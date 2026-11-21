// Generalized Silver Bullet (SB) controller plugin.
//
// SB spans the whole spectrum of static counter allocation:
//   subbank_size = 1            -> one counter per row (PRAC / Chronus-like per-row counting)
//   subbank_size = rows_per_bank -> one counter per bank (PRFM-like bank activation counting)
//
// Bookkeeping (FRAC / PENDING / LOCAL_INDEX, counter regions, highest-PENDING-first,
// round-robin LOCAL_INDEX) lives in silver_bullet_core.h and follows Yaglikci et al. 2021,
// Sections 2-3 and Table 3. This file connects it to the Ramulator command stream:
//
//   * Every row-open is attributed to the row it opens:
//       - demand ACT                        -> the ACT's row
//       - REFab / REFsb                     -> rows given by the SALT-C REF mapping (ref_coordinate)
//       - SB preventive refresh (ACT+PRE)   -> the ACT's row (observed when issued on the bus)
//       - SB preventive refresh inside RFM  -> the LOCAL_INDEX row (at RFM issue time)
//     Demand ACTs always hammer the counter regions. Refresh row-opens hammer them only if
//     count_refreshes = true (default; the 2021 report counts them).
//
//   * Consumption (R refreshes per window of T demand activations of a bank):
//       mc_act_pre   : budget[b] = R at every T-th demand ACT of bank b; while budget and PENDING
//                      remain, the MC enqueues an ACT+PRE pair to the LOCAL_INDEX row of the
//                      highest-PENDING subbank.
//       rfm          : RFM on demand. When bank b has PENDING > 0 and at least T demand ACTs have
//                      reached it since its last RFM (or since start), the MC issues RFMsb (DDR5 same-bank RFM,
//                      i.e. bank index b in every bank group). Every bank covered by an RFM
//                      performs up to R preventive refreshes.
//       periodic_rfm : PRFM. The MC issues RFMsb whenever a bank's ACT counter reaches rfm_th
//                      (identical issue rule to rfm_manager.cpp, rfm_plus = false). Every covered
//                      bank performs up to R preventive refreshes, highest PENDING first; an RFM
//                      to a bank with no PENDING refreshes nothing.

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/pluginutil/device_config.h"
#include "dram_controller/impl/plugin/silver_bullet_core.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace Ramulator {

class SilverBullet : public IControllerPlugin, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, SilverBullet, "SilverBullet",
                                      "Generalized Silver Bullet (subbank-granularity activation counting).")

private:
    enum class Consumption { McActPre, Rfm, PeriodicRfm };

    IDRAM* m_dram = nullptr;
    DeviceConfig m_cfg;
    SB::Config m_sb;

    std::vector<SB::Bank> m_banks;
    std::vector<SB::RefMapper> m_ref_maps;

    Consumption m_mode = Consumption::McActPre;
    int m_R = 1;
    int m_T = 1;
    int m_rfm_th = -1;
    bool m_count_refreshes = true;
    bool m_enforce_obs1 = true;
    int m_pending_bits = 0;
    uint32_t m_pending_cap = 0;
    int m_rows_per_ref = 1;
    bool m_debug = false;
    std::string m_region_name;
    std::string m_mode_name;

    // per flat bank
    std::vector<int> m_window_acts;     // demand ACTs in the current T window (mc_act_pre) / since last RFM (rfm)
    std::vector<int> m_budget;          // mc_act_pre: remaining refreshes in this window
    std::vector<int> m_inflight_actpre; // mc_act_pre: preventive ACTs enqueued but not yet issued
    std::vector<bool> m_rfm_inflight;   // rfm: an RFM covering this bank is enqueued
    std::vector<int> m_prfm_ctr;        // periodic_rfm: ACTs since last RFM

    int m_cmd_ACT = -1, m_cmd_REFab = -1, m_cmd_REFsb = -1, m_cmd_RFMab = -1, m_cmd_RFMsb = -1;
    int m_req_open = -1, m_req_close = -1, m_req_rfmsb = -1;
    int m_num_levels = -1;

    // stats
    uint64_t s_demand_acts = 0;
    uint64_t s_ref_row_opens = 0;
    uint64_t s_prev_refreshes = 0;          // preventive refreshes performed (rows)
    uint64_t s_prev_act_issued = 0;         // mc_act_pre: preventive ACTs observed on the bus
    uint64_t s_pending_produced = 0;
    uint64_t s_pending_produced_by_refresh = 0;
    uint64_t s_rfm_issued = 0;              // RFM commands enqueued by SB
    uint64_t s_rfm_bank_ops = 0;            // (RFM command, covered bank) pairs observed
    uint64_t s_rfm_bank_ops_empty = 0;      // ... of which found no PENDING in the bank
    uint64_t s_pending_overflows = 0;
    uint64_t s_max_pending = 0;
    uint64_t s_final_total_pending = 0;
    int s_obs1_k = 0;

public:
    void init() override {
        m_sb.subbank_size = param<int>("subbank_size").desc("Rows per subbank (power of two, 1..rows per bank; -1 = whole bank)").default_val(1024);
        m_sb.D = param<int>("D").desc("Subbank hammers per produced preventive refresh").required();
        m_region_name = param<std::string>("counter_region").desc("own | own+blast_radius_rows | own+neighbor_subarrays").default_val("own+blast_radius_rows");
        m_sb.blast_radius = param<int>("blast_radius").default_val(1);
        m_sb.subarray_size = param<int>("subarray_size").default_val(1024);
        m_mode_name = param<std::string>("consumption").desc("mc_act_pre | rfm | periodic_rfm").default_val("mc_act_pre");
        m_R = param<int>("R").desc("Preventive refreshes per window (per RFM in RFM modes)").default_val(1);
        m_T = param<int>("T").desc("Demand activations per window").default_val(-1);
        m_rfm_th = param<int>("rfm_th").desc("periodic_rfm: bank ACTs per RFM (default T)").default_val(-1);
        m_count_refreshes = param<bool>("count_refreshes").desc("Refresh row-opens (REF, preventive) hammer counter regions").default_val(true);
        m_enforce_obs1 = param<bool>("enforce_obs1").desc("Reject configs violating D >= 2(T/R+1)").default_val(true);
        m_pending_bits = param<int>("pending_bits").desc("PENDING width in bits for overflow accounting (0 = unbounded)").default_val(0);
        m_debug = param<bool>("debug").default_val(false);
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        m_ctrl = cast_parent<IDRAMController>();
        m_dram = m_ctrl->m_dram;
        m_cfg.set_device(m_ctrl);
        m_num_levels = m_dram->m_levels.size();

        m_sb.rows_per_bank = m_dram->get_level_size("row");
        if (m_sb.subbank_size <= 0) m_sb.subbank_size = m_sb.rows_per_bank;
        try {
            m_sb.region = SB::parse_region(m_region_name);
            m_sb.validate();
        } catch (const std::invalid_argument& e) {
            throw ConfigurationError("[SilverBullet] {}", e.what());
        }

        if      (m_mode_name == "mc_act_pre")   m_mode = Consumption::McActPre;
        else if (m_mode_name == "rfm")          m_mode = Consumption::Rfm;
        else if (m_mode_name == "periodic_rfm") m_mode = Consumption::PeriodicRfm;
        else throw ConfigurationError("[SilverBullet] unknown consumption '{}'", m_mode_name);

        if (m_mode == Consumption::PeriodicRfm) {
            if (m_rfm_th <= 0) m_rfm_th = m_T;
            if (m_rfm_th <= 0) throw ConfigurationError("[SilverBullet] periodic_rfm needs rfm_th (or T) > 0");
            if (m_T <= 0) m_T = m_rfm_th;
        }
        if (m_T <= 0) throw ConfigurationError("[SilverBullet] T must be > 0");
        if (m_R <= 0) throw ConfigurationError("[SilverBullet] R must be > 0");

        // Observation 1: D >= 2(T/R + 1). In periodic_rfm the window is the RFM interval.
        int T_eff = (m_mode == Consumption::PeriodicRfm) ? m_rfm_th : m_T;
        s_obs1_k = m_sb.max_subbanks_per_hammer();
        if (!SB::satisfies_obs1(m_sb.D, T_eff, m_R, 2)) {
            if (m_enforce_obs1)
                throw ConfigurationError("[SilverBullet] Observation 1 violated: D={} < 2(T/R+1) with T={} R={}", m_sb.D, T_eff, m_R);
            std::printf("[SilverBullet] WARNING: Observation 1 violated: D=%d < 2(T/R+1), T=%d R=%d\n", m_sb.D, T_eff, m_R);
        }
        if (!SB::satisfies_obs1(m_sb.D, T_eff, m_R, s_obs1_k)) {
            std::printf("[SilverBullet] WARNING: a row-open hammers up to k=%d subbanks; production can exceed "
                        "consumption unless D >= k(T/R+1) = %.1f (D=%d)\n",
                        s_obs1_k, s_obs1_k * ((double)T_eff / m_R + 1), m_sb.D);
        }

        if (m_pending_bits > 0) m_pending_cap = (m_pending_bits >= 32) ? UINT32_MAX : ((1u << m_pending_bits) - 1);

        m_rows_per_ref = std::max(1, m_sb.rows_per_bank / 8192);   // as in salt_c.cpp
        int nb = m_cfg.m_num_banks;
        m_banks.reserve(nb);
        m_ref_maps.reserve(nb);
        for (int b = 0; b < nb; b++) {
            m_banks.emplace_back(m_sb);
            m_ref_maps.emplace_back(m_sb.rows_per_bank, m_sb.subarray_size, m_rows_per_ref);
        }
        m_window_acts.assign(nb, 0);
        m_budget.assign(nb, m_R);
        m_inflight_actpre.assign(nb, 0);
        m_rfm_inflight.assign(nb, false);
        m_prfm_ctr.assign(nb, 0);

        m_cmd_ACT   = m_dram->m_commands("ACT");
        m_cmd_REFab = m_dram->m_commands("REFab");
        m_cmd_REFsb = m_dram->m_commands.contains("REFsb") ? m_dram->m_commands("REFsb") : -1;
        m_cmd_RFMab = m_dram->m_commands.contains("RFMab") ? m_dram->m_commands("RFMab") : -1;
        m_cmd_RFMsb = m_dram->m_commands.contains("RFMsb") ? m_dram->m_commands("RFMsb") : -1;
        m_req_open  = m_dram->m_requests("open-row");
        m_req_close = m_dram->m_requests("close-row");
        if (m_mode != Consumption::McActPre) {
            if (!m_dram->m_requests.contains("same-bank-rfm"))
                throw ConfigurationError("[SilverBullet] RFM consumption needs a DRAM with same-bank-rfm");
            m_req_rfmsb = m_dram->m_requests("same-bank-rfm");
        }

        register_stat(s_demand_acts).name("sb_demand_acts");
        register_stat(s_ref_row_opens).name("sb_ref_row_opens");
        register_stat(s_prev_refreshes).name("sb_preventive_refreshes");
        register_stat(s_prev_act_issued).name("sb_preventive_acts_issued");
        register_stat(s_pending_produced).name("sb_pending_produced");
        register_stat(s_pending_produced_by_refresh).name("sb_pending_produced_by_refresh_row_opens");
        register_stat(s_rfm_issued).name("sb_rfm_issued");
        register_stat(s_rfm_bank_ops).name("sb_rfm_bank_ops");
        register_stat(s_rfm_bank_ops_empty).name("sb_rfm_bank_ops_without_pending");
        register_stat(s_pending_overflows).name("sb_pending_overflows");
        register_stat(s_max_pending).name("sb_max_pending");
        register_stat(s_final_total_pending).name("sb_final_total_pending");
        register_stat(s_obs1_k).name("sb_max_subbanks_per_hammer");

        std::printf("[SilverBullet] S_SB=%d N_SB=%d D=%d region=%s B=%d SA=%d consumption=%s R=%d T=%d rfm_th=%d "
                    "count_refreshes=%d k=%d\n",
                    m_sb.subbank_size, m_sb.num_subbanks(), m_sb.D, m_region_name.c_str(), m_sb.blast_radius,
                    m_sb.subarray_size, m_mode_name.c_str(), m_R, m_T, m_rfm_th, (int)m_count_refreshes, s_obs1_k);
    }

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
        if (!request_found) return;
        Request& req = *req_it;
        int cmd = req.command;

        if (cmd == m_cmd_ACT) {
            on_act(req);
        } else if (cmd == m_cmd_REFab || (m_cmd_REFsb >= 0 && cmd == m_cmd_REFsb)) {
            for_each_covered_bank(req, [&](int b) { on_ref(b); });
        } else if (m_mode != Consumption::McActPre &&
                   ((m_cmd_RFMab >= 0 && cmd == m_cmd_RFMab) || (m_cmd_RFMsb >= 0 && cmd == m_cmd_RFMsb))) {
            for_each_covered_bank(req, [&](int b) { on_rfm(b); });
            if (m_mode == Consumption::Rfm)
                for_each_covered_bank(req, [&](int b) { try_issue_rfm(b); });
        }
    }

    void reset_stats() override {
        Implementation::reset_stats();
        for (auto& bk : m_banks) bk.reset_max_pending();
    }

    void finalize() override {
        uint64_t mx = 0, tot = 0;
        for (auto& bk : m_banks) { mx = std::max<uint64_t>(mx, bk.max_pending()); tot += bk.total_pending(); }
        s_max_pending = mx;
        s_final_total_pending = tot;
        s_obs1_k = m_sb.max_subbanks_per_hammer();
    }

private:
    template <typename F>
    void for_each_covered_bank(const Request& req, F&& f) {
        int rank = req.addr_vec[m_cfg.m_rank_level];
        int bg = req.addr_vec[m_cfg.m_bankgroup_level];
        int bank = req.addr_vec[m_cfg.m_bank_level];
        int bpr = m_cfg.m_num_banks_per_rank, bpg = m_cfg.m_num_banks_per_bankgroup;
        for (int g = 0; g < m_cfg.m_num_bankgroups; g++) {
            if (bg != -1 && g != bg) continue;
            for (int k = 0; k < bpg; k++) {
                if (bank != -1 && k != bank) continue;
                f(rank * bpr + g * bpg + k);
            }
        }
    }

    // Row-open at (bank, row). Returns pending produced.
    int hammer(int b, int row, bool is_refresh) {
        int produced = m_banks[b].hammer(row);
        if (produced) {
            s_pending_produced += produced;
            if (is_refresh) s_pending_produced_by_refresh += produced;
            if (m_pending_cap) {
                int top = m_banks[b].top_subbank();
                if (top >= 0 && m_banks[b].pending(top) > m_pending_cap) s_pending_overflows++;
            }
            if (m_debug) {
                auto [lo, hi] = m_sb.hammered_subbanks(row);
                std::printf("[SB] PRODUCE bank=%d row=%d subbanks=[%d,%d] produced=%d total_pending=%lu\n",
                            b, row, lo, hi, produced, (unsigned long)m_banks[b].total_pending());
            }
        }
        return produced;
    }

    void on_act(const Request& req) {
        int b = m_cfg.get_flat_bank_id(req);
        int row = req.addr_vec[m_cfg.m_row_level];
        bool is_prev = (req.type_id == m_req_open) && m_inflight_actpre[b] > 0;

        if (is_prev) {
            m_inflight_actpre[b]--;
            s_prev_act_issued++;
            if (m_count_refreshes) hammer(b, row, true);
        } else {
            s_demand_acts++;
            hammer(b, row, false);
        }

        switch (m_mode) {
        case Consumption::McActPre:
            if (!is_prev && ++m_window_acts[b] >= m_T) {
                m_window_acts[b] = 0;
                m_budget[b] = m_R;
            }
            try_issue_actpre(b);
            break;
        case Consumption::Rfm:
            if (!is_prev) m_window_acts[b]++;
            try_issue_rfm(b);
            break;
        case Consumption::PeriodicRfm:
            if (++m_prfm_ctr[b] >= m_rfm_th) issue_periodic_rfm(req, b);
            break;
        }
    }

    void on_ref(int b) {
        m_ref_maps[b].next_ref([&](int row) {
            s_ref_row_opens++;
            if (m_count_refreshes) hammer(b, row, true);
        });
        if (m_mode == Consumption::McActPre) try_issue_actpre(b);
        else if (m_mode == Consumption::Rfm) try_issue_rfm(b);
    }

    // An RFM reached bank b: perform up to R preventive refreshes in-DRAM.
    void on_rfm(int b) {
        s_rfm_bank_ops++;
        m_rfm_inflight[b] = false;
        if (m_mode == Consumption::Rfm) m_window_acts[b] = 0;
        if (!m_banks[b].has_pending()) s_rfm_bank_ops_empty++;
        for (int i = 0; i < m_R; i++) {
            int sb = m_banks[b].top_subbank();
            int row = m_banks[b].pop_refresh();
            if (row < 0) break;
            s_prev_refreshes++;
            if (m_debug) std::printf("[SB] RFM-REFRESH bank=%d subbank=%d row=%d\n", b, sb, row);
            if (m_count_refreshes) hammer(b, row, true);
        }
    }

    std::vector<int> bank_addr_vec(int b, int row) const {
        std::vector<int> av(m_num_levels, -1);
        int bpr = m_cfg.m_num_banks_per_rank, bpg = m_cfg.m_num_banks_per_bankgroup;
        av[0] = m_ctrl->m_channel_id;
        av[m_cfg.m_rank_level] = b / bpr;
        av[m_cfg.m_bankgroup_level] = (b % bpr) / bpg;
        av[m_cfg.m_bank_level] = (b % bpr) % bpg;
        av[m_cfg.m_row_level] = row;
        av[m_cfg.m_col_level] = 0;
        return av;
    }

    void try_issue_actpre(int b) {
        auto& bk = m_banks[b];
        while (m_budget[b] > 0 && bk.has_pending()) {
            int sb = bk.top_subbank();
            int row = bk.pop_refresh();
            m_budget[b]--;
            s_prev_refreshes++;
            m_inflight_actpre[b]++;
            auto av = bank_addr_vec(b, row);
            Request act(av, m_req_open);
            Request pre(av, m_req_close);
            if (!m_ctrl->priority_send(act) || !m_ctrl->priority_send(pre))
                throw std::runtime_error("[SilverBullet] priority_send failed for preventive ACT+PRE");
            if (m_debug) std::printf("[SB] ACTPRE bank=%d subbank=%d row=%d budget=%d\n", b, sb, row, m_budget[b]);
        }
    }

    void send_rfmsb(int b) {
        auto av = bank_addr_vec(b, 0);
        av[m_cfg.m_bankgroup_level] = -1;   // DDR5 RFMsb: this bank index in every bank group
        Request rfm(av, m_req_rfmsb);
        if (!m_ctrl->priority_send(rfm))
            throw std::runtime_error("[SilverBullet] priority_send failed for RFMsb");
        s_rfm_issued++;
        int rank = b / m_cfg.m_num_banks_per_rank;
        int k = (b % m_cfg.m_num_banks_per_rank) % m_cfg.m_num_banks_per_bankgroup;
        for (int g = 0; g < m_cfg.m_num_bankgroups; g++)
            m_rfm_inflight[rank * m_cfg.m_num_banks_per_rank + g * m_cfg.m_num_banks_per_bankgroup + k] = true;
    }

    void try_issue_rfm(int b) {
        if (m_rfm_inflight[b] || m_window_acts[b] < m_T || !m_banks[b].has_pending()) return;
        if (m_debug) std::printf("[SB] RFM-ON-DEMAND bank=%d total_pending=%lu\n", b, (unsigned long)m_banks[b].total_pending());
        send_rfmsb(b);
    }

    // Same issue rule as rfm_manager.cpp (rfm_plus = false): reset the ACT counters of this bank
    // index in every bank group of the rank and send one RFMsb.
    void issue_periodic_rfm(const Request& req, int b) {
        int rank = b / m_cfg.m_num_banks_per_rank;
        int k = (b % m_cfg.m_num_banks_per_rank) % m_cfg.m_num_banks_per_bankgroup;
        for (int g = 0; g < m_cfg.m_num_bankgroups; g++)
            m_prfm_ctr[rank * m_cfg.m_num_banks_per_rank + g * m_cfg.m_num_banks_per_bankgroup + k] = 0;
        send_rfmsb(b);
    }
};

}  // namespace Ramulator
