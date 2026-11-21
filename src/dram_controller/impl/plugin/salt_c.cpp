#include <vector>
#include <unordered_map>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/pluginutil/device_config.h"
#include "dram_controller/impl/plugin/prac.h"

#include "frontend/frontend.h"

namespace Ramulator {

class SALTC : public IControllerPlugin, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, SALTC, "SALTC", "SALT-C ColumnDisturb mitigation.")
    
    private:
        IDRAM* m_dram = nullptr;
        DeviceConfig m_cfg;
        IPRAC* m_prac = nullptr;
        IFrontEnd* m_frontend = nullptr;

        int m_row_level = -1;
        int m_RFMab_id = -1;
        int m_num_subarrays = -1;

        int m_REFab_id = -1;
        int m_rows_per_ref = -1;
        std::vector<int> m_rr_sa;
        

        int m_ath = -1;
        int m_apm = -1;
        int m_bundle = -1;
        int m_subarray_size = -1;
        bool m_neighbor_propagation = true; // always true for ColunDisturb mtiigation
        bool m_is_debug = false;

        enum class Shadow { IDLE, WAIT, DELAY };
        Shadow m_shadow = Shadow::IDLE;
        bool m_shadow_trigger = false;
        Clk_t m_clk = 0;
        Clk_t m_shadow_start = 0;
        int m_shadow_delay_rem = 0;
        int m_abo_act_ns = 180;
        int m_abo_cycles = -1;
        int m_recovery_refs = 1;
        int m_delay_acts = 4;

        // Back-off after an ALERT:
        //   prac    (default): the IPRAC plugin's own back-off, e.g. PRAC's
        //                      abo_recovery_refs RFMs per ALERT.
        //   chronus          : RFMs until no bank's selected subarray (CSS) is
        //                      at or above ATH. Needs the Chronus plugin (with
        //                      its RowHammer threshold off) placed after SALTC.
        //                      Each RFM here adds bundle counts to its subarray
        //                      and to each neighbor, so RFMs drain the counters
        //                      only if apm > 3 * bundle (apm > bundle without
        //                      neighbor_propagation); otherwise recovery never ends.
        bool m_backoff_chronus = false;
        int m_chronus_max_rounds = 100000;

        struct SubarrayState {
            int actr = 0;
            int rptr = 0;
        };
        std::vector<std::unordered_map<int,SubarrayState>> m_state; // [flat_bank][subarray]


        struct CSS{
            bool valid = false;
            int sa = 0;
            int actr = 0;
        };
        std::vector<CSS> m_css;

        int s_saltc_REF_refreshes = 0;
        int s_saltc_RFMab_refreshes = 0;
        int s_saltc_RFMab_commands = 0;
        int s_saltc_ath_events = 0;
        int s_saltc_max_actr = 0;


    public:
    void init() override {
        m_ath = param<int>("ath").desc("Alert threshold").required();
        m_apm = param<int>("apm").desc("Counter decrement per ABO").required();
        m_bundle = param<int>("bundle").desc("Rows refreshed per ABO").required();
        m_subarray_size = param<int>("subarray_size").desc("Rows per subarray").required();
        m_neighbor_propagation = param<bool>("neighbor_propagation").default_val(true);
        m_is_debug = param<bool>("debug").default_val(false);
        m_abo_act_ns = param<int>("abo_act_ns").default_val(180);
        m_delay_acts = param<int>("abo_delay_acts").default_val(4);
        std::string backoff = param<std::string>("backoff").desc("prac | chronus").default_val("prac");
        if (backoff != "prac" && backoff != "chronus")
            throw ConfigurationError("[SALTC] backoff must be prac or chronus, got {}", backoff);
        m_backoff_chronus = (backoff == "chronus");
        m_chronus_max_rounds = param<int>("chronus_max_rounds").desc("Warmup cap on RFM rounds per back-off (backoff: chronus)").default_val(100000);
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        m_ctrl = cast_parent<IDRAMController>();
        m_dram = m_ctrl->m_dram;
        m_cfg.set_device(m_ctrl);
        m_frontend = frontend;

        int rows_per_bank = m_dram->get_level_size("row");
        m_num_subarrays = rows_per_bank / m_subarray_size;

        m_row_level = m_dram->m_levels("row");
        m_RFMab_id = m_dram->m_commands("RFMab");

        m_REFab_id = m_dram->m_commands("REFab");
        m_abo_cycles = m_abo_act_ns / ((float)m_dram->m_timing_vals("tCK_ps") / 1000.0f);
        m_rows_per_ref = rows_per_bank / 8192;
        if (m_rows_per_ref < 1) m_rows_per_ref = 1;
        m_rr_sa.resize(m_cfg.m_num_banks);

        
        for (auto* plugin : m_ctrl->m_plugins){
            if (auto* p = dynamic_cast<IPRAC*>(plugin)){
                m_prac = p;
                break;
            }
        }

        if (!m_prac){
            throw ConfigurationError("[SALTC] requires a PRAC plugin for ABO-based refresh.");
        }
        if (m_backoff_chronus && dynamic_cast<Implementation*>(m_prac)->get_name() != "Chronus")
            throw ConfigurationError("[SALTC] backoff: chronus requires the Chronus plugin.");
        m_recovery_refs = m_prac->get_num_abo_recovery_refs();
        m_prac->set_pending_work_fn([this]() -> bool {
            for (int b = 0; b < m_cfg.m_num_banks; b++)
                if (css_pending(b)) return true;
            return false;
        });

        m_state.resize(m_cfg.m_num_banks);
        m_css.resize(m_cfg.m_num_banks);

        register_stat(s_saltc_REF_refreshes).name("saltc_row_refreshes_in_REF_command");
        register_stat(s_saltc_RFMab_refreshes).name("saltc_row_refreshes_in_RFM_command (total count of ranks)");
        register_stat(s_saltc_RFMab_commands).name("saltc_RFM_commands");
        register_stat(s_saltc_ath_events).name("saltc_ath_events");
        register_stat(s_saltc_max_actr).name("saltc_max_actr");


    }

    void update(bool request_found, ReqBuffer::iterator& req_it) override{
        m_clk++;
        bool warmup_active = m_frontend && !m_frontend->is_warmup_done();
        bool delay_ended = false;
        if (!warmup_active) m_shadow = Shadow::IDLE;
        else if (m_shadow == Shadow::WAIT && m_clk >= m_shadow_start) shadow_recovery();
        m_shadow_trigger = false;

        if (request_found) {
            process(*req_it);
            bool is_act = m_dram->m_command_meta(req_it->command).is_opening &&
                          m_dram->m_command_scopes(req_it->command) == m_row_level;
            if (warmup_active && is_act && m_shadow == Shadow::DELAY && --m_shadow_delay_rem == 0) {
                m_shadow = Shadow::IDLE;
                delay_ended = true;
            }
        }

        if (warmup_active && m_shadow == Shadow::IDLE) {
            bool any_pending = false;
            for (int b = 0; b < m_cfg.m_num_banks && !any_pending; b++) any_pending = css_pending(b);
            if (delay_ended ? any_pending : m_shadow_trigger) {
                m_shadow = Shadow::WAIT;
                m_shadow_start = m_clk + (delay_ended ? 1 : 0) + m_abo_cycles;
            }
        }
        m_shadow_trigger = false;
    }

    private:
    bool css_pending(int flat_bank) const {
        return m_css[flat_bank].valid && m_css[flat_bank].actr >= m_ath;
    }

    void process(Request& req){

        if (m_dram->m_command_meta(req.command).is_opening &&
            m_dram->m_command_scopes(req.command) == m_row_level) {
            int flat_bank = m_cfg.get_flat_bank_id(req);
            int row = req.addr_vec[m_row_level];
            int sa = row / m_subarray_size;
            if (m_is_debug) std::printf("[SALTV] ACT bank=%d sa=%d\n", flat_bank, sa);

            count_and_check(flat_bank, sa);

            if (m_neighbor_propagation){
                count_and_check(flat_bank, sa - 1);
                count_and_check(flat_bank, sa + 1);
            }
        }

        if (req.command == m_RFMab_id) {
            int rank = req.addr_vec[m_cfg.m_rank_level];
            int bpr = m_cfg.m_num_banks_per_rank;
            for (int b = 0; b < bpr; b++){
                refresh_due_subarray(rank * bpr + b);
            }
            s_saltc_RFMab_commands++;
        }

        if(req.command == m_REFab_id){
            int rank = req.addr_vec[m_cfg.m_rank_level];
            int bpr = m_cfg.m_num_banks_per_rank;
            for (int b = 0; b < bpr; b++){
                ref_coordinate(rank * bpr + b);
            }
        }

    }

    void shadow_recovery(){
        if (m_is_debug) std::printf("[SALTV] SHADOW-RECOVERY clk=%lu\n", (unsigned long)m_clk);
        int bpr = m_cfg.m_num_banks_per_rank;
        auto one_round = [&]() {
            for (int r = 0; r < m_cfg.m_num_ranks; r++)
                for (int b = 0; b < bpr; b++) refresh_due_subarray(r * bpr + b);
        };
        if (m_backoff_chronus) {
            // Chronus: RFMs until nothing is pending, no post-recovery delay.
            for (int i = 0; i < m_chronus_max_rounds && any_css_pending(); i++) one_round();
            m_shadow = Shadow::IDLE;
            return;
        }
        for (int i = 0; i < m_recovery_refs; i++) one_round();
        m_shadow = Shadow::DELAY;
        m_shadow_delay_rem = m_delay_acts;
    }

    bool any_css_pending() const {
        for (int b = 0; b < m_cfg.m_num_banks; b++)
            if (css_pending(b)) return true;
        return false;
    }

    void count_and_check(int flat_bank, int sa){
        if (sa < 0 || sa >= m_num_subarrays) return;
        auto& st = m_state[flat_bank][sa];
        st.actr++;
        if (st.actr > s_saltc_max_actr) s_saltc_max_actr = st.actr;

        auto& css = m_css[flat_bank];
        if(!css.valid || st.actr > css.actr){
            css.valid = true;
            css.sa = sa;
            css.actr = st.actr;

        }
        if (m_is_debug)
            std::printf("[SALTV] CNT bank=%d sa=%d actr=%d css=%d:%d:%d\n",
                        flat_bank, sa, st.actr, (int)css.valid, css.sa, css.actr);
        if (st.actr >= m_ath){
            if (m_frontend && !m_frontend->is_warmup_done())
                m_shadow_trigger = true;
            else
                m_prac->set_abo_needed(
                    true, IPRAC::ABOTrigger::SALTC, flat_bank);
            s_saltc_ath_events++;
            if (m_is_debug)
                std::printf("[SALTV] ATH bank=%d sa=%d actr=%d\n", flat_bank, sa, st.actr);
        }
    }

    void refresh_due_subarray(int flat_bank){
        auto& css = m_css[flat_bank];
        if (m_is_debug)
            std::printf("[SALTV] RFM bank=%d css=%d\n", flat_bank, css.valid ? css.sa : -1);
        if (!css.valid) return;

        int rsa = css.sa;
        auto& st = m_state[flat_bank][rsa];
        int rptr0 = st.rptr;
        for (int i=0; i< m_bundle; i++){
            st.rptr = (st.rptr + 1) % m_subarray_size;
            s_saltc_RFMab_refreshes++;
        }
        if (m_is_debug)
            std::printf("[SALTV] RFM-ROWS bank=%d sa=%d rptr %d->%d\n", flat_bank, rsa, rptr0, st.rptr);

        for (int i = 0; i < m_bundle; i++){
            count_and_check(flat_bank, rsa);
            if (m_neighbor_propagation){
                count_and_check(flat_bank, rsa - 1);
                count_and_check(flat_bank, rsa + 1);
            }
        }

        int before = st.actr;
        st.actr -= m_apm;

        if (st.actr < 0) st.actr = 0;

        if (css.valid && css.sa == rsa){
            css.actr = st.actr;
            if (css.actr <= 0) css.valid = false;
        }

        if (m_is_debug)
            std::printf("[SALTV] RFM-DEC bank=%d sa=%d actr %d->%d css=%d:%d:%d\n",
                        flat_bank, rsa, before, st.actr, (int)css.valid, css.sa, css.actr);
    }

    void ref_coordinate(int flat_bank){
        auto& bank = m_state[flat_bank];
        auto& css = m_css[flat_bank];
        int& rr = m_rr_sa[flat_bank];

        for (int k = 0; k < m_rows_per_ref; k++){
            int sa = (rr + k) % m_num_subarrays;
            auto& st = bank[sa];

            int per_dec = ((st.rptr + 1) * m_apm / m_bundle) - (st.rptr * m_apm / m_bundle);
            int rptr0 = st.rptr, actr0 = st.actr;
            st.rptr = (st.rptr + 1) % m_subarray_size;
            st.actr -= per_dec;
            if (st.actr < 0) st.actr = 0;
            s_saltc_REF_refreshes++;

            if (css.valid && css.sa == sa) {
            css.actr = st.actr;
            if (css.actr <= 0) css.valid = false;
            }
            if (m_is_debug)
                std::printf("[SALTV] REF bank=%d sa=%d rptr %d->%d actr %d->%d css=%d:%d:%d\n",
                            flat_bank, sa, rptr0, st.rptr, actr0, st.actr, (int)css.valid, css.sa, css.actr);
        }
        rr = (rr + m_rows_per_ref) % m_num_subarrays;        
    }

};

}
