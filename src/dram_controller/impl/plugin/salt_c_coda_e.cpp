#include <vector>
#include <unordered_map>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram_controller/impl/pluginutil/device_config.h"
#include "dram_controller/impl/plugin/prac.h"

#include "frontend/frontend.h"

namespace Ramulator {

// CODA-E: reduces the Adjacent-Counter-Increment (ACI) overhead of SALT-C's
// ColumnDisturb Protection (CDP) by routing neighbor propagation through a
// small Pending-Activation Counter (PAC) instead of incrementing the
// neighbor's real ACTR on every demand activation. A demand activation to a
// subarray "overrides" (decrements) that subarray's own PAC, since real
// demand activity already covers its mitigation needs. A PAC that would
// overflow its max value spills over into one real ACTR increment instead.
class SALTCCODAE : public IControllerPlugin, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, SALTCCODAE, "SALTCCODAE", "SALT-C ColumnDisturb mitigation with CODA-E reduced-ACI.")

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
        int m_pac_bits = -1;
        int m_pac_max = -1;
        bool m_is_debug = false;

        struct SubarrayState {
            int actr = 0;
            int rptr = 0;
            int pac = 0;
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

        int s_coda_e_demand_acts = 0;
        int s_coda_e_demand_decrements = 0;
        int s_coda_e_pac_increments = 0;
        int s_coda_e_pac_spillover_events = 0;


    public:
    void init() override {
        m_ath = param<int>("ath").desc("Alert threshold").required();
        m_apm = param<int>("apm").desc("Counter decrement per ABO").required();
        m_bundle = param<int>("bundle").desc("Rows refreshed per ABO").required();
        m_subarray_size = param<int>("subarray_size").desc("Rows per subarray").required();
        m_pac_bits = param<int>("pac_bits").desc("Bits allocated to the CODA-E Pending-Activation Counter").default_val(4);
        m_is_debug = param<bool>("debug").default_val(false);
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
        m_rows_per_ref = rows_per_bank / 8192;
        if (m_rows_per_ref < 1) m_rows_per_ref = 1;
        m_rr_sa.resize(m_cfg.m_num_banks);

        m_pac_max = (1 << m_pac_bits) - 1;

        for (auto* plugin : m_ctrl->m_plugins){
            if (auto* p = dynamic_cast<IPRAC*>(plugin)){
                m_prac = p;
                break;
            }
        }

        if (!m_prac){
            throw ConfigurationError("[SALTCCODAE] requires a PRAC plugin for ABO-based refresh.");
        }

        m_state.resize(m_cfg.m_num_banks);
        m_css.resize(m_cfg.m_num_banks);

        register_stat(s_saltc_REF_refreshes).name("saltc_row_refreshes_in_REF_command");
        register_stat(s_saltc_RFMab_refreshes).name("saltc_row_refreshes_in_RFM_command (total count of ranks)");
        register_stat(s_saltc_RFMab_commands).name("saltc_RFM_commands");
        register_stat(s_saltc_ath_events).name("saltc_ath_events");

        register_stat(s_coda_e_demand_acts).name("coda_e_demand_acts");
        register_stat(s_coda_e_demand_decrements).name("coda_e_demand_pac_decrements");
        register_stat(s_coda_e_pac_increments).name("coda_e_pac_increments");
        register_stat(s_coda_e_pac_spillover_events).name("coda_e_pac_spillover_events (ACI events that reached real ACTR)");
    }

    void update(bool request_found, ReqBuffer::iterator& req_it) override{
        if (!request_found)
            return;
        bool warmup_active = m_frontend && !m_frontend->is_warmup_done();
        auto& req = *req_it;

        if (m_dram->m_command_meta(req.command).is_opening &&
            m_dram->m_command_scopes(req.command) == m_row_level) {
            int flat_bank = m_cfg.get_flat_bank_id(req);
            int row = req.addr_vec[m_row_level];
            int sa = row / m_subarray_size;

            s_coda_e_demand_acts++;
            count_and_check(flat_bank, sa);
            decrement_pac(flat_bank, sa);

            aci_increment(flat_bank, sa - 1);
            aci_increment(flat_bank, sa + 1);

            if (warmup_active && m_css[flat_bank].valid && m_css[flat_bank].actr >= m_ath)
                refresh_due_subarray(flat_bank);
        }

        // RFMab and REFab act only on the banks of the addressed rank.
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


    private:
    void count_and_check(int flat_bank, int sa){
        if (sa < 0 || sa >= m_num_subarrays) return;
        auto& st = m_state[flat_bank][sa];
        st.actr++;

        auto& css = m_css[flat_bank];
        if(!css.valid || st.actr > css.actr){
            css.valid = true;
            css.sa = sa;
            css.actr = st.actr;

        }
        if (st.actr >= m_ath){
            m_prac->set_abo_needed(
                true, IPRAC::ABOTrigger::SALTC, flat_bank);
            s_saltc_ath_events++;
            if (m_is_debug)
                std::printf("[SALTC CODA-E] ATH CROSS bank=%d sa=%d actr=%d >= ath=%d -> ABO\n",
                            flat_bank, sa, st.actr, m_ath);
        }
    }

    // CODA-E: neighbor propagation goes through PAC instead of a direct
    // ACTR increment. Only when PAC would overflow do we spill one real
    // ACTR increment onto the neighbor.
    void aci_increment(int flat_bank, int sa){
        if (sa < 0 || sa >= m_num_subarrays) return;
        auto& st = m_state[flat_bank][sa];
        if (st.pac + 1 > m_pac_max){
            st.pac = 0;
            s_coda_e_pac_spillover_events++;
            count_and_check(flat_bank, sa);
            if (m_is_debug)
                std::printf("[SALTC CODA-E] PAC SPILLOVER bank=%d sa=%d -> real ACTR increment\n",
                            flat_bank, sa);
        } else {
            st.pac++;
            s_coda_e_pac_increments++;
        }
    }

    // CODA-E: a subarray that receives real activity (demand ACT or a
    // mitigation refresh to itself) no longer needs pending ACI credit, so
    // we let that activity override/cancel one pending PAC increment.
    void decrement_pac(int flat_bank, int sa){
        if (sa < 0 || sa >= m_num_subarrays) return;
        auto& st = m_state[flat_bank][sa];
        if (st.pac > 0){
            st.pac--;
            s_coda_e_demand_decrements++;
        }
    }

    void refresh_due_subarray(int flat_bank){
        auto& css = m_css[flat_bank];
        if (!css.valid) return;

        int rsa = css.sa;
        auto& st = m_state[flat_bank][rsa];
        for (int i=0; i< m_bundle; i++){
            st.rptr = (st.rptr + 1) % m_subarray_size;
            s_saltc_RFMab_refreshes++;
        }

        for (int i = 0; i < m_bundle; i++){
            count_and_check(flat_bank, rsa);
            decrement_pac(flat_bank, rsa);
            aci_increment(flat_bank, rsa - 1);
            aci_increment(flat_bank, rsa + 1);
        }

        int before = st.actr;
        st.actr -= m_apm;

        if (st.actr < 0) st.actr = 0;

        if (css.valid && css.sa == rsa){
            css.actr = st.actr;
            if (css.actr <= 0) css.valid = false;
        }

        if (m_is_debug)
            std::printf("[SALTC CODA-E] RFM DRAIN bank=%d sa=%d actr %d->%d (bundle=%d)\n",
                        flat_bank, rsa, before, st.actr, m_bundle);

            }

    void ref_coordinate(int flat_bank){
        auto& bank = m_state[flat_bank];
        auto& css = m_css[flat_bank];
        int& rr = m_rr_sa[flat_bank];

        for (int k = 0; k < m_rows_per_ref; k++){
            int sa = (rr + k) % m_num_subarrays;
            auto it = bank.find(sa);
            if (it == bank.end()) continue;
            auto& st = it->second;
            if (st.actr <= 0) continue;

            int per_dec = ((st.rptr + 1) * m_apm / m_bundle) - (st.rptr * m_apm / m_bundle);
            st.rptr = (st.rptr + 1) % m_subarray_size;
            st.actr -= per_dec;
            if (st.actr < 0) st.actr = 0;
            s_saltc_REF_refreshes++;

            if (css.valid && css.sa == sa) {
            css.actr = st.actr;
            if (css.actr <= 0) css.valid = false;
            }
        }
        rr = (rr + m_rows_per_ref) % m_num_subarrays;
    }

};

}
