#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"
#include "dram_controller/rowpolicy.h"

namespace Ramulator {

class OpenRowPolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRowPolicy, OpenRowPolicy, "OpenRowPolicy", "Open Row Policy.")
  private:
    
  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override { };

    void update(bool request_found, ReqBuffer::iterator& req_it) override { 
      // OpenRowPolicy does not need to take any actions
    };


};

class CloseRowPolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRowPolicy, CloseRowPolicy, "CloseRowPolicy", "Close Row Policy.")
  private:
    IDRAM* m_dram;
    
    int m_PRE_cmd = -1;
    int m_RD_cmd = -1;
    
    int m_cap = -1;
    int m_aqua_copy_read_id =-1;
    int m_aqua_copy_write_id =-1;
    
    int m_rank_level = -1;
    int m_bankgroup_level = -1;
    int m_bank_level = -1;
    int m_row_level = -1;
    int m_num_ranks = -1;
    int m_num_bankgroups = -1;
    int m_num_banks = -1;

    int s_num_close_reqs = 0;

    std::vector<uint64_t> m_col_accesses;

    // Rows that reached the cap and still have to be closed, per flat bank
    // (empty = nothing to close). The PRE is issued in a command slot where the
    // controller issues nothing else. Sending it through priority_send() would
    // block all other requests until the PRE can issue (tRTP/tWR/tRAS).
    std::vector<AddrVec_t> m_to_close;
    std::vector<int> m_marked;   // flat bank ids with a pending close
    size_t m_rr = 0;

    void reset_bank(int flat_bank_id) {
      m_col_accesses[flat_bank_id] = 0;
      m_to_close[flat_bank_id].clear();   // dropped from m_marked lazily
    }

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IDRAMController>();
      m_dram = m_ctrl->m_dram;

      m_cap = param<int>("cap").default_val(10000000); // TODO

      // m_rank_level = m_dram->m_levels("pseudochannel");
      m_rank_level = m_dram->m_levels("rank");
      m_bankgroup_level = m_dram->m_levels("bankgroup");
      m_bank_level = m_dram->m_levels("bank");
      m_row_level = m_dram->m_levels("row");

      m_PRE_cmd = m_dram->m_commands("PRE");
      m_RD_cmd = m_dram->m_commands("RD");
      m_aqua_copy_read_id = m_dram->m_requests("aqua-copy-read");
      m_aqua_copy_write_id = m_dram->m_requests("aqua-copy-write");


      // m_num_ranks = m_dram->get_level_size("pseudochannel");
      m_num_ranks = m_dram->get_level_size("rank");
      m_num_bankgroups = m_dram->get_level_size("bankgroup");
      m_num_banks = m_dram->get_level_size("bank");
      
      m_col_accesses.resize(m_num_banks * m_num_bankgroups * m_num_ranks, 0);
      m_to_close.resize(m_num_banks * m_num_bankgroups * m_num_ranks);

      register_stat(s_num_close_reqs).name("num_close_reqs");
    };

    void update(bool request_found, ReqBuffer::iterator& req_it) override {

      if (!request_found) {
        close_in_idle_slot();
        return;
      }

      if (m_dram->m_command_meta(req_it->command).is_closing ||
          m_dram->m_command_meta(req_it->command).is_refreshing)  // PRE or REF 
      {  

        if (req_it->addr_vec[m_bankgroup_level] == -1 && req_it->addr_vec[m_bank_level] == -1) {  // all bank closes
          for (int b = 0; b < m_num_banks; b++) {
            for (int bg = 0; bg < m_num_bankgroups; bg++) {
              int rank_id = req_it->addr_vec[m_rank_level];
              int flat_bank_id = b + bg * m_num_banks + rank_id * m_num_banks * m_num_bankgroups;
              reset_bank(flat_bank_id);
            }
          }
        } else if (req_it->addr_vec[m_bankgroup_level] == -1) {  // same bank closes
          for (int bg = 0; bg < m_num_bankgroups; bg++) {
            int bank_id = req_it->addr_vec[m_bank_level];
            int rank_id = req_it->addr_vec[m_rank_level];
            int flat_bank_id = bank_id + bg * m_num_banks + rank_id * m_num_banks * m_num_bankgroups;
            reset_bank(flat_bank_id);
          }
        } else {  // single bank closes  (PRE, VRR, RDA, WRA)
          int flat_bank_id = req_it->addr_vec[m_bank_level] + 
                             req_it->addr_vec[m_bankgroup_level] * m_num_banks + 
                             req_it->addr_vec[m_rank_level] * m_num_banks * m_num_bankgroups;

          reset_bank(flat_bank_id);
        }
      } else if (m_dram->m_command_meta(req_it->command).is_accessing)  // RD or WR
      {
        if (req_it->type_id == m_aqua_copy_read_id || req_it->type_id == m_aqua_copy_write_id)
          return;
        
        int flat_bank_id = req_it->addr_vec[m_bank_level] + 
                           req_it->addr_vec[m_bankgroup_level] * m_num_banks + 
                           req_it->addr_vec[m_rank_level] * m_num_banks * m_num_bankgroups;
        
        m_col_accesses[flat_bank_id]++;

        if (m_col_accesses[flat_bank_id] >= m_cap) {
          if (m_to_close[flat_bank_id].empty()) m_marked.push_back(flat_bank_id);
          m_to_close[flat_bank_id] = req_it->addr_vec;
          m_col_accesses[flat_bank_id] = 0;
        }
      }
    };

  private:
    // Issue at most one PRE, for a marked bank whose row is still open and ready to close.
    void close_in_idle_slot() {
      for (size_t n = 0; n < m_marked.size(); ) {
        size_t i = (m_rr + n) % m_marked.size();
        int fb = m_marked[i];
        AddrVec_t& av = m_to_close[fb];
        if (av.empty() || !m_dram->check_rowbuffer_hit(m_RD_cmd, av)) {
          av.clear();                         // reset, or the row was closed by another command
          m_marked.erase(m_marked.begin() + i);
          continue;                           // re-check the element shifted into i
        }
        if (m_dram->check_ready(m_PRE_cmd, av)) {
          m_dram->issue_command(m_PRE_cmd, av);
          s_num_close_reqs++;
          av.clear();
          m_marked.erase(m_marked.begin() + i);
          m_rr = m_marked.empty() ? 0 : i % m_marked.size();
          return;
        }
        n++;
      }
    }
};

}       // namespace Ramulator
