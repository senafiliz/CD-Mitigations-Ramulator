/**
 * @file DDR5-VRR-RFM.cpp
 * @brief DDR5 with both VRR and RFM/DRFM commands.
 *
 * Merges the upstream DDR5 (RFM energy and RFM cycle stats) and DDR5-VRR (VRR command)
 * models, with DRAMPower-style energy accounting and a DDR5-3200 timing preset whose
 * nRCD/nRAS/nRP/nWR can be overridden from YAML with circuit-calibrated values.
 */

#include <set>
#include <string>

#include "dram/dram.h"
#include "dram/lambdas.h"

namespace Ramulator {

class DDR5_VRR_RFM : public IDRAM, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAM, DDR5_VRR_RFM,
                                    "DDR5_VRR_RFM",
                                    "DDR5 with VRR and RFM/DRFM commands and RFM energy accounting")

private:
  int m_RH_radius = -1;

public:
  //=========================================================================
  // Organization presets (same physical geometry as DDR5)
  //=========================================================================
  inline static const std::map<std::string, Organization> org_presets = {
    //   name        density   DQ   Ch Ra Bg Ba    Ro     Co
    {"DDR5_8Gb_x8",   { 8<<10,   8, {1, 1, 8, 2, 1<<16, 1<<10}}},
    {"DDR5_16Gb_x8",  {16<<10,   8, {1, 1, 8, 4, 1<<16, 1<<10}}},
    {"DDR5_32Gb_x8",  {32<<10,   8, {1, 1, 8, 4, 1<<17, 1<<10}}},
  };

  //=========================================================================
  // Timing presets
  //
  // DDR5_3200AN: DDR5-3200 speed grade.
  //   Circuit-calibrated overrides (nRCD, nRAS, nRP, nWR) are applied via YAML config.
  //
  // Column order:
  //   rate  nBL  nCL nRCD  nRP  nRAS  nRC   nWR nRTP nCWL nPPD
  //   nCCDS nCCDS_WR nCCDS_WTR nCCDL nCCDL_WR nCCDL_WTR
  //   nRRDS nRRDL nFAW
  //   nRFC1 nRFC2 nRFCsb nREFI nREFSBRD
  //   nRFM1 nRFM2 nRFMsb nDRFMab nDRFMsb nVRR nCS tCK_ps
  //=========================================================================
  inline static const std::map<std::string, std::vector<int>> timing_presets = {
    // nRCD/nRAS/nRP/nWR can be overridden in YAML with circuit-calibrated values.
    {"DDR5_3200AN", {3200,  8,  24,  20,  24,  52,  76,  48,  12,  22,  2,
                         8,   8, 34,   8,  16,  46,   8,  -1,  -1,
                        -1,  -1,  -1,  -1,  30,
                        -1,  -1,  -1,  -1,  -1,  -1,  2,  625}},
  };

  inline static const std::map<std::string, std::vector<double>> voltage_presets = {
    {"Default", {1.1, 1.8}},
  };

  inline static const std::map<std::string, std::vector<double>> current_presets = {
    // name         IDD0  IDD2N  IDD3N  IDD4R  IDD4W  IDD5B  IPP0  IPP2N IPP3N IPP4R IPP4W IPP5B
    {"Default",     {60,   50,    55,    145,   145,   362,    3,    3,    3,    3,    3,    48}},
  };

  //=========================================================================
  // Organization hierarchy
  //=========================================================================
  const int m_internal_prefetch_size = 16;

  inline static constexpr ImplDef m_levels = {
    "channel", "rank", "bankgroup", "bank", "row", "column",
  };

  //=========================================================================
  // Commands
  //
  // Standard DDR5 commands only.
  //=========================================================================
  inline static constexpr ImplDef m_commands = {
    "ACT",
    "PRE", "PREA", "PREsb",
    "RD",  "WR",  "RDA",  "WRA",
    "REFab",  "REFsb",  "REFab_end",  "REFsb_end",
    "RFMab",  "RFMsb",  "RFMab_end",  "RFMsb_end",
    "DRFMab", "DRFMsb", "DRFMab_end", "DRFMsb_end",

    "VRR", "VRR_end",
  };

  inline static const ImplLUT m_command_scopes = LUT (
    m_commands, m_levels, {
      {"ACT",         "row"},
      {"PRE",         "bank"},  {"PREA",  "rank"},  {"PREsb",  "bank"},
      {"RD",          "column"}, {"WR", "column"}, {"RDA", "column"}, {"WRA", "column"},
      {"REFab",       "rank"}, {"REFsb",  "bank"}, {"REFab_end",  "rank"}, {"REFsb_end",  "bank"},
      {"RFMab",       "rank"}, {"RFMsb",  "bank"}, {"RFMab_end",  "rank"}, {"RFMsb_end",  "bank"},
      {"DRFMab",      "rank"}, {"DRFMsb", "bank"}, {"DRFMab_end", "rank"}, {"DRFMsb_end", "bank"},

      {"VRR",         "bank"}, {"VRR_end", "bank"},
    }
  );

  inline static const ImplLUT m_command_meta = LUT<DRAMCommandMeta> (
    m_commands, {
                    // open?  close?  access?  refresh?
      {"ACT",       {true,  false,  false,   false}},
      {"PRE",       {false, true,   false,   false}},
      {"PREA",      {false, true,   false,   false}},
      {"PREsb",     {false, true,   false,   false}},
      {"RD",        {false, false,  true,    false}},
      {"WR",        {false, false,  true,    false}},
      {"RDA",       {false, true,   true,    false}},
      {"WRA",       {false, true,   true,    false}},
      {"REFab",     {false, false,  false,   true }},
      {"REFsb",     {false, false,  false,   true }},
      {"REFab_end", {false, true,   false,   false}},
      {"REFsb_end", {false, true,   false,   false}},
      {"RFMab",     {false, false,  false,   true }},
      {"RFMsb",     {false, false,  false,   true }},
      {"RFMab_end", {false, true,   false,   false}},
      {"RFMsb_end", {false, true,   false,   false}},
      {"DRFMab",    {false, false,  false,   true }},
      {"DRFMsb",    {false, false,  false,   true }},
      {"DRFMab_end",{false, true,   false,   false}},
      {"DRFMsb_end",{false, true,   false,   false}},

      {"VRR",       {false, false,  false,   true }},
      {"VRR_end",   {false, true,   false,   false}},
    }
  );

  inline static constexpr ImplDef m_requests = {
    "read", "write",
    "all-bank-refresh", "same-bank-refresh",
    "rfm", "same-bank-rfm",
    "directed-rfm", "same-bank-directed-rfm",
    "open-row", "close-row",
    "close-all-bank",
    "aqua-copy-read", "aqua-copy-write",

    "victim-row-refresh",
  };

  inline static const ImplLUT m_request_translations = LUT (
    m_requests, m_commands, {
      {"read",                      "RD"},
      {"write",                     "WR"},
      {"all-bank-refresh",          "REFab"},
      {"same-bank-refresh",         "REFsb"},
      {"rfm",                       "RFMab"},
      {"same-bank-rfm",             "RFMsb"},
      {"directed-rfm",              "DRFMab"},
      {"same-bank-directed-rfm",    "DRFMsb"},
      {"open-row",                  "ACT"},
      {"close-row",                 "PRE"},
      {"close-all-bank",            "PREA"},
      {"aqua-copy-read",            "RD"},
      {"aqua-copy-write",           "WR"},

      {"victim-row-refresh",        "VRR"},
    }
  );

  //=========================================================================
  // Timing parameter names
  //=========================================================================
  inline static constexpr ImplDef m_timings = {
    "rate",
    "nBL", "nCL", "nRCD", "nRP", "nRAS", "nRC", "nWR", "nRTP", "nCWL",
    "nPPD",
    "nCCDS", "nCCDS_WR", "nCCDS_WTR",
    "nCCDL", "nCCDL_WR", "nCCDL_WTR",
    "nRRDS", "nRRDL",
    "nFAW",
    "nRFC1", "nRFC2", "nRFCsb", "nREFI", "nREFSBRD",
    "nRFM1", "nRFM2", "nRFMsb",
    "nDRFMab", "nDRFMsb",
    "nVRR",
    "nCS",
    "tCK_ps",
  };

  //=========================================================================
  // Power
  //=========================================================================
  inline static constexpr ImplDef m_voltages = { "VDD", "VPP" };

  inline static constexpr ImplDef m_currents = {
    "IDD0", "IDD2N", "IDD3N", "IDD4R", "IDD4W", "IDD5B",
    "IPP0", "IPP2N", "IPP3N", "IPP4R", "IPP4W", "IPP5B"
  };

  inline static constexpr ImplDef m_cmds_counted = {
    "ACT", "PRE", "RD", "WR", "REF", "RFMsb", "RFMab", "VRR"
  };

  //=========================================================================
  // Node states
  //=========================================================================
  inline static constexpr ImplDef m_states = {
    "Opened", "Closed", "PowerUp", "N/A", "Refreshing"
  };

  inline static const ImplLUT m_init_states = LUT (
    m_levels, m_states, {
      {"channel",   "N/A"},
      {"rank",      "PowerUp"},
      {"bankgroup", "N/A"},
      {"bank",      "Closed"},
      {"row",       "Closed"},
      {"column",    "N/A"},
    }
  );

public:
  struct Node : public DRAMNodeBase<DDR5_VRR_RFM> {
    Node(DDR5_VRR_RFM* dram, Node* parent, int level, int id)
        : DRAMNodeBase<DDR5_VRR_RFM>(dram, parent, level, id) {}
  };
  std::vector<Node*> m_channels;

  FuncMatrix<ActionFunc_t<Node>>  m_actions;
  FuncMatrix<PreqFunc_t<Node>>    m_preqs;
  FuncMatrix<RowhitFunc_t<Node>>  m_rowhits;
  FuncMatrix<RowopenFunc_t<Node>> m_rowopens;
  FuncMatrix<PowerFunc_t<Node>>   m_powers;

  double s_total_rfm_energy = 0.0;
  double s_total_vrr_energy = 0.0;
  std::vector<size_t> s_total_rfm_cycles;

  int m_BRC = 2;

  //=========================================================================
  // IDRAM interface
  //=========================================================================
  void tick() override {
    m_clk++;
    for (int i = m_future_actions.size() - 1; i >= 0; i--) {
      auto& fa = m_future_actions[i];
      if (fa.clk == m_clk) {
        handle_future_action(fa.cmd, fa.addr_vec);
        m_future_actions.erase(m_future_actions.begin() + i);
      }
    }
  }

  void init() override {
    RAMULATOR_DECLARE_SPECS();
    set_organization();
    set_timing_vals();
    set_actions();
    set_preqs();
    set_rowhits();
    set_rowopens();
    set_powers();
    create_nodes();
  }

  void issue_command(int command, const AddrVec_t& addr_vec) override {
    int ch = addr_vec[m_levels["channel"]];
    m_channels[ch]->update_timing(command, addr_vec, m_clk);
    m_channels[ch]->update_powers(command, addr_vec, m_clk);
    m_channels[ch]->update_states(command, addr_vec, m_clk);
    check_future_action(command, addr_vec);

  }

  int get_preq_command(int command, const AddrVec_t& addr_vec) override {
    int ch = addr_vec[m_levels["channel"]];
    return m_channels[ch]->get_preq_command(command, addr_vec, m_clk);
  }

  bool check_ready(int command, const AddrVec_t& addr_vec) override {
    int ch = addr_vec[m_levels["channel"]];
    return m_channels[ch]->check_ready(command, addr_vec, m_clk);
  }

  bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec) override {
    int ch = addr_vec[m_levels["channel"]];
    return m_channels[ch]->check_rowbuffer_hit(command, addr_vec, m_clk);
  }

  bool check_node_open(int command, const AddrVec_t& addr_vec) override {
    int ch = addr_vec[m_levels["channel"]];
    return m_channels[ch]->check_node_open(command, addr_vec, m_clk);
  }

private:
  //=========================================================================
  // Future action dispatch (identical to DDR5 pattern)
  //=========================================================================
  void check_future_action(int command, const AddrVec_t& addr_vec) {
    switch (command) {
      case m_commands("REFab"):
        m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFC1") - 1});
        break;
      case m_commands("REFsb"):
        m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFCsb") - 1});
        break;
      case m_commands("RFMab"):
        m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFM1") - 1});
        break;
      case m_commands("RFMsb"):
        m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFMsb") - 1});
        break;
      case m_commands("DRFMab"):
        m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nDRFMab") - 1});
        break;
      case m_commands("DRFMsb"):
        m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nDRFMsb") - 1});
        break;
      case m_commands("VRR"):
        m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nVRR") - 1});
        break;
      default: break;
    }
  }

  void handle_future_action(int command, const AddrVec_t& addr_vec) {
    int ch = addr_vec[m_levels["channel"]];
    switch (command) {
      case m_commands("REFab"):
        m_channels[ch]->update_powers(m_commands("REFab_end"), addr_vec, m_clk);
        m_channels[ch]->update_states(m_commands("REFab_end"), addr_vec, m_clk);
        break;
      case m_commands("REFsb"):
        m_channels[ch]->update_powers(m_commands("REFsb_end"), addr_vec, m_clk);
        m_channels[ch]->update_states(m_commands("REFsb_end"), addr_vec, m_clk);
        break;
      case m_commands("RFMab"):
        m_channels[ch]->update_powers(m_commands("RFMab_end"), addr_vec, m_clk);
        m_channels[ch]->update_states(m_commands("RFMab_end"), addr_vec, m_clk);
        break;
      case m_commands("RFMsb"):
        m_channels[ch]->update_powers(m_commands("RFMsb_end"), addr_vec, m_clk);
        m_channels[ch]->update_states(m_commands("RFMsb_end"), addr_vec, m_clk);
        break;
      case m_commands("DRFMab"):
        m_channels[ch]->update_powers(m_commands("DRFMab_end"), addr_vec, m_clk);
        m_channels[ch]->update_states(m_commands("DRFMab_end"), addr_vec, m_clk);
        break;
      case m_commands("DRFMsb"):
        m_channels[ch]->update_powers(m_commands("DRFMsb_end"), addr_vec, m_clk);
        m_channels[ch]->update_states(m_commands("DRFMsb_end"), addr_vec, m_clk);
        break;
      case m_commands("VRR"):
        m_channels[ch]->update_powers(m_commands("VRR_end"), addr_vec, m_clk);
        m_channels[ch]->update_states(m_commands("VRR_end"), addr_vec, m_clk);
        break;
      default: break;
    }
  }

  //=========================================================================
  // set_organization
  //=========================================================================
  void set_organization() {
    m_channel_width = param_group("org").param<int>("channel_width").default_val(64);

    m_organization.count.resize(m_levels.size(), -1);

    if (auto preset_name = param_group("org").param<std::string>("preset").optional()) {
      if (org_presets.count(*preset_name) > 0) {
        m_organization = org_presets.at(*preset_name);
      } else {
        throw ConfigurationError("Unrecognized org preset \"{}\" in {}!", *preset_name, get_name());
      }
    }

    if (auto dq = param_group("org").param<int>("dq").optional())
      m_organization.dq = *dq;

    for (int i = 0; i < m_levels.size(); i++) {
      auto level_name = m_levels(i);
      if (auto sz = param_group("org").param<int>(level_name).optional())
        m_organization.count[i] = *sz;
    }

    if (auto density = param_group("org").param<int>("density").optional())
      m_organization.density = *density;

    size_t _density = size_t(m_organization.count[m_levels["bankgroup"]]) *
                      size_t(m_organization.count[m_levels["bank"]]) *
                      size_t(m_organization.count[m_levels["row"]]) *
                      size_t(m_organization.count[m_levels["column"]]) *
                      size_t(m_organization.dq);
    _density >>= 20;
    if (m_organization.density != _density) {
      throw ConfigurationError(
          "Calculated {} density {} Mb != provided {} Mb!",
          get_name(), _density, m_organization.density);
    }

    int num_channels = m_organization.count[m_levels["channel"]];
    int num_ranks    = m_organization.count[m_levels["rank"]];
    s_total_rfm_cycles.resize(num_channels * num_ranks, 0);
    for (int r = 0; r < num_channels * num_ranks; r++)
      register_stat(s_total_rfm_cycles[r]).name("total_rfm_cycles_rank{}", r);

  }

  //=========================================================================
  // set_timing_vals
  //=========================================================================
  void set_timing_vals() {
    m_timing_vals.resize(m_timings.size(), -1);

    bool preset_provided = false;
    if (auto preset_name = param_group("timing").param<std::string>("preset").optional()) {
      if (timing_presets.count(*preset_name) > 0) {
        const auto& pv = timing_presets.at(*preset_name);
        for (int i = 0; i < (int)pv.size() && i < (int)m_timing_vals.size(); i++)
          m_timing_vals(i) = pv[i];
        preset_provided = true;
      } else {
        throw ConfigurationError("Unrecognized timing preset \"{}\" in {}!", *preset_name, get_name());
      }
    }

    int tCK_ps = 1E6 / (m_timing_vals("rate") / 2);
    m_timing_vals("tCK_ps") = tCK_ps;

    // Refresh timing from density
    int density_id = [](int d) -> int {
      switch (d) {
        case 8192:  return 0;
        case 16384: return 1;
        case 32768: return 2;
        default:    return -1;
      }
    }(m_organization.density);

    constexpr int tRFC_TABLE[2][3] = {{ 195, 295, 410 }, { 130, 160, 220 }};
    constexpr int tRFCsb_TABLE[1][3] = {{ 115, 130, 190 }};
    constexpr int tREFI_BASE = 3900;

    m_RH_radius = param<int>("RH_radius").default_val(2);

    const int nVRR_base_ns = 60;
    m_timing_vals("nVRR") = JEDEC_rounding_DDR5(nVRR_base_ns * m_RH_radius * 2, tCK_ps);

    if (density_id >= 0) {
      m_timing_vals("nRFC1")  = JEDEC_rounding_DDR5(tRFC_TABLE[0][density_id], tCK_ps);
      m_timing_vals("nRFC2")  = JEDEC_rounding_DDR5(tRFC_TABLE[1][density_id], tCK_ps);
      m_timing_vals("nRFCsb") = JEDEC_rounding_DDR5(tRFCsb_TABLE[0][density_id], tCK_ps);
      m_timing_vals("nREFI")  = JEDEC_rounding_DDR5(tREFI_BASE, tCK_ps);
    }

    m_BRC = param_group("RFM").param<int>("BRC").default_val(2);
    m_timing_vals("nRFM1")   = m_timing_vals("nRFC1");
    m_timing_vals("nRFM2")   = m_timing_vals("nRFC2");
    m_timing_vals("nRFMsb")  = m_timing_vals("nRFCsb") * m_RH_radius;

    constexpr int tRRFsb_TABLE[2][3] = {{ 70, 70, 70 }, { 60, 60, 60 }};
    if (density_id >= 0) {
      m_timing_vals("nDRFMab") = 2 * m_BRC * JEDEC_rounding_DDR5(tRRFsb_TABLE[0][density_id], tCK_ps);
      m_timing_vals("nDRFMsb") = 2 * m_BRC * JEDEC_rounding_DDR5(tRRFsb_TABLE[1][density_id], tCK_ps);
    }

    // DQ-width and rate dependent timings (same tables as upstream DDR5)
    int dq_id = [](int dq) -> int {
      switch (dq) {
        case  4: return 0;
        case  8: return 1;
        case 16: return 2;
        default: return -1;
      }
    }(m_organization.dq);

    int rate_id = [](int rate) -> int {
      switch (rate) {
        case 3200: return 0;
        default:   return -1;
      }
    }(m_timing_vals("rate"));

    // nRRDL and nFAW (not provided by the preset; derived from DQ width)
    // JEDEC JESD79-5: tRRD_L = max(8 nCK, 5 ns) = 8 nCK at 3200 MT/s
    constexpr int nRRDL_TABLE[3][1] = {{ 8 }, { 8 }, { 8 }};   // x4, x8, x16
    constexpr int nFAW_TABLE[3][1]  = {{ 40 }, { 32 }, { 32 }}; // x4, x8, x16
    if (dq_id != -1 && rate_id != -1) {
      m_timing_vals("nRRDL") = nRRDL_TABLE[dq_id][rate_id];
      m_timing_vals("nFAW")  = nFAW_TABLE[dq_id][rate_id];
    }

    // nREFSBRD: same-bank refresh to activate delay (use default 30 from preset
    // or compute from nRFCsb if preset left it unset)
    if (m_timing_vals("nREFSBRD") == -1 && density_id >= 0) {
      m_timing_vals("nREFSBRD") = JEDEC_rounding_DDR5(30.0f, tCK_ps); // ~30 ns, standard DDR5
    }

    // Allow user to override any standard DDR5 timing field explicitly
    // (e.g. circuit-calibrated nRCD, nRAS, nRP, nWR).
    static const std::set<std::string> skip_fields = { "tCK_ps" };
    for (int i = 1; i < m_timings.size(); i++) {
      auto timing_name = std::string(m_timings(i));
      if (skip_fields.count(timing_name)) continue;
      if (auto v = param_group("timing").param<int>(timing_name).optional())
        m_timing_vals(i) = *v;
      else if (auto v = param_group("timing").param<float>(timing_name.replace(0,1,"t")).optional())
        m_timing_vals(i) = JEDEC_rounding_DDR5(*v, tCK_ps);
    }


    // Sanity: ensure no uninitialized field
    for (int i = 0; i < m_timing_vals.size(); i++) {
      if (m_timing_vals(i) == -1)
        throw ConfigurationError("In \"{}\", timing {} not specified!", get_name(), m_timings(i));
    }


    m_read_latency = m_timing_vals("nCL") + m_timing_vals("nBL");

    build_timing_constraints();
  }

  void build_timing_constraints() {
    #define V(t) (m_timing_vals(t))
    auto all_cmds = std::vector<std::string_view>(m_commands.begin(), m_commands.end());
    populate_timingcons(this, {
      // Channel: two-cycle commands
      {.level="channel", .preceding={"ACT","RD","RDA","WR","WRA"}, .following=all_cmds, .latency=2},
      // Data bus occupancy
      {.level="channel", .preceding={"RD","RDA"}, .following={"RD","RDA"}, .latency=V("nBL")},
      {.level="channel", .preceding={"WR","WRA"}, .following={"WR","WRA"}, .latency=V("nBL")},

      // Rank: CAS <-> CAS
      {.level="rank", .preceding={"RD","RDA"}, .following={"RD","RDA"}, .latency=V("nCCDS")},
      {.level="rank", .preceding={"WR","WRA"}, .following={"WR","WRA"}, .latency=V("nCCDS_WR")},
      {.level="rank", .preceding={"RD","RDA"}, .following={"WR","WRA"}, .latency=V("nCL")+V("nBL")+2-V("nCWL")+2},
      {.level="rank", .preceding={"WR","WRA"}, .following={"RD","RDA"}, .latency=V("nCCDS_WTR")},
      {.level="rank", .preceding={"RD","RDA"}, .following={"RD","RDA","WR","WRA"}, .latency=V("nBL")+V("nCS"), .is_sibling=true},
      {.level="rank", .preceding={"WR","WRA"}, .following={"RD","RDA"}, .latency=V("nCL")+V("nBL")+V("nCS")-V("nCWL"), .is_sibling=true},
      {.level="rank", .preceding={"RD"}, .following={"PREA"}, .latency=V("nRTP")},
      {.level="rank", .preceding={"WR"}, .following={"PREA"}, .latency=V("nCWL")+V("nBL")+V("nWR")},
      // RAS <-> RAS
      {.level="rank", .preceding={"ACT"}, .following={"ACT"}, .latency=V("nRRDS")},
      {.level="rank", .preceding={"ACT"}, .following={"ACT"}, .latency=V("nFAW"), .window=4},
      {.level="rank", .preceding={"ACT"}, .following={"PREA"}, .latency=V("nRAS")},
      {.level="rank", .preceding={"PREA"}, .following={"ACT","VRR"}, .latency=V("nRP")},
      // RAS <-> REF
      {.level="rank", .preceding={"ACT"}, .following={"REFab","RFMab","DRFMab"}, .latency=V("nRC")},
      {.level="rank", .preceding={"PRE","PREsb"}, .following={"REFab","RFMab","DRFMab"}, .latency=V("nRP")},
      {.level="rank", .preceding={"PREA"}, .following={"REFab","RFMab","DRFMab","REFsb","RFMsb","DRFMsb"}, .latency=V("nRP")},
      {.level="rank", .preceding={"RDA"}, .following={"REFab","RFMab","DRFMab"}, .latency=V("nRP")+V("nRTP")},
      {.level="rank", .preceding={"WRA"}, .following={"REFab","RFMab","DRFMab"}, .latency=V("nCWL")+V("nBL")+V("nWR")+V("nRP")},
      {.level="rank", .preceding={"REFab"}, .following={"ACT","VRR","PREA","REFab","RFMab","DRFMab","REFsb","RFMsb","DRFMsb"}, .latency=V("nRFC1")},
      {.level="rank", .preceding={"RFMab"}, .following={"ACT","VRR","PREA","REFab","RFMab","DRFMab","REFsb","RFMsb","DRFMsb"}, .latency=V("nRFM1")},
      {.level="rank", .preceding={"DRFMab"}, .following={"ACT","VRR","PREA","REFab","RFMab","DRFMab","REFsb","RFMsb","DRFMsb"}, .latency=V("nDRFMab")},
      {.level="rank", .preceding={"REFsb"}, .following={"PREA","REFab","RFMab","DRFMab"}, .latency=V("nRFCsb")},
      {.level="rank", .preceding={"RFMsb"}, .following={"PREA","REFab","RFMab","DRFMab"}, .latency=V("nRFMsb")},
      {.level="rank", .preceding={"DRFMsb"}, .following={"PREA","REFab","RFMab","DRFMab"}, .latency=V("nDRFMsb")},

      {.level="rank", .preceding={"VRR"}, .following={"PREA"}, .latency=V("nVRR")},
      {.level="rank", .preceding={"VRR"}, .following={"REFab","RFMab","DRFMab"}, .latency=V("nVRR")},

      // BankGroup: CAS <-> CAS
      {.level="bankgroup", .preceding={"RD","RDA"}, .following={"RD","RDA"}, .latency=V("nCCDL")},
      {.level="bankgroup", .preceding={"WR","WRA"}, .following={"WR","WRA"}, .latency=V("nCCDL_WR")},
      {.level="bankgroup", .preceding={"WR","WRA"}, .following={"RD","RDA"}, .latency=V("nCCDL_WTR")},
      {.level="bankgroup", .preceding={"ACT"}, .following={"ACT"}, .latency=V("nRRDL")},

      // Bank
      {.level="bank", .preceding={"ACT"}, .following={"ACT","VRR","REFsb","RFMsb","DRFMsb"}, .latency=V("nRC")},
      {.level="bank", .preceding={"ACT"}, .following={"RD","RDA","WR","WRA"}, .latency=V("nRCD")},
      {.level="bank", .preceding={"ACT"}, .following={"PRE","PREsb"}, .latency=V("nRAS")},
      {.level="bank", .preceding={"PRE","PREsb"}, .following={"ACT","VRR","REFsb","RFMsb","DRFMsb"}, .latency=V("nRP")},
      {.level="bank", .preceding={"RD"}, .following={"PRE","PREsb"}, .latency=V("nRTP")},
      {.level="bank", .preceding={"WR"}, .following={"PRE","PREsb"}, .latency=V("nCWL")+V("nBL")+V("nWR")},
      {.level="bank", .preceding={"RDA"}, .following={"ACT","VRR","REFsb","RFMsb","DRFMsb"}, .latency=V("nRTP")+V("nRP")},
      {.level="bank", .preceding={"WRA"}, .following={"ACT","VRR","REFsb","RFMsb","DRFMsb"}, .latency=V("nCWL")+V("nBL")+V("nWR")+V("nRP")},
      {.level="bank", .preceding={"WR"}, .following={"RDA"}, .latency=V("nCWL")+V("nBL")+V("nWR")-V("nRTP")},
      {.level="bank", .preceding={"REFsb"}, .following={"ACT","VRR","REFsb","RFMsb","DRFMsb"}, .latency=V("nRFCsb")},
      {.level="bank", .preceding={"RFMsb"}, .following={"ACT","VRR","REFsb","RFMsb","DRFMsb"}, .latency=V("nRFMsb")},
      {.level="bank", .preceding={"DRFMsb"}, .following={"ACT","VRR","REFsb","RFMsb","DRFMsb"}, .latency=V("nDRFMsb")},

      {.level="bank", .preceding={"VRR"}, .following={"ACT","VRR","REFsb","RFMsb","DRFMsb"}, .latency=V("nVRR")},
    });
    #undef V
  }

  //=========================================================================
  // Actions, preqs, rowhits, rowopens, powers (reuse DDR5 lambdas)
  //=========================================================================
  void set_actions() {
    m_actions.resize(m_levels.size(), std::vector<ActionFunc_t<Node>>(m_commands.size()));
    m_actions[m_levels["rank"]][m_commands["PREA"]]        = Lambdas::Action::Rank::PREab<DDR5_VRR_RFM>;
    m_actions[m_levels["rank"]][m_commands["REFab"]]       = Lambdas::Action::Rank::REFab<DDR5_VRR_RFM>;
    m_actions[m_levels["rank"]][m_commands["REFab_end"]]   = Lambdas::Action::Rank::REFab_end<DDR5_VRR_RFM>;
    m_actions[m_levels["rank"]][m_commands["RFMab"]]       = Lambdas::Action::Rank::REFab<DDR5_VRR_RFM>;
    m_actions[m_levels["rank"]][m_commands["RFMab_end"]]   = Lambdas::Action::Rank::REFab_end<DDR5_VRR_RFM>;
    m_actions[m_levels["rank"]][m_commands["DRFMab"]]      = Lambdas::Action::Rank::REFab<DDR5_VRR_RFM>;
    m_actions[m_levels["rank"]][m_commands["DRFMab_end"]]  = Lambdas::Action::Rank::REFab_end<DDR5_VRR_RFM>;
    m_actions[m_levels["bankgroup"]][m_commands["PREsb"]]      = Lambdas::Action::BankGroup::PREsb<DDR5_VRR_RFM>;
    m_actions[m_levels["bankgroup"]][m_commands["REFsb"]]      = Lambdas::Action::BankGroup::REFsb<DDR5_VRR_RFM>;
    m_actions[m_levels["bankgroup"]][m_commands["REFsb_end"]]  = Lambdas::Action::BankGroup::REFsb_end<DDR5_VRR_RFM>;
    m_actions[m_levels["bankgroup"]][m_commands["RFMsb"]]      = Lambdas::Action::BankGroup::REFsb<DDR5_VRR_RFM>;
    m_actions[m_levels["bankgroup"]][m_commands["RFMsb_end"]]  = Lambdas::Action::BankGroup::REFsb_end<DDR5_VRR_RFM>;
    m_actions[m_levels["bankgroup"]][m_commands["DRFMsb"]]     = Lambdas::Action::BankGroup::REFsb<DDR5_VRR_RFM>;
    m_actions[m_levels["bankgroup"]][m_commands["DRFMsb_end"]] = Lambdas::Action::BankGroup::REFsb_end<DDR5_VRR_RFM>;
    m_actions[m_levels["bank"]][m_commands["ACT"]]      = Lambdas::Action::Bank::ACT<DDR5_VRR_RFM>;
    m_actions[m_levels["bank"]][m_commands["PRE"]]      = Lambdas::Action::Bank::PRE<DDR5_VRR_RFM>;
    m_actions[m_levels["bank"]][m_commands["RDA"]]      = Lambdas::Action::Bank::PRE<DDR5_VRR_RFM>;
    m_actions[m_levels["bank"]][m_commands["WRA"]]      = Lambdas::Action::Bank::PRE<DDR5_VRR_RFM>;
    m_actions[m_levels["bank"]][m_commands["VRR"]]      = Lambdas::Action::Bank::VRR<DDR5_VRR_RFM>;
    m_actions[m_levels["bank"]][m_commands["VRR_end"]]  = Lambdas::Action::Bank::VRR_end<DDR5_VRR_RFM>;
  }

  void set_preqs() {
    m_preqs.resize(m_levels.size(), std::vector<PreqFunc_t<Node>>(m_commands.size()));
    m_preqs[m_levels["rank"]][m_commands["PREA"]]   = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5_VRR_RFM>;
    m_preqs[m_levels["rank"]][m_commands["REFab"]]  = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5_VRR_RFM>;
    m_preqs[m_levels["rank"]][m_commands["RFMab"]]  = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5_VRR_RFM>;
    m_preqs[m_levels["rank"]][m_commands["DRFMab"]] = Lambdas::Preq::Rank::RequireAllBanksClosed<DDR5_VRR_RFM>;
    m_preqs[m_levels["rank"]][m_commands["REFsb"]]  = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5_VRR_RFM>;
    m_preqs[m_levels["rank"]][m_commands["RFMsb"]]  = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5_VRR_RFM>;
    m_preqs[m_levels["rank"]][m_commands["DRFMsb"]] = Lambdas::Preq::Rank::RequireSameBanksClosed<DDR5_VRR_RFM>;
    m_preqs[m_levels["bank"]][m_commands["RD"]]  = Lambdas::Preq::Bank::RequireRowOpen<DDR5_VRR_RFM>;
    m_preqs[m_levels["bank"]][m_commands["WR"]]  = Lambdas::Preq::Bank::RequireRowOpen<DDR5_VRR_RFM>;
    m_preqs[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Preq::Bank::RequireRowOpen<DDR5_VRR_RFM>;
    m_preqs[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Preq::Bank::RequireBankClosed<DDR5_VRR_RFM>;
    m_preqs[m_levels["bank"]][m_commands["VRR"]] = Lambdas::Preq::Bank::RequireBankClosed<DDR5_VRR_RFM>;
  }

  void set_rowhits() {
    m_rowhits.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));
    m_rowhits[m_levels["bank"]][m_commands["RD"]] = Lambdas::RowHit::Bank::RDWR<DDR5_VRR_RFM>;
    m_rowhits[m_levels["bank"]][m_commands["WR"]] = Lambdas::RowHit::Bank::RDWR<DDR5_VRR_RFM>;
  }

  void set_rowopens() {
    m_rowopens.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));
    m_rowopens[m_levels["bank"]][m_commands["RD"]] = Lambdas::RowOpen::Bank::RDWR<DDR5_VRR_RFM>;
    m_rowopens[m_levels["bank"]][m_commands["WR"]] = Lambdas::RowOpen::Bank::RDWR<DDR5_VRR_RFM>;
  }

  void set_powers() {
    m_drampower_enable = param<bool>("drampower_enable").default_val(false);
    if (!m_drampower_enable) return;

    m_voltage_vals.resize(m_voltages.size(), -1);
    if (auto p = param_group("voltage").param<std::string>("preset").optional()) {
      if (voltage_presets.count(*p)) m_voltage_vals = voltage_presets.at(*p);
      else throw ConfigurationError("Unrecognized voltage preset \"{}\"!", *p);
    }

    m_current_vals.resize(m_currents.size(), -1);
    if (auto p = param_group("current").param<std::string>("preset").optional()) {
      if (current_presets.count(*p)) m_current_vals = current_presets.at(*p);
      else throw ConfigurationError("Unrecognized current preset \"{}\"!", *p);
    }

    m_power_debug = param<bool>("power_debug").default_val(false);

    int num_channels = m_organization.count[m_levels["channel"]];
    int num_ranks    = m_organization.count[m_levels["rank"]];
    m_power_stats.resize(num_channels * num_ranks);
    for (int i = 0; i < num_channels; i++)
      for (int j = 0; j < num_ranks; j++) {
        m_power_stats[i*num_ranks+j].rank_id = i*num_ranks+j;
        m_power_stats[i*num_ranks+j].cmd_counters.resize(m_cmds_counted.size(), 0);
      }

    m_powers.resize(m_levels.size(), std::vector<PowerFunc_t<Node>>(m_commands.size()));
    m_powers[m_levels["bank"]][m_commands["ACT"]]      = Lambdas::Power::Bank::ACT<DDR5_VRR_RFM>;
    m_powers[m_levels["bank"]][m_commands["PRE"]]      = Lambdas::Power::Bank::PRE<DDR5_VRR_RFM>;
    m_powers[m_levels["bank"]][m_commands["RD"]]       = Lambdas::Power::Bank::RD<DDR5_VRR_RFM>;
    m_powers[m_levels["bank"]][m_commands["WR"]]       = Lambdas::Power::Bank::WR<DDR5_VRR_RFM>;
    m_powers[m_levels["bank"]][m_commands["VRR"]]      = Lambdas::Power::Bank::VRR<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["VRR"]]       = Lambdas::Power::Rank::VRR<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["VRR_end"]]   = Lambdas::Power::Rank::VRR_end<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["REFab"]]     = Lambdas::Power::Rank::REFab<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Power::Rank::REFab_end<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["RFMab"]]     = Lambdas::Power::Rank::RFMab<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["RFMab_end"]] = Lambdas::Power::Rank::RFMab_end<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["RFMsb"]]     = Lambdas::Power::Rank::RFMsb<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["RFMsb_end"]] = Lambdas::Power::Rank::RFMsb_end<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["ACT"]]       = Lambdas::Power::Rank::ACT<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["PRE"]]       = Lambdas::Power::Rank::PRE<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["PREA"]]      = Lambdas::Power::Rank::PREA<DDR5_VRR_RFM>;
    m_powers[m_levels["rank"]][m_commands["PREsb"]]     = Lambdas::Power::Rank::PREsb<DDR5_VRR_RFM>;

    register_stat(s_total_background_energy).name("total_background_energy");
    register_stat(s_total_cmd_energy).name("total_cmd_energy");
    register_stat(s_total_energy).name("total_energy");
    register_stat(s_total_rfm_energy).name("total_rfm_energy");
    register_stat(s_total_vrr_energy).name("total_vrr_energy");

    for (auto& ps : m_power_stats) {
      register_stat(ps.total_background_energy).name("total_background_energy_rank{}", ps.rank_id);
      register_stat(ps.total_cmd_energy).name("total_cmd_energy_rank{}", ps.rank_id);
      register_stat(ps.total_energy).name("total_energy_rank{}", ps.rank_id);
      register_stat(ps.act_background_energy).name("act_background_energy_rank{}", ps.rank_id);
      register_stat(ps.pre_background_energy).name("pre_background_energy_rank{}", ps.rank_id);
      register_stat(ps.active_cycles).name("active_cycles_rank{}", ps.rank_id);
      register_stat(ps.idle_cycles).name("idle_cycles_rank{}", ps.rank_id);
    }
  }

  void create_nodes() {
    int num_channels = m_organization.count[m_levels["channel"]];
    for (int i = 0; i < num_channels; i++)
      m_channels.push_back(new Node(this, nullptr, 0, i));
  }

  void reset_power_counters() override {
    IDRAM::reset_power_counters();
    // Fix active/idle start cycles so post-warmup accumulation begins from now.
    for (auto& ps : m_power_stats) {
      ps.active_start_cycle = m_clk;
      ps.idle_start_cycle   = m_clk;
    }
  }

  void finalize() override {
    if (!m_drampower_enable) return;
    int num_channels = m_organization.count[m_levels["channel"]];
    int num_ranks    = m_organization.count[m_levels["rank"]];
    for (int i = 0; i < num_channels; i++)
      for (int j = 0; j < num_ranks; j++)
        process_rank_energy(m_power_stats[i*num_ranks+j], m_channels[i]->m_child_nodes[j]);

  }

  void process_rank_energy(PowerStats& rs, Node* rank_node) {
    Lambdas::Power::Rank::finalize_rank<DDR5_VRR_RFM>(rank_node, 0, AddrVec_t(), m_clk);
    int num_bgs = m_organization.count[m_levels["bankgroup"]];
    auto TS = [&](std::string_view t) { return m_timing_vals(t); };
    auto VE = [&](std::string_view v) { return m_voltage_vals(v); };
    auto CE = [&](std::string_view c) { return m_current_vals(c); };
    double tCK_ns = (double)TS("tCK_ps") / 1000.0;

    rs.act_background_energy = (VE("VDD")*CE("IDD3N") + VE("VPP")*CE("IPP3N"))
                               * rs.active_cycles * tCK_ns / 1E3;
    rs.pre_background_energy = (VE("VDD")*CE("IDD2N") + VE("VPP")*CE("IPP2N"))
                               * rs.idle_cycles * tCK_ns / 1E3;
    double act_e = (VE("VDD")*(CE("IDD0")-CE("IDD3N")) + VE("VPP")*(CE("IPP0")-CE("IPP3N")))
                   * rs.cmd_counters[m_cmds_counted("ACT")] * TS("nRAS") * tCK_ns / 1E3;
    double pre_e = (VE("VDD")*(CE("IDD0")-CE("IDD2N")) + VE("VPP")*(CE("IPP0")-CE("IPP2N")))
                   * rs.cmd_counters[m_cmds_counted("PRE")] * TS("nRP") * tCK_ns / 1E3;
    double rd_e  = (VE("VDD")*(CE("IDD4R")-CE("IDD3N")) + VE("VPP")*(CE("IPP4R")-CE("IPP3N")))
                   * rs.cmd_counters[m_cmds_counted("RD")] * TS("nBL") * tCK_ns / 1E3;
    double wr_e  = (VE("VDD")*(CE("IDD4W")-CE("IDD3N")) + VE("VPP")*(CE("IPP4W")-CE("IPP3N")))
                   * rs.cmd_counters[m_cmds_counted("WR")] * TS("nBL") * tCK_ns / 1E3;
    double ref_e = (VE("VDD")*CE("IDD5B") + VE("VPP")*CE("IPP5B"))
                   * rs.cmd_counters[m_cmds_counted("REF")] * TS("nRFC1") * tCK_ns / 1E3;
    double rfmsb_e = (VE("VDD")*(CE("IDD0")-CE("IDD3N")) + VE("VPP")*(CE("IPP0")-CE("IPP3N"))) * num_bgs
                     * rs.cmd_counters[m_cmds_counted("RFMsb")] * TS("nRFMsb") * tCK_ns / 1E3;
    double rfmab_e = (VE("VDD")*CE("IDD5B") + VE("VPP")*CE("IPP5B"))
                     * rs.cmd_counters[m_cmds_counted("RFMab")] * TS("nRFM1") * tCK_ns / 1E3;
    double rfm_e = rfmsb_e + rfmab_e;
    double vrr_e = (VE("VDD")*(CE("IDD0")-CE("IDD3N")) + VE("VPP")*(CE("IPP0")-CE("IPP3N")))
                   * rs.cmd_counters[m_cmds_counted("VRR")] * TS("nVRR") * tCK_ns / 1E3;

    rs.total_background_energy = rs.act_background_energy + rs.pre_background_energy;
    rs.total_cmd_energy = act_e + pre_e + rd_e + wr_e + ref_e + rfm_e + vrr_e;
    rs.total_energy = rs.total_background_energy + rs.total_cmd_energy;

    s_total_background_energy += rs.total_background_energy;
    s_total_cmd_energy        += rs.total_cmd_energy;
    s_total_energy            += rs.total_energy;
    s_total_rfm_energy        += rfm_e;
    s_total_vrr_energy        += vrr_e;
    s_total_rfm_cycles[rs.rank_id] = rs.cmd_counters[m_cmds_counted("RFMsb")] * TS("nRFMsb")
                                   + rs.cmd_counters[m_cmds_counted("RFMab")] * TS("nRFM1");
  }
};

} // namespace Ramulator
