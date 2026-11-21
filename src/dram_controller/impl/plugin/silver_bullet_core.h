#ifndef RAMULATOR_PLUGIN_SILVER_BULLET_CORE_H_
#define RAMULATOR_PLUGIN_SILVER_BULLET_CORE_H_

// Silver Bullet (SB) bookkeeping, independent of Ramulator so that it can be unit tested.
//
// Reference: Yaglikci et al., "Security Analysis of the Silver Bullet Technique for RowHammer
// Prevention", 2021, Sections 2-3 and Table 3.
//
//   * A bank is split into N_SB = S_B / S_SB subbanks of S_SB contiguous rows.
//   * Each subbank has FRAC, PENDING and LOCAL_INDEX.
//   * A row-open (activation, preventive refresh or periodic refresh) "hammers" every subbank
//     whose counter region contains the opened row: FRAC++, and when FRAC reaches D,
//     PENDING++ and FRAC = 0.
//   * A preventive refresh is taken from the subbank with the highest PENDING; it targets the row
//     at LOCAL_INDEX of that subbank, then PENDING-- and LOCAL_INDEX = (LOCAL_INDEX + 1) % S_SB.
//
// Counter region (which subbanks a row-open at row r hammers):
//   Own          : only the subbank that contains r.
//   BlastRadius  : every subbank that has a row within [r - B, r + B] (extended counter region).
//   NeighborSA   : every subbank that has a row in subarrays sa(r) - 1, sa(r), sa(r) + 1
//                  (ColumnDisturb, open-bitline: a subarray is disturbed by its own and both
//                  neighbouring subarrays' activations).

#include <algorithm>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace Ramulator {
namespace SB {

enum class Region { Own, BlastRadius, NeighborSA };

inline Region parse_region(const std::string& s) {
    if (s == "own") return Region::Own;
    if (s == "own+blast_radius_rows" || s == "blast_radius") return Region::BlastRadius;
    if (s == "own+neighbor_subarrays" || s == "neighbor_subarrays") return Region::NeighborSA;
    throw std::invalid_argument("unknown SB counter_region: " + s);
}

inline bool is_pow2(int x) { return x > 0 && (x & (x - 1)) == 0; }

struct Config {
    int rows_per_bank = 131072;
    int subbank_size = 1024;     // S_SB, power of two in [1, rows_per_bank]
    int D = 64;
    Region region = Region::BlastRadius;
    int blast_radius = 1;        // B (rows), used by Region::BlastRadius
    int subarray_size = 1024;    // used by Region::NeighborSA and by the REF row mapping

    void validate() const {
        if (!is_pow2(subbank_size) || subbank_size > rows_per_bank)
            throw std::invalid_argument("SB: subbank_size must be a power of two in [1, rows_per_bank]");
        if (rows_per_bank % subbank_size != 0)
            throw std::invalid_argument("SB: rows_per_bank must be a multiple of subbank_size");
        if (D < 1) throw std::invalid_argument("SB: D must be >= 1");
        if (region == Region::BlastRadius && blast_radius < 0)
            throw std::invalid_argument("SB: blast_radius must be >= 0");
        if (region == Region::NeighborSA && (subarray_size < 1 || rows_per_bank % subarray_size != 0))
            throw std::invalid_argument("SB: subarray_size must divide rows_per_bank");
    }

    int num_subbanks() const { return rows_per_bank / subbank_size; }

    // Inclusive range of subbanks whose counter region contains `row`.
    std::pair<int, int> hammered_subbanks(int row) const {
        int lo_row = row, hi_row = row;
        switch (region) {
        case Region::Own: break;
        case Region::BlastRadius:
            lo_row = row - blast_radius;
            hi_row = row + blast_radius;
            break;
        case Region::NeighborSA: {
            int sa = row / subarray_size;
            lo_row = (sa - 1) * subarray_size;
            hi_row = (sa + 2) * subarray_size - 1;
            break;
        }
        }
        lo_row = std::max(lo_row, 0);
        hi_row = std::min(hi_row, rows_per_bank - 1);
        return {lo_row / subbank_size, hi_row / subbank_size};
    }

    // Largest number of subbanks that a single row-open can hammer (k). Observation 1 of the
    // 2021 report assumes k = 2 (a row lies in at most two overlapping counter regions); the
    // general production-rate bound is D >= k (T/R + 1).
    int max_subbanks_per_hammer() const {
        int k = 1;
        // The span is translation invariant except for clamping; check every alignment offset
        // within one subbank / subarray period away from the bank edges.
        int period = std::max(subbank_size, region == Region::NeighborSA ? subarray_size : 1);
        int start = std::min(rows_per_bank - 1, std::max(0, rows_per_bank / 2 - period));
        int end = std::min(rows_per_bank, start + 2 * period);
        for (int r = start; r < end; r++) {
            auto [lo, hi] = hammered_subbanks(r);
            k = std::max(k, hi - lo + 1);
        }
        return k;
    }
};

// Observation 1 of the report (k = 2): D >= 2 (T/R + 1).
inline bool satisfies_obs1(int D, int T, int R, int k = 2) {
    // D >= k (T/R + 1)  <=>  D * R >= k (T + R)
    return (long long)D * R >= (long long)k * ((long long)T + R);
}

class Bank {
public:
    explicit Bank(const Config& cfg) : m_cfg(cfg) {
        int n = cfg.num_subbanks();
        m_frac.assign(n, 0);
        m_pending.assign(n, 0);
        m_local_index.assign(n, 0);
    }

    // A row-open at `row`. Returns the number of preventive refreshes produced.
    int hammer(int row) {
        auto [lo, hi] = m_cfg.hammered_subbanks(row);
        int produced = 0;
        for (int sb = lo; sb <= hi; sb++) {
            if (++m_frac[sb] >= (uint32_t)m_cfg.D) {
                m_frac[sb] = 0;
                set_pending(sb, m_pending[sb] + 1);
                produced++;
            }
        }
        m_hammers += hi - lo + 1;
        return produced;
    }

    bool has_pending() const { return !m_order.empty(); }
    uint64_t total_pending() const { return m_total_pending; }

    // Subbank with the highest PENDING (ties: lowest subbank index), or -1.
    int top_subbank() const { return m_order.empty() ? -1 : m_order.begin()->second; }

    // Consume one pending preventive refresh. Returns the row to refresh, or -1 if none pending.
    int pop_refresh() {
        int sb = top_subbank();
        if (sb < 0) return -1;
        int row = sb * m_cfg.subbank_size + (int)m_local_index[sb];
        m_local_index[sb] = (m_local_index[sb] + 1) % (uint32_t)m_cfg.subbank_size;
        set_pending(sb, m_pending[sb] - 1);
        return row;
    }

    uint32_t frac(int sb) const { return m_frac[sb]; }
    uint32_t pending(int sb) const { return m_pending[sb]; }
    uint32_t local_index(int sb) const { return m_local_index[sb]; }
    uint32_t max_pending() const { return m_max_pending; }
    void reset_max_pending() { m_max_pending = 0; for (auto& [neg, sb] : m_order) m_max_pending = std::max(m_max_pending, (uint32_t)-neg); }
    uint64_t subbank_hammers() const { return m_hammers; }

private:
    void set_pending(int sb, uint32_t v) {
        uint32_t old = m_pending[sb];
        if (old > 0) m_order.erase({-(long long)old, sb});
        m_pending[sb] = v;
        if (v > 0) m_order.insert({-(long long)v, sb});
        m_total_pending += (int64_t)v - (int64_t)old;
        m_max_pending = std::max(m_max_pending, v);
    }

    const Config& m_cfg;
    std::vector<uint32_t> m_frac;
    std::vector<uint32_t> m_pending;
    std::vector<uint32_t> m_local_index;
    std::set<std::pair<long long, int>> m_order;  // (-PENDING, subbank) for PENDING > 0
    uint64_t m_total_pending = 0;
    uint32_t m_max_pending = 0;
    uint64_t m_hammers = 0;
};

// Periodic-refresh (REF) row mapping, identical to salt_c.cpp's ref_coordinate(): every REF
// refreshes `rows_per_ref` rows per bank, one in each of `rows_per_ref` consecutive subarrays
// (round robin over subarrays); within a subarray, a per-subarray pointer walks the rows.
class RefMapper {
public:
    RefMapper(int rows_per_bank, int subarray_size, int rows_per_ref)
        : m_subarray_size(subarray_size), m_num_subarrays(rows_per_bank / subarray_size),
          m_rows_per_ref(std::max(1, rows_per_ref)), m_ptr(m_num_subarrays, 0) {}

    template <typename F>
    void next_ref(F&& on_row) {
        for (int k = 0; k < m_rows_per_ref; k++) {
            int sa = (m_rr + k) % m_num_subarrays;
            int row = sa * m_subarray_size + m_ptr[sa];
            m_ptr[sa] = (m_ptr[sa] + 1) % m_subarray_size;
            on_row(row);
        }
        m_rr = (m_rr + m_rows_per_ref) % m_num_subarrays;
    }

private:
    int m_subarray_size;
    int m_num_subarrays;
    int m_rows_per_ref;
    int m_rr = 0;
    std::vector<int> m_ptr;
};

}  // namespace SB
}  // namespace Ramulator

#endif  // RAMULATOR_PLUGIN_SILVER_BULLET_CORE_H_
