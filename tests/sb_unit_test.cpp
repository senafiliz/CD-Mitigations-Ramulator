// Standalone unit tests for the Silver Bullet core (src/dram_controller/impl/plugin/silver_bullet_core.h).
// Build & run:  g++ -std=c++20 -O2 -I src tests/sb_unit_test.cpp -o build/sb_unit_test && ./build/sb_unit_test
#include "dram_controller/impl/plugin/silver_bullet_core.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace Ramulator::SB;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; std::printf("  FAIL %s:%d: %s | ", __FILE__, __LINE__, #cond); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct Snap { uint32_t frac, pending, li; };
static Snap snap(const Bank& b, int sb) { return {b.frac(sb), b.pending(sb), b.local_index(sb)}; }

// ---------------------------------------------------------------------------------------------
// 1. FRAC / PENDING / LOCAL_INDEX evolve exactly as in the report (counter region = own).
static void test_exact_evolution_own() {
    std::printf("[test] exact FRAC/PENDING/LOCAL_INDEX evolution (own region, S=4, D=3)\n");
    Config c; c.rows_per_bank = 16; c.subbank_size = 4; c.D = 3; c.region = Region::Own; c.validate();
    Bank b(c);
    // Reference model written independently of Bank.
    std::vector<uint32_t> F(4, 0), P(4, 0), L(4, 0);
    auto ref_hammer = [&](int row) { int s = row / 4; if (++F[s] == 3) { F[s] = 0; P[s]++; } };
    auto ref_pop = [&]() -> int {
        int best = -1; for (int s = 0; s < 4; s++) if (P[s] > 0 && (best < 0 || P[s] > P[best])) best = s;
        if (best < 0) return -1; int row = best * 4 + L[best]; L[best] = (L[best] + 1) % 4; P[best]--; return row; };
    auto same = [&](const char* when) {
        for (int s = 0; s < 4; s++) {
            auto x = snap(b, s);
            CHECK(x.frac == F[s] && x.pending == P[s] && x.li == L[s],
                  "%s sb%d got F=%u P=%u L=%u want F=%u P=%u L=%u", when, s, x.frac, x.pending, x.li, F[s], P[s], L[s]);
        }
    };
    // Hand-checked prefix: rows 0,1,2 -> subbank0 FRAC 1,2,0 and PENDING 0,0,1.
    b.hammer(0); CHECK(b.frac(0) == 1 && b.pending(0) == 0, "after 1 act");
    b.hammer(1); CHECK(b.frac(0) == 2 && b.pending(0) == 0, "after 2 acts");
    b.hammer(3); CHECK(b.frac(0) == 0 && b.pending(0) == 1, "FRAC reached D -> PENDING++ and FRAC=0");
    ref_hammer(0); ref_hammer(1); ref_hammer(3);
    // Subbank 2 to PENDING 2, subbank 1 to PENDING 1.
    for (int i = 0; i < 6; i++) { b.hammer(9); ref_hammer(9); }
    for (int i = 0; i < 3; i++) { b.hammer(5); ref_hammer(5); }
    same("after production");
    CHECK(b.total_pending() == 4, "total pending %lu", (unsigned long)b.total_pending());
    // Highest PENDING first: subbank 2 (P=2) -> row 8, LOCAL_INDEX 0->1.
    int r = b.pop_refresh(); int rr = ref_pop();
    CHECK(r == 8 && rr == 8, "first refresh row=%d", r);
    same("after pop 1");
    // Now P = {1,1,1,0}: tie -> lowest subbank index (0), row 0.
    r = b.pop_refresh(); rr = ref_pop(); CHECK(r == 0 && rr == 0, "tie-break row=%d", r);
    r = b.pop_refresh(); rr = ref_pop(); CHECK(r == 4 && rr == 4, "row=%d", r);
    r = b.pop_refresh(); rr = ref_pop(); CHECK(r == 9 && rr == 9, "subbank2 LOCAL_INDEX advanced: row=%d", r);
    CHECK(b.pop_refresh() == -1 && !b.has_pending(), "no pending left");
    same("after all pops");
    // LOCAL_INDEX wraps modulo S: refresh subbank 3 five times.
    for (int i = 0; i < 15; i++) { b.hammer(12); ref_hammer(12); }
    std::vector<int> rows;
    for (int i = 0; i < 5; i++) { rows.push_back(b.pop_refresh()); ref_pop(); }
    CHECK(rows == std::vector<int>({12, 13, 14, 15, 12}), "round robin rows %d %d %d %d %d", rows[0], rows[1], rows[2], rows[3], rows[4]);
    same("after wrap");
    // Randomized differential check vs the reference model.
    std::mt19937 g(7);
    for (int i = 0; i < 100000; i++) {
        if (g() % 4) { int row = g() % 16; b.hammer(row); ref_hammer(row); }
        else { int x = b.pop_refresh(), y = ref_pop(); if (x != y) { CHECK(x == y, "pop mismatch %d vs %d at %d", x, y, i); break; } }
    }
    same("after 100k random ops");
}

// ---------------------------------------------------------------------------------------------
// 2. Counter regions.
static void test_regions() {
    std::printf("[test] counter regions\n");
    Config c; c.rows_per_bank = 64; c.subbank_size = 4;
    c.region = Region::Own; c.validate();
    CHECK(c.hammered_subbanks(5) == std::make_pair(1, 1), "own");
    CHECK(c.max_subbanks_per_hammer() == 1, "own k");
    c.region = Region::BlastRadius; c.blast_radius = 1;
    CHECK(c.hammered_subbanks(4) == std::make_pair(0, 1), "B=1 row 4 -> subbanks 0,1");
    CHECK(c.hammered_subbanks(5) == std::make_pair(1, 1), "B=1 row 5 -> subbank 1");
    CHECK(c.hammered_subbanks(7) == std::make_pair(1, 2), "B=1 row 7 -> subbanks 1,2");
    CHECK(c.hammered_subbanks(0) == std::make_pair(0, 0), "bank edge clamps");
    CHECK(c.hammered_subbanks(63) == std::make_pair(15, 15), "bank edge clamps (top)");
    CHECK(c.max_subbanks_per_hammer() == 2, "S=4,B=1 k=%d", c.max_subbanks_per_hammer());
    c.subbank_size = 1; c.blast_radius = 2;
    CHECK(c.hammered_subbanks(10) == std::make_pair(8, 12), "S=1,B=2 -> 5 per-row counters");
    CHECK(c.max_subbanks_per_hammer() == 5, "k=%d", c.max_subbanks_per_hammer());
    c.region = Region::NeighborSA; c.subarray_size = 8; c.subbank_size = 4;
    CHECK(c.hammered_subbanks(9) == std::make_pair(0, 5), "CD: row 9 (SA1) -> SA0..SA2 -> subbanks 0..5");
    CHECK(c.hammered_subbanks(2) == std::make_pair(0, 3), "CD: SA0 -> SA0..SA1");
    CHECK(c.hammered_subbanks(60) == std::make_pair(12, 15), "CD: last SA -> SA6..SA7");
    c.subbank_size = 16;   // subbank spans two subarrays
    CHECK(c.hammered_subbanks(17) == std::make_pair(0, 1), "CD S=16: row 17 (SA2) -> SA1..SA3 -> subbanks 0..1");
    c.subbank_size = 64;   // whole bank
    CHECK(c.hammered_subbanks(33) == std::make_pair(0, 0), "subbank = bank");
    // Extended-region test from the report's Figure 1: an activation in the overlap hammers both subbanks.
    Config f; f.rows_per_bank = 64; f.subbank_size = 8; f.region = Region::BlastRadius; f.blast_radius = 2; f.D = 2; f.validate();
    Bank b(f);
    b.hammer(7); b.hammer(7);   // row 7 is within B=2 of subbank 1 (rows 8..15)
    CHECK(b.pending(0) == 1 && b.pending(1) == 1 && b.pending(2) == 0, "overlap act produces for both subbanks");
}

// ---------------------------------------------------------------------------------------------
// 3. Refresh row-opens: count_refreshes is a boolean property of the mechanism.
//    Driver identical to the plugin's RFM path: pop, then hammer the refreshed row iff count_refreshes.
static void test_count_refreshes() {
    std::printf("[test] count_refreshes (refresh row-opens hammer counter regions)\n");
    for (bool cnt : {true, false}) {
        Config c; c.rows_per_bank = 32; c.subbank_size = 4; c.D = 2; c.region = Region::BlastRadius; c.blast_radius = 1; c.validate();
        Bank b(c);
        b.hammer(5); b.hammer(5);                 // subbank 1 -> PENDING 1
        int row = b.pop_refresh();                 // row 4: within B of subbanks 0 and 1
        CHECK(row == 4, "row=%d", row);
        if (cnt) b.hammer(row);
        CHECK(b.frac(0) == (cnt ? 1u : 0u) && b.frac(1) == (cnt ? 1u : 0u),
              "count_refreshes=%d frac0=%u frac1=%u", (int)cnt, b.frac(0), b.frac(1));
    }
}

// ---------------------------------------------------------------------------------------------
// 4. REF row mapping equals salt_c.cpp ref_coordinate().
static void test_ref_mapping() {
    std::printf("[test] REF row mapping (salt_c.cpp ref_coordinate)\n");
    int rows = 131072, sa = 1024, rpr = rows / 8192;   // 16 rows per REF per bank, 128 subarrays
    RefMapper m(rows, sa, rpr);
    // Independent re-implementation of ref_coordinate's row choice.
    std::vector<int> ptr(rows / sa, 0); int rr = 0;
    bool ok = true; long n = 0;
    std::vector<int> hits(rows, 0);
    for (int ref = 0; ref < 8192; ref++) {     // one refresh window (tREFW / tREFI = 8192 REFs)
        std::vector<int> got; m.next_ref([&](int r) { got.push_back(r); hits[r]++; });
        for (int k = 0; k < rpr; k++) {
            int s = (rr + k) % (rows / sa);
            int want = s * sa + ptr[s]; ptr[s] = (ptr[s] + 1) % sa;
            ok &= (got[k] == want); n++;
        }
        rr = (rr + rpr) % (rows / sa);
    }
    CHECK(ok, "mapping differs");
    bool each_once = true; for (int h : hits) each_once &= (h == 1);
    CHECK(each_once, "every row refreshed exactly once per 8192 REFs");
    CHECK(n == rows, "n=%ld", n);
}

// ---------------------------------------------------------------------------------------------
// 5. Observation 1: D >= 2(T/R + 1) prevents PENDING overflow.
// Model of Section 3.3: within each window of T activations, R preventive refreshes are performed
// at the end of the window, highest PENDING first, and every refresh row-open hammers its counter
// regions (count_refreshes = true). Worst case for production: every row-open hammers exactly
// two subbanks (S=2, B=1: row 2s hammers s-1,s; row 2s+1 hammers s,s+1).
struct RunResult { uint32_t max_pending; uint64_t final_pending; };

template <typename NextRow>
static RunResult run_windows(Config c, int T, int R, long acts, NextRow next_row) {
    c.validate();
    Bank b(c);
    uint64_t mx_total = 0;
    for (long i = 1; i <= acts; i++) {
        b.hammer(next_row(i));
        if (i % T == 0)
            for (int k = 0; k < R; k++) { int row = b.pop_refresh(); if (row < 0) break; b.hammer(row); }
        mx_total = std::max<uint64_t>(mx_total, b.total_pending());
    }
    (void)mx_total;
    return {b.max_pending(), b.total_pending()};
}

static void test_obs1() {
    std::printf("[test] Observation 1: D >= 2(T/R+1) bounds PENDING\n");
    const int T = 8, R = 2;    // bound: D >= 2(8/2 + 1) = 10
    CHECK(satisfies_obs1(10, T, R) && !satisfies_obs1(9, T, R), "bound arithmetic");
    Config c; c.rows_per_bank = 1024; c.subbank_size = 2; c.region = Region::BlastRadius; c.blast_radius = 1;
    CHECK(c.max_subbanks_per_hammer() == 2, "k=2 layout");
    auto attack = [](long) { return 511; };   // single aggressor in the overlap of subbanks 255 and 256
    std::printf("  single-row attack, T=%d R=%d (bound D>=10)\n", T, R);
    std::printf("  %4s %10s %14s %14s\n", "D", "acts", "max PENDING", "final PENDING");
    for (int D : {8, 9, 10, 11, 16}) {
        c.D = D;
        RunResult r1 = run_windows(c, T, R, 100000, attack);
        RunResult r2 = run_windows(c, T, R, 1000000, attack);
        std::printf("  %4d %10d %14u %14lu\n", D, 100000, r1.max_pending, (unsigned long)r1.final_pending);
        std::printf("  %4d %10d %14u %14lu\n", D, 1000000, r2.max_pending, (unsigned long)r2.final_pending);
        if (satisfies_obs1(D, T, R)) {
            CHECK(r2.max_pending == r1.max_pending && r2.max_pending <= (uint32_t)std::ceil(R / 2.0) + 1,
                  "D=%d satisfies Obs.1 but PENDING grows: %u -> %u", D, r1.max_pending, r2.max_pending);
        } else {
            CHECK(r2.final_pending > 5 * r1.final_pending && r2.final_pending > 1000,
                  "D=%d violates Obs.1; expected unbounded growth, got %lu -> %lu", D,
                  (unsigned long)r1.final_pending, (unsigned long)r2.final_pending);
        }
    }
    // Random many-row traffic with D at the bound: PENDING stays within the Table 3 width bound
    // ceil(log2(N_SB)) + R/2 (random traffic is weaker than the worst-case wave attack).
    c.D = 10;
    std::mt19937 g(1);
    for (int span : {4, 64, 1024}) {
        RunResult r = run_windows(c, T, R, 2000000, [&](long) { return (int)(g() % span); });
        double bound = std::log2((double)c.num_subbanks()) + R / 2.0;
        std::printf("  random rows in [0,%d): max PENDING=%u (Table 3 bound log2(N_SB)+R/2 = %.1f)\n", span, r.max_pending, bound);
        CHECK(r.max_pending <= bound, "span=%d max pending %u > %.1f", span, r.max_pending, bound);
    }
    // Same attack with refresh row-opens not counted: only 2T/D production, bound becomes D >= 2T/R = 8.
    {
        Config c2 = c; c2.D = 8; c2.validate();
        Bank b(c2); uint32_t mx1 = 0;
        for (long i = 1; i <= 1000000; i++) {
            b.hammer(511);
            if (i % T == 0) for (int k = 0; k < R; k++) if (b.pop_refresh() < 0) break;
            if (i == 100000) mx1 = b.max_pending();
        }
        std::printf("  count_refreshes=false, D=8: max PENDING %u (100k) / %u (1M)\n", mx1, b.max_pending());
        CHECK(b.max_pending() == mx1, "D=8 without refresh counting should be bounded");
    }
}

int main() {
    test_exact_evolution_own();
    test_regions();
    test_count_refreshes();
    test_ref_mapping();
    test_obs1();
    std::printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
