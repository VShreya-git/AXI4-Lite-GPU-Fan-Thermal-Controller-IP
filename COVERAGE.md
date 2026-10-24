# Coverage report (Verilator 5.020, all three testbenches)

Measured on the RTL only (`rtl/*.v`, 13 files).  Reproduce with `./scripts/run_cov.sh`.
Raw table: `coverage_summary.txt`; uncovered points: `coverage_uncovered.txt`.

| metric | union of all TBs | strict (system TB only, each instance counted) |
|---|---|---|
| Line   | **98.7 %** (150 pts) | 98.0 % |
| Branch | **97.4 %** (152 pts) | 95.8 % |
| Toggle | **83.2 %** (4,854 pts) | 82.7 % |

*Union* merges the three testbenches and treats every instance of a module as one.
*Strict* counts every instance separately and uses only the full-system testbench (the harder number).

## What the numbers mean
* **Line / branch** (the important ones): every RTL file is at 100 % except the points below.
* **Toggle** is lower by nature: wide counters and registers (24-bit timeouts, 32-bit gate, 16-bit phase on unused channels, upper bits of
  count registers) never flip every bit in a short simulation. Per file it ranges 48 %..100 %.

## What is still not covered (and why)
| where | what | reason |
|---|---|---|
| `fan_curve.v` default state, `nt <= bt` check | defensive code | unreachable with a monotonic curve table |
| `pwm_core.v` dead-time FSM `default` | illegal-state recovery | unreachable (2-bit state, 3 legal values) |
| `fan_core.v` stall counter saturate at 15 | saturation | needs 15+ consecutive stalled windows beyond the fault threshold |
| `safety_ctrl.v` stale/watchdog counter saturate | saturation | needs 16.7 M cycles |
| toggle: `r_phase[4..7]`, `r_duty[4..7]` | channels 4-7 | design instantiated with NCH = 4 |

## Gaps that coverage found in MY tests (and that I then fixed)
1. Most registers were never **read back** (read path untested), and some never written. -> added a register walk (write two patterns, read, restore) for every writable register.
2. Fan-curve **segment 2 (60-75 C)** never exercised in the regression. -> added check (68 C -> 57889).
3. Command registers: **"write of 0" / wrong byte-strobe does nothing** never exercised. -> added checks.
4. Dead-time = 0 complementary mode never exercised. -> added check.
Line coverage went 82.7 % -> 98.7 %, toggle 63.1 % -> 83.2 % because of these additions.

## Caveats
* Coverage shows what was *executed*, not that it was *checked*. A line can be covered by a test that never checks its result
  (this is why bug injection is run as well: 21 of 22 injected bugs detected).
* Not measured: functional coverage (cross of modes x faults x clock ratios), FSM-transition coverage, assertion coverage.
* Verilator and Icarus Verilog both run the same testbenches and both pass, which also rules out simulator-specific behaviour.
