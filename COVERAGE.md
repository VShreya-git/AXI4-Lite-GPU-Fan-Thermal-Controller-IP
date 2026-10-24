# Coverage report 

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



