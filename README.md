# AXI4-Lite GPU Fan & Thermal Controller IP (pure Verilog-2001)

Two asynchronous clock domains, no vendor primitives, no SystemVerilog.

```
 AXI4-Lite master ──► axi4l_slave_if ──► fan_regs ──(cfg bundle, CDC handshake)──► fan_core ─► PWM_OUT[3:0]
      ACLK domain         ▲  W1C IRQ / lock      ◄──(status bundle, CDC)──────────  PCLK domain
                          └─ IRQ                  TACH_IN ─► tach_capture ─► PI ─┐
                                                  TEMP_IN ─► hysteresis ─► fan_curve (4-pt, divider)
                                                  FAULT_IN/watchdog/stale/stall/over-temp ─► safety_ctrl ─► safe
```

## Modules (rtl/)
| file | what |
|---|---|
| `axi4l_slave_if.v` | AXI4-Lite slave: AW/W independent, one outstanding, SLVERR, registered outputs |
| `fan_regs.v` | register file, WSTRB merge, write-lock, IRQ enable/status (W1C) |
| `cdc_bus_hs.v`, `sync_ff.v`, `reset_sync.v` | coherent multi-bit CDC (toggle handshake), 2-FF sync, async-assert/sync-deassert reset |
| `pwm_core.v` | N-channel PWM: shadow regs, edge/center aligned, phase, prescaler, gamma, polarity, complementary + dead time, safe override, sync in/out |
| `tach_capture.v` | gate-window pulse counter -> RPM |
| `fan_curve.v` + `div_seq.v` | 4-point piecewise-linear temp→level with sequential divider |
| `pi_ctrl.v` | fixed-point PI (Q8.8 gains), anti-windup, bumpless transfer |
| `safety_ctrl.v` | sticky faults, watchdog, stale-sensor timeout, fault counter |
| `fan_core.v` | PCLK-domain integration (modes, hysteresis, stall/over-temp detect, telemetry) |
| `fan_ctrl_top.v` | top level (`NCH` = 1..8) |
| `fan_cfg.vh`, `fan_stat.vh` | single source of truth for CDC bundle layouts |
| `fan_ctrl_all.v` | everything concatenated into one file |

## Control modes (CTRL[2:1])
`0` manual duty · `1` curve (open loop) · `2` closed-loop RPM, target = curve×RPM_MAX · `3` closed-loop RPM, target = DUTY0×RPM_MAX.
Duty everywhere is a 16-bit fraction: `0xFFFF` = exactly 100 %, `0x0000` = exactly 0 %.

## Register map (byte offsets, 32-bit)
| addr | name | notes |
|---|---|---|
| 00 | ID (RO) | 0x46414E01 |
| 04 | CTRL | [0] EN [2:1] MODE [3] CENTER [4] COMP_EN [5] TEMP_SRC_SW [6] SYNC_EN |
| 08 | STATUS (RO) | [0] SAFE [1] LOCKED [2] IRQ |
| 0C / 10 | IRQ_EN / IRQ_STATUS (W1C) | [0] fault [1] tach window [2] PWM frame |
| 14 | LOCK | write 0xA5 lock, 0x5A unlock (other → SLVERR) |
| 18 1C 20 24 28 | PRESCALE, PERIOD, DEAD_TIME, POLARITY, FAULT_LEVEL | 🔒 lockable |
| 2C | GAMMA_EN | per channel, x² curve |
| 30..4C | DUTY[0..7] | 16-bit fraction |
| 50..6C | PHASE[0..7] | edge-aligned phase offset (counts) |
| 70 | CURVE_T | {t3,t2,t1,t0} °C |
| 74..80 | CURVE_L0..L3 | 16-bit levels |
| 84 88 8C 90 94 | HYST, T_CRIT 🔒, FAN_MIN, TEMP_SW, TEMP_NOW (RO) | |
| 98 9C A0 | KP, KI (Q8.8), RPM_MAX | |
| A4 A8 AC | TACH_GATE (cycles), TACH_MULT, STALL_CFG | [7:0] duty thr (top byte) [11:8] windows |
| B0 B4 B8 BC C0 | FAULT_EN 🔒, STALE_TO 🔒, WDOG_TO 🔒, WDOG_CTRL 🔒, WDOG_KICK (WO) | |
| C4 C8 CC | FAULT_STATUS (RO), FAULT_CLR, TELE_CLR | faults: [0] pin [1] stall [2] stale [3] wdog [4] over-temp |
| D0..E0 | RPM, DUTY_NOW, DUTY_MIN, DUTY_MAX, FAULT_COUNT (RO) | |

Unmapped, unaligned, read-only-written, nonexistent-channel and locked-register writes return **SLVERR**.

## Run
```
./scripts/run_all.sh            # lint, unit + system tests, synthesis
./scripts/run_all.sh --mutate   # + bug injection
```
Needs: iverilog, verilator, yosys, python3 (all free).

## Verification
* `tb_pwm_core`: randomized vs an independent position-based reference model, cycle-accurate; dead-time/overlap monitor; 0/100 % exactness; safe override; sync.
* `tb_cdc`: bus toggled every source clock at 3 clock ratios; white-box coherence checks.
* `tb_system`: AXI BFM (AW/W orderings, delays, pipelined writes, WSTRB, SLVERR, lock), protocol monitor, fan-plant model (closed loop, saturation/windup recovery), faults, IRQ, reset mid-transaction.
* `scripts/mutate.py`: injects one RTL bug at a time; every mutant must make a testbench fail.

## Known limits (be upfront about these)
* Zero-delay RTL simulation cannot show metastability/skew: run a CDC tool / STA before silicon or FPGA sign-off.
* One reset source assumed for both domains (req/ack toggles start equal).
* PWM dithering and APB/Wishbone wrappers are not implemented.
* `FAULT_IN` takes 5 PCLK cycles to reach the pin (2-FF sync + latch + safe + output flop); internal faults / `safe` take 1 flop to the pin.
