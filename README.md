# AXI4-Lite-PWM-LED-Controller-RTL-Simulation

A synthesizable AXI4-Lite slave peripheral written in Verilog: a memory-mapped PWM/LED controller with configurable duty cycle, an auto-breathing mode, and an interrupt-generating status register. Built to demonstrate AXI4-Lite protocol implementation (explicit write/read FSMs) alongside a real hardware datapath behind the bus interface.

## Features

- Full AXI4-Lite slave interface (AW/W/B/AR/R channels) with explicit FSMs for write and read paths
- Byte-strobe (`WSTRB`) aware register writes
- Memory-mapped control/status/data registers
- Free-running PWM engine with configurable period and duty cycle
- "Breathing LED" auto mode (duty cycle ramps up/down automatically)
- Sticky, write-1-to-clear overflow status flag
- Level-triggered interrupt output (`PWM_IRQ`)
- Self-checking testbench, simulated in Icarus Verilog with waveform capture

## Register Map

| Offset | Name   | Bits                              | Description                          |
|--------|--------|-----------------------------------|---------------------------------------|
| 0x0    | CTRL   | [0] enable, [1] mode (0=fixed, 1=breathe) | Control register              |
| 0x4    | DUTY   | [7:0]                             | PWM duty cycle (0-255)               |
| 0x8    | STATUS | [0] busy, [1] overflow (W1C)      | Status / interrupt flag              |
| 0xC    | PERIOD | [15:0]                            | PWM period, in clock cycles          |




