
# Pico‑ICE Lab Manual: Beginner → Professional

A step‑by‑step, hands‑on path to master the **Pico‑ICE** (RP2040 + Lattice iCE40UP5K) with useful‑only labs. Every lab lists **objectives, prerequisites, estimated time, steps, validation, troubleshooting,** and **GitHub references** so you can go fast and stay practical.

> **Primary Repos** (bookmark these):
> - **Pico‑ICE board repo**: https://github.com/tinyvision-ai-inc/pico-ice
> - **Pico‑ICE SDK (RP2040 firmware)**: https://github.com/tinyvision-ai-inc/pico-ice-sdk
> - **Pico‑ICE MicroPython builds**: https://github.com/tinyvision-ai-inc/pico-ice-micropython

---

## Lab 0 — Prep & Installation
**Objective.** Install tooling, clone repos, verify USB connectivity.

**Prereqs.**
- PC on Windows/macOS/Linux with USB‑C cable
- Basic shell/terminal familiarity

**Estimated time.** 20–40 min

**Steps.**
1) **Clone repos** (you’ll pull examples from these throughout):
   ```bash
   git clone https://github.com/tinyvision-ai-inc/pico-ice
   git clone https://github.com/tinyvision-ai-inc/pico-ice-sdk
   ```
2) **Install toolchains** (pick one or both):
   - **Open‑source FPGA flow**: Yosys + nextpnr + IcePack (via OSS CAD Suite/Tabby CAD).
   - **RP2040 C/C++**: CMake + ninja + arm‑none‑eabi‑gcc (Raspberry Pi pico‑sdk requirements). See examples inside **pico‑ice‑sdk**.
3) **Connect board** and confirm a serial device appears (you’ll use it in later labs).

**Validate.** You can `cd pico-ice` and see `Board/`, `Docs/`, `Firmware/`, `rtl/` etc. You can also open `pico-ice-sdk/examples`.

**Troubleshooting.** If Windows doesn’t enumerate DFU later, install `libusbK` for the DFU interface (see SDK notes and typical DFU workflows).

**GitHub refs.**  
- Pico‑ICE: https://github.com/tinyvision-ai-inc/pico-ice  
- SDK: https://github.com/tinyvision-ai-inc/pico-ice-sdk

---

## Lab 1 — Board Bring‑Up & Default Firmware
**Objective.** Verify the shipped firmware, learn BOOTSEL and MSC/DFU modes.

**Prereqs.** Lab 0

**Estimated time.** 10–20 min

**Steps.**
1) Plug Pico‑ICE via USB‑C. You should see the RGB LED blink (default firmware).  
2) **Enter RP2040 BOOTSEL**: hold BOOT (BT to GND) then press RESET; the board mounts as `RPI-RP2`.  
3) From releases in **pico‑ice** (or from **pico‑ice‑sdk** examples you build later), drag a `.uf2` to update the firmware.  
4) Power‑cycle; open your serial terminal to note which **USB CDC ports** enumerate.

**Validate.** Board reboots cleanly; `RPI-RP2` shows up only in BOOTSEL; normal mode shows CDC ports.

**Troubleshooting.** If it remains in `RPI-RP2`, ensure the BT pad is not still grounded before resetting.

**GitHub refs.**  
- Board repo (firmware releases, docs): https://github.com/tinyvision-ai-inc/pico-ice  
- SDK (default firmware sources): https://github.com/tinyvision-ai-inc/pico-ice-sdk

---

## Lab 2 — FPGA Hello World (Blink)
**Objective.** Build and load a minimal FPGA bitstream to blink the on‑board LED.

**Prereqs.** Lab 0–1; open‑source FPGA toolchain.

**Estimated time.** 30–45 min

**Steps.**
1) In `pico-ice/rtl/` locate a simple example (e.g., `blinky` or a minimal top). If not present, create `blinky.v`:
   ```verilog
   module top(
     input  wire clk,
     output wire led_r
   );
     reg [23:0] cnt;
     always @(posedge clk) cnt <= cnt + 1;
     assign led_r = cnt[23];
   endmodule
   ```
2) Use the **board PCF** constraints from the repo (pin names for LED/clock). If a ready PCF isn’t provided in your folder, copy one from `pico-ice/Docs/` or the repo issues/examples.
3) Synthesize, place/route, and pack:
   ```bash
   yosys -p "synth_ice40 -top top -json top.json" blinky.v
   nextpnr-ice40 --up5k --package sg48 --json top.json --pcf pico_ice.pcf --asc top.asc
   icepack top.asc top.bin
   ```
4) **Load to CRAM (volatile) via DFU** for quick iterate:
   ```bash
   dfu-util --alt 1 --download top.bin
   ```

**Validate.** LED blinks. Adjust divider `cnt[23]` for different rates.

**Troubleshooting.** If DFU device not found on Windows, install `libusbK` on the DFU interface.

**GitHub refs.**  
- Board source & RTL folder: https://github.com/tinyvision-ai-inc/pico-ice  
- SDK (use later for clock/reset helpers): https://github.com/tinyvision-ai-inc/pico-ice-sdk

---

## Lab 3 — RP2040 Baseline Firmware (SDK Example)
**Objective.** Build and flash a working RP2040 firmware using **pico‑ice‑sdk**.

**Prereqs.** Lab 0–1; CMake toolchain for RP2040.

**Estimated time.** 30–60 min

**Steps.**
1) `cd pico-ice-sdk/examples/pico_usb_uart`
2) Initialize submodules (if not already):
   ```bash
   git -C ../../ submodule update --init
   git -C ../../pico-sdk submodule update --init lib/tinyusb
   ```
3) Build:
   ```bash
   mkdir build && cd build
   cmake -DPICO_BOARD=pico_ice ..
   make -j
   ```
4) Enter BOOTSEL (Lab 1) and drag the produced `.uf2` onto `RPI-RP2`.

**Validate.** Open the enumerated USB‑CDC port and confirm UART passthrough banner or activity.

**Troubleshooting.** If a C++ compiler error occurs, ensure `arm-none-eabi-g++` is installed.

**GitHub refs.**  
- SDK (examples, includes): https://github.com/tinyvision-ai-inc/pico-ice-sdk

---

## Lab 4 — MCU‑Driven FPGA Control (Clock/Reset + UF2 Programming)
**Objective.** Control the FPGA clock/reset from RP2040 and learn UF2 programming to Flash.

**Prereqs.** Lab 2–3

**Estimated time.** 45–75 min

**Steps.**
1) In your firmware, use SDK helpers (see `include/ice_fpga.h`) to **start/stop FPGA** and set an export **clock frequency**:
   ```c
   #include "ice_fpga.h"
   int main(){
     ice_fpga_init(12);  // 12 MHz to FPGA
     bool ok = ice_fpga_start();
     // ...
   }
   ```
2) Convert a `*.bin` bitstream into **UF2** and drag‑drop to the **pico‑ice MSC** interface (the firmware exposes this when built accordingly). Use Microsoft’s UF2 tool if needed: https://github.com/microsoft/uf2
3) Reboot FPGA and verify design comes up from Flash.

**Validate.** Design persists after power cycle.

**Troubleshooting.** If Flash stays off at boot, confirm your firmware enables it or reload via DFU once.

**GitHub refs.**  
- SDK headers: https://github.com/tinyvision-ai-inc/pico-ice-sdk/tree/main/include  
- UF2 tool: https://github.com/microsoft/uf2

---

## Lab 5 — MCU↔FPGA SPI Register Map
**Objective.** Implement a small memory‑mapped register file in FPGA, accessed from RP2040 over SPI.

**Prereqs.** Lab 2–4

**Estimated time.** 60–120 min

**Steps.**
1) In FPGA RTL, add a simple SPI slave with 8‑bit address + 8‑bit data (R/W). Expose regs: `0x00` status, `0x01` LED control, `0x02` counter low, `0x03` counter high.
2) In RP2040 firmware, use SDK **SPI** helpers (see `include/ice_spi.h`) to transact. Example pseudo‑code:
   ```c
   #include "ice_spi.h"
   uint8_t rd(uint8_t a){ uint8_t b[2]={a|0x80,0}; ice_spi_xfer(b,2); return b[1]; }
   void wr(uint8_t a,uint8_t v){ uint8_t b[2]={a&0x7F,v}; ice_spi_xfer(b,2); }
   ```
3) Write tests: toggle LED via register; read back the counter.

**Validate.** Register writes/reads match logic analyzer traces.

**Troubleshooting.** Check CPOL/CPHA and bit‑endian; probe PMOD pins to confirm clock/data.

**GitHub refs.**  
- SDK SPI include: https://github.com/tinyvision-ai-inc/pico-ice-sdk/blob/main/include/ice_spi.h  
- SDK examples folder (use as scaffolding): https://github.com/tinyvision-ai-inc/pico-ice-sdk/tree/main/examples

---

## Lab 6 — External qSPI SRAM as Buffer
**Objective.** Store/stream data using the on‑board 8 MB qSPI SRAM from FPGA/MCU.

**Prereqs.** Lab 5

**Estimated time.** 60–120 min

**Steps.**
1) Review SDK **SRAM** helpers (see `include/ice_sram.h`) and data‑path around the main SPI bus.
2) Write RP2040 firmware that **streams a burst** into SRAM, then signals the FPGA via a register.
3) FPGA reads from SRAM and processes (e.g., moving average on samples), then exposes results via registers or DMA‑like streaming back to MCU.

**Validate.** Host script (Python) sends data → MCU → SRAM → FPGA → MCU → PC, with integrity checks.

**Troubleshooting.** Confirm chip‑select switching between FPGA/SRAM/Flash if sharing the bus; use CDC‑SPI for quick diagnostics.

**GitHub refs.**  
- SDK SRAM include: https://github.com/tinyvision-ai-inc/pico-ice-sdk/blob/main/include/ice_sram.h  
- SDK USB SPI/CDC patterns (examples/tools): https://github.com/tinyvision-ai-inc/pico-ice-sdk/tree/main/examples

---

## Lab 7 — PMOD Expansion (Sensor In, Real‑Time Process)
**Objective.** Attach a PMOD sensor (e.g., ADC/IMU), capture in FPGA, forward features to MCU.

**Prereqs.** Lab 5–6

**Estimated time.** 60–180 min (depends on PMOD)

**Steps.**
1) Pick a PMOD and wire to a **FPGA‑dedicated PMOD** header.
2) Use SDK **PMOD unions** (pin naming convenience) and/or define pins in your PCF/RTL. See `include/ice_pmod.h`.
3) Implement SPI/I²C controller in FPGA or use RP2040 PIO on the **RP‑only PMOD** as a baseline, then migrate to FPGA for hard‑real‑time.

**Validate.** Live sensor plots on host; deterministic pre‑processing in FPGA.

**Troubleshooting.** Double‑check PMOD pinout orientation; probe SCL/SDA/MISO/MOSI.

**GitHub refs.**  
- SDK PMOD include: https://github.com/tinyvision-ai-inc/pico-ice-sdk/blob/main/include/ice_pmod.h  
- SDK includes index: https://github.com/tinyvision-ai-inc/pico-ice-sdk/tree/main/include

---

## Lab 8 — MicroPython Fast Path (No C Toolchain)
**Objective.** Program the FPGA from MicroPython and toggle pins/registers quickly for demos.

**Prereqs.** None beyond Lab 1

**Estimated time.** 20–40 min

**Steps.**
1) Download a MicroPython build for Pico‑ICE and drag the UF2 to `RPI-RP2`:  
   https://github.com/tinyvision-ai-inc/pico-ice-micropython/releases
2) Copy your `bitstream.bin` to the board storage (or fetch via REPL).
3) Use the built‑in `ice` module to start the FPGA and `cram()` the bitstream:
   ```python
   from machine import Pin
   import ice
   fpga = ice.fpga(cdone=Pin(26), clock=Pin(24), creset=Pin(27),
                   cram_cs=Pin(9), cram_mosi=Pin(8), cram_sck=Pin(10), frequency=12)
   with open('bitstream.bin','br') as f:
       fpga.start(); fpga.cram(f)
   ```

**Validate.** Design runs immediately; adjust `frequency` to match your constraints.

**Troubleshooting.** If `cram()` succeeds but design misbehaves, verify the FPGA clock frequency and PCF pin mapping.

**GitHub refs.**  
- MicroPython releases: https://github.com/tinyvision-ai-inc/pico-ice-micropython/releases

---

## Lab 9 — USB CDC‑SPI Host Bridge (PC ↔ FPGA/SRAM/Flash)
**Objective.** Control the on‑board SPI fabric directly from a PC over USB CDC for bring‑up/tests.

**Prereqs.** Lab 3–6

**Estimated time.** 45–90 min

**Steps.**
1) Build/run the **USB‑SPI/CDC** example in the SDK (see `examples`); it exposes a tiny protocol over CDC to switch CS and read/write SPI.
2) Write a short Python host script to send frames and dump reads; use it to verify SRAM and FPGA register access without custom firmware loops.
3) Integrate with your CI to smoke‑test boards by reading an FPGA ID register or SRAM pattern.

**Validate.** Round‑trip reads/writes to SPI devices match logic analyzer traces.

**Troubleshooting.** If no CDC port appears, ensure TinyUSB is enabled and `tud_task()` is serviced frequently in firmware.

**GitHub refs.**  
- SDK examples (USB‑SPI/CDC patterns): https://github.com/tinyvision-ai-inc/pico-ice-sdk/tree/main/examples  
- SDK library sources (USB, SPI): https://github.com/tinyvision-ai-inc/pico-ice-sdk/tree/main/src

---

## Lab 10 — Capstone: Mini Hardware Accelerator
**Objective.** Build a streaming **FIR filter** (or Sobel edge detector) in FPGA with MCU orchestration and host benchmarks.

**Prereqs.** Lab 2–7

**Estimated time.** 1–3 days

**Steps.**
1) **Spec** a simple streaming interface (e.g., SPI in/out, or SRAM ring‑buffer).
2) **Implement** the accelerator in FPGA RTL; simulate if you wish using the repo’s `rtl/sim` scaffolding.
3) **Integrate** with RP2040 firmware to feed data, manage buffers, and collect metrics.
4) **Benchmark** throughput/latency vs. a pure‑MCU implementation; plot results on host.

**Validate.** Stable throughput; correctness vs. software reference.

**Troubleshooting.** Clock‑domain crossings, backpressure, and CS timing are common pain points—instrument with counters and a spare LED/error register.

**GitHub refs.**  
- Board repo RTL/sim: https://github.com/tinyvision-ai-inc/pico-ice/tree/main/rtl  
- SDK examples (baseline app): https://github.com/tinyvision-ai-inc/pico-ice-sdk/tree/main/examples

---

## Appendix A — Pinout & Constraints
- Pinouts, schematics, and board docs live in the board repo (check `Docs/`, `Board/`, and README):  
  https://github.com/tinyvision-ai-inc/pico-ice
- For SDK pin helpers: browse `include/` headers such as `pico_ice.h`, `ice_pmod.h`, `ice_fpga.h`, `ice_spi.h`:  
  https://github.com/tinyvision-ai-inc/pico-ice-sdk/tree/main/include

## Appendix B — Useful Commands
```bash
# DFU to Flash (persistent) vs CRAM (volatile)
dfu-util --alt 0 --download design.bin --reset

dfu-util --alt 1 --download design.bin

# Convert BIN -> UF2 (drag to MSC)
# see https://github.com/microsoft/uf2
bin2uf2 -o design.uf2 design.bin
```

## Appendix C — Checklists
**Before P&R:** PCF pins match schematic; clock net named and constrained.  
**Before DFU:** Board enumerates DFU; correct alt‑setting for Flash vs CRAM.  
**Before streaming:** SPI mode agreed (CPOL/CPHA); CS lines not shared unexpectedly; analyzer ready.

---

**You’re done!** Keep this manual beside your clones of:
- https://github.com/tinyvision-ai-inc/pico-ice
- https://github.com/tinyvision-ai-inc/pico-ice-sdk
- https://github.com/tinyvision-ai-inc/pico-ice-micropython
