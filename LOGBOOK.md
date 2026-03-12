# ESP32 Trovis 5575 SPI Slave – Development Logbook

## Protocol Background

The Trovis 5575 heating controller acts as SPI **master**. An external slave (ESP32) can respond to read register values. Key protocol facts discovered through reverse-engineering and Benvorth's prior work:

- **MISO bytes are logically inverted**: `miso_byte = 0xFF - cmd_byte`
  - e.g. Modbus header `0x06` → MISO physical byte `0xF9`
- **MOSI bytes are NOT inverted** — read directly
- **SPI mode**: CPOL=1 (clock idles HIGH), CPHA=1 — Trovis drives MOSI on falling SCK edge, slave samples on rising edge → **SPI_MODE3**
- **CS polarity**: Trovis CS is **active-HIGH** (idle LOW). ESP32 SPI hardware requires active-LOW → must invert
- **CS inversion**: GPIO25 (Trovis CS input) → software ISR → GPIO26 → GPIO5 (ESP32 SPI CS)
- **Clock speed**: ~50 kHz (20 µs/cycle)
- **Timing**:
  - CS↑ to first SCK↓: **52 µs** (setup window — ISR latency ~5 µs fits easily)
  - Inter-byte gap (SCK↑ to next SCK↓): **30 µs**
  - Inter-transaction gap (CS↓ to next CS↑): **~997 ms**

### Transaction sequence (Benvorth stage model)

| Stage | Name | MISO | Trovis sees | Expected rxLen |
|---|---|---|---|---|
| 0 | PSV | `FF FF FF ...` | `00 00 00 ...` | 4B (poll only) |
| 1 | ANN | `F9 FF FF ...` | `06 00 00 ...` | 32B (Trovis extends) |
| 2..N+1 | CMD | `F9 F3 FF 05 ...` | full Modbus CMD | 32B |
| → back to 0 | | | | |

### CMD frame (14 bytes, non-inverted)

```
[0]     0x06        header (→ MISO 0xF9)
[1]     0x0C        length
[2-4]   00 FA 08    fixed preamble
[5]     0xFF        device selector
[6]     0x03        Modbus fn: read holding registers
[7-8]   reg_hi/lo   register address
[9-10]  00 01       count = 1
[11-12] crc_hi/lo   CRC-16 Modbus over bytes [5..10]
[13]    0x55        terminator
```

Register response from Trovis at MOSI bytes `[CMD_LEN+5]` (hi) and `[CMD_LEN+6]` (lo).

---

## Experiments

### Experiment 1–3 (pre-context)
Early exploration with passive mode and T1/T2 split (two separate SPI slots). Not fully documented here.

---

### Experiment 4 — 2026-03-10
**Serial log**: `serial-log-2026-03-10-4.txt`
**PulseView CSV**: `trovis-comms-spi-esp32-2026-03-10-4-a.csv`

**Setup**: Two-slot approach (T1 = 4B poll, T2 = CMD). SPI_MODE2.

**Observations**:
- T2/CMD always received `MOSI = 30 00 00 00` (same as T1 poll data)
- `rxLen = 4` throughout — Trovis never extended to 32B
- PulseView: MISO = `00` for all transactions
- T1 (4B) and T2 (1–2B) appeared as sub-transfers within a single CS window

**Conclusions**:
- The T1/T2 split was incorrect — Trovis uses ONE CS assertion for both poll and response
- MISO = 0x00 on wire: ESP32 not driving MISO before first clock edge
- Serial.printf between `wait()` and `queue()` was causing ~10 ms re-arm delay (later found irrelevant given 997 ms inter-transaction gap)
- T1/T2 concept eliminated; moved to single-slot architecture

---

### Experiment 5 — 2026-03-11
**Serial log**: `serial-log-2026-03-11-1.txt`
**PulseView CSV**: `trovis-comms-spi-esp32-2026-03-11-1-a.csv`

**Setup**: Single-slot, SPI_MODE3, separate ANN transaction before CMD loop. No T1/T2 split.

**Observations**:
- Serial log: `MOSI = 18 00 00 00` (was `30 00 00 00` in MODE2 experiments)
- `0x18 = 0x30 >> 1` — one-bit right shift
- `rxLen = 4` throughout — still no 32B extension
- PulseView (decoded with MODE2 decoder): MISO still = `00 00 00 00`
- PulseView decoded MOSI as `30 00 00 00` (correct with MODE2 decoder)

**Analysis**:
- MOSI shift `0x30 → 0x18`: ISR latency causes GPIO5 to go LOW a few µs after Trovis CS↑. First SCK↓ (at 52 µs) is well after GPIO5↓ (~5 µs), so ESP32 is ready. The 1-bit shift was due to **PulseView decoder mismatch** (decoder in MODE2, ESP32 in MODE3 → decoder sampled at transition point → garbled). ESP32 receiving `0x18` likely a transient artefact.
- MISO = 0x00: ESP32 SPI hardware not driving MISO. Root cause still under investigation.
- Confirmed: **Trovis drives MOSI on falling SCK edge** (directly observed in PulseView) → MODE3 is correct.
- PulseView decoder must be set to **MODE3** to match wire behaviour.

**Changes after experiment 5**:
- Confirmed SPI_MODE3 is correct; kept it
- Restructured task loop: `queue()` called immediately after `wait()` — **before** any Serial output — to minimise re-arm latency
- Added PSV (stage 0, MISO=0xFF) before ANN to match Benvorth's stage sequence exactly
- ISR changed from `cs = !cs` toggle to `digitalRead(pin_inv_SS)` for robustness

---

### Experiment 6 — 2026-03-11
**Serial log**: `serial-log-2026-03-11-6.txt`
**PulseView CSV**: `trovis-comms-spi-esp32-2026-03-11-6-a.csv`

**Setup**: Single-slot state machine (PSV → ANN → CMD × 8 → PSV). SPI_MODE3. ISR uses `digitalRead()`. Re-arm with `queue()` immediately after `wait()`, before any Serial output. ESP32SPISlave v0.8.0.

**Observations**:
- **Progress**: MISO is no longer 0x00 — ESP32 IS driving the MISO line. (Fixed since experiments 4–5 by code restructuring.)
- **Serial log**: PSV shows `MOSI = 00 00 00 00`, all ANN/CMD show `MOSI = 18 00 00 00`, `rxLen = 4`, "too short" throughout.
- **PulseView (MODE3 decoder, 5× oversampling)**:
  - Txn 1: `MISO = F3 FF FF FF`, `MOSI = 30 00 00 00`
  - Txn 2: `MISO = F3 CF F8`, `MOSI = 30 00 00`
  - Txn 3: `MISO = F3 CF F8 1E`, `MOSI = 30 00 00 00`
- Signal quality: stable, no glitches. Same MISO decode across all SPI modes 0–3 in PulseView.
- MOSI 1-bit shift persists: wire = `0x30`, ESP32 receives `0x18`.

**Analysis**:
- MISO outputs **F3** instead of the expected **F9**. Both MISO and MOSI show a consistent 1-bit shift:
  - MOSI: `0x30` (00110000) on wire → ESP32 reads `0x18` (00011000) = right-shift by 1
  - MISO: ESP32 sends `0xF9` (11111001) → wire shows `0xF3` (11110011)
- The MISO values for CMD transactions (`F3 CF F8`) do NOT match a simple bit shift of the expected CMD bytes (`F9 F3 FF 05`). The corruption pattern is more complex than a single-bit phase offset.
- Because Trovis never sees `0x06` (inverted from `0xF9`), it doesn't extend the transaction to 32 bytes. All CMDs remain 4B with "too short" errors.
- **Root cause hypothesis**: The new ESP32SPISlave v0.8.0 API (`slave.queue()` + `slave.wait()`) loads the SPI TX FIFO differently from Benvorth's proven old API (`slave.wait(rx, tx, size)` + `slave.pop()`). Benvorth's version produced correct F9 on MISO. This is the only significant difference between the two setups.

### Experiment 7 — 2026-03-11
**Serial log**: `serial-log-2026-03-11-7.txt`
**PulseView CSV**: `trovis-comms-spi-esp32-2026-03-11-7-a.csv`

**Setup**: Two-task architecture (matching Benvorth). **ESP32SPISlave v0.3.0** (old API: `slave.wait(rx, tx, size)` + `slave.pop()`). SPI_MODE3. ISR uses `cs = !cs` toggle (Benvorth style). `pinMode(pin_inv_SS, INPUT)` without PULLDOWN. ESP32 Arduino core 3.3.7.

**Observations**:
- **MISO still F3 on wire** — identical to experiment 6
- PulseView: `MISO = F3 CF F8`, `MOSI = 30 00 00` (same as exp 6)
- Serial log: MOSI = `18 00 00 00 F7 ...` (32-byte buffers now visible with old API)
- First cycle: ESP32 reads back its own MISO as MOSI (PSV: FF, ANN: F9) — likely crosstalk during boot before Trovis drives MOSI

**Conclusions**:
- **Library version is NOT the root cause.** v0.3.0 (old API) produces identical MISO corruption as v0.8.0 (new API).
- Both APIs are thin wrappers around ESP-IDF's `spi_slave_transmit()` / `spi_slave_queue_trans()`. The corruption is in the ESP-IDF SPI slave driver layer.
- Benvorth likely used Arduino ESP32 core 2.x (ESP-IDF 4.4). We use core 3.3.7 (ESP-IDF 5.x). The ESP-IDF SPI slave driver was reworked between versions.

---

### Experiment 8 — 2026-03-12
**Serial log**: `serial-log-2026-03-12-8.txt`
**PulseView CSV**: `trovis-comms-spi-esp32-2026-03-12-8-a.csv`

**Setup**: Same as experiment 7 + direct SPI register patch after `slave.begin()`:
```c
SPI3.pin.ck_idle_edge = 1;  // Clock idles HIGH (CPOL=1)
SPI3.user.ck_i_edge = 1;    // Input sampled on correct edge
```
Motivated by known ESP-IDF bug reports: GitHub issues espressif/esp-idf #7698, #15762, #9058.

**Observations**:
- **MISO still F3 on wire** — unchanged
- **MOSI reception changed**: ESP32 now reads `F9` (its own MISO output) instead of `18` (shifted Trovis MOSI). The `ck_i_edge` change shifted the MOSI sampling edge, causing ESP32 to read back its own MISO via crosstalk.

**Conclusions**:
- The one-time register patch in `setupSPISlave()` either (a) addresses the wrong registers, or (b) is overwritten by `spi_slave_transmit()` which reconfigures SPI registers before each transaction.
- The `ck_i_edge = 1` change DID affect MOSI sampling (proving the register write works), but did not fix MISO output timing. This suggests a different register controls MISO output edge.
- The ESP-IDF bug reports (#7698, #15762, #9058) mostly describe first-bit-only corruption, which is a narrower symptom than our multi-bit corruption across all bytes.

---

## Key Findings

| Finding | Detail |
|---|---|
| SPI mode | MODE3 (CPOL=1, CPHA=1) confirmed by wire observation |
| MISO inversion | Physical MISO = `0xFF - logical_byte` |
| MOSI inversion | None — read directly |
| CS inversion | Software ISR GPIO25→GPIO26→GPIO5; 52 µs setup window is sufficient |
| Inter-transaction gap | ~997 ms — re-arm timing is not the constraint |
| MISO = 0x00 (exps 4–5) | Fixed by code restructuring (immediate re-arm after wait()) |
| MISO = 0xF3 (exps 6–8) | ESP32 drives MISO but value is wrong. Persists across both library versions AND register fix. Root cause is in ESP-IDF 5.x SPI slave driver (core 3.3.7) |
| Library version irrelevant | v0.3.0 and v0.8.0 produce identical MISO corruption — both are thin wrappers around ESP-IDF |
| Register patch insufficient | One-time `ck_idle_edge`/`ck_i_edge` fix after `slave.begin()` did not fix MISO; may be overwritten by `spi_slave_transmit()` per-transaction |
| No DMA | `spi_slave_initialize()` hardcoded with `SPI_DMA_DISABLED` |
| Single-slot is correct | Trovis uses one CS assertion for entire exchange; T1/T2 split was wrong |

---

## Current Code State

File: `src/SPI/MySPI.cpp`

State machine:
```
ST_PASSIVE → ST_ANN → ST_CMD[0] → ST_CMD[1] → ... → ST_CMD[7] → ST_PASSIVE → ...
```

Critical coding rule: **never put Serial.print in time-critical sections** (between `wait()` and `queue()`). Always re-arm with `queue()` first, log after.

---

## CS Blip in Passive Mode

Observed on the wire: Trovis CS (active-HIGH) drops LOW for ~2 µs immediately after the first 4 bytes of a passive-mode transaction, coinciding with the 30 µs inter-byte clock gap. CS then goes HIGH again for the remaining 2 bytes.

```
Trovis CS:  ___HIGH(4B)__LOW(2µs)__HIGH(2B)___
ISR output: ___LOW(4B)___HIGH(2µs)__LOW(2B)___
ESP32 SPI:  [— txn A: 4B —][end]   [— txn B: 2B —][end]
```

The ISR amplifies the 2 µs glitch into a full CS deassert+reassert, so ESP32 sees two separate transactions: 4B (`30 00 00 00`) and 2B (`57 <stellsignal>`).

**Impact on state machine**: None significant.
- PSV: `wait()` returns after 4B (txn A). Re-arm with ANN begins immediately. txn B (2B) arrives ~2 µs later — too fast for `slave.queue()` to execute — so it hits an unarmed ESP32 and is silently dropped. Stellsignal value is lost, which is acceptable.
- ANN/CMD: Benvorth's trace confirms the blip does NOT occur when MISO[0]=0xF9. Trovis sees `0x06` and extends the transaction to 24–32B cleanly with no CS interruption.

The `too short` results in experiments 4–5 were caused by MISO=0x00 (Trovis never sees `0x06`), not by this blip.

---

### Experiment 9 — 2026-03-12
**Serial log**: `serial-log-2026-03-12-9.txt`
**PulseView CSV**: `trovis-comms-spi-esp32-2026-03-12-9-a.csv`

**Setup**: Same as experiment 8, but register fix moved from one-time `setupSPISlave()` to `post_setup_cb` callback in local ESP32SPISlave.h copy. This ensures the fix is applied before every transaction (not overwritten by `spi_slave_transmit()`).
```c
inline void spi_slave_setup_done(spi_slave_transaction_t* trans) {
    SPI3.pin.ck_idle_edge = 1;  // Clock idles HIGH (CPOL=1)
    SPI3.user.ck_i_edge = 1;   // Correct input edge (CPHA=1)
}
```

**Observations**:
- **MISO still F3 on wire** — unchanged from experiments 6–8
- PulseView: first ~4 transactions have `MISO = 00 00 00` (ESP32 not driving MISO initially), then `MISO = F3 CF F8 1E` stabilizes
- Serial log: MOSI alternates between `F9` (own MISO readback, early txns) and `0x18` (Trovis MOSI, later txns)
- MOSI = `F9` readback confirms `ck_i_edge = 1` IS taking effect (samples at MISO transition edge)
- Trovis still does not extend transactions — never sees `0x06`
- PSV transactions show `FF FF FF FF` on MISO (correct), but ANN/CMD show F3 corruption

**Conclusions**:
- **The `post_setup_cb` IS being called** — `ck_i_edge = 1` measurably affects MOSI sampling.
- **But `ck_idle_edge` and `ck_i_edge` do NOT control MISO output timing.** These registers control clock polarity and input sampling edge, not the output data edge.
- The register fix hypothesis from ESP-IDF bug #7698 was incomplete — those bugs describe first-bit-only corruption, while our problem is a consistent 1-bit shift across all bytes.

---

## ESP-IDF SPI Slave Register Analysis

**Source**: `spi_ll.h` in ESP-IDF 5.x (Arduino ESP32 core 3.3.7)

ESP-IDF `spi_ll_slave_set_mode()` for MODE3 sets:
```c
hw->pin.ck_idle_edge = 0;      // Slave inverts CPOL perception!
hw->user.ck_i_edge = 0;        // Controls input edge + MISO delay interaction
hw->ctrl2.miso_delay_mode = 1; // ← THIS controls MISO output timing
hw->ctrl2.miso_delay_num = 0;
hw->ctrl2.mosi_delay_mode = 0;
hw->ctrl2.mosi_delay_num = 0;
```

**Critical discovery**: The slave driver **intentionally inverts `ck_idle_edge`** relative to the master. From the ESP32 TRM, the slave perceives clock polarity inverted because it observes rather than drives the clock. So for MODE3 (CPOL=1), the master uses `ck_idle_edge=1` but the slave correctly uses `ck_idle_edge=0`.

**Our experiments 8–9 were setting the WRONG values!** By forcing `ck_idle_edge=1` and `ck_i_edge=1` (master MODE3 values), we were fighting the hardware design.

### MISO output timing registers

| Register | Controls | MODE3 default |
|---|---|---|
| `ck_idle_edge` | Clock idle polarity (inverted for slave) | 0 |
| `ck_i_edge` | Input (MOSI) sampling edge; also interacts with miso_delay | 0 |
| `ck_out_edge` | Not used in slave mode | — |
| **`miso_delay_mode`** | **MISO output delay** | **1** |
| `miso_delay_num` | Additional system clock delay on MISO | 0 |

Per the ESP32 TRM, `miso_delay_mode` with `ck_i_edge=0`:
- `0`: no delay
- `1`: delayed by one full SPI clock cycle (current default)
- `2`: delayed by half SPI clock cycle

The 1-bit MISO shift (F9→F3) is consistent with the MISO delay being wrong by half a clock cycle.

---

### Experiment 10 — 2026-03-12
**Setup**: Reverted `ck_idle_edge` and `ck_i_edge` to ESP-IDF defaults (0, 0). Instead, change `miso_delay_mode` in `post_setup_cb`:
```c
inline void spi_slave_setup_done(spi_slave_transaction_t* trans) {
    SPI3.ctrl2.miso_delay_mode = 2;  // Try 0, 1, 2, 3 — default is 1
}
```
Starting with `miso_delay_mode = 2` (half cycle delay instead of full cycle).

---

## Next Steps

- **Experiment 10**: Sweep `miso_delay_mode` (0, 1, 2, 3) to find correct MISO output timing
- **Alternative**: Try `SPI_MODE2` (CPOL=1, CPHA=0) in case ESP-IDF has modes swapped for slave
- **Fallback**: Downgrade Arduino ESP32 core from 3.3.7 to 2.x (ESP-IDF 4.4) to match Benvorth's proven environment
