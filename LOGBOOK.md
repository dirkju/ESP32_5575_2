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

## Key Findings

| Finding | Detail |
|---|---|
| SPI mode | MODE3 (CPOL=1, CPHA=1) confirmed by wire observation |
| MISO inversion | Physical MISO = `0xFF - logical_byte` |
| MOSI inversion | None — read directly |
| CS inversion | Software ISR GPIO25→GPIO26→GPIO5; 52 µs setup window is sufficient |
| Inter-transaction gap | ~997 ms — re-arm timing is not the constraint |
| MISO = 0x00 (root cause) | Under investigation. Library confirmed CPU FIFO mode (no DMA). SPI hardware not driving MISO before first clock — possibly interrupt latency to load TX FIFO after CS↓ |
| No DMA | `spi_slave_initialize()` hardcoded with `SPI_DMA_DISABLED` in ESP32SPISlave v0.8.0 |
| Single-slot is correct | Trovis uses one CS assertion for entire exchange; T1/T2 split was wrong |
| Benvorth API difference | Old: `slave.wait(rx, tx, size)` + `slave.pop()`. New: `slave.queue(tx, rx, size)` + `slave.wait()` returns results |

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

## Next Steps

- **Experiment 6**: Upload current code, capture serial log + PulseView CSV
  - Success criteria: MISO shows `F9 FF FF ...` (PSV/ANN) and `F9 F3 FF 05 ...` (CMD) on wire; `rxLen = 32` for ANN and CMD transactions
  - If MISO still 0x00: investigate whether GPIO19 (MISO) is correctly configured as SPI output; check if `slave.begin(VSPI, SCK, MISO, MOSI, SS)` pin mapping is correct for the board in use
