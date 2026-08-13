# e220m30s transport layer review findings

Review date: 2026-08-13. Reviewed `e220m30s.cpp`, `include/e220m30s.h`, and the caller in `main/main.c`. All RadioLib assumptions verified against the pinned commit `10e7925` in `managed_components/radiolib`.

## Verification of the critical mechanics (checked against pinned RadioLib)

These are the things that *would* be bugs if RadioLib behaved differently — confirmed they are not:

- **IRQ flag clearing**: `stageMode(RX)` calls `clearIrqStatus()` before arming (`SX126x.cpp:973`), `readData()` clears flags after reading, and `finishTransmit()`/`finishReceive()` clear flags. So the re-arm loop in `listen()` cannot double-deliver a packet from stale `RX_DONE`, and a stale `TX_DONE` cannot produce a false success in `transmit_data()`. The "notification is not proof, latched flags are" pattern is correctly implemented.
- **RX timeout units**: `ms * 64` = units of 15.625 µs is correct, and the guard `elapsed < timeout_ticks` guarantees `rx_timeout_raw >= 64` — never 0, which on SX126x would mean "receive forever". 1200 ms -> 76800, well under the 24-bit limit.
- **Airtime math**: (255+10)·8·1000/2400 ≈ 883 ms < `listen_window_ms` (1200 ms) — a max-size packet fits the window as the comment claims.
- **Ping-pong timing**: ground responds immediately after receiving, so even a max-size response (~880 ms) lands inside the rocket's 1200 ms post-TX listen window. The roles in `init_narrowband(commandQueue, sensorDataQueue)` match the header docs for both modes.
- **Memory**: every malloc/free path is balanced (pack frees after copy including truncation, unpack frees on queue-full, `handle_receive` frees on all exits), ownership is documented in the header, and `xQueuePeek`+`xQueueReceive` is safe with the single-consumer invariant. No exceptions, all allocations checked.

## Findings

**1. Over-long messages are transmitted truncated (low–medium, design).**
`pack_messages` (`e220m30s.cpp:238-242`) sends the first 254 bytes of an over-long message as a valid frame. The receiver has no way to distinguish a truncated frame from a complete one, so a corrupt sensor payload would be delivered as good data. Dropping over-long messages (or stealing a flag bit from the length byte) is safer. Only triggers if a producer ever enqueues >254 bytes — today nothing does.

**2. No ACK/retry/sequencing, no addressing (low, design — confirm this is a conscious decision).**
Commands can be silently lost (CRC drop, collision, or RX queue-full drop — queues are only 10 deep), and telemetry gaps are undetectable without sequence numbers. Also, any transmitter with matching 434 MHz / 2.4 kbps / 11.7 kHz BW and default sync word can inject "commands". Reasonable for a first flight link; if command delivery must be reliable, an app-level ACK/sequence field is needed. `setSyncWord()` works as a poor-man's network ID.

**3. Ground response can be clipped by the rocket's listen window (low, edge case).**
If the ground receives the rocket's packet late in its own window and answers with a large command batch (~880 ms airtime), the tail of the response can overrun the rocket's remaining listen window and be aborted by the hardware RX timeout. With genuinely short commands there is ample margin; only relevant if command payloads grow.

**4. Duty cycle (regulatory note, not code).**
With a busy sensor queue the rocket transmits ~40% duty cycle at 22 dBm on 434 MHz, above the 10% SRD limit for that band. Likely irrelevant for a short flight, but it's a trade-off worth having made deliberately.

## Style / consistency nits

- `transmit_data(std::span<uint8_t>)` never writes through the span — should be `std::span<const uint8_t>` (`e220m30s.cpp:300`).
- `unpack_messages` treats a `0` length byte as "skip, reserved for future use" (`e220m30s.cpp:269`) while `pack_messages` never emits zero-length frames (it drops them, `e220m30s.cpp:231`). So on RX a 0 byte means corruption, and optimistically continuing to parse the rest is inconsistent with that — treating it as end-of-packet would match the TX side better. Harmless in practice due to CRC.
- Header doc for `init_narrowband` documents both queue params with the identical sentence "pointer to FreeRTOS queue for narrowband data" — copy-paste; the direction/ownership distinction described above it is the useful part.
- `typedef struct message_t { ... } message_t;` — ESP-IDF convention is an anonymous struct in the typedef; cosmetic only.
- The `RadioType` template parameter is used only for trampoline dispatch (the base never calls derived methods). It works and avoids vtables; a one-line comment saying so would prevent it looking like unfinished CRTP.
- 4096-byte task stack with a 255-byte stack buffer plus `ESP_LOGx` and RadioLib call depth is plausible but not generous — worth one `uxTaskGetStackHighWaterMark` check during bring-up.

## Verdict

The transport design (single rxtx task owning the radio, ISR -> notification -> verify latched IRQ flags, whole-message `[len][payload]` framing, hardware single-shot RX with timeouts) is sound and appropriate for rocket telemetry + short uplink commands. No logic errors, no memory leaks, no unhandled-failure paths. The only finding to act on before flight is #1 (drop rather than truncate), with #2 as an explicit design sign-off.
