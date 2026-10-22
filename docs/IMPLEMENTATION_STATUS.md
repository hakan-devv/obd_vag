# Implementation and validation status

## Implemented

- CAN startup discovery listens passively across 250/500 kbit/s and 11/29-bit
  candidates. It requires repeated frames with a matching ID and DLC and rejects
  a listen window when MCP2515 reports a message error. Active OBD verification
  remains gated by `CONFIG_OBD_ACTIVE_TX`; Normal mode uses one-shot TX.
- Each ECU response is dispatched through one ISO-TP transport shared by OBD
  Mode 01 and opt-in UDS, avoiding duplicate Flow Control. Dashboard values
  stay pinned to the first responding ECU for each PID.
- Mode 01 support bitmap pages are discovered through the continuation bit.
- The scalar numeric PID catalog and decoder formulas are documented in
  [PARAMETER_CATALOG.md](./PARAMETER_CATALOG.md). Only a PID both reported by an
  ECU and present in the decoder catalog is scheduled. Other advertised PIDs
  are logged as lacking a decoder; bitfield and packed multi-sensor PIDs are
  not represented as fabricated scalar values.
- PID scheduling chooses the oldest due supported PID, with configured
  priority as a tie-breaker. Per-PID request time, success time, error count,
  and latest error are available through the OBD client API.
- Mode 01 timeouts, negative responses, malformed payload lengths, and TX
  errors are recorded without permanently stopping polling. The dashboard
  immediately marks failed data rather than continuing to present it as live.
- The onboard ST7789 display uses three-card pages built from the discovered
  supported catalog. CST826 horizontal swipe handling changes one page per
  gesture; small and predominantly vertical movements are ignored.
- Numeric values use a seven-segment renderer with adaptive sizing, finite
  value checks, and inline units. LCD transfers wait for the DMA completion
  callback before drawing buffers can be reused. Missing, stale, or invalid
  values are not presented as live data. Value updates are queued to a
  dedicated UI task and redraw only the value area.
- ISO-TP, UDS, parameter manager, scheduler, and swipe protocol tests are
  available with `CONFIG_OBD_RUN_PROTOCOL_TESTS=y`.
- Serial screenshot/framebuffer streaming support and its capture tooling have
  been removed; the LCD is not captured or validated through a framebuffer
  mirror.

## Build and test evidence

- ESP-IDF v5.5.2 production firmware builds successfully for ESP32-S3. The
  app image is 307,712 bytes (`0x4B200`), with 71% free in the 1 MiB app
  partition.
- The protocol-test image builds successfully. Its app image is 239,632 bytes
  (`0x3A810`), with 77% free in the app partition. All 24 tests passed when
  flashed and run on the ESP32-S3 over COM13. The production image was flashed
  back afterward.
- `git diff --check` passes. No physical CAN, ECU simulator, or real-vehicle
  test was performed for this revision.

## Physical verification limits and remaining work

- No CAN or ECU-simulator runtime test was performed for this revision, and no
  real-vehicle test has been performed.
- LCD controller and CST826 initialization are implemented. LCD colors,
  asynchronous redraw timing, and physical swipe navigation remain unverified.
  Gesture tests passed in the protocol-test image, but physical swipe
  navigation is unverified.
- The standard defines additional bitfield, enum, packed-sensor, and other
  Mode 01 PIDs beyond the scalar catalog. These remain explicitly unsupported
  until their response semantics and UI representation are implemented.
- MCP2515 RX still reads status, frame registers, and data separately;
  READ RX BUFFER batching has not been implemented or benchmarked. Validate
  overflow behavior under representative bus traffic before changing this path.
- A before/after RAM baseline was not captured. The compiler map and runtime
  logs do not provide a like-for-like RAM comparison for the earlier image.
- No vehicle-specific UDS DID decoder is included. UDS `0x22` responses remain
  raw until supported by model/year/ECU-specific documentation and tests.
