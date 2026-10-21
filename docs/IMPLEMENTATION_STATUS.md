# Implementation and validation status

## Implemented

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
  values are not presented as live data.
- ISO-TP, UDS, parameter manager, scheduler, and swipe protocol tests are
  available with `CONFIG_OBD_RUN_PROTOCOL_TESTS=y`.

## Build and test evidence

- ESP-IDF v5.5.2 production firmware builds successfully for ESP32-S3. The
  current app image is 305,952 bytes (`0x4AB20`), with 71% free in the 1 MiB
  app partition.
- The opt-in serial screenshot firmware also builds for ESP32-S3. Its current
  app image is 305,968 bytes (`0x4AB30`) and allocates an additional 150 KiB
  framebuffer at runtime. A screenshot was captured, but its framebuffer
  colors did not match the photographed LCD; it is not considered visual proof.
- The protocol-test image also builds successfully. Its app image is 251,232
  bytes (`0x3D560`), with 76% free in the app partition.
- The protocol tests have **not been executed for this revision**. The
  protocol-test image has not been flashed or run on hardware.
- The normal production firmware was restored to the LCD board after the
  temporary screenshot capture. No further physical verification was made
  after the latest display-source changes.

## Physical verification limits and remaining work

- No CAN or ECU-simulator runtime test was performed for this revision, and no
  real-vehicle test has been performed.
- LCD controller and CST826 initialization are implemented. The last screenshot
  did not match the physical LCD colors, so visual correctness remains
  unverified. Physical swipe navigation is also unverified for this revision.
  Gesture tests are present in the protocol-test image but were not run.
- The standard defines additional bitfield, enum, packed-sensor, and other
  Mode 01 PIDs beyond the scalar catalog. These remain explicitly unsupported
  until their response semantics and UI representation are implemented.
- A before/after RAM baseline was not captured. The compiler map and runtime
  logs do not provide a like-for-like RAM comparison for the earlier image.
- No vehicle-specific UDS DID decoder is included. UDS `0x22` responses remain
  raw until supported by model/year/ECU-specific documentation and tests.
