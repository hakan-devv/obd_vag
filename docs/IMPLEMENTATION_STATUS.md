# Implementation and validation status

## Implemented

- MCP2515 CAN setup can be configured for 250/500 kbit/s and standard/extended
  CAN identifiers. Active OBD requests remain an explicit Kconfig choice.
- Supported Mode 01 PID bitmaps are cached per responding ECU. The request
  scheduler discovers bitmap pages and polls supported catalog entries by
  priority and target update period.
- ISO-TP supports Single, First, Consecutive, and Flow Control frames, including
  receive reassembly, sequence checks, FC handling, STmin/Block Size, and timeouts.
- The Parameter Manager validates response lengths and ranges, records support,
  values, validity, timeouts, and stale data for the documented PID catalog.
- The opt-in UDS client supports physical-addressing requests for
  `0x10 03`, `0x3E 00`, and `0x22 <DID>`, with positive/negative responses,
  `0x78` pending, timeouts, and session tracking. It does not send requests at
  boot, implement write services, or assign meanings to unknown DIDs.
- Focused protocol tests cover ISO-TP framing, Flow Control direction,
  reassembly, sequence errors, timeouts, PID bitmap mapping, decoder scaling,
  invalid data, stale transitions, UDS DID responses, and pending timeouts.

## Build and test evidence

- ESP-IDF v5.5.2 default firmware configuration: `idf.py build` completed
  successfully for ESP32-S3.
- `CONFIG_OBD_RUN_PROTOCOL_TESTS=y` test-image configuration: firmware compiled
  successfully. The test image was not flashed, so its assertions have not
  been executed on-device.
- No new-code physical CAN or vehicle test was performed. A successful build or
  MCP2515 internal loopback is not evidence of CAN bus communication.
- The current saved configuration keeps the protocol test option disabled and
  retains the project's active-TX setting. Review CAN wiring, bitrate, ID width,
  and electrical protection before enabling transmission in a vehicle setup.

## Remaining verification

- Exercise the optional test image on the S3 and capture its pass/fail log.
- Recheck physical CAN request/response on the bench after flashing the default
  image; do not treat previous firmware behavior as validation of this change.
- Confirm MCP2515 timing against the actual 8 MHz modules and the licensed
  datasheet; the public datasheet PDF was not retrievable during research.
- Obtain a model/year/ECU-specific authoritative source before adding any
  VAG-specific DID profile. Until then, OEM values remain `UNKNOWN/TODO`.
