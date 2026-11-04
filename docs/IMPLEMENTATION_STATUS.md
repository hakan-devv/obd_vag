# Implementation and validation status

## Final status

- The production firmware for the ESP32-S3 builds successfully under ESP-IDF
  v5.5.2.
- The protocol-test image builds successfully and all 24 protocol tests pass on
  the target hardware.
- Real CAN validation was performed on three vehicles and the resulting logs
  are stored in [../test_logs](../test_logs): BMW 420 Gran Coupe 2014, Fiat
  Grande Punto IAW5SF3, and Skoda Octavia.
- The firmware successfully handled live traffic on both 11-bit and 29-bit
  response families, accepted real `41 00` support-bitmap responses, and
  decoded supported PID values into numeric dashboard outputs. Unsupported PIDs
  are intentionally omitted rather than fabricated.
- The ISO-TP path, CAN discovery, and parameter decoding were exercised against
  real vehicle traffic and not only against simulator or loopback-only inputs.
  - The OEM UDS-DID layer is implemented as a deny-by-default architecture. No DID
    is requested automatically, and only an explicit, profile-matched and
    source-backed descriptor can be scheduled. The current catalog contains only
    placeholder/experimental entries and every production-facing route is gated by
    validation and explicit profile selection.

## Build and live validation evidence

- ESP-IDF v5.5.2 production firmware builds successfully for ESP32-S3. The
  app image is 307,712 bytes (`0x4B200`), with 71% free in the 1 MiB app
  partition.
- The protocol-test image builds successfully. Its app image is 239,632 bytes
  (`0x3A810`), with 77% free in the app partition. All 24 tests passed when
  flashed and run on the ESP32-S3 over COM13.
- Live vehicle logs confirm real OBD traffic and valid responses:
  - BMW 420 Gran Coupe 2014: engine RPM values around 780-782 rpm, throttle
    about 84.71%, and calculated engine load around 23-25% from live CAN data.
  - Fiat Grande Punto IAW5SF3: 29-bit extended response family `0x18DAF110`
    with valid live OBD requests and decoded throttle values near 85.88%.
  - Skoda Octavia: 11-bit `0x7E8/0x7E9` response family with valid decoding,
    including control module voltage around 12.36 V.
- `git diff --check` passes.

## Profile-based supported-DID polling (automatic VEH layer)

- The VEH identity layer starts automatically once the CAN link is locked and the ECU
  channel exists (alongside the PID bitmap, no bitmap restart). `v` re-runs it manually
  and also restarts the bitmap. VEH reads the core list plus every `vag-uds-identity`
  catalog DID (ISO 14229 F180-F19F etc.), so identification DIDs never enter the
  engine-data pool. identity reply is logged as `[VEH_DECODE]` (decoded text plus `RAW=[...]`
  bytes; VIN additionally split into WMI/brand/VDS/year/plant/serial).
- After the sequence `[PROFILE]` lists the brand (from the VIN WMI, `did_profile.c`)
  and the candidate catalog profiles (BMW only when F197 reports N47/DDE7).
- Round 1 asks every decodable DID of those profiles once. NRC 0x31 (and other
  hard rejections) excludes the DID permanently; 0x78/0x22/0x21 mark it
  supported-but-not-ready. A DID with no reply at all is excluded after 2 consecutive
  timeouts only while untested; once supported it is never dropped, just retried after 5 s. Surviving DIDs are polled (500 ms, 1 s for temperatures
  and levels, 2 s when not ready), alternating with PID requests. Results are logged
  as `[DID_POLL]` with decoded value and raw bytes and drawn as DID cards on the LCD
  (`onx_display_update_did`), with "NOT READY" shown for pending DIDs.
- `d` and `v` scan behaviour is unchanged and unfiltered; the polling pauses while
  a `d` scan runs. The OBD PID, ISO-TP and UDS client layers were not modified.
- Same-DID/different-profile conflicts are resolved by brand-first ordering; the
  values are family candidates, not an exact ECU match, until vehicle verified.

- Profile logging: `[PROFILE]` prints `ENGINE_FAMILY` (F197 engine code lookup in
  `did_profile_engine_family`, codes unverified) and the pool size instead of the full
  candidate list. The family is a label only and does not narrow the pool yet.
- `[DID_OUT_OF_PROFILE]` is logged when a manual `d` scan gets a positive reply for a
  DID that is not in the active poll set, to find DIDs that must join the profile.
- Community profile `vag-engine-community` (38 cross-model DIDs, see
  `UDS_DID_CATALOG.md`) is last in the VAG pool.
- Pending: BMW gasoline DIDs (N55 source is ENET/HSFZ; the OBD route 7E0 vs 6F1/612 is
  decided after a real BMW `d` log), NVS persistence of discovery, manual model selection.

## Scope and limits

- The implementation is validated for the supported CAN combinations used by the
  tested vehicles: 250/500 kbit/s and 11-/29-bit identifiers.
- This is not a universal detector for every CAN transport, every ECU family,
  or every OEM-specific parameter set.
- Custom OEM DIDs remain opt-in and raw unless a model-specific decoder is
  explicitly implemented and documented. Unknown or unsupported standard PIDs
  are logged and excluded; they are never inferred or invented.
- The supported-PID catalog is intentionally limited to scalar values with a
  verified decoding path. Bitfield, packed multi-sensor, and other non-scalar
  identifiers remain explicit unsupported entries unless they are implemented
  and validated with a real ECU response.
