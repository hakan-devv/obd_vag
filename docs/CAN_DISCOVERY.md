# CAN and OBD discovery

## Reference implementation review

The reference reader's `Basic_Code/CAN.ino` iterates four candidates:
11-bit/250 kbit/s, 29-bit/250 kbit/s, 11-bit/500 kbit/s, and 29-bit/500
kbit/s. For each it starts the ESP32 TWAI in `TWAI_MODE_NORMAL`, sends Mode 01
PID `00`, and treats `readData() > 0` as connection success. The library
`src/OBD2_CanBus.cpp` uses the same selection and check; `readData()` accepts
only a short hard-coded response-ID list and does not validate a positive
`41 00` payload before setting connection status. Normal mode also enables
transmission/ACK before the candidate has been established.

Sources:

- [OBD2_CAN_Bus_Reader Basic_Code/CAN.ino](https://github.com/muki01/OBD2_CAN_Bus_Reader/blob/main/Basic_Code/CAN.ino)
- [OBD2_CAN_Bus_Library src/OBD2_CanBus.cpp](https://github.com/muki01/OBD2_CAN_Bus_Library/blob/main/src/OBD2_CanBus.cpp)

This project retains the four tested CAN combinations but uses a passive
Listen-Only phase before any diagnostic request. A combination is eligible for
active verification only after at least three valid, non-RTR frames repeat the
same identifier, DLC, and identifier width during its listen window, in either
identifier width. Passive traffic proves the presence of CAN activity at that
bitrate and format; some gateways show only 29-bit gateway traffic on the OBD
bus while OBD-II responses still use the standard 11-bit response family. The
candidate's own width (11-bit `0x7DF` or 29-bit `0x18DB33F1`) is decided by
the active probe. A latched MCP2515 message-error flag rejects the candidate.
It is accepted for OBD only when an ISO-TP response from the corresponding
response-ID family matches the outstanding Mode 01 PID `00` request and
contains the exact positive `41 00` support bitmap.

## Runtime behavior and safety

- The initial candidate is the currently configured CAN combination, followed by
  the other supported combinations. MCP2515 timing and ID format are changed
  only in Configuration mode, then verified by the existing driver.
- The Listen-Only phase accepts both standard and extended frames; it does not
  ACK or transmit. `can_receive()` reports only frames the MCP2515 accepted as
  valid, and the discovery state machine additionally checks RTR, repeated
  ID/DLC, and MCP2515 message-error status.
- With `CONFIG_OBD_ACTIVE_TX=n`, valid traffic can identify the observed
  bitrate/ID-width candidate, but the firmware does not claim that OBD service
  is available and sends no request.
- With `CONFIG_OBD_ACTIVE_TX=y`, a candidate with passive traffic evidence is
  switched to response-ID filters and Normal mode before one Mode 01 `00`
  verification request is sent. Normal mode verifies the MCP2515 one-shot
  setting; failed attempts are not automatically retransmitted. One Mode 01
  `00` timeout is retried once before the candidate is rejected. There are no
  diagnostic requests during candidate scanning.
- A negative response, malformed bitmap, or timeout rejects that candidate and
  resumes Listen-Only scanning. If all candidates fail or no traffic is
  observed, discovery reports an inconclusive result, leaves the MCP2515 in
  Configuration mode, and retries after five seconds; it does not fall back to
  a guessed configuration.
- A valid bitmap locks the CAN configuration and hands off to the supported-PID
  bitmap discovery and polling scheduler.

## Vehicle validation status

The discovery logic and OBD verification path were exercised on three real
vehicles, and the logs in [../test_logs](../test_logs) contain the corresponding
raw traffic and successful live responses. The results include valid 11-bit and
29-bit OBD response handling as well as successful `41 00` bitmap verification
for the tested ECUs.

## Scope and limits

Only 250/500 kbit/s CAN with 11-/29-bit identifiers are searched. This is not
an automatic detector for every CAN bitrate, SAE J1850, ISO 9141/K-line, or
other diagnostic transports. Passive traffic is evidence for a CAN candidate,
not proof of OBD-II compatibility; only the active, explicitly enabled
`41 00` exchange verifies an OBD Mode 01 endpoint. Error counters are not used
as bitrate evidence because MCP2515 counters are not meaningful in Listen-Only
mode. The real-vehicle validation is limited to the vehicles in the project log
set and is not a blanket claim for every ECU or every bus.
