# Vehicle and bench validation record

Use only a suitable diagnostic connector and a confirmed compatible interface.
Start in Listen-Only. Enable active OBD requests only after confirming the
selected bitrate, identifier format, wiring, and test setup. Do not connect
bench hardware directly to a vehicle unless the electrical interface and power
protection are appropriate.

## Setup

- Date/time:
- Operator:
- Vehicle make/model/year/market:
- Engine and transmission:
- ECU/module identity and software version:
- Diagnostic connector/interface and transceiver:
- MCP2515 oscillator and module:
- CAN bitrate and identifier format:
- Bus termination and measured CANH-CANL resistance:
- Ignition/engine state:
- Firmware build ID/configuration:

## Test record

| Test | Expected result | Observed result / log reference | Pass/Fail |
| --- | --- | --- | --- |
| MCP2515 reset/register readback | Read/write checks succeed | | |
| MCP2515 internal loopback | Test frame matches; no physical CAN traffic | | |
| Passive receive | Valid bus frames or documented idle bus | | |
| Supported PID discovery | Bitmap response cached per ECU | | |
| Mode 01 RPM (`01 0C`) | `41 0C A B`; RPM `(256*A+B)/4` | | |
| Mode 01 coolant (`01 05`) | `41 05 A`; temperature `A-40` deg C | | |
| Other supported standard PIDs | Formula, range, unit, and response length verified | | |
| ISO-TP multi-frame | Correct FC, sequence, and completion behavior | | |
| Timeout / negative response | Explicit error state; no success-shaped value | | |
| UDS `0x22` read | Only a documented DID; raw payload retained | | |

## Results and limitations

- Frames captured:
- ECU response IDs:
- CAN controller error counters and overflow flags:
- Unsupported or timed-out requests:
- DIDs verified and source/documentation:
- Physical CAN tests completed:
- Tests not run:
- Follow-up actions:

Do not report physical vehicle compatibility from a firmware build, simulator
test, or MCP2515 loopback test alone.
