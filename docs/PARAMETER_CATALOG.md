# Standard Mode 01 parameter catalog

Only standardized Mode 01 PIDs with implemented decoders are listed here.
Runtime requests are limited to PIDs reported as supported by a discovered ECU.
Priorities use lower numbers first; periods are requested refresh targets, not
guaranteed ECU update rates.

| PID | Parameter | Unit | Response bytes | Decoder | Target period | Priority |
| --- | --- | --- | ---: | --- | ---: | ---: |
| `04` | Calculated engine load | % | 1 | `100*A/255` | 500 ms | 2 |
| `05` | Engine coolant temperature | deg C | 1 | `A-40` | 1000 ms | 3 |
| `06` | Short-term fuel trim, Bank 1 | % | 1 | `(A-128)*100/128` | 1000 ms | 4 |
| `07` | Long-term fuel trim, Bank 1 | % | 1 | `(A-128)*100/128` | 1000 ms | 4 |
| `0B` | Intake manifold absolute pressure | kPa | 1 | `A` | 500 ms | 4 |
| `0C` | Engine speed | rpm | 2 | `(256*A+B)/4` | 100 ms | 0 |
| `0D` | Vehicle speed | km/h | 1 | `A` | 100 ms | 1 |
| `0E` | Ignition timing advance | deg | 1 | `A/2-64` | 500 ms | 5 |
| `0F` | Intake air temperature | deg C | 1 | `A-40` | 1000 ms | 5 |
| `10` | Mass air flow | g/s | 2 | `(256*A+B)/100` | 500 ms | 4 |
| `11` | Throttle position | % | 1 | `100*A/255` | 250 ms | 2 |
| `1F` | Run time since engine start | s | 2 | `256*A+B` | 5000 ms | 7 |
| `2F` | Fuel tank level input | % | 1 | `100*A/255` | 5000 ms | 7 |
| `42` | Control module voltage | V | 2 | `(256*A+B)/1000` | 2000 ms | 6 |
| `46` | Ambient air temperature | deg C | 1 | `A-40` | 5000 ms | 7 |
| `49` | Accelerator pedal position D | % | 1 | `100*A/255` | 500 ms | 5 |
| `4A` | Accelerator pedal position E | % | 1 | `100*A/255` | 500 ms | 5 |
| `5C` | Engine oil temperature | deg C | 1 | `A-40` | 5000 ms | 7 |
| `62` | Actual engine percent torque | % | 1 | `A-125` | 1000 ms | 6 |
| `63` | Engine reference torque | Nm | 2 | `256*A+B` | 5000 ms | 7 |

`A` and `B` are the first and second data bytes after the positive response
service and PID. `AB` is big-endian. The response length and physical range are
validated before a value is marked valid.

Not currently decoded: oxygen-sensor families, fuel-rail pressure, catalyst
temperature, and additional standardized PIDs. They must not be displayed as
implemented until their response shapes and formulas are added and tested.

## OEM DIDs

No vehicle-specific DID is included. UDS `0x22` only specifies the read service;
it does not provide a universal VAG/BMW DID catalogue or scaling. Keep OEM
profiles `UNKNOWN/TODO` until documentation is tied to a specific model,
model year, and ECU family and the payload decoder is independently verified.

The firmware exposes opt-in physical-addressing calls for `0x10 03`,
`0x3E 00`, and `0x22 <DID>` through the OBD client API. No UDS service is
requested automatically, and positive `0x62` payloads are retained as raw
bytes rather than assigned an unverified meaning.
