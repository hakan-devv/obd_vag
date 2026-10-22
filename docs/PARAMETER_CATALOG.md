# Standard Mode 01 parameter catalog

The firmware requests a PID only after an ECU reports it in its Mode 01
support bitmap and a decoder is present in the catalog below. Bitmap support
does not guarantee that a value is available in every ECU operating state.
Refresh periods are scheduler targets; the oldest-due scheduler may defer a
PID when the CAN response time exceeds the target.

| PID | Parameter | Unit | Bytes | Conversion | Target |
| --- | --- | --- | ---: | --- | ---: |
| `04` | Calculated engine load | % | 1 | `100*A/255` | 500 ms |
| `05` | Engine coolant temperature | deg C | 1 | `A-40` | 1000 ms |
| `06` | Short-term fuel trim, Bank 1 | % | 1 | `(A-128)*100/128` | 1000 ms |
| `07` | Long-term fuel trim, Bank 1 | % | 1 | `(A-128)*100/128` | 1000 ms |
| `08` | Short-term fuel trim, Bank 2 | % | 1 | `(A-128)*100/128` | 1000 ms |
| `09` | Long-term fuel trim, Bank 2 | % | 1 | `(A-128)*100/128` | 1000 ms |
| `0A` | Fuel pressure | kPa | 1 | `3*A` | 1000 ms |
| `0B` | Intake manifold absolute pressure | kPa | 1 | `A` | 500 ms |
| `0C` | Engine speed | rpm | 2 | `(256*A+B)/4` | 100 ms |
| `0D` | Vehicle speed | km/h | 1 | `A` | 100 ms |
| `0E` | Ignition timing advance | deg | 1 | `A/2-64` | 500 ms |
| `0F` | Intake air temperature | deg C | 1 | `A-40` | 1000 ms |
| `10` | Mass air flow | g/s | 2 | `(256*A+B)/100` | 500 ms |
| `11` | Throttle position | % | 1 | `100*A/255` | 250 ms |
| `1F` | Run time since engine start | s | 2 | `256*A+B` | 5000 ms |
| `21` | Distance with MIL on | km | 2 | `256*A+B` | 5000 ms |
| `22` | Fuel rail pressure, relative to manifold vacuum | kPa | 2 | `0.079*(256*A+B)` | 1000 ms |
| `23` | Fuel rail gauge pressure | kPa | 2 | `10*(256*A+B)` | 1000 ms |
| `2C` | Commanded EGR | % | 1 | `100*A/255` | 1000 ms |
| `2D` | EGR error | % | 1 | `(A-128)*100/128` | 1000 ms |
| `2E` | Evaporative purge | % | 1 | `100*A/255` | 1000 ms |
| `2F` | Fuel tank level input | % | 1 | `100*A/255` | 5000 ms |
| `31` | Distance since DTC clear | km | 2 | `256*A+B` | 5000 ms |
| `33` | Barometric pressure | kPa | 1 | `A` | 5000 ms |
| `3C`-`3F` | Catalyst temperature (B1S1, B2S1, B1S2, B2S2) | deg C | 2 | `(256*A+B)/10-40` | 5000 ms |
| `42` | Control module voltage | V | 2 | `(256*A+B)/1000` | 2000 ms |
| `43` | Absolute engine load | % | 2 | `100*(256*A+B)/255` | 500 ms |
| `44` | Commanded equivalence ratio | ratio | 2 | `2*(256*A+B)/65536` | 1000 ms |
| `45` | Relative throttle position | % | 1 | `100*A/255` | 500 ms |
| `46` | Ambient air temperature | deg C | 1 | `A-40` | 5000 ms |
| `47`-`48` | Absolute throttle position B-C | % | 1 | `100*A/255` | 500 ms |
| `49`-`4B` | Accelerator pedal position D-F | % | 1 | `100*A/255` | 500 ms |
| `4C` | Commanded throttle actuator | % | 1 | `100*A/255` | 500 ms |
| `4D` | Time with MIL on | min | 2 | `256*A+B` | 5000 ms |
| `4E` | Time since DTC clear | min | 2 | `256*A+B` | 5000 ms |
| `52` | Ethanol fuel percentage | % | 1 | `100*A/255` | 5000 ms |
| `53` | Absolute evaporative pressure | kPa | 2 | `(256*A+B)/200` | 5000 ms |
| `5A` | Relative accelerator pedal position | % | 1 | `100*A/255` | 500 ms |
| `5C` | Engine oil temperature | deg C | 1 | `A-40` | 5000 ms |
| `5D` | Fuel injection timing | deg | 2 | `(256*A+B)/128-210` | 1000 ms |
| `5E` | Engine fuel rate | L/h | 2 | `0.05*(256*A+B)` | 1000 ms |
| `62` | Actual engine percent torque | % | 1 | `A-125` | 1000 ms |
| `63` | Engine reference torque | Nm | 2 | `256*A+B` | 5000 ms |
| `74` | Turbocharger RPM | rpm | 2 | `(256*A+B)/4` | 1000 ms |
| `A1` | Driver demand engine torque | % | 1 | `A-125` | 1000 ms |
| `A2` | Actual engine torque | % | 1 | `A-125` | 1000 ms |
| `A3` | Engine reference torque | Nm | 2 | `256*A+B` | 5000 ms |
| `A6` | Odometer | km | 4 | `(A*2^24+B*2^16+C*256+D)/10` | 5000 ms |

`A` through `D` are response data bytes after the positive response service
and PID; multi-byte values are big-endian. Exact response length and decoded
physical range are checked before a value is marked valid. The catalog is a
scalar numeric UI: standardized bitfield, enumerated, packed multi-sensor,
and other non-scalar PIDs without an appropriate decoder are intentionally
not queried or shown. When an ECU bitmap reports one of those PIDs, firmware
logs that its decoder is not implemented.

The supported-PID bitmap pages use their standard continuation bit to discover
the next page. Only the catalog entries supported by at least one discovered
ECU are shown and scheduled. For example, the `ecu_sim` ECU advertises and
answers `04`, `05`, `0B`, `0C`, `0D`, `0F`, and `11`.

## OEM DIDs

No vehicle-specific DID is included. UDS `0x22` only specifies the read service;
it does not provide a universal VAG/BMW DID catalogue or scaling. Keep OEM
profiles `UNKNOWN/TODO` until documentation is tied to a specific model,
model year, and ECU family and the payload decoder is independently verified.

The firmware exposes opt-in physical-addressing calls for `0x10 03`,
`0x3E 00`, and `0x22 <DID>` through the OBD client API. No UDS service is
requested automatically, and positive `0x62` payloads are retained as raw
bytes rather than assigned an unverified meaning.
