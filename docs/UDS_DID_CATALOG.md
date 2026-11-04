# UDS OEM DID Catalog and Field-Safety Rules

## Scope

This catalog is a **manual, read-only UDS `0x22 ReadDataByIdentifier` probe list**. It is not a universal VAG DID map. Every model-specific record is attached to a model/profile label and retains its source URL, payload length, byte offset, signedness, byte order, scale, offset, units, and evidence level.

No catalog record is automatically polled. `auto_query_allowed` is `false` for every record. The current implementation sends only `22 DID_H DID_L` for catalog probes. Manual `v` identity collection reads exactly fifteen VAG identity DIDs: `F190` first (VIN), then `F187`, `F189`, `F191`, `F1A3`, `F197`, `F1AD`, `F19E`, `F1A2`, `F1AA`, `F17C`, `F1F4`, `F1A5`, `F804`, and `F806`, and only after the active OBD link has been verified. It does not sweep the optional/reserved `F180–F19F` range. The unsafe early Mode 09 transmit path has been removed; identity reads are physical UDS `0x22` transactions. The current engine-role manual queue contains 186 unique identifiers (not one request per descriptor): descriptors that share one DID across per-profile meanings are transmitted once and interpreted against the candidate definitions. It does not issue `0x10` DiagnosticSessionControl, `0x27` SecurityAccess, `0x2E` WriteDataByIdentifier, `0x2F` InputOutputControl, `0x31` RoutineControl, ECU reset, download, or actuator requests. The public `obd_client` wrappers for extended-session and TesterPresent are deliberately unsupported in the field scanner. The additional VAG identity DIDs `F1AA`, `F17C`, `F1F4`, `F1A5`, `F804`, and `F806` are sourced from VW_Flash identification records.

## CAN route and safety invariant

The DID layer **inherits** the CAN link already selected and verified by the existing discovery/Mode 01 path. It does not call discovery, reconfigure the MCP2515, change bitrate, switch 11/29-bit ID format, or change CAN operating mode.

Before a DID request is permitted, all of the following must be true:

1. The existing discovery path has moved the controller from Listen-Only to Normal only after observing live frames and a valid `41 00` OBD supported-PID response.
2. `s_can_link_verified` is true.
3. The UDS channel was derived from an observed physical ECU response ID on that same selected bus configuration.
4. The current configured ID width matches the route.
5. The physical TX/RX pair is allow-listed.
6. The ECU has replied to `01 00` on the verified link.
7. No other OBD/UDS transaction is outstanding.

Address pairing used by the current code:

- 11-bit: response `7E8` maps to physical request `7E0`; `7E9` maps to `7E1`; subsequent OBD response IDs use the same offset.
- 29-bit: for an observed physical reply such as `18DAF110`, the paired request is derived as `18DA10F1`. The actual observed ECU address is retained; the code does not force a universal ECU target address.

If no valid `41 00` verification has occurred, a UDS request is rejected. Arbitrary ISO-TP First Frames are not passed into the transport before link verification, because ISO-TP would otherwise transmit a Flow Control frame. The `0x22` layer never tries another bitrate or another identifier width. The live MCP2515 bitrate and identifier-width configuration is re-read and compared with the discovery lock before VIN/DID requests; a mismatch aborts the diagnostic operation rather than reconfiguring the controller.

The Mode 01 PID scheduler, PID catalog/decoder, `pid_scheduler.c`, CAN discovery candidate ordering, MCP2515 bit-timing selection, and HMI/UART pin setup are outside this DID patch. The existing PID scheduler continues to decide when Mode 01 frames are sent; UDS only occupies the single outstanding diagnostic transaction slot.

## Catalog size

Current catalog table: **284 descriptors**:

- Engine role: 263 descriptors, including 40 identity-only entries and 223 model-specific entries.
- Transmission role: 17 descriptors, 16 unique DID values.
- Haldex research-only role: 4 descriptors; manual querying is disabled.
- 240 descriptors are marked for explicit manual probing; automatic querying is disabled for all 284 descriptors.
- Unique manual request identifiers: 186 engine-role DIDs and 16 transmission-role DIDs (202 across both roles).

The engine scanner queue capacity is 256 unique DID values, accommodating the current 186 unique manually-queryable engine-role DIDs. The transmission-role queue currently has 16 unique manually-queryable DIDs. The role-filtered helper buffer is also 256 entries. Duplicate DID descriptors (for example, one DID containing several per-cylinder fields or a DID reused by different vehicle profiles) are queried once per ECU role and decoded against all applicable candidate descriptors. When profile identity is not exact, output is a set of labelled candidate interpretations, never a single asserted live-data value. A full automatic profile match remains disabled until a validated fingerprint mapping exists.

## Evidence levels

- `SOURCE_BACKED`: a DID, payload layout, and formula are present in the cited vehicle parameter source. This does **not** mean it has been tested on the user's car. Entries whose source labels a year as “In testing” remain candidates.
- `SAMPLE_CONFIRMED`: a concrete sample response/formula is published for one or more model years. This is still not the same as vehicle verification on the user's ECU.
- `VEHICLE_VERIFIED`: reserved for evidence captured on the actual target ECU and confirmed by its fingerprint. No current catalog row is marked this level.
- `EXPERIMENTAL`: used for custom Haldex research records, which are not active in the normal OBD/UDS scanner.

A missing unit is kept as `raw`; no pressure/temperature/torque unit is guessed from a multiplier alone. Lambda remains a lambda ratio and is not relabelled as AFR without a known fuel stoichiometric ratio.

## Target parameter mapping

| Desired measurement | Current model-specific candidates | Important limit |
|---|---|---|
| Knock retard | VW Up `10E6`, `10E5`; Touareg cylinders 3/5 `13F4`, `13F6` (`u8 × 3/8 − 48°`) | Several vehicle-year entries are marked testing. Do not confuse with knock-sensor voltage/raw-signal DIDs such as Up `1126`/`1127` or Touareg `13FC`/`13FA`/`13F9`/`13FD`. |
| Transmission oil temperature | Jetta TCU `704F` | This is **lifetime maximum ATF temperature**, formula `u8 - 50 °C`, not live ATF temperature. Clutch and auxiliary-pump temperatures are separate signals. |
| Engine oil pressure | VW Up `177D`, `381D` actual signed 16-bit `/10 kPa`; `1503` alternate `/416 psi`; `4F70` and `25CF` targets `/10 kPa` | Model/ECU-specific; target and actual are distinct. |
| Low-pressure fuel-line pressure | VW Up `2025` actual `u16/1000 bar`; `2932` specified `u16/1000 bar`; `1AB8` actual alternate signed `/10 kPa`; `11CE` commanded `/10 kPa` | Do not mix commanded and actual values. |
| Boost actuator / N75-like duty | Up `11A3` commanded regulation %, `16EE` / `1219` actuator/regulation duty; Jetta `16EE`; Taigo `1219`; Ateca `1108`, `3EA9` and related valve feedback/position/voltage DIDs `3D01`, `3D77`, `39A3`, `3EAC`, `16EC`, `16ED`, `29C7` | These are model-specific boost regulation, actuator or sensor values, not one universal “N75” DID. Up `39FF` is a bypass-valve activation signal, not N75 duty. |
| Haldex lock percentage | OpenHaldex-Edge custom protocol references | Not a stock generic OBD UDS DID. The current normal OBD DID scanner cannot send on the custom `70F→779` route. `FDA2` write/control is not catalogued or sent. |
| Engine oil temperature | Up `10A7`; Jetta `20A1`; Taigo `201A`, `179E`, `202F`, `40D5`; Touareg `10E6` | Same DID `10E6` means Up knock retard versus Touareg oil temperature; profile separation is mandatory. |
| Lambda | Up `10D5`; Seat Mii `15D4` | Lambda ratio only; no automatic AFR conversion. |
| Injection duration | Up `15E7`, `128C`, `2003`, `10D3`, `10DE`; Jetta `128C`, `2003`; Touareg `14CF`, `14CC` plus injector rail-pressure calibration points | Different data widths/scales, so profile+length matter. Calibration-point signals are not a direct current injection-duration measurement. |
| Injection timing / auxiliary air and environmental values | VW Up `12C8`, `12C7` pilot injection start angles; `14D1`, `121A` pilot injection duration; `121D`, `1631` pilot-injection activation/start; `121B`, `1131` post-injection timing; `081A`, `081B`, `205D`, `20C0` post-injection quantity/duration records. Up `121C` air filling (`u16 × 11719/500000`, range 0–100%); `11B8` air leakage raw; `11EE` pressure-before-throttle raw; `10CA`/`12B0` longitudinal acceleration (`s16/1000`); `F41F` engine runtime (seconds), `F433` barometric pressure (kPa), `F446` ambient temperature (`u8−40 °C`). | These expand the manual candidate set beyond the 12 headline parameters. Raw units remain raw where the source does not establish a physical unit. All are model-specific and not vehicle-verified. |
| Boost pressure target/actual | Up `11F0` actual bar, `111E` actual raw; Jetta `1118` target, `111E` actual raw and `11F0` model-specific raw; Taigo `1118` target, `111E` actual raw, `1057` actual bar; Ateca `2029` target raw and `1057` actual bar | DID `11F0` has different profile-specific meanings/scales in Up vs Jetta; never decode without the exact descriptor. Raw values remain raw when the source does not state a physical pressure unit. |
| Engine torque-related values | VW Up `1010`, `2002`, `437C` (signed 16-bit `/10 Nm`, testing candidates); Up `209C` engine drag torque (`u16/10 Nm`), `2929` adaptation of loss torque (`s16/10 Nm`), `108F` torque limit (`s16/10 Nm`), `203F` and `20A0` torque limits (`u16/10 Nm`), `2122` torque-loss adaptation (`s16/10 Nm`); `147F` is **drive-motor torque** (`s16/3 Nm`), not combustion-engine torque. Jetta `11E0` commanded crankshaft torque (signed `/10 Nm`; published example decodes to an implausible 1978.5 Nm, so candidate only), `29D4`/`4380` driver torque requests (`u16/10 Nm`), `5847`/`5849`/`584B` per-cylinder internal torque | Commanded/requested torque, engine drag/loss/limits, electric drive-motor torque and per-cylinder internal torque are not the same as measured total delivered combustion-engine torque. Jetta TCU `1010` is converter-loss torque. |
| Injector deviations / calibration | Up `2046`, `2048`, `2049` quantity deviation × `0.00036623`; `204A` alternate cylinder-2 quantity deviation `/100`; `2000`, `207B`, `21C1`, `2100`, `2101` injector-time deviations (signed raw). Touareg `10FF`, `1105`, `1100`, `1104` signed `/100` samples and `1106`/`1107` testing alternatives. | Some entries expose raw values with no reliable engineering unit. Do not invent µs or mg/stroke conversions. `10D0`–`10E1` calibration-point values are signed `/20000 ms`, with model/point-specific IDs. |

## Duplicate DID semantics

### Cross-model community profile (`vag-engine-community`)

38 engine DIDs from the OBDb community signalsets (the data behind Pelican) were added as profile `vag-engine-community`: per-cylinder timing retard (`200A`–`200D`), oil/coolant/intake/intercooler/EGT/catalyst temperatures, VVT actual/specified (`136E`, `1373`, `136C`, `1371`, `127D`), early-ignition (LSPI) and misfire counters, low-pressure fuel pressure/pump duty, fuel level in litres, oil level and engine torque. Selection rules: the same name and payload format in at least two OBDb vehicle repositories, no conflicting definition elsewhere in those repositories, and an explicit unit/scale. DIDs with an unresolved unit (e.g. `1056` charge pressure, `159F` rail pressure) were left out on purpose. The profile is polled last, so a model-specific entry with the same DID always wins. Everything here is `SOURCE_BACKED`, not vehicle-verified.

Not added: BMW N55/B58 values. The only public source found (kaiwen-z BMW ENET dashboard) reads them with `2C 01 F300` dynamic definitions followed by `22 F300`, which the UDS client does not implement, and its scale factors are partly estimated.

The same numeric DID may mean different values in different ECU/vehicle profiles and even different ECU roles. Examples intentionally kept separate:

- `10E6`: VW Up “Knock retard 2” versus VW Touareg engine-oil temperature.
- `1057`: boost pressure actual on Jetta, Ateca, and Taigo with model-specific payload interpretations/applicability.
- `1118`: boost target on certain engine ECU profiles; not a universal definition for every module.
- `1010`: VW Up engine torque candidate versus Jetta TCU torque-converter loss torque (different ECU roles).

When ECU profile is unknown, output is a set of labelled candidate interpretations, never a single asserted live-data value. The scanner queries one unique DID once per role, then shows every matching profile interpretation as `NOT VERIFIED FOR THIS ECU`. A full automatic profile match remains disabled until a validated fingerprint mapping exists.

## Source repositories and limits

- `bri3d/VW_Flash`: used for VAG-specific UDS identity DID constants and protocol context. It is a VAG ECU flashing repository, not a generic live-data DID catalogue. No flashing, programming, security bypass, or write workflow was copied into this project.
- `python-udsoncan`: used as a UDS client/response-validation reference. The firmware implements a smaller asynchronous embedded subset: one outstanding request, service/echo validation, NRC handling, timeout, and ISO-TP transport.
- `commaai/opendbc`: used only as a CAN/DBC signal reference for MQB signals. DBC CAN signals are not UDS DIDs and are not inserted as `0x22` identifiers.
- OBDb/Pelican-published Audi/VW parameter datasets: used as independent corroboration for specific payload layouts and formulas where an equivalent DID appears on another model. Cross-model matches are kept as separate profile descriptors, not assumed universal.
- `Forbes-Automotive/OpenHaldex-C6` and `Kile-Thomson/OpenHaldex-Edge`: used only to understand Haldex-specific architectures/custom CAN data. Their protocol is not treated as generic OEM UDS DID traffic.
- Pelican vehicle parameter pages are the primary source for the model-specific DID numbers, payload layouts, scales, units, sample examples, and model-year “In testing” / “Not supported” labels. The Jetta catalog additionally includes sample-confirmed `51D4` cylinder-block temperature, `1065` cylinder-head temperature, and `4267` direct-injection count. The Up catalog also includes `147D`, `147F`, `2929`, `209C`, `108F`, `203F`, `20A0`, `2122`, `121C`, `11B8`, `1029`, `1059`, `10CA`, `12B0`, `51D4`, `1065`, `523B`, `120E`, `1212`, `10E9`, `1230`, `2032`, `4267`, `417F`, `4183`, `4185`, pilot/post-injection timing DIDs (`12C8`, `12C7`, `14D1`, `121A`, `121D`, `1631`, `121B`, `1131`, `081A`, `081B`, `205D`, `20C0`), `11EE`, `1221`, `2097`, `1139`, `4268`, `F41F`, `F433`, `F446`, and lambda `15D4`. Other additions include Up `11F0`, `11A3`, `11FF`–`1202`, `39FF`, `1B04`, `2CD8`, `11B9`; Ateca actuator/sensor `3D01`, `3D77`, `39A3`, `3EAC`, `16EC`, `16ED`, `29C7`; Touareg knock retard/voltage records `13F4`, `13F6`, `13FC`, `13FA`, `13F9`, `13FD`, `120E`, `1210`, `120F`, `1211`; and Touareg TCU `2114`, `38F1`. A page entry is not proof that another ECU or vehicle supports the same DID.

## Field-test procedure

1. Allow the existing passive CAN discovery and Mode 01 `41 00` verification to complete.
2. Confirm the log states the verified bitrate and 11-bit/29-bit format. DID scan log includes the locked TX/RX IDs.
3. Turn off periodic PID polling with the runtime command if desired; this does not turn off the verified CAN link or the UDS transaction path.
4. Invoke the existing manual DID scan command. It sends one `22 DID_H DID_L` request at a time, with a 250 ms minimum gap between requests, logs the exact TX route, raw `62` data, negative response NRC, or timeout, then applies only labelled model-specific candidate decoders.
5. Do not treat positive response as proof of the decoder formula. Capture the ECU's F187/F189/F191/F1A3/F197/F1AD/F19E/F1A2 fingerprint and compare against a validated profile before promoting any candidate.

Host-side strict C compilation and decoder/route/UDS protocol harnesses are used during development. Physical CAN vehicle validation and a full ESP-IDF build on the target ESP32-S3/MCP2515 hardware are still required before field deployment.
