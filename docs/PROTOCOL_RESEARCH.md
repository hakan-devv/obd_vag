# OBD-II, ISO-TP, UDS, and MCP2515 notes

## Scope and evidence

This note records public-source findings used by the firmware. The full SAE/ISO
normative texts were not publicly available during research, so implementation
details should be checked against the licensed editions before claiming formal
conformance. Open-source implementation references below are corroboration, not
substitutes for the standards.

## OBD-II over CAN

- SAE J1979 defines the standardized emissions-diagnostics services and data;
  the official listing is [SAE J1979_202202](https://www.sae.org/standards/content/j1979_202202/).
- The official catalogs for [ISO 15765-4](https://www.iso.org/search.html?q=ISO%2015765-4)
  and [ISO 15765-2](https://www.iso.org/search.html?q=ISO%2015765-2) are the
  references for emissions diagnostics over CAN and the transport protocol.
- Python-OBD enumerates the 11-bit/29-bit and 250/500 kbit/s CAN combinations
  used by common emissions-diagnostics protocols:
  [protocol_can.py](https://github.com/brendan-w/python-OBD/blob/a378bdd81d58c67d08050e4244173a9a7dbda73d/obd/protocols/protocol_can.py#L280-L309).
- Mode 01 support bitmaps use requests `01 00`, `01 20`, `01 40`, and so on.
  A bitmap page describes the next 32 PIDs; its final bit indicates whether a
  subsequent page is available. The mapping is corroborated by
  [Python-OBD command definitions](https://github.com/brendan-w/python-OBD/blob/a378bdd81d58c67d08050e4244173a9a7dbda73d/obd/commands.py#L53-L69)
  and its [discovery loop](https://github.com/brendan-w/python-OBD/blob/a378bdd81d58c67d08050e4244173a9a7dbda73d/obd/obd.py#L112-L140).
- A bitmap is an ECU's report of supported standardized PIDs. A missing response
  may also indicate addressing, connection, ignition, or bitrate problems; it
  does not alone prove that a PID is unsupported.

Representative standardized formulas implemented by the Parameter Manager:

| PID | Parameter | Formula |
| --- | --- | --- |
| `04` | Calculated engine load | `100 * A / 255` % |
| `05` | Coolant temperature | `A - 40` deg C |
| `0C` | Engine speed | `(256 * A + B) / 4` rpm |
| `0D` | Vehicle speed | `A` km/h |
| `10` | MAF | `(256 * A + B) / 100` g/s |

Further formulas and catalog coverage are listed in
[PARAMETER_CATALOG.md](./PARAMETER_CATALOG.md). Python-OBD's command and decoder
sources corroborate the representative conversions:
[commands.py](https://github.com/brendan-w/python-OBD/blob/a378bdd81d58c67d08050e4244173a9a7dbda73d/obd/commands.py#L53-L69)
and [decoders.py](https://github.com/brendan-w/python-OBD/blob/a378bdd81d58c67d08050e4244173a9a7dbda73d/obd/decoders.py#L102-L123).

## ISO-TP

- Classical CAN with normal addressing carries up to 7 application bytes in a
  Single Frame. Larger payloads use First, Consecutive, and Flow Control frames.
- Flow Control provides status, Block Size, and STmin. The Linux
  [ISO-TP documentation](https://docs.kernel.org/networking/iso15765-2.html#flow-control-options)
  documents STmin encodings `00..7F` (0..127 ms) and `F1..F9` (100..900 us);
  reserved encodings are rejected by this implementation.
- ECU Flow Control arrives using the ECU response CAN identifier; the tester
  sends Flow Control for a received multi-frame PDU using its request identifier.
- The MCP2515 is a Classical-CAN controller; its support for extended CAN IDs
  does not imply CAN-FD support. Linux's
  [link-layer options](https://docs.kernel.org/networking/iso15765-2.html#link-layer-options)
  distinguish CAN-FD link-layer settings from classical CAN.
- The project's ISO-TP implementation has bounded payloads, sequence checking,
  FC status handling, BS/STmin pacing, and receive/transmit timeouts. Its
  behavior has focused test cases in the optional protocol-test build; this is
  not a claim of full ISO conformance.
- To build the isolated protocol test image, enable
  `CONFIG_OBD_RUN_PROTOCOL_TESTS=y`. That image runs only pure protocol/decoder
  tests and does not initialize or transmit on CAN. Its assertions have been
  compiled here but must still be executed on an ESP32-S3 test device.

## Read-only UDS boundary

- UDS `ReadDataByIdentifier` uses request service `0x22`; the positive service
  response is `0x62`. `python-udsoncan` documents the service serialization and
  DID codecs in [ReadDataByIdentifier.py](https://github.com/pylessard/python-udsoncan/blob/78e85f1b5dac968b6a2d478fce9c0cc5adba2b50/udsoncan/services/ReadDataByIdentifier.py#L14-L36).
- Negative responses have the form `7F <request SID> <NRC>`; `0x78` means
  response pending. A valid `0x62` response does not establish a DID's meaning,
  scaling, or safety.
- The service `0x10 03` changes diagnostic session state; `0x3E 00` keeps an
  active session alive. These should be explicit, opt-in operations with
  timeouts, positive/negative response tracking, and no automatic session
  changes at boot.
- No VAG- or BMW-specific DID is enabled by default. A DID must be associated
  with a named ECU family and documented encoding before it can be exposed as a
  decoded parameter. Unknown values remain `UNKNOWN/TODO`.

## MCP2515 bit timing and SPI

- The primary reference is Microchip's
  [MCP2515 datasheet DS20001801J](https://ww1.microchip.com/downloads/en/DeviceDoc/20001801J.pdf).
  The official PDF was not retrievable in the research environment, so this
  note does not independently certify any concrete CNF register tuple.
- Bit timing depends on both the oscillator fitted to the module and the
  selected bitrate/sample point. The user confirmed 8 MHz oscillators on the
  two bench modules; firmware timing values still require module readback and
  physical verification. Do not infer oscillator frequency from a board label
  or reuse timing values for a different clock.
- ESP-IDF v5.5.2's official [SPI master guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/api-reference/peripherals/spi_master.html)
  documents transaction, DMA, and bus-acquisition requirements. Firmware
  starts SPI at a conservative 1 MHz.

## Other references and licensing notes

- [Linux ISO-TP documentation](https://docs.kernel.org/networking/iso15765-2.html)
- [`python-can-isotp` protocol implementation](https://github.com/pylessard/python-can-isotp/blob/5593c219f35e739554d743a98d3af68c9332da31/isotp/protocol.py)
- [`python-udsoncan` service implementation](https://github.com/pylessard/python-udsoncan/blob/78e85f1b5dac968b6a2d478fce9c0cc5adba2b50/udsoncan/services/ReadDataByIdentifier.py)

The cited libraries have their own licenses (Python-OBD GPL-2.0-only;
`python-can-isotp` and `python-udsoncan` MIT). This project uses them as
references and does not copy their implementation code.
