# UDS Diagnostic ECU with Self-Checking Tests (ESP32)

An ESP32 sketch (Arduino IDE) that implements a small subset of the UDS diagnostic protocol (ISO 14229-1) as an ECU, and tests itself. A tester function sends 13 request frames over a simulated CAN bus, and the sketch compares each response byte-for-byte against the expected frame, then prints `PASS` / `FAIL` and a summary.

> **Scope:** the CAN bus is simulated with two FreeRTOS queues. No CAN controller, transceiver or second node is used, so this verifies the protocol logic, not CAN electrical behaviour.

## What it implements

| Service | SID | Behaviour |
|---|---|---|
| DiagnosticSessionControl | `0x10` | Sub-functions `0x01` (default) and `0x03` (extended). Positive response carries P2 = 50 ms and P2* = 5000 ms. Other sub-functions return NRC `0x12`. |
| ReadDataByIdentifier | `0x22` | One DID per request: `0x0100` engine RPM (2850), `0x0101` coolant temperature (88 °C, sent as +40 offset), `0x0200` software version `0x0102` (**extended session only**). Unknown DID or wrong session returns NRC `0x31`. |
| TesterPresent | `0x3E` | Sub-function `0x00`. Restarts the session timer. |
| Anything else | | NRC `0x11` (service not supported) |

Other behaviour:

- **Request validation:** wrong request length returns NRC `0x13`. Only ISO-TP single frames (PCI type 0) are accepted; other frame types are ignored.
- **Session timeout (S3):** the ECU falls back to the default session after 5 s without a valid request.
- **Framing:** request ID `0x7E0`, response ID `0x7E8`, 8-byte frames. The tester pads requests with `0x55` and the ECU pads responses with `0xAA`. The test also checks the padding bytes.

## How the tests work

```text
  Tester (in the sketch)                      ECU
  ──────────────────────  queue (0x7E0)  ──────────────────
  send request frame      ───────────────▶   Ecu_Process()
  compare response        ◀───────────────   queue (0x7E8)
  PASS if bytes + padding match the expected frame
```

| # | Test | Request | Expected response |
|---|---|---|---|
| 1 | RDBI engine RPM | `03 22 01 00` | `05 62 01 00 0B 22` |
| 2 | RDBI coolant temperature | `03 22 01 01` | `04 62 01 01 80` |
| 3 | RDBI unknown DID `0x9999` | `03 22 99 99` | `03 7F 22 31` |
| 4 | RDBI wrong length | `01 22` | `03 7F 22 13` |
| 5 | RDBI software version, default session | `03 22 02 00` | `03 7F 22 31` |
| 6 | Enter extended session | `02 10 03` | `06 50 03 00 32 01 F4` |
| 7 | RDBI software version, extended session | `03 22 02 00` | `05 62 02 00 01 02` |
| 8 | Tester Present | `02 3E 00` | `02 7E 00` |
| 9 | DSC invalid sub-function | `02 10 7E` | `03 7F 10 12` |
| 10 | Unsupported SID `0x99` | `01 99` | `03 7F 99 11` |
| 11 | Back to default session | `02 10 01` | `06 50 01 00 32 01 F4` |
| 12 | Extended session again | `02 10 03` | `06 50 03 00 32 01 F4` |
| 13 | S3 timeout: software version after 5.5 s idle | `03 22 02 00` | `03 7F 22 31` |

## Results

Run on an ESP32 WROOM-32 dev board (ESP32-D0WD-V3), Arduino-ESP32 core 3.3.12:

```text
[SUMMARY] 13/13 PASS
```

| Resource | Usage |
|---|---|
| Program storage | 271,876 bytes (20% of 1,310,720) |
| Dynamic memory | 22,124 bytes (6% of 327,680) |

Sending `r` in the Serial Monitor runs the whole suite again.

## Limitations

- **Simulated bus only.** There is no real CAN bus, so arbitration, bit timing, error frames and bus-off are not tested.
- **ISO-TP single frames only.** Multi-frame transfers and flow control (ISO 15765-2) are not implemented, so responses are limited to 7 payload bytes.
- **Small service set.** No DTC reading (`0x19`), security access (`0x27`), ECU reset (`0x11`) or write services.
- **The tester lives in the same firmware** as the ECU, so the tests prove the ECU matches the expected frames, not interoperability with an external tool.
- **P2 / P2\* are reported but not enforced** as response-timing limits.

## Build and run

1. Install the Arduino IDE and add the **esp32 by Espressif Systems** board package.
2. Open `CAN_UDS_Node.ino`, select **ESP32 Dev Module** and your COM port, then upload.
3. Open the Serial Monitor at **115200** baud. The suite runs about 6 seconds after boot.

No wiring is needed beyond the USB cable.

```text
└── CAN_UDS_Node/
    └── CAN_UDS_Node.ino
```

## Roadmap

- [ ] Run on real CAN with two ESP32 nodes and SN65HVD230 transceivers (TWAI driver)
- [ ] Multi-frame ISO-TP with flow control
- [ ] Add `0x19` (read DTCs), `0x27` (security access) and `0x11` (ECU reset)
- [ ] Drive the ECU from an external tester (python-can)
- [ ] Fault injection: dropped, delayed and malformed frames
