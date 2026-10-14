/*
 * CAN / UDS Diagnostic ECU - self-checking simulation (ESP32, Arduino IDE)
 *
 *  - The "CAN bus" is a pair of FreeRTOS queues (tester -> ECU, ECU -> tester).
 *    No CAN hardware or transceiver is used.
 *  - ISO-TP: single frames only (no multi-frame / flow control).
 *  - UDS services: 0x10 DiagnosticSessionControl, 0x22 ReadDataByIdentifier,
 *    0x3E TesterPresent. Anything else returns NRC 0x11.
 *  - Every request has an expected response; the sketch prints PASS/FAIL
 *    and a summary. Send 'r' in the Serial Monitor to run the tests again.
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define UDS_REQ_CAN_ID   0x7E0
#define UDS_RESP_CAN_ID  0x7E8
#define S3_TIMEOUT_MS    5000UL

#define SID_DSC   0x10
#define SID_RDBI  0x22
#define SID_TP    0x3E
#define SID_NEG   0x7F

#define NRC_SERVICE_NOT_SUPPORTED      0x11
#define NRC_SUBFUNCTION_NOT_SUPPORTED  0x12
#define NRC_INCORRECT_LENGTH           0x13
#define NRC_REQUEST_OUT_OF_RANGE       0x31

#define SESSION_DEFAULT   0x01
#define SESSION_EXTENDED  0x03

#define DID_ENGINE_RPM    0x0100
#define DID_COOLANT_TEMP  0x0101
#define DID_SW_VERSION    0x0200   // readable in the extended session only

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

typedef struct {
    const char *name;
    uint8_t     req[8];
    uint8_t     req_len;
    uint8_t     exp[8];      // expected payload incl. PCI byte; rest must be 0xAA
    uint8_t     exp_len;
    uint32_t    wait_ms;     // idle time before sending the request
} test_case_t;

static QueueHandle_t busToEcu;      // tester -> ECU
static QueueHandle_t busToTester;   // ECU -> tester

static uint8_t  current_session = SESSION_DEFAULT;
static uint32_t last_request_ms = 0;

static const uint16_t engine_rpm     = 2850;   // 0x0B22
static const int8_t   coolant_temp_c = 88;     // sent as 88 + 40 = 0x80
static const uint16_t sw_version     = 0x0102;

/* ------------------------------ Simulated bus ----------------------------- */

static bool Bus_Send(QueueHandle_t bus, uint32_t id, const uint8_t *payload,
                     uint8_t len, uint8_t pad) {
    if (len > 8) return false;
    can_frame_t f;
    f.id  = id;
    f.dlc = 8;
    memset(f.data, pad, sizeof(f.data));
    memcpy(f.data, payload, len);
    return xQueueSend(bus, &f, 0) == pdTRUE;
}

/* --------------------------------- ECU ------------------------------------ */

static void Ecu_Respond(const uint8_t *payload, uint8_t len) {
    Bus_Send(busToTester, UDS_RESP_CAN_ID, payload, len, 0xAA);
}

static void Ecu_Nrc(uint8_t sid, uint8_t nrc) {
    uint8_t r[4] = {0x03, SID_NEG, sid, nrc};
    Ecu_Respond(r, 4);
}

static void Ecu_Process(const can_frame_t &f) {
    uint8_t pci = f.data[0];
    if ((pci & 0xF0) != 0x00) return;               // only ISO-TP single frames
    uint8_t n = pci & 0x0F;                         // payload length in bytes
    if (n == 0 || n > 7 || f.dlc < (uint8_t)(n + 1)) return;

    uint8_t sid = f.data[1];
    last_request_ms = millis();                     // restarts the S3 timer

    switch (sid) {
        case SID_DSC: {
            if (n != 2) { Ecu_Nrc(sid, NRC_INCORRECT_LENGTH); break; }
            uint8_t sub = f.data[2];
            if (sub == SESSION_DEFAULT || sub == SESSION_EXTENDED) {
                current_session = sub;
                // session, P2 = 50 ms (0x0032), P2* = 5000 ms (0x01F4 x 10 ms)
                uint8_t r[7] = {0x06, (uint8_t)(sid + 0x40), sub, 0x00, 0x32, 0x01, 0xF4};
                Ecu_Respond(r, 7);
            } else {
                Ecu_Nrc(sid, NRC_SUBFUNCTION_NOT_SUPPORTED);
            }
            break;
        }

        case SID_TP: {
            if (n != 2) { Ecu_Nrc(sid, NRC_INCORRECT_LENGTH); break; }
            if (f.data[2] == 0x00) {
                uint8_t r[3] = {0x02, (uint8_t)(sid + 0x40), 0x00};
                Ecu_Respond(r, 3);
            } else {
                Ecu_Nrc(sid, NRC_SUBFUNCTION_NOT_SUPPORTED);
            }
            break;
        }

        case SID_RDBI: {
            if (n != 3) { Ecu_Nrc(sid, NRC_INCORRECT_LENGTH); break; }
            uint16_t did = ((uint16_t)f.data[2] << 8) | f.data[3];
            if (did == DID_ENGINE_RPM) {
                uint8_t r[6] = {0x05, (uint8_t)(sid + 0x40), f.data[2], f.data[3],
                                (uint8_t)(engine_rpm >> 8), (uint8_t)(engine_rpm & 0xFF)};
                Ecu_Respond(r, 6);
            } else if (did == DID_COOLANT_TEMP) {
                uint8_t r[5] = {0x04, (uint8_t)(sid + 0x40), f.data[2], f.data[3],
                                (uint8_t)(coolant_temp_c + 40)};
                Ecu_Respond(r, 5);
            } else if (did == DID_SW_VERSION && current_session == SESSION_EXTENDED) {
                uint8_t r[6] = {0x05, (uint8_t)(sid + 0x40), f.data[2], f.data[3],
                                (uint8_t)(sw_version >> 8), (uint8_t)(sw_version & 0xFF)};
                Ecu_Respond(r, 6);
            } else {
                Ecu_Nrc(sid, NRC_REQUEST_OUT_OF_RANGE);
            }
            break;
        }

        default:
            Ecu_Nrc(sid, NRC_SERVICE_NOT_SUPPORTED);
            break;
    }
}

static void Ecu_Service(void) {
    if (current_session != SESSION_DEFAULT &&
        (millis() - last_request_ms) > S3_TIMEOUT_MS) {
        current_session = SESSION_DEFAULT;
        Serial.println("   [ECU] S3 timeout: session fell back to default");
    }
    can_frame_t f;
    while (xQueueReceive(busToEcu, &f, 0) == pdTRUE) {
        if (f.id == UDS_REQ_CAN_ID) Ecu_Process(f);
    }
}

/* ----------------------------- Self-checking tests ------------------------- */

static const test_case_t tests[] = {
    {"RDBI engine RPM (0x0100)",               {0x03,0x22,0x01,0x00}, 4, {0x05,0x62,0x01,0x00,0x0B,0x22}, 6, 0},
    {"RDBI coolant temp (0x0101)",             {0x03,0x22,0x01,0x01}, 4, {0x04,0x62,0x01,0x01,0x80},      5, 0},
    {"RDBI unknown DID (0x9999) -> NRC 0x31",  {0x03,0x22,0x99,0x99}, 4, {0x03,0x7F,0x22,0x31},           4, 0},
    {"RDBI bad length -> NRC 0x13",            {0x01,0x22},           2, {0x03,0x7F,0x22,0x13},           4, 0},
    {"RDBI SW version in default session",     {0x03,0x22,0x02,0x00}, 4, {0x03,0x7F,0x22,0x31},           4, 0},
    {"DSC extended session (0x10 03)",         {0x02,0x10,0x03},      3, {0x06,0x50,0x03,0x00,0x32,0x01,0xF4}, 7, 0},
    {"RDBI SW version in extended session",    {0x03,0x22,0x02,0x00}, 4, {0x05,0x62,0x02,0x00,0x01,0x02}, 6, 0},
    {"Tester Present (0x3E 00)",               {0x02,0x3E,0x00},      3, {0x02,0x7E,0x00},                3, 0},
    {"DSC invalid sub-function -> NRC 0x12",   {0x02,0x10,0x7E},      3, {0x03,0x7F,0x10,0x12},           4, 0},
    {"Unsupported SID 0x99 -> NRC 0x11",       {0x01,0x99},           2, {0x03,0x7F,0x99,0x11},           4, 0},
    {"DSC back to default session (0x10 01)",  {0x02,0x10,0x01},      3, {0x06,0x50,0x01,0x00,0x32,0x01,0xF4}, 7, 0},
    {"DSC extended session again",             {0x02,0x10,0x03},      3, {0x06,0x50,0x03,0x00,0x32,0x01,0xF4}, 7, 0},
    {"S3 timeout: SW version after idle",      {0x03,0x22,0x02,0x00}, 4, {0x03,0x7F,0x22,0x31},           4, S3_TIMEOUT_MS + 500},
};

#define NUM_TESTS (sizeof(tests) / sizeof(tests[0]))

static void Print_Hex(const uint8_t *d, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) Serial.printf("%02X ", d[i]);
}

static void Wait_Servicing(uint32_t ms) {
    uint32_t t0 = millis();
    while ((millis() - t0) < ms) {
        Ecu_Service();
        delay(10);
    }
}

static bool Run_Test(const test_case_t &t, int index) {
    Serial.printf("\n[TEST %02d] %s\n", index, t.name);
    if (t.wait_ms) {
        Serial.printf("   (idling %u ms)\n", (unsigned)t.wait_ms);
        Wait_Servicing(t.wait_ms);
    }

    can_frame_t stale;
    while (xQueueReceive(busToTester, &stale, 0) == pdTRUE) {}   // clear old replies

    Bus_Send(busToEcu, UDS_REQ_CAN_ID, t.req, t.req_len, 0x55);
    Ecu_Service();

    can_frame_t r;
    bool got = (xQueueReceive(busToTester, &r, pdMS_TO_TICKS(100)) == pdTRUE);
    bool ok = got && r.id == UDS_RESP_CAN_ID && memcmp(r.data, t.exp, t.exp_len) == 0;
    if (ok) {
        for (int i = t.exp_len; i < 8; i++) {
            if (r.data[i] != 0xAA) ok = false;        // padding must be 0xAA
        }
    }

    Serial.print("   REQ : "); Print_Hex(t.req, t.req_len); Serial.println();
    Serial.print("   RESP: ");
    if (got) Print_Hex(r.data, 8); else Serial.print("(none)");
    Serial.println();
    Serial.print("   EXP : "); Print_Hex(t.exp, t.exp_len); Serial.println("(+ AA padding)");
    Serial.println(ok ? "   -> PASS" : "   -> FAIL");
    return ok;
}

static void Run_All_Tests(void) {
    current_session = SESSION_DEFAULT;
    last_request_ms = millis();

    Serial.println("\n--- Running UDS self-checking tests ---");
    int pass = 0;
    for (unsigned i = 0; i < NUM_TESTS; i++) {
        if (Run_Test(tests[i], (int)i + 1)) pass++;
    }
    Serial.printf("\n[SUMMARY] %d/%u PASS\n", pass, (unsigned)NUM_TESTS);
    Serial.println("Send 'r' to run the tests again.");
}

/* -------------------------------- Arduino --------------------------------- */

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=======================================================");
    Serial.println("  CAN / UDS Diagnostic ECU - Self-Checking Simulation");
    Serial.println("  Bus: simulated (FreeRTOS queues) | ISO-TP: single frame");
    Serial.println("=======================================================");

    busToEcu    = xQueueCreate(8, sizeof(can_frame_t));
    busToTester = xQueueCreate(8, sizeof(can_frame_t));

    Run_All_Tests();
}

void loop() {
    Ecu_Service();
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r' || c == 'R') Run_All_Tests();
    }
    delay(20);
}