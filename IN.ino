#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

const uint8_t RF_CE   = 9;
const uint8_t RF_CSN  = 3;
const uint8_t PIN_SERVO_TILT = 7;
const uint8_t PIN_SERVO_PAN  = 6;
const uint8_t PIN_LASER      = 5;
const uint8_t PIN_STOP       = 4;

RF24 link(RF_CE, RF_CSN);
Servo srvTilt;
Servo srvPan;

const byte PIPE_HOST2[6] = "HOST1";
const byte PIPE_NODE2[6] = "NODE1";

const uint8_t SELF_ID = 1;

const uint16_t LASER_ON_MS  = 2500;
const uint16_t MOVE_DELAY_MS = 400;

enum NodeCmd : uint8_t {
    NC_NOP = 0,
    NC_RUN_ALL,
    NC_LASER,
    NC_SERVO,
    NC_RADIO,
    NC_MCU,
    NC_SCAN_H,
    NC_SCAN_V,
    NC_SCAN_D1,
    NC_SCAN_D2,
    NC_ZERO,
    NC_SCAN_FULL
};

enum NodeMode : uint8_t {
    NM_IDLE = 0,
    NM_WAIT,
    NM_SCAN_H,
    NM_SCAN_V,
    NM_SCAN_D1,
    NM_SCAN_D2,
    NM_DIAG
};

enum NodeDiag : uint8_t {
    ND_NOP = 0,
    ND_LASER,
    ND_SERVOS,
    ND_RADIO,
    ND_MCU,
    ND_ALL
};

struct NodeCmdFrame {
    uint8_t node;
    NodeCmd cmd;
};

struct NodeTelFrame {
    uint8_t node;
    int8_t tilt;
    int8_t pan;
    NodeMode mode;
};

struct NodeDiagFrame {
    uint8_t node;
    NodeDiag test;
    bool started;
    bool finished;
    bool passed;
};

int8_t tiltDeg = 0;
int8_t panDeg  = 0;
NodeMode modeNow = NM_IDLE;

int degToServo(int8_t d) {
    if (d < -40) d = -40;
    if (d > 40)  d = 40;
    return map(d, -40, 40, 50, 130);
}

bool stopPressed() {
    return digitalRead(PIN_STOP) == LOW;
}

void moveHead(int8_t tiltTarget, int8_t panTarget, uint16_t stepMs = 20) {
    int tEnd = constrain(tiltTarget, -40, 40);
    int pEnd = constrain(panTarget, -40, 40);
    int tStart = tiltDeg;
    int pStart = panDeg;
    int dT = abs(tEnd - tStart);
    int dP = abs(pEnd - pStart);
    int steps = max(dT, dP);

    if (steps <= 0) {
        srvTilt.write(degToServo(tEnd));
        srvPan.write(degToServo(pEnd));
        tiltDeg = tEnd;
        panDeg = pEnd;
        return;
    }

    for (int i = 1; i <= steps; ++i) {
        if (stopPressed()) break;
        int tCur = tStart + (tEnd - tStart) * i / steps;
        int pCur = pStart + (pEnd - pStart) * i / steps;
        srvTilt.write(degToServo(tCur));
        srvPan.write(degToServo(pCur));
        delay(stepMs);
    }

    tiltDeg = tEnd;
    panDeg = pEnd;
}

void sendState() {
    NodeTelFrame f;
    f.node = SELF_ID;
    f.tilt = -tiltDeg;
    f.pan  = panDeg;
    f.mode = modeNow;

    link.stopListening();
    link.openWritingPipe(PIPE_HOST2);
    link.write(&f, sizeof(f));
    link.startListening();
}

void sendDiag(NodeDiag t, bool st, bool fin, bool ok) {
    NodeDiagFrame f;
    f.node     = SELF_ID;
    f.test     = t;
    f.started  = st;
    f.finished = fin;
    f.passed   = ok;

    link.stopListening();
    link.openWritingPipe(PIPE_HOST2);
    link.write(&f, sizeof(f));
    link.startListening();
}

void runLaserDiag() {
    modeNow = NM_DIAG;
    sendDiag(ND_LASER, true, false, true);

    for (int i = 0; i < 3; ++i) {
        if (stopPressed()) {
            sendDiag(ND_LASER, false, true, false);
            return;
        }
        digitalWrite(PIN_LASER, HIGH);
        delay(300);
        digitalWrite(PIN_LASER, LOW);
        delay(300);
    }

    sendDiag(ND_LASER, false, true, true);
}

void runServosDiag() {
    modeNow = NM_DIAG;
    sendDiag(ND_SERVOS, true, false, true);

    moveHead(0, 0, 10);
    delay(300);
    if (stopPressed()) { sendDiag(ND_SERVOS, false, true, false); return; }

    moveHead(-40, -40, 10);
    delay(500);
    if (stopPressed()) { sendDiag(ND_SERVOS, false, true, false); return; }

    moveHead(40, 40, 10);
    delay(500);
    if (stopPressed()) { sendDiag(ND_SERVOS, false, true, false); return; }

    moveHead(0, 0, 10);
    delay(300);

    sendDiag(ND_SERVOS, false, true, true);
}

void runRadioDiag() {
    modeNow = NM_DIAG;
    sendDiag(ND_RADIO, true, false, true);

    for (int i = 0; i < 3; ++i) {
        if (stopPressed()) {
            sendDiag(ND_RADIO, false, true, false);
            return;
        }
        sendState();
        delay(500);
    }

    sendDiag(ND_RADIO, false, true, true);
}

void runMCUDiag() {
    modeNow = NM_DIAG;
    sendDiag(ND_MCU, true, false, true);

    for (int i = 0; i < 2; ++i) {
        if (stopPressed()) {
            sendDiag(ND_MCU, false, true, false);
            return;
        }
        digitalWrite(PIN_LASER, HIGH);
        delay(100);
        digitalWrite(PIN_LASER, LOW);
        delay(100);
    }

    sendDiag(ND_MCU, false, true, true);
}

void runAllDiag() {
    modeNow = NM_DIAG;
    sendDiag(ND_ALL, true, false, true);

    runLaserDiag();
    runServosDiag();
    runRadioDiag();
    runMCUDiag();

    sendDiag(ND_ALL, false, true, true);
}

void patternH() {
    modeNow = NM_SCAN_H;
    for (int t = 40; t >= -40; t -= 10) {
        if (stopPressed()) break;
        moveHead(t, 0);
        delay(100);
        digitalWrite(PIN_LASER, HIGH);
        sendState();
        delay(LASER_ON_MS);
        digitalWrite(PIN_LASER, LOW);
        delay(MOVE_DELAY_MS);
    }
}

void patternV() {
    modeNow = NM_SCAN_V;
    for (int p = -40; p <= 40; p += 10) {
        if (stopPressed()) break;
        moveHead(0, p);
        delay(100);
        digitalWrite(PIN_LASER, HIGH);
        sendState();
        delay(LASER_ON_MS);
        digitalWrite(PIN_LASER, LOW);
        delay(MOVE_DELAY_MS);
    }
}

void patternD1() {
    modeNow = NM_SCAN_D1;
    for (int a = -40; a <= 40; a += 10) {
        if (stopPressed()) break;
        moveHead(-a, a);
        delay(100);
        digitalWrite(PIN_LASER, HIGH);
        sendState();
        delay(LASER_ON_MS);
        digitalWrite(PIN_LASER, LOW);
        delay(MOVE_DELAY_MS);
    }
}

void patternD2() {
    modeNow = NM_SCAN_D2;
    for (int a = -40; a <= 40; a += 10) {
        if (stopPressed()) break;
        moveHead(-a, -a);
        delay(100);
        digitalWrite(PIN_LASER, HIGH);
        sendState();
        delay(LASER_ON_MS);
        digitalWrite(PIN_LASER, LOW);
        delay(MOVE_DELAY_MS);
    }
}

void patternFull() {
    patternH();
    if (stopPressed()) return;
    patternV();
    if (stopPressed()) return;
    patternD1();
    if (stopPressed()) return;
    patternD2();
}

void goCenter() {
    moveHead(0, 0, 10);
    modeNow = NM_IDLE;
    sendState();
}

void handleCmd(const NodeCmdFrame& c) {
    if (c.node != SELF_ID && c.node != 255) return;

    switch (c.cmd) {
        case NC_RUN_ALL:
            runAllDiag();
            goCenter();
            break;
        case NC_LASER:
            runLaserDiag();
            break;
        case NC_SERVO:
            runServosDiag();
            break;
        case NC_RADIO:
            runRadioDiag();
            break;
        case NC_MCU:
            runMCUDiag();
            break;
        case NC_SCAN_H:
            patternH();
            goCenter();
            break;
        case NC_SCAN_V:
            patternV();
            goCenter();
            break;
        case NC_SCAN_D1:
            patternD1();
            goCenter();
            break;
        case NC_SCAN_D2:
            patternD2();
            goCenter();
            break;
        case NC_SCAN_FULL:
            patternFull();
            goCenter();
            break;
        case NC_ZERO:
            goCenter();
            break;
        default:
            break;
    }

    modeNow = NM_WAIT;
}

void setup() {
    pinMode(PIN_LASER, OUTPUT);
    digitalWrite(PIN_LASER, LOW);
    pinMode(PIN_STOP, INPUT_PULLUP);

    srvTilt.attach(PIN_SERVO_TILT);
    srvPan.attach(PIN_SERVO_PAN);
    moveHead(0, 0, 10);

    link.begin();
    link.setChannel(90);
    link.setDataRate(RF24_250KBPS);
    link.setPALevel(RF24_PA_LOW);
    link.openReadingPipe(1, PIPE_NODE2);
    link.startListening();

    modeNow = NM_WAIT;
    delay(500);
    sendState();
}

void loop() {
    if (stopPressed()) {
        digitalWrite(PIN_LASER, LOW);
        delay(100);
        return;
    }

    if (link.available()) {
        NodeCmdFrame f;
        link.read(&f, sizeof(f));
        handleCmd(f);
    }
}
