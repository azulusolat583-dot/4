#include <SPI.h>
#include <RF24.h>

enum CtrlCmd : uint8_t {
    CC_NOP = 0,
    CC_RUN_ALL,
    CC_CHECK_LASER,
    CC_CHECK_SERVOS,
    CC_CHECK_LINK,
    CC_CHECK_MCU,
    CC_SCAN_H,
    CC_SCAN_V,
    CC_SCAN_D1,
    CC_SCAN_D2,
    CC_GOTO_ZERO,
    CC_SCAN_FULL
};

enum CtrlMode : uint8_t {
    CM_IDLE = 0,
    CM_WAIT,
    CM_H,
    CM_V,
    CM_D1,
    CM_D2,
    CM_DIAG
};

enum CtrlDiag : uint8_t {
    CD_NONE = 0,
    CD_LASER,
    CD_SERVOS,
    CD_RADIO,
    CD_MCU,
    CD_ALL
};

struct CtrlCmdFrame {
    uint8_t node;
    CtrlCmd cmd;
};

struct CtrlTelFrame {
    uint8_t node;
    int8_t tilt;
    int8_t pan;
    CtrlMode mode;
};

struct CtrlDiagFrame {
    uint8_t node;
    CtrlDiag test;
    bool started;
    bool finished;
    bool passed;
};

const uint8_t PIN_RF_CE  = 7;
const uint8_t PIN_RF_CSN = 8;

RF24 rf(PIN_RF_CE, PIN_RF_CSN);

const uint8_t NODE_ID = 1;

const byte PIPE_HOST[6] = "HOST1";
const byte PIPE_NODE[6] = "NODE1";

void sendCommand(CtrlCmd c) {
    CtrlCmdFrame f;
    f.node = NODE_ID;
    f.cmd  = c;

    rf.stopListening();
    rf.openWritingPipe(PIPE_NODE);
    bool ok = rf.write(&f, sizeof(f));
    rf.startListening();

    Serial.print("cmd ");
    Serial.print((int)c);
    Serial.print(" sent ");
    Serial.println(ok ? "OK" : "FAIL");
}

const char* modeName(CtrlMode m) {
    switch (m) {
        case CM_IDLE: return "IDLE";
        case CM_WAIT: return "WAIT";
        case CM_H:    return "H";
        case CM_V:    return "V";
        case CM_D1:   return "D1";
        case CM_D2:   return "D2";
        case CM_DIAG: return "DIAG";
        default:      return "?";
    }
}

const char* diagName(CtrlDiag t) {
    switch (t) {
        case CD_LASER:  return "LASER";
        case CD_SERVOS: return "SERVOS";
        case CD_RADIO:  return "RADIO";
        case CD_MCU:    return "MCU";
        case CD_ALL:    return "ALL";
        default:        return "NONE";
    }
}

void printMenu() {
    Serial.println("Console ready");
    Serial.println("Keys:");
    Serial.println(" 1 - all tests");
    Serial.println(" 2 - laser");
    Serial.println(" 3 - servos");
    Serial.println(" 4 - radio");
    Serial.println(" 5 - mcu");
    Serial.println(" h - scan H");
    Serial.println(" v - scan V");
    Serial.println(" x - scan diag 1");
    Serial.println(" y - scan diag 2");
    Serial.println(" s - scan full");
    Serial.println(" 0 - home");
}

void processKey(char c) {
    if (c == '1') sendCommand(CC_RUN_ALL);
    else if (c == '2') sendCommand(CC_CHECK_LASER);
    else if (c == '3') sendCommand(CC_CHECK_SERVOS);
    else if (c == '4') sendCommand(CC_CHECK_LINK);
    else if (c == '5') sendCommand(CC_CHECK_MCU);
    else if (c == 'h') sendCommand(CC_SCAN_H);
    else if (c == 'v') sendCommand(CC_SCAN_V);
    else if (c == 'x') sendCommand(CC_SCAN_D1);
    else if (c == 'y') sendCommand(CC_SCAN_D2);
    else if (c == 's') sendCommand(CC_SCAN_FULL);
    else if (c == '0') sendCommand(CC_GOTO_ZERO);
}

void showDiag(const CtrlDiagFrame* d) {
    Serial.print("[D] id=");
    Serial.print(d->node);
    Serial.print(" test=");
    Serial.print(diagName(d->test));
    Serial.print(" st=");
    Serial.print(d->started ? '1' : '0');
    Serial.print(" end=");
    Serial.print(d->finished ? '1' : '0');
    Serial.print(" ok=");
    Serial.println(d->passed ? '1' : '0');
}

void showTel(const CtrlTelFrame* t) {
    Serial.print("[S] id=");
    Serial.print(t->node);
    Serial.print(" m=");
    Serial.print(modeName(t->mode));
    Serial.print(" tilt=");
    Serial.print(t->tilt);
    Serial.print(" pan=");
    Serial.println(t->pan);
}

void receiveFrame() {
    uint8_t buf[32];
    rf.read(&buf, sizeof(buf));

    CtrlDiagFrame* d = (CtrlDiagFrame*)buf;
    bool isDiag =
        d->node == NODE_ID &&
        d->test >= CD_LASER &&
        d->test <= CD_ALL;

    if (isDiag) {
        showDiag(d);
    } else {
        CtrlTelFrame* t = (CtrlTelFrame*)buf;
        showTel(t);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    printMenu();

    rf.begin();
    rf.setChannel(90);
    rf.setDataRate(RF24_250KBPS);
    rf.setPALevel(RF24_PA_LOW);
    rf.openReadingPipe(1, PIPE_HOST);
    rf.startListening();
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        processKey(c);
    }

    if (rf.available()) {
        receiveFrame();
    }
}
