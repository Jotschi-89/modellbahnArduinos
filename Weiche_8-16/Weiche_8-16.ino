#include <SPI.h>
#include <TimerOne.h>
#include <mcp2515.h>
#include <zcan.h>
#include <Servo.h>

// PIN Belegung CAN-Modul -> Arduino Mega 2560
// Int -> 2
// SCK -> 52
// SI  -> 51
// SO  -> 50
// CS  -> 53
// GND -> GND
// VCC -> +5V 

// CAN Params
#define NETWORK_ID (40202)
#define PORT (1)

MCP2515 mcp2515(53); // SPI CS Pin
struct can_frame canMsg;

// Weichen Nrs
#define WEICHEN_NR_FROM (8)
#define WEICHEN_NR_TO (16)

// Config                             8     9    10    11    12    13    14    15    16   
int servoPin[9]                 = {   4,    5,    6,    7,    8,    9,   10,   11,   12};
int relayPin[9]                 = {  31,   32,   33,   34,   35,   36,   37,   38,   39};
int weichenStellungGerade[9]    = {  55,   55,   50,   50,   50,   55,   55,   45,   50};
int weichenStellungAbgebogen[9] = { 125,  125,  120,  140,  130,  125,  125,  111,  125};
int relayDir[9]                 ={false, true, true, true, true, true,false,false,false};

// state
bool weichenState[9] =            {true, true, true, true, true, true, true, true, true};
Servo servo[9];

// winkel state
int actWinkelList[]  = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int zielWinkelList[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// Lokschuppen Variablen & Config
Servo servoDoorLeft;
Servo servoDoorRight;
const int doorLeftPin = 44;
const int doorRightPin = 46;
const int doorLeftClosed = 30;
const int doorRightClosed = 120;
const int doorLeftOpen = 145;
const int doorRightOpen = 5;
const int doorNid = 1201;
bool doorClosed = true;
int actWinkelDoorLeft = doorLeftClosed;
int actWinkelDoorRight = doorRightClosed;
int zielWinkelDoorLeft = doorLeftClosed;
int zielWinkelDoorRight = doorRightClosed;


unsigned long lastWinkelStep = 0;
unsigned long lastWinkelStepDoor = 0;

int getRelayDirection(int weichenNr) {
  return relayDir[weichenNr - WEICHEN_NR_FROM];
}

int getWeichenStellungAbgebogen(int weichenNr) {
  return weichenStellungAbgebogen[weichenNr - WEICHEN_NR_FROM];
}

int getWeichenStellungGerade(int weichenNr) {
  return weichenStellungGerade[weichenNr - WEICHEN_NR_FROM];
}

bool getWeichenState(int weichenNr) {
  return weichenState[weichenNr - WEICHEN_NR_FROM];
}

void setWeichenState(int weichenNr, bool stellungGerade) {
  weichenState[weichenNr - WEICHEN_NR_FROM] = stellungGerade;
}

void setActWinkel(int weichenNr, int actwinkel) {
  actWinkelList[weichenNr - WEICHEN_NR_FROM] = actwinkel;
}

int getActWinkel(int weichenNr) {
  return actWinkelList[weichenNr - WEICHEN_NR_FROM];
}

void setZielWinkel(int weichenNr, int zielwinkel) {
  zielWinkelList[weichenNr - WEICHEN_NR_FROM] = zielwinkel;
}

int getZielWinkel(int weichenNr) {
  return zielWinkelList[weichenNr - WEICHEN_NR_FROM];
}

int getServoPin(int weichenNr) {
  return servoPin[weichenNr - WEICHEN_NR_FROM];
}

int getRelayPin(int weichenNr) {
  return relayPin[weichenNr - WEICHEN_NR_FROM];
}

int getZubehoerNid(int weichenNr) {
  return weichenNr + 1000;
}

void setServoTo(int weichenNr, bool stellungGerade) {
  int zielWinkel = stellungGerade ? getWeichenStellungGerade(weichenNr) : getWeichenStellungAbgebogen(weichenNr);
  setZielWinkel(weichenNr, zielWinkel);
}

void sendCan(int weichenNr, ZCAN_MODE mode) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::ZUBEHOER;
  zcanMessage.command = 4;  // Command.PORT4
  zcanMessage.mode = mode;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = (getZubehoerNid(weichenNr)) & 0xff;
  zcanMessage.data[1] = ((getZubehoerNid(weichenNr)) >> 8);
  zcanMessage.data[2] = PORT;
  zcanMessage.data[3] = getWeichenState(weichenNr) ? 0b1 : 0b0;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void sendCanDoor(ZCAN_MODE mode) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::ZUBEHOER;
  zcanMessage.command = 4;  // Command.PORT4
  zcanMessage.mode = mode;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = ((doorNid) & 0xff);
  zcanMessage.data[1] = ((doorNid) >> 8);
  zcanMessage.data[2] = PORT;
  zcanMessage.data[3] = doorClosed ? 0b1 : 0b0;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void initWeiche(int weichenNr) {
  bool stellungGerade = getWeichenState(weichenNr);
  digitalWrite(getRelayPin(weichenNr), stellungGerade ? (getRelayDirection(weichenNr) ? LOW : HIGH) : (getRelayDirection(weichenNr) ? HIGH : LOW));
  int zielWinkel = stellungGerade ? getWeichenStellungGerade(weichenNr) : getWeichenStellungAbgebogen(weichenNr);
  setActWinkel(weichenNr, zielWinkel);
  setZielWinkel(weichenNr, zielWinkel);
  int weichenIndex = weichenNr - WEICHEN_NR_FROM;
  servo[weichenIndex].attach(servoPin[weichenIndex]);
  servo[weichenIndex].write(zielWinkel);
  delay(1000);
  servo[weichenIndex].detach();
}

void initDoors() {
  actWinkelDoorLeft = zielWinkelDoorLeft;
  actWinkelDoorRight = zielWinkelDoorRight;
  servoDoorLeft.attach(doorLeftPin);
  servoDoorLeft.write(zielWinkelDoorLeft);
  delay(1000);
  servoDoorLeft.detach();
  servoDoorRight.attach(doorRightPin);
  servoDoorRight.write(zielWinkelDoorRight);
  delay(1000);
  servoDoorRight.detach();
}

void computeCommand(int weichenNr, bool stellungGerade) {
  bool oldState = getWeichenState(weichenNr);
  if (oldState == stellungGerade) {
    sendCan(weichenNr, ZCAN_MODE::EVENT);
    return;
  }
  setWeichenState(weichenNr, stellungGerade);
  sendCan(weichenNr, ZCAN_MODE::EVENT);
  digitalWrite(getRelayPin(weichenNr), stellungGerade ? (getRelayDirection(weichenNr) ? LOW : HIGH) : (getRelayDirection(weichenNr) ? HIGH : LOW));
  setServoTo(weichenNr, stellungGerade);
}

void computeDoorCommand(bool doorClosedNew) {
  if (doorClosed == doorClosedNew) {
    sendCanDoor(ZCAN_MODE::EVENT);
    return;
  }
  doorClosed = doorClosedNew;
  sendCanDoor(ZCAN_MODE::EVENT);
  zielWinkelDoorLeft = doorClosed ? doorLeftClosed : doorLeftOpen;
  zielWinkelDoorRight = doorClosed ? doorRightClosed : doorRightOpen;
}

void setup() {
  
  for (int i = WEICHEN_NR_FROM; i <= WEICHEN_NR_TO; i++) {
    pinMode(getRelayPin(i), OUTPUT);
    // servo[i-WEICHEN_NR_FROM].attach(getServoPin(i));
  }
  
  // init CAN
  SPI.begin();
  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setConfigMode();
  mcp2515.setFilterMask(MCP2515::MASK0, true, MASK_GROUP);
  mcp2515.setFilter(MCP2515::RXF0, true, FILTER_ZUBEHOER);
  mcp2515.setNormalMode();

  // init weichenStates
  for (int i = WEICHEN_NR_FROM; i <= WEICHEN_NR_TO; i++) {
    initWeiche(i);
  }

  // init lokschuppen doors
  initDoors();
}

void loop() { 
  // poll CAN message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    zcan_message zcanMsg = toZCanMessage(canMsg);   
    
    if (zcanMsg.data[2] == PORT && zcanMsg.command == 4) {
      // print_zcanMsg(zcanMsg);  // debug print CAN
      
      uint16_t nid = zcanMsg.data[0] | zcanMsg.data[1] << 8;
      int weichenNr = 0;
      
      for (int i = WEICHEN_NR_FROM; i <= WEICHEN_NR_TO; i++) {
        if (nid == getZubehoerNid(i)) {
          weichenNr = i;
          break;
        }
      }
      if (weichenNr > 0) {
        // answere request with current state
        if (zcanMsg.mode == ZCAN_MODE::REQUEST) {
          sendCan(weichenNr, ZCAN_MODE::EVENT);
        }
        // compute command
        if (zcanMsg.mode == ZCAN_MODE::COMMAND) {
          bool stellungGerade = (zcanMsg.data[3] & 0b1) > 0;
          computeCommand(weichenNr, stellungGerade);
        }
      } else if (nid == doorNid) {
        // answere request with current state
        if (zcanMsg.mode == ZCAN_MODE::REQUEST) {
          sendCanDoor(ZCAN_MODE::EVENT);
        }
        // compute command
        if (zcanMsg.mode == ZCAN_MODE::COMMAND) {
          bool doorClosedNew = (zcanMsg.data[3] & 0b1) > 0;
          computeDoorCommand(doorClosedNew);
        }
      }
    }
  }

  // stelle Weichen
  if (millis() - lastWinkelStep >= 8) {
    lastWinkelStep = millis();
    for (int i = WEICHEN_NR_FROM; i <= WEICHEN_NR_TO; i++) {
      int actWinkel = getActWinkel(i);
      int zielWinkel = getZielWinkel(i);
      int weicheIndex = i - WEICHEN_NR_FROM;
      if (actWinkel != zielWinkel) {
        int inc = ((zielWinkel - actWinkel) > 0) ? 1 : -1;
        setActWinkel(i, actWinkel + inc);
        if (!servo[weicheIndex].attached()) {
          servo[weicheIndex].attach(servoPin[weicheIndex]);
        }
        servo[weicheIndex].write(actWinkel + inc);
        if (zielWinkel == actWinkel + inc) {
          servo[weicheIndex].detach();
        }
        return;
      }
    }
  }

  // stelle Door
  if (millis() - lastWinkelStepDoor >= 16) {
    lastWinkelStepDoor = millis();
    if (actWinkelDoorLeft != zielWinkelDoorLeft) {
      int inc = ((zielWinkelDoorLeft - actWinkelDoorLeft) > 0) ? 1 : -1;
      actWinkelDoorLeft = actWinkelDoorLeft + inc;
      if (!servoDoorLeft.attached()) {
        servoDoorLeft.attach(doorLeftPin);
      }
      servoDoorLeft.write(actWinkelDoorLeft);
      if (zielWinkelDoorLeft == actWinkelDoorLeft) {
        servoDoorLeft.detach();
      }
    }
    if (actWinkelDoorRight != zielWinkelDoorRight) {
      int inc = ((zielWinkelDoorRight - actWinkelDoorRight) > 0) ? 1 : -1;
      actWinkelDoorRight = actWinkelDoorRight + inc;
      if (!servoDoorRight.attached()) {
        servoDoorRight.attach(doorRightPin);
      }
      servoDoorRight.write(actWinkelDoorRight);
      if (zielWinkelDoorRight == actWinkelDoorRight) {
        servoDoorRight.detach();
      }
    }    
  }
}
