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
#define NETWORK_ID (40204)
#define PORT (1)

MCP2515 mcp2515(53); // SPI CS Pin
struct can_frame canMsg;

// Weichen Nrs
#define WEICHEN_NR_FROM (21)
#define WEICHEN_NR_TO (29)

// Config                            21    22    23    24    25    26    27    28    29
int servoPin[9]                 = {   3,    4,    5,    6,    7,    8,    9,   10,   11};
int relayPin[9]                 = {  30,   31,   32,   33,   34,   35,   36,   37,   38};
int weichenStellungGerade[9]    = {  60,   60,   40,   60,   60,   30,   95,  135,   40};
int weichenStellungAbgebogen[9] = { 120,  120,  110,  120,  120,   90,   50,   45,   98};
int relayDir[9]                 = {true, true, true, true, true,false, true, true, false};
// state
bool weichenState[9] =            {true, true, true,false,false,false, true, true, true};
Servo servo[9];

// winkel state
int actWinkelList[]  = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int zielWinkelList[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

unsigned long lastWinkelStep = 0;

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
  zcanMessage.data[0] = (getZubehoerNid(weichenNr)) & 0xff;;
  zcanMessage.data[1] = ((getZubehoerNid(weichenNr)) >> 8);
  zcanMessage.data[2] = PORT;
  zcanMessage.data[3] = getWeichenState(weichenNr) ? 0b1 : 0b0;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void initWeiche(int weichenNr) {
  bool stellungGerade = getWeichenState(weichenNr);
  digitalWrite(getRelayPin(weichenNr), stellungGerade ? (getRelayDirection(weichenNr) ? LOW : HIGH) : (getRelayDirection(weichenNr) ? HIGH : LOW));
  int zielWinkel = stellungGerade ? getWeichenStellungGerade(weichenNr) : getWeichenStellungAbgebogen(weichenNr);
  setActWinkel(weichenNr, zielWinkel);
  setZielWinkel(weichenNr, zielWinkel);
  servo[weichenNr - WEICHEN_NR_FROM].write(zielWinkel);
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

void setup() {
  // Serial.begin(9600); 
  
  // init Relays
  // init servos
  for (int i = WEICHEN_NR_FROM; i <= WEICHEN_NR_TO; i++) {
    pinMode(getRelayPin(i), OUTPUT);
    servo[i-WEICHEN_NR_FROM].attach(getServoPin(i));
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
      }
    }
  }

  // stelle Weichen
  if (millis() - lastWinkelStep >= 12) {
    lastWinkelStep = millis();
    for (int i = WEICHEN_NR_FROM; i <= WEICHEN_NR_TO; i++) {
      int actWinkel = getActWinkel(i);
      int zielWinkel = getZielWinkel(i);
      if (actWinkel != zielWinkel) {
        int inc = ((zielWinkel - actWinkel) > 0) ? 1 : -1;
        setActWinkel(i, actWinkel + inc);
        servo[i - WEICHEN_NR_FROM].write(actWinkel + inc);
      }
    }
  }
}
