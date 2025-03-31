#include <SPI.h>
#include <mcp2515.h>
#include <zcan.h>
#include <TPIC6B595.h>

// PIN Belegung 

// Shift Register Wiring:
// the serial out pin (18) is connected to
// the serial in pin  (3) of the next chip.
#define SER_IN (3)  // SER IN - serial input 
#define SRCK   (4)  // SRCK   - shift register clock 
#define RCK    (5)  // RCK    - register clock 
#define SRCLR  (6)  // SRCLR  - shift register clear
TPIC6B595 shiftRegister(SER_IN, SRCK, RCK, SRCLR);

// CAN-Module -> Arduino NANO
// Int -> 2
// SCK -> 13
// SI  -> 11
// SO  -> 12
// CS  -> 10
// GND -> GND
// VCC -> +5V 
#define NETWORK_ID (40206)
#define PORT (1)

MCP2515 mcp2515(10); // SPI CS Pin
struct can_frame canMsg;

// config
#define SIGNAL_COUNT (7)
#define REGISTER_COUNT (4)
// 1 data byte, bits: [HALT (Rot oder Weiße schräg), HALT2 (weiße schräg auf Hauptsignal), FREI (grün rechts oben oder weiße horizontal), FREI2 (rechts unten), GELB, LEER, LEER, LEER]
int signalNIDs[SIGNAL_COUNT]       = {       1306,       1307,       1308,       1309,       1310,       1311,       1312};
byte signalState[SIGNAL_COUNT]     = { 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001};
byte usedBitsMask[SIGNAL_COUNT]    = { 0b00011101, 0b00011101, 0b00011101, 0b00011101, 0b00011101, 0b00011101, 0b00011101};
byte registerState[REGISTER_COUNT] = {0, 0, 0, 0};

int getSignalIndex(int signalNID) {
  for (int i = 0; i < SIGNAL_COUNT; i++) {
    if (signalNIDs[i] == signalNID) {
      return i;
    }
  }
  return -1;
}

byte getSignalState(int signalNID) {
  return signalState[getSignalIndex(signalNID)];
}

void setSignalState(int signalNID, byte state) {
  signalState[getSignalIndex(signalNID)] = state;
}

void sendCan(ZCAN_MODE mode, int signalNID) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::ZUBEHOER;
  zcanMessage.command = 4;  // Command.PORT4
  zcanMessage.mode = mode;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = (signalNID) & 0xff;
  zcanMessage.data[1] = (signalNID) >> 8;
  zcanMessage.data[2] = PORT;
  zcanMessage.data[3] = getSignalState(signalNID);
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}

void updateRegisterState() {
  int registerIndex = 0;
  int registerBitNr = 0;
  registerState[registerIndex] = 0;
  for (int signalIndex = 0; signalIndex < SIGNAL_COUNT; signalIndex++) {
    // per signal byte
    byte signalStateByte = signalState[signalIndex];
    for (int j = 0; j < usedBits[signalIndex]; j++) {
      // per significant bit
      byte nextBit = signalStateByte % 2;
      signalStateByte = signalStateByte >> 1;
      registerState[registerIndex] = registerState[registerIndex] + nextBit*128;
      registerBitNr = registerBitNr + 1;
      if (registerBitNr == 8) {
        registerBitNr = 0;
        registerIndex = registerIndex + 1;
        registerState[registerIndex] = 0;
      } else {
        registerState[registerIndex] = registerState[registerIndex] >> 1;
      }
    }
  }
}

void updateShiftRegister() {
  updateRegisterState();
  for (int i = 0; i < REGISTER_COUNT; i++) {
    shiftRegister.write(registerState[i]);
  }
}

void computeCommand(int signalNID, byte newState) {
  byte oldState = getSignalState(signalNID);
  if (oldState == newState) {
    sendCan(ZCAN_MODE::EVENT, signalNID);
    return;
  }
  setSignalState(signalNID, newState);
  updateShiftRegister();
  sendCan(ZCAN_MODE::EVENT, signalNID);
}

void setup() {
  SPI.begin();
  // Serial.begin(115200);

  shiftRegister.begin();  
  shiftRegister.clear();

  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setConfigMode();
  // MASK0 -> RXF0, RXF1; MASK1 -> RXF2, RXF3, RXF4, RXF5
  mcp2515.setFilterMask(MCP2515::MASK0, true, MASK_GROUP);
  mcp2515.setFilter(MCP2515::RXF0, true, FILTER_ZUBEHOER);
  mcp2515.setNormalMode(); 

  updateShiftRegister();
}

void loop(){
  // poll CAN message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    zcan_message zcanMsg = toZCanMessage(canMsg);   
    uint16_t nid = zcanMsg.data[0] | zcanMsg.data[1] << 8;
    int signalNID = nid;
    int signalIndex = getSignalIndex(signalNID);
    if (signalIndex >= 0) {
      //print_zcanMsg(zcanMsg);  // debug print CAN
      if (zcanMsg.mode == REQUEST) {
        sendCan(ZCAN_MODE::EVENT, signalNID);
      }
      if (zcanMsg.mode == COMMAND) {
        computeCommand(signalNID, zcanMsg.data[3]);
      }
    }
  }

}
