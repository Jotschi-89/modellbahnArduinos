#include <SPI.h>
#include <TimerOne.h>
#include <mcp2515.h>
#include <zcan.h>
#include <EEPROM.h>
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
#define MAX_SIMULTANEOUS_SERVOS (2)
#define MAX_ARTIFACTS (25)

MCP2515 mcp2515(53); // SPI CS Pin
struct can_frame canMsg;

// EEPROM
#define EEPROM_INIT_ADDRESS (0)
#define EEPROM_CALIBRATION_VALUES_ADDRESS (1)

enum SETTINGS_TYPE {
  DELETE = 1,
  SAVE_ALL = 2,
  PAGE_1 = 3,
  PAGE_2 = 4
};

// settings values
typedef struct {
  uint16_t zubehoerNid = 0;
  byte relayPinWechsler = 0;     // used at Weichen for Herzstueckpolarisierung
  bool relayPinWechslerFlipped = false;  
  byte relayPinSchlieser = 0;    // used at Weichen for Servo Strom Shutoff if servoPin is set, otherwise just closed on stateON = true
  byte servoPin = 0;             // Number Mappings: PINs 1-28 29➔44 30➔45 31➔46 
  byte millisPerDegree = 8;      // servo Speed: formular speed[0-7]: 2^(7-speed) => millisPerDegree
  byte servoPositionA = 0;       // in Grad, 0-255
  byte servoPositionB = 0;       // in Grad, 0-255
}__attribute__((packed)) ZubehoerSetting;
ZubehoerSetting settings[MAX_ARTIFACTS];

// live working variables
typedef struct {
  bool stateON = false;
  int actWinkel = 0;
  int zielWinkel = 0;
  unsigned long lastWinkelStep = 0;
}__attribute__((packed)) ZuebehoerState;
ZuebehoerState state[MAX_ARTIFACTS];
Servo servo[MAX_ARTIFACTS];
byte attachedServosCount = 0;


void loadEEPROMState() {
  // todo:
  if (EEPROM.read(EEPROM_INIT_ADDRESS) != 255) {
    // EEPROM not yet initialized -> save initial state
    EEPROM.put(EEPROM_CALIBRATION_VALUES_ADDRESS, settings);
    EEPROM.write(EEPROM_INIT_ADDRESS, 255);
  }
  EEPROM.get(EEPROM_CALIBRATION_VALUES_ADDRESS, settings);
}


void saveEEPROMState() {
  EEPROM.put(EEPROM_CALIBRATION_VALUES_ADDRESS, settings);
}


bool initZuebehoerState(uint16_t zubehoerNid) {
  // find settings
  byte i = MAX_ARTIFACTS;
  for (int j = 0; j < MAX_ARTIFACTS; j++) {
    if (settings[j].zubehoerNid == zubehoerNid) {
      i = j;
      break;
    }
  }
  if (i >= MAX_ARTIFACTS) {
    // settings unkown
    return false;
  }
  state[i].stateON = false;
  state[i].actWinkel = 0;
  state[i].zielWinkel = 0;  
  state[i].lastWinkelStep = 0;  

  // init Wechsler
  if (settings[i].relayPinWechsler != 0) {
    pinMode(settings[i].relayPinWechsler, OUTPUT);
    digitalWrite(settings[i].relayPinWechsler, state[i].stateON ? (settings[i].relayPinWechslerFlipped ? LOW : HIGH) : (settings[i].relayPinWechslerFlipped ? HIGH : LOW));
  }
  // init Servo
  if (settings[i].servoPin > 0) {
    state[i].zielWinkel = state[i].stateON ? settings[i].servoPositionA : settings[i].servoPositionB;
    state[i].actWinkel = state[i].zielWinkel;
    if (settings[i].relayPinSchlieser > 0) {
      pinMode(settings[i].relayPinSchlieser, OUTPUT);
      digitalWrite(settings[i].relayPinSchlieser, HIGH);
    }
    servo[i].attach(settings[i].servoPin);
    servo[i].write(state[i].zielWinkel);
    delay(1000);
    if (settings[i].relayPinSchlieser > 0) {
      digitalWrite(settings[i].relayPinSchlieser, LOW);
    }    
    servo[i].detach();
  }
  // init Schlieser (when not used for servo)
  if (settings[i].relayPinSchlieser > 0 && settings[i].servoPin == 0) {
    pinMode(settings[i].relayPinSchlieser, OUTPUT);
    digitalWrite(settings[i].relayPinSchlieser, state[i].stateON ? HIGH : LOW);
  }
}


void updateZuebehoerSettingsPage1(byte i, byte relayPinWechsler, bool relayPinWechslerFlipped, byte relayPinSchlieser) {
  bool changed = settings[i].relayPinWechsler != relayPinWechsler || 
                 settings[i].relayPinWechslerFlipped != relayPinWechslerFlipped || 
                 settings[i].relayPinSchlieser != relayPinSchlieser;
  settings[i].relayPinWechsler = relayPinWechsler;
  settings[i].relayPinWechslerFlipped = relayPinWechslerFlipped;
  settings[i].relayPinSchlieser = relayPinSchlieser;
  if (changed) {
    initZuebehoerState(settings[i].zubehoerNid); 
  }
}


void updateZuebehoerSettingsPage2(byte i, byte servoPin, byte millisPerDegree, byte servoPositionA, byte servoPositionB) {
  bool servoPinChanged = settings[i].servoPin != servoPin;
  settings[i].servoPin = servoPin;
  settings[i].millisPerDegree = millisPerDegree;
  settings[i].servoPositionA = servoPositionA;
  settings[i].servoPositionB = servoPositionB;
  if (servoPinChanged) {
    initZuebehoerState(settings[i].zubehoerNid);
  } else if (settings[i].servoPin > 0) {
    // act on Servo
      state[i].zielWinkel = state[i].stateON ? settings[i].servoPositionA : settings[i].servoPositionB;
    } 
  }
}

byte createZuebehoerSettingsPage1(uint16_t nid, byte relayPinWechsler, byte relayPinWechslerFlipped, byte relayPinSchlieser) {
  // find free space in settings
  byte i = MAX_ARTIFACTS;
  for (int j = 0; j < MAX_ARTIFACTS; j++) {
    if (settings[j].zubehoerNid == 0) {
      i = j;
      break;
    }
  }
  if (i >= MAX_ARTIFACTS) {
    // no free space
    return MAX_ARTIFACTS;
  }
  settings[i].zubehoerNid = nid;
  settings[i].relayPinWechsler = relayPinWechsler;
  settings[i].relayPinWechslerFlipped = relayPinWechslerFlipped;
  settings[i].relayPinSchlieser = relayPinSchlieser;
  settings[i].servoPin = 0;
  settings[i].millisPerDegree = 0;
  settings[i].servoPositionA = 0;
  settings[i].servoPositionB = 0;  
  initZuebehoerState(nid);
  return i;
}


byte createZuebehoerSettingsPage2(uint16_t nid, byte servoPin, byte millisPerDegree, byte servoPositionA, byte servoPositionB) {
  // find free space in settings
  byte i = MAX_ARTIFACTS;
  for (int j = 0; j < MAX_ARTIFACTS; j++) {
    if (settings[j].zubehoerNid == 0) {
      i = j;
      break;
    }
  }
  if (i >= MAX_ARTIFACTS) {
    // no free space
    return MAX_ARTIFACTS;
  }
  settings[i].zubehoerNid = nid;
  settings[i].relayPinWechsler = 0;
  settings[i].relayPinWechslerFlipped = false;
  settings[i].relayPinSchlieser = 0;
  settings[i].servoPin = servoPin;
  settings[i].millisPerDegree = millisPerDegree;
  settings[i].servoPositionA = servoPositionA;
  settings[i].servoPositionB = servoPositionB;  
  initZuebehoerState(nid);
  return i;
}

void deleteZuebehoerSettings(byte i) {
  if (state[i].actWinkel != state[i].zielWinkel) {
    return;
  }
  settings[i].zubehoerNid = 0;
  settings[i].relayPinWechsler = 0;
  settings[i].relayPinWechslerFlipped = false;
  settings[i].relayPinSchlieser = 0;
  settings[i].servoPin = 0;
  settings[i].millisPerDegree = 0;
  settings[i].servoPositionA = 0;
  settings[i].servoPositionB = 0;  
  state[i].stateON = false;
  state[i].actWinkel = 0;
  state[i].zielWinkel = 0;
  state[i].lastWinkelStep = 0;
}

byte servoCanNrToPin(byte canNr) {
  if (canNr == 29) {
    return 44;
  } else if (canNr == 30) {
    return 45;
  } else if (canNr == 31) {
    return 46;
  } else {
    return canNr;
  }
}

byte servoPinToCanNr(byte pin) {
  if (pin == 44) {
    return 29;
  } else if (pin == 45) {
    return 30;
  } else if (pin == 46) {
    return 31;
  } else {
    return pin;
  }
}

byte speedToMillisPerDegree(byte speed) {
  if (speed >= 7) {
    return 1;
  } else if (speed == 6) {
    return 2;
  } else if (speed == 5) {
    return 4;
  } else if (speed == 4) {
    return 8;
  } else if (speed == 3) {
    return 16;
  } else if (speed == 2) {
    return 32;
  } else if (speed == 1) {
    return 64;
  } else if (speed == 0) {
    return 128;
  }
  return 1;
}

byte millisPerDegreeToSpeed(byte millisPerDegree) {
  if (millisPerDegree == 128) {
    return 0;
  } else if (millisPerDegree == 64) {
    return 1;
  } else if (millisPerDegree == 32) {
    return 2;
  } else if (millisPerDegree == 16) {
    return 3;
  } else if (millisPerDegree == 8) {
    return 4;
  } else if (millisPerDegree == 4) {
    return 5;
  } else if (millisPerDegree == 2) {
    return 6;
  } else {
    return 7;
  }
}

void sendZubehoerCan(byte i, ZCAN_MODE mode) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::ZUBEHOER;
  zcanMessage.command = 4;  // Command.PORT4
  zcanMessage.mode = mode;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.dataLength = 4;
  zcanMessage.data[0] = settings[i].zubehoerNid & 0xff;
  zcanMessage.data[1] = settings[i].zubehoerNid >> 8;
  zcanMessage.data[2] = PORT;
  zcanMessage.data[3] = state[i].stateON ? 0b1 : 0b0;
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}


void computeZubehoerCommand(byte i, bool stateON) {
  if (state[i].stateON == stateON) {
    sendZubehoerCan(i, ZCAN_MODE::EVENT);
    return;
  }
  state[i].stateON = stateON;
  sendZubehoerCan(i, ZCAN_MODE::EVENT);
  // act on Wechsler
  if (settings[i].relayPinWechsler > 0) { 
    digitalWrite(settings[i].relayPinWechsler, state[i].stateON ? (settings[i].relayPinWechslerFlipped ? LOW : HIGH) : (settings[i].relayPinWechslerFlipped ? HIGH : LOW));
  }
  // act on Servo
  if (settings[i].servoPin > 0) {
    state[i].zielWinkel = state[i].stateON ? settings[i].servoPositionA : settings[i].servoPositionB;
  } 
  // act on Schlieser (when not used for servo)
  if (settings[i].relayPinSchlieser > 0 && settings[i].servoPin == 0) {
    digitalWrite(settings[i].relayPinSchlieser, state[i].stateON ? HIGH : LOW);
  }
}

void sendSettingsCan(byte i, byte type) {
  struct zcan_message zcanMessage;
  zcanMessage.group = ZCAN_GROUP::FREE;
  zcanMessage.command = 1;  // Command.ZUBEHOER_SETTING
  zcanMessage.mode = ZCAN_MODE::ACKNOWLEDGE;
  zcanMessage.networkId = NETWORK_ID;
  zcanMessage.data[0] = NETWORK_ID & 0xff;
  zcanMessage.data[1] = NETWORK_ID >> 8;
  zcanMessage.data[2] = type;
  if (type == SETTINGS_TYPE::SAVE_ALL) { 
    zcanMessage.dataLength = 3;
  } else {  
    zcanMessage.data[3] = settings[i].zubehoerNid & 0xff;
    zcanMessage.data[4] = settings[i].zubehoerNid >> 8;
    if (type == SETTINGS_TYPE::DELETE) { 
      zcanMessage.dataLength = 5;
    } else if (type == SETTINGS_TYPE::PAGE_1) { // PAGE_1
      zcanMessage.dataLength = 8;
      zcanMessage.data[5] = settings[i].relayPinWechsler;
      zcanMessage.data[6] = settings[i].relayPinWechslerFlipped;
      zcanMessage.data[7] = settings[i].relayPinSchlieser;
    } else if (type == SETTINGS_TYPE::PAGE_2) { // PAGE_2
      zcanMessage.dataLength = 8;
      zcanMessage.data[5] = millisPerDegreeToSpeed(settings[i].millisPerDegree) * 32 + servoPinToCanNr(settings[i].servoPin);
      zcanMessage.data[6] = settings[i].servoPositionA;
      zcanMessage.data[7] = settings[i].servoPositionB;
    }
  }
  mcp2515.sendMessage(&toCanFrame(zcanMessage));
}



void setup() {
  for (int j = 0; j < MAX_ARTIFACTS; j++) {
    settings[j].zubehoerNid = 0;
  }

  // load EEPROM State
  loadEEPROMState();

  // init loaded settings states
  for (int i = 0; i < MAX_ARTIFACTS; i++) {
    if (settings[i].zubehoerNid > 0) {
      initZuebehoerState(settings[i].zubehoerNid);
    }
  }
  
  // init CAN
  SPI.begin();
  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setConfigMode();
  // Note: Mask0 has 2 filters (0 and 1). Mask 1 has four filters (2, 3, 4, and 5).
  mcp2515.setFilterMask(MCP2515::MASK0, true, MASK_GROUP);
  mcp2515.setFilter(MCP2515::RXF0, true, FILTER_ZUBEHOER);
  mcp2515.setFilter(MCP2515::RXF1, true, FILTER_FREE);
  mcp2515.setNormalMode();
}


void loop() { 
  // poll CAN message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    zcan_message zcanMsg = toZCanMessage(canMsg);   
    
    // ZCanZubehoerMessage with Command == PORT4 (4) -> Schaltbefehl
    if (zcanMsg.group == ZCAN_GROUP::ZUBEHOER && zcanMsg.data[2] == PORT && zcanMsg.command == 4) {
      uint16_t nid = zcanMsg.data[0] | zcanMsg.data[1] << 8;
      byte i = MAX_ARTIFACTS;
      // find nid in settings
      for (int j = 0; j < MAX_ARTIFACTS; j++) {
        if (settings[j].zubehoerNid == nid) {
          i = j;
          break;
        }
      }
      if (i < MAX_ARTIFACTS) {
        // answere request with current state
        if (zcanMsg.mode == ZCAN_MODE::REQUEST) {
          sendZubehoerCan(i, ZCAN_MODE::EVENT);
        }
        // compute command
        if (zcanMsg.mode == ZCAN_MODE::COMMAND) {
          bool stateON = (zcanMsg.data[3] & 0b1) > 0;
          computeZubehoerCommand(i, stateON);
        }
      }
    }
    // ZCanFreeZubehoerSettingMessage with Command == ZUBEHOER_SETTING (1)
    else if (zcanMsg.group == ZCAN_GROUP::FREE && zcanMsg.command == 1) {
      // check if it's this device that's adressed by the settings message
      uint16_t canDeviceNetworkId = zcanMsg.data[0] | zcanMsg.data[1] << 8;
      if (canDeviceNetworkId == NETWORK_ID) {
        byte type = zcanMsg.data[2];
        if (type == SETTINGS_TYPE::SAVE_ALL && zcanMsg.mode == ZCAN_MODE::COMMAND) {
          // SAVE_ALL does not have a nid
          saveEEPROMState();
          sendSettingsCan(0, type);
        } else {
          // all other commands have a ZubehoerNid
          uint16_t nid = zcanMsg.data[3] | zcanMsg.data[4] << 8;
          byte i = MAX_ARTIFACTS;
          // find nid in settings
          for (int j = 0; j < MAX_ARTIFACTS; j++) {
            if (settings[j].zubehoerNid == nid) {
              i = j;
              break;
            }
          }
          if (type == SETTINGS_TYPE::DELETE && zcanMsg.mode == ZCAN_MODE::COMMAND && i < MAX_ARTIFACTS) {
            deleteZuebehoerSettings(i);
            sendSettingsCan(i, type);
          }  else if ((type == SETTINGS_TYPE::PAGE_1 || type == SETTINGS_TYPE::PAGE_2) && zcanMsg.mode == ZCAN_MODE::REQUEST && i < MAX_ARTIFACTS) {
            sendSettingsCan(i, type);
          } else if (type == SETTINGS_TYPE::PAGE_1 && zcanMsg.mode == ZCAN_MODE::COMMAND) {
            byte relayPinWechsler = zcanMsg.data[5];
            bool relayPinWechslerFlipped = zcanMsg.data[6];
            byte relayPinSchlieser = zcanMsg.data[7];            
            if (i < MAX_ARTIFACTS) {
              updateZuebehoerSettingsPage1(i, relayPinWechsler, relayPinWechslerFlipped, relayPinSchlieser);
              sendSettingsCan(i, type);
            } else {
              i = createZuebehoerSettingsPage1(nid, relayPinWechsler, relayPinWechslerFlipped, relayPinSchlieser);
              if (i < MAX_ARTIFACTS) {
                sendSettingsCan(i, type);
              }
            }
          } else if (type == SETTINGS_TYPE::PAGE_2 && zcanMsg.mode == ZCAN_MODE::COMMAND) {
            byte servoPin = servoCanNrToPin(zcanMsg.data[5] % 32);
            byte millisPerDegree = speedToMillisPerDegree(zcanMsg.data[5] >> 5);
            byte servoPositionA = zcanMsg.data[6];
            byte servoPositionB = zcanMsg.data[7]; 
            if (i < MAX_ARTIFACTS) {
              updateZuebehoerSettingsPage2(i, servoPin, millisPerDegree, servoPositionA, servoPositionB);
              sendSettingsCan(i, type);
            } else {
              i = createZuebehoerSettingsPage2(nid, servoPin, millisPerDegree, servoPositionA, servoPositionB);
              if (i < MAX_ARTIFACTS) {
                sendSettingsCan(i, type);
              }
            }
          }
        }
      }
    }
  }

  // do a servo step
  for (int i = 0; i < MAX_ARTIFACTS; i++) {
    if (state[i].actWinkel != state[i].zielWinkel && 
        millis() - state[i].lastWinkelStep >= settings[i].millisPerDegree) {
      state[i].lastWinkelStep = millis();
      // inc act Winkel
      int inc = ((state[i].zielWinkel - state[i].actWinkel) > 0) ? 1 : -1;
      state[i].actWinkel = state[i].actWinkel + inc;
      // if servo was not used before attach it
      if (!servo[i].attached()) {
        if (attachedServosCount < MAX_SIMULTANEOUS_SERVOS) {
          attachedServosCount++;
          if (settings[i].relayPinSchlieser > 0) {
            digitalWrite(settings[i].relayPinSchlieser, HIGH);
          }
          servo[i].attach(settings[i].servoPin);
        } else {
          // skip this servo until another servo gets detached
          continue;
        }
      }
      // move servo
      servo[i].write(state[i].actWinkel);
      // if servo arrived shut it down
      if (state[i].actWinkel == state[i].zielWinkel) {
        attachedServosCount--;
        state[i].lastWinkelStep = 0;
        servo[i].detach();
        if (settings[i].relayPinSchlieser > 0) {
          digitalWrite(settings[i].relayPinSchlieser, LOW);
        }  
      }
    }
  }
}
