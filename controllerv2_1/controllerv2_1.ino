#include <Arduino.h>
#include <U8g2lib.h>
#include "Adafruit_FRAM_SPI.h"
#include <MIDI.h>

#include <Adafruit_MCP23X08.h>

#define REF_BLUE 6
#define REF_GREEN 7
#define REF_STATUS 8

#define I2C_RESET 12

Adafruit_MCP23X08 mcp_matrix1;
Adafruit_MCP23X08 mcp_matrix2;
Adafruit_MCP23X08 mcp_matrix3;
Adafruit_MCP23X08 mcp_status1;
Adafruit_MCP23X08 mcp_status2;
Adafruit_MCP23X08 mcp_status3;

#define INT_MATRIX 32
#define INT_STATUS 42

#include "linkedList.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

bool serialOn = true;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiInst);

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

void SerialMuted(char * text) {
  if(serialOn)
    Serial.print(text);
}

void SerialMuted(int text) {
  if(serialOn)
    Serial.print(text);
}

void SerialMuted(float text) {
  if(serialOn)
    Serial.print(text);
}

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

void printFreeMemory() {
  SerialMuted("Memory: ");
  SerialMuted(freeMemory());
  SerialMuted("\n");
}

U8G2_ST7565_NHD_C12864_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ 4, /* dc=*/ 3, /* reset=*/ 2);
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(10);
uint8_t addrSizeInBytes = 3;

int loopDelay = 1;
int debounceDelay = 10;
int tapTempoDelay = 4;
int longPressDelay = 1000 / loopDelay;
int menuClockDelay = 350 / loopDelay;
int menuClockDelayFast = 150 / loopDelay;
int storeDelay = 1000;
int exprDelay = 50/loopDelay;
int readMatrixDelay = 100/loopDelay;
int readMatrixCount = 0;
bool readMatrixFlag = false;

int exprCount = 0;

const int noBanks = 10;
const int noPresets = 10;

int maxNoPresets = 20;
int bank = 1;
int presetBank = 10;
int preset = 10;

uint8_t midiInChannel = 1;
uint8_t midiBankCtrl = 1;
bool midiInOn = true;
bool midiOutOn = true;
bool midiThroughOn = true;

linkedList * presetList = newList();

const int noMidiMsg = 16;
const int noTapTempo = 3;
const int noLoops = 12;
const int noOuts = 3;
const int noMixers = 3;
const int noCtrl = 4;
const int noExpr = 2;

uint16_t exprVals[noExpr];
int exprDelta = 1;
int exprMax = pow(2, 8) - 1;
int exprPins[] = {A0, A1};

char header[25];
bool phaseReverse[noLoops + noOuts - 1];
int stereoLoops[noLoops/2];

const uint32_t stereoLoopsAddr = 0;
const uint32_t phaseReverseAddr = sizeof(stereoLoops);
const uint32_t bankAddr = phaseReverseAddr + sizeof(phaseReverse);
const uint32_t midiInChannelAddr = bankAddr + sizeof(bank);
const uint32_t midiInOnAddr = midiInChannelAddr + sizeof(midiInChannel);
const uint32_t midiOutOnAddr = midiInOnAddr + sizeof(midiInOn);
const uint32_t midiThroughOnAddr = midiOutOnAddr + sizeof(midiOutOn);
const uint32_t midiBankCtrlAddr = midiThroughOnAddr + sizeof(midiOutOn);

const uint32_t presetStartAddr = 1024;

void saveMidiBankCtrl() {
  fram.writeEnable(true);
  fram.write(midiBankCtrlAddr, (uint8_t*)&midiBankCtrl, sizeof(midiBankCtrl));
  fram.writeEnable(false);
}

void readMidiBankCtrl() {
  fram.read(midiBankCtrlAddr, (uint8_t*)&midiBankCtrl, sizeof(midiBankCtrl));
}

void saveMidiInChannel() {
  fram.writeEnable(true);
  fram.write(midiInChannelAddr, (uint8_t*)&midiInChannel, sizeof(midiInChannel));
  fram.writeEnable(false);
}

void readMidiInChannel() {
  fram.read(midiInChannelAddr, (uint8_t*)&midiInChannel, sizeof(midiInChannel));
}

void saveMidiInOn() {
  fram.writeEnable(true);
  fram.write(midiInOnAddr, (uint8_t*)&midiInOn, sizeof(midiInOn));
  fram.writeEnable(false);
}

void readMidiInOn() {
  fram.read(midiInOnAddr, (uint8_t*)&midiInOn, sizeof(midiInOn));
}

void saveMidiOutOn() {
  fram.writeEnable(true);
  fram.write(midiOutOnAddr, (uint8_t*)&midiOutOn, sizeof(midiOutOn));
  fram.writeEnable(false);
}

void readMidiOutOn() {
  fram.read(midiOutOnAddr, (uint8_t*)&midiOutOn, sizeof(midiOutOn));
}

void saveMidiThroughOn() {
  fram.writeEnable(true);
  fram.write(midiThroughOnAddr, (uint8_t*)&midiThroughOn, sizeof(midiThroughOn));
  fram.writeEnable(false);
}

void readMidiThroughOn() {
  fram.read(midiThroughOnAddr, (uint8_t*)&midiThroughOn, sizeof(midiThroughOn));
}

void saveStereoLoops() {
  fram.writeEnable(true);
  fram.write(stereoLoopsAddr, (uint8_t*)&stereoLoops, sizeof(stereoLoops));
  fram.writeEnable(false);
}

void readStereoLoops() {
  fram.read(stereoLoopsAddr, (uint8_t*)&stereoLoops, sizeof(stereoLoops));
}

void savePhaseReverse() {
  fram.writeEnable(true);
  fram.write(phaseReverseAddr, (uint8_t*)&phaseReverse, sizeof(phaseReverse));
  fram.writeEnable(false);
}

void readPhaseReverse() {
  fram.read(phaseReverseAddr, (uint8_t*)&phaseReverse, sizeof(phaseReverse));
}

void saveBank() {
  fram.writeEnable(true);
  fram.write(bankAddr, (uint8_t*)&bank, sizeof(int));
  fram.writeEnable(false);
}

void readBank() {
  fram.read(bankAddr, (uint8_t*)&bank, sizeof(int));
}

bool programm = false;

int noMenuPins = 6;
int enterPin = 2;
int backPin = 3;
int upPin = 4;
int downPin = 5;
int leftPin = 6;
int rightPin = 7;
int menuPins[] = {enterPin, backPin, upPin, downPin, leftPin, rightPin};
bool menuMcpVals[] = {false, false, false, false, false, false};
bool menuEdges[] = {false, false, false, false, false, false};
bool menuDetect[] = {false, false, false, false, false, false};
int menuCount[] = {0, 0, 0, 0, 0, 0};
int menuLongCount[] = {0, 0, 0, 0, 0, 0};
bool menuLongPressDetect[] = {false, false, false, false, false, false};
bool menuLongPress[] = {false, false, false, false, false, false};
int menuClockCount[] = {0, 0, 0, 0, 0, 0};
bool menuClock[] = {false, false, false, false, false, false};
int menuClockFastCount[] = {0, 0, 0, 0, 0, 0};
bool menuClockFast[] = {false, false, false, false, false, false};
bool menuClockDetect[] = {false, false, false, false, false, false};
bool menuNegEdges[] = {false, false, false, false, false, false};
bool menuNegDetect[] = {false, false, false, false, false, false};
bool menuNegDetectFlag[] = {false, false, false, false, false, false};
int menuNegCount[] = {0, 0, 0, 0, 0, 0};

int noPresetPins = 10;
int presetPins[] = {22, 23, 24, 25, 27, 26, 50, 51, 52, 53};
int presetEdges[] = {false, false, false, false, false, false, false, false, false, false};
bool presetDetect[] = {false, false, false, false, false, false, false, false, false, false};
int presetCount[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool loopConnsSend[] = {false, false, false, false, false, false, false, false, false, false};
bool loopConnsReturn[] = {false, false, false, false, false, false, false, false, false, false};
bool outConns[] = {false, false};
bool exprConns[] = {false, false};


const uint8_t outIds[] = {20, 21, 22};
const uint8_t mixerIds[] = {30, 31, 32};

enum midiTypes {midiPC = 0, midiCC = 1, expr1 = 2, expr2 = 3};
enum tapTypes {tapCC = 0, tapRemote1 = 1, tapRemote2 = 2, tapRemote3 = 3, tapRemote4 = 4};
enum stompModes {offStomp = 0, normalStomp = 1, permanentStomp = 2, toggleStomp = 3, permanentBankStomp = 4};

struct preset_t {

  uint8_t bankNo;
  uint8_t presetNo;
  char presetName[20];

  bool loopsOn[noLoops];
  bool outsOn[noOuts];
  bool outPhaseReverse[noOuts-1];
  bool tunerOn;
  uint8_t loopOrder[noLoops + noOuts + noMixers][noOuts];

  bool ctrlOn[noCtrl];

  bool midiOn[noMidiMsg];
  midiTypes midiType[noMidiMsg];
  uint8_t midiChannel[noMidiMsg];
  uint8_t midiPC[noMidiMsg];
  uint8_t midiCC[noMidiMsg];
  uint8_t midiCCValue[noMidiMsg][2];

  bool tapOn[noTapTempo];
  tapTypes tapType[noTapTempo];
  uint8_t tapChannel[noTapTempo];
  uint8_t tapCC[noTapTempo];
  uint8_t tapCCOn[noTapTempo];
  uint8_t tapCCOff[noTapTempo];

  stompModes stompMode;
  
};

const uint32_t noActivatedPresetAddr = presetStartAddr + noBanks * noPresets * sizeof(preset_t);
const uint32_t activatedPresetAddr = noActivatedPresetAddr + sizeof(uint32_t);

bool checkActivePresets() {
  listItem* iter = presetList->first;
  bool hasPreset = false;
  while(iter) {
    preset_t* preset = (preset_t*) iter->item;
    if(preset->stompMode == offStomp) {
      hasPreset = true;
      break;
    }
    iter = (listItem*) iter->next;
  }
  return hasPreset;
}

void savePresetFnc(struct preset_t* presetVar, uint32_t addr) {
  fram.writeEnable(true);
  fram.write(addr, (uint8_t*) presetVar, sizeof(preset_t));
  fram.writeEnable(false);
}

void savePresetFncBankNPreset(struct preset_t* presetVar, uint8_t bank, uint8_t preset) {
  uint32_t addr = presetStartAddr + ((bank-1) * noPresets + preset-1) * sizeof(preset_t);
  savePresetFnc(presetVar, addr);
}

void readPresetFnc(struct preset_t* presetVar, uint32_t addr) {
  fram.read(addr, (uint8_t*) presetVar, sizeof(preset_t));
}

void readPresetFncBankNPreset(struct preset_t* presetVar, uint8_t bank, uint8_t preset) {
  uint32_t addr = presetStartAddr + ((bank-1) * noPresets + preset-1) * sizeof(preset_t);
  readPresetFnc(presetVar, addr);
}

void saveActivePresets() {
  uint32_t count = 0;
  listItem* iter = presetList->first;
  while(iter) {
    preset_t* preset = (preset_t*) iter->item;
    uint32_t addr = activatedPresetAddr + count * sizeof(preset_t);
    savePresetFnc(preset, addr);
    iter = (listItem*) iter->next;
    count++;
  }
  fram.writeEnable(true);
  fram.write(noActivatedPresetAddr, (uint8_t*)&count, sizeof(uint32_t));
  fram.writeEnable(false);
}

void checkExpr(int exprNo) {
  uint16_t exprVal = exprVals[exprNo];
  midiTypes exprTyp;
  switch(exprNo) {
    case 0: exprTyp = expr1; break;
    case 1: exprTyp = expr2; break;
  }
  listItem* iter = presetList->first;
  while(iter) {
    preset_t* preset = (preset_t*) iter->item;
    for(int i=0; i<noMidiMsg; i++) {
      if(preset->midiOn[i] && preset->midiType[i] == exprTyp) {
        uint8_t toeVal = preset->midiCCValue[i][1];
        uint8_t hellVal = preset->midiCCValue[i][0];
        float targetVal = hellVal + (toeVal - hellVal) / 127.0 * exprVal/exprMax * 127;
        SerialMuted("Writing Expr. CC ");
        SerialMuted(preset->midiCC[i]);
        SerialMuted(" ");
        SerialMuted(targetVal);
        SerialMuted(" ");
        SerialMuted(preset->midiChannel[i]);
        SerialMuted("\n");
        
        midiInst.sendControlChange((byte) preset->midiCC[i], (byte) targetVal, (byte) preset->midiChannel[i]);
      }
    }
    iter = (listItem*) iter->next;
  }
}

bool checkMidi = false;
bool checkExprOnLoad = false;

void hardwareActivatePreset() {
  SerialMuted("Hardware Activate Start\n");
  listItem* iter = presetList->first;
  while(iter) {
    preset_t* preset = (preset_t*) iter->item;
    SerialMuted("Bank: ");
    SerialMuted(preset->bankNo);
    SerialMuted(" Preset No: ");
    SerialMuted(preset->presetNo);
    SerialMuted(" Stomp Type: ");
    char* stompTypeCur;
    switch(preset->stompMode) {
      case offStomp: stompTypeCur = "Off"; break;
      case normalStomp: stompTypeCur = "Normal"; break;
      case permanentStomp: stompTypeCur = "Permanent"; break;
      case toggleStomp: stompTypeCur = "Toggle"; break;
      case permanentBankStomp: stompTypeCur = "Permanent Bank\n"; break;
      default: stompTypeCur = "Unset";
    }
    SerialMuted(stompTypeCur);
    SerialMuted(" Loops:");
    for(int i=0; i<noLoops; i++) {
      SerialMuted(" ");
      if(preset->loopsOn[i]) {
        SerialMuted(1);
      } else {
        SerialMuted(0);
      }
    }
    SerialMuted("\n");
    iter = (listItem*) iter->next;
  }
  if(checkMidi && midiOutOn) {
    checkMidi = false;
    preset_t* preset = (preset_t*) presetList->last->item;
    for(int i=0; i<noMidiMsg; i++) {
      if(preset->midiOn[i]) {
        if(preset->midiType[i] == midiPC) {
          SerialMuted("Midi PC send: ");
          SerialMuted(preset->midiPC[i]);
          SerialMuted(" ");
          SerialMuted(preset->midiChannel[i]);
          SerialMuted("\n");
          midiInst.sendProgramChange((byte) preset->midiPC[i], (byte) preset->midiChannel[i]);
        } else if(preset->midiType[i] == midiCC) {
          midiInst.sendControlChange((byte) preset->midiCC[i], (byte) preset->midiCCValue[i][0], (byte) preset->midiChannel[i]);
          SerialMuted("Midi CC send: ");
          SerialMuted(preset->midiCC[i]);
          SerialMuted(" ");
          SerialMuted(preset->midiChannel[i]);
          SerialMuted(" ");
          SerialMuted(preset->midiCCValue[i][0]);
          SerialMuted("\n");
        }
      }
    }
  }
  if(checkExprOnLoad) {
    for(int i=0; i<noExpr; i++) {
      checkExpr(i);
    }
  }
  SerialMuted("Hardware Activate End\n");
  printFreeMemory();
}

bool keepLast = false;

bool activatePreset(uint8_t bankNo, uint8_t presetNo) {
  preset_t* newPreset = new preset_t;
  readPresetFncBankNPreset(newPreset, bankNo, presetNo);
  bool toAppend = false;
  bool keepLastAppend = false;
  uint8_t keepBank = 0;
  uint8_t keepPreset = 0;
  bool presetAlreadyActive = false;
  if(newPreset->stompMode == offStomp) {
    int deleteCount = 0;
    preset_t* deletePresets[maxNoPresets];

    listItem* iter = presetList->first;
    while(iter) {
      if(iter == presetList->last && keepLast) {
        preset_t* keepPresetPtr = (preset_t*) iter->item;
        keepLastAppend = true;
        keepBank = keepPresetPtr->bankNo;
        keepPreset = keepPresetPtr->presetNo;
      }
      preset_t* item = (preset_t*) iter->item;
      if(item->stompMode == normalStomp || item->stompMode == toggleStomp || item->stompMode == offStomp) {
        deletePresets[deleteCount] = item;
        deleteCount++;
      } else if(item->stompMode == permanentBankStomp && item->bankNo != newPreset->bankNo) {
        deletePresets[deleteCount] = item;
        deleteCount++;
      }
      if(item->bankNo == bankNo && item->presetNo == presetNo) {
        presetAlreadyActive = true;
      }
      iter = (listItem*) iter->next;
    } 

    for(int i=0; i<deleteCount; i++) {
      deleteItem(presetList, deletePresets[i]);
      delete deletePresets[i];
    }
    toAppend = true;
  } else {
    preset_t* deletePreset = NULL;
    preset_t* activePreset;
    listItem* iter =  presetList->first;
    while(iter) {
      activePreset = (preset_t*) iter->item;
      if(activePreset->bankNo == bankNo && activePreset->presetNo == presetNo) {
        deletePreset = activePreset;
        toAppend = false;
      }
      if(iter == presetList->last && keepLast) {
        keepLastAppend = true;
        keepBank = activePreset->bankNo;
        keepPreset = activePreset->presetNo;
      }
      iter = (listItem*) iter->next;
    }
    if(deletePreset) {
      deleteItem(presetList, deletePreset);
      delete deletePreset;
    }
    else
      toAppend = true;
  }
  if(toAppend) {
    listAppend(presetList, newPreset);
    checkMidi = true;
    checkExprOnLoad = true;
  }
  else
    delete newPreset;
  keepLast = false;
  if(keepLastAppend) {
    bool checkMidiSave = checkMidi;
    activatePreset(keepBank, keepPreset);
    checkMidi = checkMidiSave;
  }
  if(listLength(presetList) >= maxNoPresets) {
    preset_t* toDelete = (preset_t*) deleteIdx(presetList, 0);
    delete toDelete;
  }
  printFreeMemory();
  hardwareActivatePreset();
  saveActivePresets();
  return presetAlreadyActive;
}

void findBackupPreset() {
  for(uint32_t i=0; i<noBanks; i++) {
    for(uint32_t j=0; j<noPresets; j++) {
      preset_t* preset = new preset_t;
      readPresetFncBankNPreset(preset, i+1, j+1);
      if(preset->stompMode == offStomp) {
        char printStr[50];
        sprintf(printStr, "Backup Bank %d Preset %d\n", i+1, j+1);
        SerialMuted(printStr);
        activatePreset(i+1, j+1);
        return;
      }
    }
  }
}

void readActivePresets() {
  uint32_t noPresetsAct;
  fram.read(noActivatedPresetAddr, (uint8_t*)&noPresetsAct, sizeof(uint32_t));
  if(noPresetsAct >= maxNoPresets || noPresetsAct < 1) {
    noPresetsAct = 0;
    clearList(presetList);
    findBackupPreset();
  }
  for(int i=0; i<noPresetsAct; i++) {
    preset_t* curPreset = new preset_t;
    uint32_t addr = activatedPresetAddr + i * sizeof(preset_t);
    readPresetFnc(curPreset, addr);
    listAppend(presetList, curPreset);
  }
}

preset_t* createDefaultPreset() {
  preset_t* defaultPreset = new preset_t;
  defaultPreset->tunerOn = false;
  for(int i = 0; i < noOuts; i++) {
    if(i == 0) {
      defaultPreset->outsOn[i] = true;
    } else {
      defaultPreset->outsOn[i] = false;
    }
    if(i < noOuts-1) {
      defaultPreset->outPhaseReverse[i] = 0;
    }
  }
  for(uint8_t i = 0; i<noLoops; i++) {
    defaultPreset->loopsOn[i] = false;
    defaultPreset->loopOrder[i][0] = i+1;
    for(uint8_t j = 1; j < noOuts; j++) {
      defaultPreset->loopOrder[i][j] = 0;
    }
  }
  for(uint8_t i = 0; i<noOuts; i++) {
    defaultPreset->loopOrder[noLoops][i] = outIds[i];
  }
  for(uint8_t i = noLoops+1; i < noLoops + noOuts + noMixers; i++) {
    for(uint8_t j = 1; j < noOuts; j++) {
      defaultPreset->loopOrder[i][j] = 0;
    }
  }
  for(uint8_t i = 0; i < noCtrl; i++) {
    defaultPreset->ctrlOn[i] = false;
  }
  for(int i=0; i<noMidiMsg; i++) {
    defaultPreset->midiOn[i] = false;
    defaultPreset->midiType[i] = midiPC;
    defaultPreset->midiChannel[i] = 1;
    defaultPreset->midiPC[i] = 0;
    defaultPreset->midiCC[i] = 1;
    defaultPreset->midiCCValue[i][0] = 0;
    defaultPreset->midiCCValue[i][1] = 127;
  }
  for(int i=0; i<noTapTempo; i++) {
    defaultPreset->tapOn[i] = false;
    defaultPreset->tapType[i] = tapCC;
    defaultPreset->tapChannel[i] = 1;
    defaultPreset->tapCC[i] = 1;
    defaultPreset->tapCCOn[i] = 127;
    defaultPreset->tapCCOff[i] = 0;
  }
  defaultPreset->stompMode = offStomp;
  strcpy(defaultPreset->presetName, "");
  return defaultPreset;
}

void resetPreset(uint8_t bank, uint8_t preset, preset_t* templ=NULL) {
  bool deletePreset = false;
  if(!templ) {
    templ = createDefaultPreset();
    deletePreset = true;
  }
  templ->bankNo = bank;
  templ->presetNo = preset;
  uint32_t presetAddr = presetStartAddr + ((bank - 1) * noPresets + (preset-1)) * sizeof(preset_t);
  savePresetFnc(templ, presetAddr);
  if(deletePreset) {
    delete templ;
  }
}

bool tapCheckOn = false;
bool tapCheckFromPreset = false;
int tapCheckCount = 0;
linkedList tapPresets;

void sendTapTempo(bool on) {
  listItem* iter = tapPresets.first;
  while(iter) {
    preset_t* tapPreset = (preset_t*) iter->item;
    for(int i=0; i<noTapTempo; i++) {
      if(tapPreset->tapOn[i]) {
        if(tapPreset->tapType[i] == tapCC) {
          uint8_t value;
          if(on) {
            value = tapPreset->tapCCOn[i];
          } else {
            value = tapPreset->tapCCOff[i];
          }
          midiInst.sendControlChange((byte) tapPreset->tapCC[i], (byte) value, (byte) tapPreset->tapChannel[i]);
        }
      }
    }
    iter = (listItem*) iter->next;
  }
}

void cancelTapTempo() {
  listItem* iter = tapPresets.first;
  for(int i=0; i<listLength(&tapPresets); i++) {
    preset_t* deletePreset = (preset_t*) deleteIdx(&tapPresets, i);
    delete deletePreset;
  }
  tapCheckOn = false;
  tapCheckCount = 0;
  tapCheckFromPreset = false;
}

void checkTapTempo(bool fromPreset=false) {
  if(tapCheckOn) {
    tapCheckCount++;
    if(tapCheckCount == tapTempoDelay) {
      sendTapTempo(false);
      cancelTapTempo();
    }
  } else {
    tapCheckOn = true;
    listItem * iter = presetList->first;
    while(iter) {
      preset_t * tapPreset = (preset_t*) iter->item;
      bool hasTapTempo = false;
      for(int i=0; i<noTapTempo; i++) {
        if(tapPreset->tapOn[i]) {
          hasTapTempo = true;
          break;
        }
      }
      if(fromPreset && tapPreset->stompMode == offStomp && hasTapTempo) {
        preset_t * cpyPreset = new preset_t;
        for(int i=0; i<sizeof(preset_t); i++) {
          *(((byte*)cpyPreset)+i) = *(((byte*)tapPreset)+i);
        }
        listAppend(&tapPresets, cpyPreset);
      }
      iter = (listItem*) iter->next;
    }
    if(listLength(&tapPresets)) {
      sendTapTempo(true);
    }
  }
}

void factoryReset() {
  clearList(presetList);
  
  bank = 1;
  saveBank();
  midiInChannel = 1;
  saveMidiInChannel();
  midiBankCtrl = 1;
  saveMidiBankCtrl();
  midiInOn = true;
  saveMidiInOn();
  midiOutOn = true;
  saveMidiOutOn();
  midiThroughOn = true;
  saveMidiThroughOn();
  midiInst.turnThruOn();
  for(int i=0; i<noLoops + noOuts - 1; i++) {
    phaseReverse[i] = false;
  }
  savePhaseReverse();
  for(int i=0; i<noLoops/2; i++)  {
    stereoLoops[i] = false;
  }
  saveStereoLoops();
  
  preset_t* defaultPreset = createDefaultPreset();;
  for(uint8_t bankNo = 0; bankNo < noBanks; bankNo++) {
    for(uint8_t presetNo=0; presetNo < noPresets; presetNo++) {
      resetPreset(bankNo+1, presetNo+1, defaultPreset);
    }
  }
  delete defaultPreset;
  activatePreset(1, 1);
}

void detectMenuEdges() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuDetect[i]) {
      if(menuMcpVals[i]) {
        menuDetect[i] = false;
        menuCount[i] = 0;
        menuLongPressDetect[i] = false;
        menuClockCount[i] = 0;
        menuClockDetect[i] = false;
        menuClock[i] = false;
        menuLongCount[i] = 0;
      } else if (menuCount[i] == debounceDelay) {
        menuEdges[i] = true;
        menuCount[i]++;
        menuLongPressDetect[i] = true;
        menuNegDetectFlag[i] = true;
      } else if(menuCount[i] < debounceDelay) {
        menuCount[i]++;
      }
    } else if(!menuMcpVals[i]) {
      menuDetect[i] = true;
    }
    if(menuLongPressDetect[i]) {
      if(menuLongCount[i] < longPressDelay) {
        menuLongCount[i]++;
      } else if(menuLongCount[i] == longPressDelay) {
        menuLongPress[i] = true;
        menuClockDetect[i] = true;
        menuNegDetectFlag[i] = false;
        menuLongCount[i]++;
      }
    }
    if(menuClockDetect[i]) {
      if(menuClockCount[i] < menuClockDelay) {
        menuClockCount[i]++;
      } else {
        menuClock[i] = true;
        menuClockCount[i] = 0;
      }
      if(menuClockFastCount[i] < menuClockDelayFast) {
        menuClockFastCount[i]++;
      } else {
        menuClockFast[i] = true;
        menuClockFastCount[i] = 0;
      }
    }
    if(menuNegDetect[i]) {
      if(!menuMcpVals[i]) {
        menuNegDetect[i] = false;
        menuNegCount[i] = 0;
      } else if (menuNegCount[i] == debounceDelay) {
        menuNegEdges[i] = true;
        menuNegCount[i]++;
      } else if(menuNegCount[i] < debounceDelay) {
        menuNegCount[i]++;
      }
    } else if(menuMcpVals[i] && menuNegDetectFlag[i]) {
      menuNegDetect[i] = true;
      menuNegDetectFlag[i] = false;
    }
  }
}

void detectPresetEdges() {
  for(int i = 0; i < noPresetPins; i++) {
    if(!digitalRead(presetPins[i]) && !presetDetect[i]) {
      presetEdges[i] = true;
      presetDetect[i] = true;
    } else if(digitalRead(presetPins[i]) && presetDetect[i]) {
      if(presetCount[i] == debounceDelay) {
        presetDetect[i] = false;
        presetCount[i] = 0;
      } else {
        presetCount[i]++;
      }
    } else if(!digitalRead(presetPins[i]) && presetDetect[i]){
      presetCount[i] = 0;
    }
  }
}

#define startup_width 119
#define startup_height 35
static const unsigned char startup_graphic[] U8X8_PROGMEM  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x80, 0x3F, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0xE0, 0xFF, 0x00, 0x00, 
  0x00, 0x00, 0xF8, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xF1, 
  0xFF, 0x01, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 
  0xF8, 0xFF, 0xFB, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0x7F, 0x00, 
  0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xF8, 
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 
  0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 
  0xFF, 0x07, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 
  0xFC, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x01, 
  0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0xF8, 
  0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFC, 0xE7, 0xFF, 0xFC, 0x07, 0x00, 
  0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFC, 0xC3, 0x7F, 
  0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 0x07, 0xFF, 0x03, 0xC0, 0x1F, 0x00, 
  0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 0x07, 0xFE, 0x03, 
  0xF8, 0xFF, 0x00, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 
  0x07, 0xFE, 0x01, 0xFC, 0xFF, 0x01, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 
  0xE1, 0x1F, 0xF8, 0x07, 0xFF, 0x01, 0xFE, 0xFF, 0x03, 0xFC, 0xC3, 0x7F, 
  0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 0xE7, 0xFF, 0x01, 0xFF, 0xFF, 0x07, 
  0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 0xE7, 0xFF, 0x80, 
  0xFF, 0xFF, 0x0F, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 
  0xE7, 0x7F, 0xC0, 0xFF, 0xFF, 0x1F, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 
  0xE1, 0x1F, 0xF8, 0xE7, 0x3F, 0xC0, 0xFF, 0xFF, 0x1F, 0xFC, 0xC3, 0x7F, 
  0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 0xE7, 0x1F, 0xE0, 0xFF, 0xFF, 0x3F, 
  0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 0xE7, 0x1F, 0xE0, 
  0x7F, 0xF0, 0x3F, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 
  0xE7, 0x3F, 0xE0, 0x3F, 0xE0, 0x3F, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 
  0xE1, 0x1F, 0xF8, 0xE7, 0x7F, 0xE0, 0x3F, 0xE0, 0x3F, 0xFC, 0xC3, 0x7F, 
  0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 0xE7, 0x7F, 0xE0, 0x3F, 0xE0, 0x3F, 
  0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xE1, 0x1F, 0xF8, 0xC7, 0x7F, 0xE0, 
  0x7F, 0xF0, 0x3F, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xF3, 0x1F, 0xF8, 
  0xC7, 0xFF, 0xE0, 0xFF, 0xFF, 0x3F, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 
  0xFF, 0x1F, 0xF8, 0x87, 0xFF, 0xC0, 0xFF, 0xFF, 0x1F, 0xFC, 0xC3, 0x7F, 
  0xF8, 0x07, 0xFE, 0xFF, 0x1F, 0xF8, 0x87, 0xFF, 0xC1, 0xFF, 0xFF, 0x1F, 
  0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xFF, 0x1F, 0xF8, 0x07, 0xFF, 0x81, 
  0xFF, 0xFF, 0x0F, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFE, 0xFF, 0x1F, 0xF8, 
  0x07, 0xFF, 0x01, 0xFF, 0xFF, 0x07, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xFC, 
  0xFF, 0x0F, 0xF8, 0x07, 0xFE, 0x03, 0xFE, 0xFF, 0x03, 0xFC, 0xC3, 0x7F, 
  0xF8, 0x07, 0xFC, 0xFF, 0x0F, 0xF8, 0x07, 0xFE, 0x03, 0xFC, 0xFF, 0x01, 
  0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xF8, 0xFF, 0x07, 0xF8, 0x07, 0xFC, 0x07, 
  0xF8, 0xFF, 0x00, 0xFC, 0xC3, 0x7F, 0xF8, 0x07, 0xE0, 0xFF, 0x01, 0xF8, 
  0x07, 0xFC, 0x07, 0xC0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

int startupDisplay = 0;
int mainDisplay = 1;
int presetProgDisplay = 2;
int presetSecProgDisplay = 3;
int bankSelectDisplay = 4;
int saveDisplay = 5;
int discardDisplay = 6;
int menuDisplay = 7;
int factoryResetDisplay = 8;
int startupPresetDisplay = 9;
int configDisplay = 10;

int displayPage = startupDisplay;
int lastDisplayPage = -1;
bool externalDisplayRefresh = false;

int menuScrollBarWidth = 5;

void intToStr0(int value, char* str, int digits=2) {
  for(int i=0; i<digits; i++) {
    int factor = 1;
    for(int j=0; j<digits-i-1; j++) 
      factor *= 10;
    /*if(i<digits-1) {*/
    str[i] = 48 + (value % (factor*10)) / factor;
    /*} else {
      str[i] = 48 + value % 10;
    }*/
  }
  str[digits] = '\0';
}

void presetBankDraw(int xCoor, int yCoor, bool centered=false, bool last=true) {
  int curPreset = 0;
  int curBank = 0;
  
  if(last && presetList->last) {
    preset_t * preset = (preset_t*) presetList->last->item;
    curBank = preset->bankNo;
    curPreset = preset->presetNo;
  } else {
    listItem* iter = presetList->first;
    while(iter) {
      preset_t * preset = (preset_t*) iter->item;
      if(preset->stompMode == offStomp) {
        curBank = preset->bankNo;
        curPreset = preset->presetNo;
        break;
      }
      iter = (listItem*) iter->next;
    }
  }
  char presetDisp[3];
  intToStr0(curPreset, presetDisp);
  char presetBankDisp[3];
  intToStr0(curBank, presetBankDisp); 
  
  int lengthPreset = u8g2.getStrWidth("Preset ");
  int lengthPresetNo = u8g2.getStrWidth(presetDisp);
  int lengthPresetBank = u8g2.getStrWidth("Bank ");
  int lengthBankNo = u8g2.getStrWidth(presetBankDisp);
  int lengthSpace = u8g2.getStrWidth(" ");
  
  if(centered) {
    xCoor = (128 - lengthPreset - lengthPresetNo - lengthSpace - lengthPresetBank - lengthBankNo) / 2;
  }
  u8g2.drawStr(xCoor, yCoor, "Preset ");
  u8g2.drawStr(xCoor+lengthPreset, yCoor, presetDisp);
  u8g2.drawStr(xCoor+lengthPreset + lengthPresetNo, yCoor, " ");
  u8g2.drawStr(xCoor + lengthPreset + lengthPresetNo + lengthSpace, yCoor, "Bank ");
  u8g2.drawStr(xCoor+lengthPreset + lengthPresetNo + lengthSpace + lengthPresetBank, yCoor, presetBankDisp);
}

struct state {
  char * name;
  void (*activate) ();
  void (*deactivate) ();
  char * (*transitions) ();
  bool isMenu;
  char ** menuItems;
  void (*menuDisp) (int, char*);
  int menuDispHeight;
  char * currentItem;
  char * menuHeader;
  int noItems;
  int menuStart;
  bool isConfig;
  int noConfig;
  char ** configTitles;
  char ** configValues;
  int * configDigits;
  int configIdx;
};

const int noStates = 24;
state* states[noStates];
state* curState;
int menuStart = 0;

char* returnState = NULL;

void display() {
  
  if (lastDisplayPage != displayPage || externalDisplayRefresh) {
    externalDisplayRefresh = false;
    u8g2_prepare();
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    if (displayPage == startupDisplay) {
      u8g2.drawFrame(48, 46, 69, 11);
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(50, 47, "Engineering");
      u8g2.drawXBMP(4, 6, startup_width, startup_height, startup_graphic);
    } else if (displayPage == mainDisplay) {
      u8g2.setFont(u8g2_font_courB24_tf);
      //u8g2.setFont(u8g2_font_fub20_tf);
      int bankLength = u8g2.drawStr(0, 0, "Bank");
      char bankDisp[3];
      intToStr0(bank, bankDisp);
      u8g2.drawStr(bankLength + 9, 0, bankDisp);
      u8g2.setFont(u8g2_font_6x10_tf);
      presetBankDraw(0, 24, true, false);
      int lengthOrder = u8g2.drawStr(1, 34, "Loop Order");
      u8g2.drawStr(0, 44, "Test");
      u8g2.setFont(u8g2_font_5x7_tf);
      int pos = 0;
      for(int i=12; i>0; i--) {
        char disp[3];
        sprintf(disp, "%d", i);
        int strwidth = u8g2.drawStr(pos, 54, disp);
        pos += strwidth;
        u8g2.drawLine(pos, 57, pos+1, 57);
        pos += 3;
      }
      u8g2.drawStr(pos, 54, "C");
      u8g2.setFont(u8g2_font_6x10_tf);
    } else if (displayPage == presetProgDisplay || displayPage == presetSecProgDisplay) {
      u8g2.setFont(u8g2_font_6x10_tf);
      presetBankDraw(0, 1, true);
      char * header;
      if(displayPage == presetProgDisplay) {
        header = "Primary Func.";
      } else {
        header = "Secondary Func.";
      }
      int loopsWidth = u8g2.getStrWidth(header);
      u8g2.drawStr((128 - loopsWidth)/2, 11, header);

      preset_t* curPreset = (preset_t*) presetList->last->item;
      
      int loopOnHeight = 11;
      int frameLeft = 2;
      int frameRight = 2;
      int delta = 1;
      int rowPos = delta;
      int row = 1;
      for(int i=0; i<noLoops; i++) {
        char disp[3];
        intToStr0(i+1, disp);
        int dispWidth = u8g2.getStrWidth(disp);
        if(rowPos + dispWidth + frameRight + delta > 127) {
          rowPos = delta;
          row++;
        }
        if(curPreset->loopsOn[i]) {
          u8g2.setDrawColor(1);
          u8g2.drawBox(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight);
          u8g2.setDrawColor(2);
          u8g2.setFontMode(1);
          u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
          u8g2.setDrawColor(1);
          u8g2.setFontMode(0);
        } else {
          u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
          u8g2.drawFrame(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight); 
        }
        rowPos += delta + dispWidth + frameLeft + frameRight;
      }
      for(int i=0; i<noOuts; i++) {
        char disp[5];
        sprintf(disp, "Out%c", 'A' + i);
        int dispWidth = u8g2.getStrWidth(disp);
        if(rowPos + dispWidth + frameRight + delta > 127) {
          rowPos = delta;
          row++;
        }
        if(curPreset->outsOn[i]) {
          u8g2.setDrawColor(1);
          u8g2.drawBox(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight);
          u8g2.setDrawColor(2);
          u8g2.setFontMode(1);
          u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
          u8g2.setDrawColor(1);
          u8g2.setFontMode(0);
        } else {
          u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
          u8g2.drawFrame(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight); 
        }
        rowPos += delta + dispWidth + frameLeft + frameRight;
      }
      for(int i=1; i<noOuts; i++) {
        char disp[5];
        sprintf(disp, "Ph.%c", 'A' + i);
        int dispWidth = u8g2.getStrWidth(disp);
        if(rowPos + dispWidth + frameRight + delta > 127) {
          rowPos = delta;
          row++;
        }
        if(curPreset->outPhaseReverse[i-1]) {
          u8g2.setDrawColor(1);
          u8g2.drawBox(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight);
          u8g2.setDrawColor(2);
          u8g2.setFontMode(1);
          u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
          u8g2.setDrawColor(1);
          u8g2.setFontMode(0);
        } else {
          u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
          u8g2.drawFrame(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight); 
        }
        rowPos += delta + dispWidth + frameLeft + frameRight;
      }
      char disp[6];
      sprintf(disp, "SB");
      int dispWidth = u8g2.getStrWidth(disp);
      if(rowPos + dispWidth + frameRight + delta > 127) {
        rowPos = delta;
        row++;
      }
      if(curPreset->stompMode != offStomp) {
        u8g2.setDrawColor(1);
        u8g2.drawBox(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight);
        u8g2.setDrawColor(2);
        u8g2.setFontMode(1);
        u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
        u8g2.setDrawColor(1);
        u8g2.setFontMode(0);
      } else {
        u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
        u8g2.drawFrame(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight); 
      }
      rowPos += delta + dispWidth + frameLeft + frameRight;
      sprintf(disp, "Tu");
      dispWidth = u8g2.getStrWidth(disp);
      if(rowPos + dispWidth + frameRight + delta > 127) {
        rowPos = delta;
        row++;
      }
      if(curPreset->tunerOn) {
        u8g2.setDrawColor(1);
        u8g2.drawBox(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight);
        u8g2.setDrawColor(2);
        u8g2.setFontMode(1);
        u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
        u8g2.setDrawColor(1);
        u8g2.setFontMode(0);
      } else {
        u8g2.drawStr(rowPos + frameLeft,63 - row * (loopOnHeight+1) - delta + 1, disp);
        u8g2.drawFrame(rowPos, 63 - row * (loopOnHeight+1) - delta, dispWidth + frameLeft + frameRight, loopOnHeight); 
      }
  } else if (displayPage == bankSelectDisplay) {
      u8g2.setFont(u8g2_font_courB24_tf);
      int bankWidth = u8g2.getStrWidth("Bank");
      u8g2.drawStr((128 - bankWidth)/2 , 6, "Bank");
      int selectWidth = u8g2.getStrWidth("Select");
      u8g2.drawStr((128 - selectWidth)/2, 32, "Select");
    } else if (displayPage == saveDisplay) {
      u8g2.setFont(u8g2_font_courB24_tf);
      int saveWidth = u8g2.getStrWidth("Save");
      u8g2.drawStr((128 - saveWidth)/2, 19, "Save");
    } else if (displayPage == discardDisplay) {
      u8g2.setFont(u8g2_font_courB24_tf);
      int discardWidth = u8g2.getStrWidth("Reject");
      u8g2.drawStr((128 - discardWidth)/2, 19, "Reject");
    } else if (displayPage == menuDisplay) {
      int offset = 0;
      u8g2.setFont(u8g2_font_6x10_tf);
      if (curState->menuHeader) {
        offset = 10;
        if (strcmp(curState->menuHeader, "banknpreset") == 0) {
          presetBankDraw(0, 0, true);
        } else {
          int headerWidth = u8g2.getStrWidth(curState->menuHeader);
          u8g2.drawStr((128 - headerWidth)/2, 0, curState->menuHeader);
        }
        u8g2.drawLine(0, offset+1, 127, offset+1);
        offset += 3;
      }

      int itemHeight;
      if(curState->menuDisp) {
        itemHeight = curState->menuDispHeight;
      } else {
        itemHeight = 10;
      }
      int noItems = (64 - offset)/itemHeight;
      int curSelect;
      for(curSelect = 0; curSelect < curState->noItems; curSelect++) {
        if(strcmp(curState->menuItems[curSelect], curState->currentItem) == 0)
          break;
      }
      menuStart = curState->menuStart;
      if(curSelect >= menuStart + noItems)
        menuStart = curSelect - noItems + 1;
      else if(curSelect < menuStart)
        menuStart = curSelect;
      curState->menuStart = menuStart;
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, offset+(curSelect-menuStart)*itemHeight, 127-menuScrollBarWidth - 1, itemHeight);
      u8g2.setDrawColor(2);
      u8g2.setFontMode(1);
      int i;
      for(i=0; i<noItems && i+menuStart < curState->noItems; i++) {
        if(curState->menuDisp) {
          curState->menuDisp(offset+i*itemHeight, curState->menuItems[menuStart+i]);
        } else {
          int textWidth = u8g2.getStrWidth(curState->menuItems[menuStart+i]);
          u8g2.drawStr((128-textWidth)/2, offset+i*itemHeight, curState->menuItems[menuStart+i]);
        }
      }
      u8g2.setDrawColor(1);
      u8g2.setFontMode(0);
      int scrollHeight = 64-offset;
      int scrollBarHeight = (int) scrollHeight * ((float) i)/(curState->noItems);
      int scrollBarStart = menuStart * scrollBarHeight/i;
      u8g2.drawBox(127 - menuScrollBarWidth, offset + scrollBarStart, menuScrollBarWidth, scrollBarHeight);
    } else if(displayPage == factoryResetDisplay) {
      u8g2.setFont(u8g2_font_inr16_mf);
      int factoryLength = u8g2.getStrWidth("Factory");
      u8g2.drawStr((128-factoryLength)/2, 10, "Factory");
      int resetLength = u8g2.getStrWidth("Reset");
      u8g2.drawStr((128-resetLength)/2 , 36, "Reset");
      u8g2.setFont(u8g2_font_6x10_tf);
    } else if(displayPage == configDisplay) {
      u8g2.setFont(u8g2_font_6x10_tf);
      int offset = 0;
      if (curState->menuHeader) {
        offset = 10;
        if (strcmp(curState->menuHeader, "banknpreset") == 0) {
          presetBankDraw(0, 0, true);
        } else {
          int headerWidth = u8g2.getStrWidth(curState->menuHeader);
          u8g2.drawStr((128 - headerWidth)/2, 0, curState->menuHeader);
        }
        u8g2.drawLine(0, offset+1, 127, offset+1);
        offset += 3;
      }
      int widths[curState->noConfig];
      for(int i=0; i<curState->noConfig; i++) {
        int titleWidth = u8g2.getStrWidth(curState->configTitles[i]);
        int valueWidth = u8g2.getStrWidth(curState->configValues[i]);
        widths[i] = titleWidth > valueWidth ? titleWidth : valueWidth;
      }
      int delta = 6;
      int triangleSpace = 7;
      int pos = triangleSpace + delta/2;
      int drawStart = 0;
      for(int i=0; i<curState->configIdx+1; i++) {
        pos += widths[i] + delta;
        if(pos + delta/2 > 127-triangleSpace) {
          drawStart = i;
          pos = delta/2;
        }
      }
      int totalWidth = delta/2;
      for(int i=drawStart; i<curState->noConfig; i++) {
        int width = widths[i];
        if(totalWidth + width + delta/2 > 127-2*triangleSpace)
          break;
        totalWidth += width + delta;
      }
      totalWidth += delta/2;
      pos = triangleSpace + (127 - totalWidth)/2;
      int titleY = offset + 5;
      int valueY = offset + 25;
      if(drawStart > 0) {
        u8g2.drawTriangle(1, valueY + 5, 6, valueY, 6, valueY + 10);
      }
      bool drawTriangleRight;
      for(int i=drawStart; i<curState->noConfig; i++) {
        int width = widths[i];
        if(pos + width + delta/2 > 127-triangleSpace)
          break;
        int valueWidth = u8g2.getStrWidth(curState->configValues[i]);
        int valueOffset = 0;
        if(valueWidth < width)
          valueOffset = (width - valueWidth) / 2;
        int headerWidth = u8g2.getStrWidth(curState->configTitles[i]);
        int headerOffset = 0;
        if(headerWidth < width)
          headerOffset = (width - headerWidth) / 2;
        if(i==curState->configIdx) {
          u8g2.setDrawColor(1);
          u8g2.drawBox(pos - delta/2, valueY - delta/2, width + delta, 10 + delta);
          u8g2.setDrawColor(2);
          u8g2.drawFrame(pos - delta/2+1, valueY - delta/2+1, width + delta-2, 10 + delta-2);
          u8g2.setDrawColor(1);
        } else {
          u8g2.setDrawColor(1);
          u8g2.drawFrame(pos - delta/2+1, valueY - delta/2+1, width + delta-2, 10 + delta-2);
        }
        u8g2.setDrawColor(2);
        u8g2.setFontMode(1);
        u8g2.drawStr(pos + valueOffset, valueY, curState->configValues[i]);
        u8g2.setDrawColor(1);
        u8g2.setFontMode(0);
        u8g2.drawStr(pos + headerOffset, titleY, curState->configTitles[i]);
        pos += width + delta;
        drawTriangleRight = i != curState->noConfig-1;
      }
      if(drawTriangleRight > 0) {
        u8g2.drawTriangle(127-1, valueY + 5, 127-6, valueY, 127-6, valueY + 10);
      } 
    }
    
    lastDisplayPage = displayPage;
    u8g2.sendBuffer();
    printFreeMemory();
  }
}

bool externalStateChange = false;

void loadPresetActivate() {
  SerialMuted("Load Activate\n");
  if(!checkActivePresets()) {
    SerialMuted("Search for Backup Preset\n");
    findBackupPreset();
  }
  hardwareActivatePreset();
  externalStateChange = true;
}

char * loadPresetTransitions() {
  SerialMuted("Load Transistion\n");
  if(returnState) {
    char* retval = new char[30];
    strcpy(retval, returnState);
    returnState = NULL;
    externalDisplayRefresh = true;
    return retval;
  }
  return "run";
}

void loadPresetDeactivate() {
  SerialMuted("Load Deactivate\n");
  return;
}

state loadPresetState;

void runActivate() {
  SerialMuted("Run Activate\n");
  displayPage = mainDisplay;
}

char * runTransitions() {
  SerialMuted("Run Transistion\n");
  for(int i = 0; i < noMenuPins; i++) {
    if(menuEdges[i]) {
      
    }
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        return "loopProgram";
      } else if(menuPins[i] == upPin) {
        return "bankSelect";
      }
    }
    if(menuLongPress[i]) {
      if(menuPins[i] == enterPin) {
        return "mainMenu";
      }
    }
  }
  for(int i = 0; i< noPresetPins; i++) {
    if(presetEdges[i]) {
      /*SerialMuted("Preset Edge: ");
      SerialMuted(i);
      SerialMuted("\n");*/
      bool presetWasThere = activatePreset(bank, i+1);
      if(!presetWasThere) {
        sendTapTempo(false);
        cancelTapTempo();
      }
      checkTapTempo(true);
      externalDisplayRefresh = true;
      return NULL;
    }
  }

  return NULL;
}

void runDeactivate() {
  SerialMuted("Run Deactivate\n");
  return;
}

state runState;

void loopProgramActivate() {
  SerialMuted("Loop Program Activate\n");
  displayPage = presetProgDisplay;
}

char * loopProgramTransitions() {
  SerialMuted("Loop Program Transistion\n");
  for(int i = 0; i < noMenuPins; i++) {
    if(menuEdges[i]) {
      
    }
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        return "secondaryProgram";
      }
    }
    if(menuLongPress[i]) {
      if(menuPins[i] == enterPin) {
        return "save";
      } else if(menuPins[i] == backPin) {
        return "discard";
      }
    }
  }
  for(int i = 0; i<noPresetPins && i<noLoops; i++) {
    if(presetEdges[i]) {
      if(stereoLoops[i/2]) {
        if(i%2 == 0) {
          ((preset_t*) presetList->last->item)->loopsOn[i] ^= true;
          ((preset_t*) presetList->last->item)->loopsOn[i+1] = ((preset_t*) presetList->last->item)->loopsOn[i];
        } else {
          ((preset_t*) presetList->last->item)->loopsOn[i] ^= true;
          ((preset_t*) presetList->last->item)->loopsOn[i-1] = ((preset_t*) presetList->last->item)->loopsOn[i];
        }
        externalDisplayRefresh = true;
      } else {
        ((preset_t*) presetList->last->item)->loopsOn[i] ^= true;  
        externalDisplayRefresh = true;
      }
      hardwareActivatePreset();
    }
  }
  return NULL;
}

void loopProgramDeactivate() {
  SerialMuted("Loop Program Deactivate\n");
  return;
}

state loopProgramState;

void secondaryProgramActivate() {
  SerialMuted("Secondary Program Activate\n");
  displayPage = presetSecProgDisplay;
}

char * secondaryProgramTransitions() {
  SerialMuted("Secondary Program Transistion\n");
  for(int i = 0; i < noMenuPins; i++) {
    if(menuEdges[i]) {
      
    }
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        return "loopProgram";
      }
    }
    if(menuLongPress[i]) {
      if(menuPins[i] == enterPin) {
        return "save";
      } else if(menuPins[i] == backPin) {
        return "discard";
      }
    }
  }
  int outOffset = noLoops - noPresetPins;
  outOffset = outOffset < 0 ? 0 : outOffset;
  for(int j = 0; j<noPresetPins; j++) {
    if(presetEdges[j]) {
      if(j+noPresetPins < noLoops) {
        int i = j+noPresetPins;
        if(stereoLoops[i/2]) {
          if(i%2 == 0) {
            ((preset_t*) presetList->last->item)->loopsOn[i] ^= true;
            ((preset_t*) presetList->last->item)->loopsOn[i+1] = ((preset_t*) presetList->last->item)->loopsOn[i];
          } else {
            ((preset_t*) presetList->last->item)->loopsOn[i] ^= true;
            ((preset_t*) presetList->last->item)->loopsOn[i-1] = ((preset_t*) presetList->last->item)->loopsOn[i];
          }
          externalDisplayRefresh = true;
        } else {
          ((preset_t*) presetList->last->item)->loopsOn[i] ^= true;  
          externalDisplayRefresh = true;
        }
      } else if(j-outOffset < noOuts) {
        ((preset_t*) presetList->last->item)->outsOn[j-outOffset] ^= true;
        externalDisplayRefresh = true;
      } else if(j-outOffset-noOuts < noOuts-1) {
        ((preset_t*) presetList->last->item)->outPhaseReverse[j-outOffset-noOuts] ^= true;
        externalDisplayRefresh = true;
      } else if(j-outOffset-noOuts-noOuts-1 == 0) {
        if(((preset_t*) presetList->last->item)->stompMode == offStomp) {
          ((preset_t*) presetList->last->item)->stompMode = normalStomp;
          externalDisplayRefresh = true;
        } else {
          ((preset_t*) presetList->last->item)->stompMode = offStomp;
          externalDisplayRefresh = true;
        }
      } else if(j-outOffset-noOuts-noOuts-1 == 1) {
        ((preset_t*) presetList->last->item)->tunerOn ^= true;
      }
      
      hardwareActivatePreset();
    }
  }
  return NULL;
}

void secondaryProgramDeactivate() {
  SerialMuted("Secondary Program Deactivate\n");
  return;
}

state secondaryProgramState;

void saveActivate() {
  SerialMuted("Save Activate\n");
  displayPage = saveDisplay;
  preset_t* preset = (preset_t*) presetList->last->item;
  preset_t* oldPreset;
  readPresetFncBankNPreset(oldPreset, preset->bankNo, preset->presetNo);
  savePresetFncBankNPreset(preset, preset->bankNo, preset->presetNo);
  if(oldPreset->stompMode != offStomp && preset->stompMode == offStomp) {
    preset_t* actPreset = (preset_t*)presetList->last->item;
    activatePreset(actPreset->bankNo, actPreset->presetNo);
  }
  saveActivePresets();
  externalDisplayRefresh = true;
  externalStateChange = true;
}

char * saveTransitions() {
  SerialMuted("Save Transistion\n");
  delay(storeDelay);
  return "loadPreset";
}

void saveDeactivate() {
  SerialMuted("Save Deactivate\n");
  return;
}

state saveState;

void discardActivate() {
  SerialMuted("Discard Activate\n");
  displayPage = discardDisplay;
  preset_t* preset = (preset_t*) presetList->last->item;
  readPresetFncBankNPreset(preset, preset->bankNo, preset->presetNo);
  externalDisplayRefresh = true;
  externalStateChange = true;
}

char * discardTransitions() {
  SerialMuted("Discard Transistion\n");
  delay(storeDelay);
  return "loadPreset";
}

void discardDeactivate() {
  SerialMuted("Discard Deactivate\n");
  return;
}

state discardState;

void bankSelectActivate() {
  SerialMuted("Bank Select Activate\n");
  displayPage = bankSelectDisplay;
}

char * bankSelectTransitions() {
  /*for(int i = 0; i< noPresetPins; i++) {
    if(presetEdges[i]) {
      bank = i + 1;
      saveBank();
      externalDisplayRefresh = true;
      return "run";
    }
  }*/
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        return "run";
      }
    }
  }
  return NULL;
}

void bankSelectDeactivate() {
  SerialMuted("Bank Select Deactivate\n");
}

state bankSelectState;

state mainMenuState;
char* mainMenuItems[7];

bool mainMenuClearSelect = false;

void mainMenuActivate() {
  SerialMuted("Main Menu Activate\n");
  externalDisplayRefresh = true;
}

char * mainMenuTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        mainMenuClearSelect = true;
        return "run";
      } else if(menuPins[i] == enterPin) {
        if(strcmp(mainMenuState.currentItem, "Loop Order") == 0)
          return "loopOrder";
        else if(strcmp(mainMenuState.currentItem, "Stomp Box Mode") == 0)
          return "stompBoxMode";
        else if(strcmp(mainMenuState.currentItem, "Midi") == 0)
          return "midi";
        else if(strcmp(mainMenuState.currentItem, "Tap Tempo") == 0)
          return "tapTempo";
        else if(strcmp(mainMenuState.currentItem, "Copy") == 0)
          return "copyPreset";
        else if(strcmp(mainMenuState.currentItem, "Reset") == 0)
          return "resetPreset";  
        else if(strcmp(mainMenuState.currentItem, "System Config") == 0)
          return "sysConfig";
      }
    }
  }
  return NULL;
}

state resetPresetState;
char* resetPresetItems[2];

void resetPresetActivate() {
  SerialMuted("Reset Preset Activate\n");
  externalDisplayRefresh = true;
}

void resetPresetDeactivate() {
  SerialMuted("Reset Preset Deactivate\n");
}

char * resetPresetTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        if(strcmp(resetPresetState.currentItem, "Ok") == 0) {
          uint8_t bankNo = ((preset_t*)presetList->last->item)->bankNo;
          uint8_t presetNo = ((preset_t*)presetList->last->item)->presetNo;
          resetPreset(bankNo, presetNo);
          deleteIdx(presetList, listLength(presetList)-1);
          activatePreset(bankNo, presetNo);
        }
        return "mainMenu";
      }
    }
  }
  return NULL;
}

void mainMenuDeactivate() {
  SerialMuted("Main Menu Deactivate\n");
  if(mainMenuClearSelect) {
    for(int i=0; i<noStates; i++) {
      if(states[i]->isMenu) {
        states[i]->currentItem = states[i]->menuItems[0];
        states[i]->menuStart = 0;
      }
    }
  }
}

state sysConfigState;
char* sysConfigItems[7];

void sysConfigActivate() {
  SerialMuted("System Config Activate\n");
  externalDisplayRefresh = true;
}

void sysConfigDeactivate() {
  SerialMuted("System Config Deactivate\n");
}

char* sysConfigTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        return "mainMenu";
      } else if(menuPins[i] == enterPin) {
        if(strcmp(sysConfigState.currentItem, "Bank Config") == 0)
          return "bankConfig";
        else if(strcmp(sysConfigState.currentItem, "Startup Preset") == 0)
          return "startupPreset";
        else if(strcmp(sysConfigState.currentItem, "Brightness") == 0)
          return "brightness";
        else if(strcmp(sysConfigState.currentItem, "Factory Reset") == 0)
          return "factoryReset";
        else if(strcmp(sysConfigState.currentItem, "Stereo Loops") == 0)
          return "stereoLoops";
        else if(strcmp(sysConfigState.currentItem, "Phase Reverse") == 0)
          return "phaseReverse";
        else if(strcmp(sysConfigState.currentItem, "Midi Config") == 0)
          return "midiSysConfig";
      }
    }
  }
  return NULL;
}

state midiSysConfigState;
char* midiSysConfigItems[5];

void midiSysConfigActivate() {
  SerialMuted("Midi Sys Config Activate\n");
  externalDisplayRefresh = true;
}

void midiSysConfigDeactivate() {
  SerialMuted("Midi Sys Config Deactivate\n");
}

char * midiSysConfigTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        return "sysConfig";
      } else if(menuPins[i] == enterPin) {
        if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[0]) == 0)
          return "midiSysBool";
        else if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[1]) == 0)
          return "midiSysBool";
        else if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[2]) == 0)
          return "midiSysBool";
        else if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[3]) == 0)
          return "midiSysInChannel";
        else if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[4]) == 0)
          return "midiSysBankCtrl";
      }
    }
  }
  return NULL;
}

void midiSysConfigDisplay(int y, char* item) {

  int xOffsetText = 10;
  int xOffsetStatus = 95;

  char stat[5];
  if(strcmp(item, midiSysConfigItems[3]) == 0) {
    intToStr0(midiInChannel, stat);
  } else if(strcmp(item, midiSysConfigItems[4]) == 0) {
    intToStr0(midiBankCtrl, stat, 3);
  } else {
    bool boolStat;
    if(strcmp(item, midiSysConfigItems[0]) == 0) {
      boolStat = midiInOn;
    }
    else if(strcmp(item, midiSysConfigItems[1]) == 0) {
      boolStat = midiOutOn;
    }
    else if(strcmp(item, midiSysConfigItems[2]) == 0) {
      boolStat = midiThroughOn;
    }
    if(boolStat) {
      strcpy(stat, "On");
    } else {
      strcpy(stat, "Off");
    }
  }
  u8g2.drawStr(xOffsetText, y + 2, item);
  int statWidth = u8g2.drawStr(xOffsetStatus, y + 2, stat);
  u8g2.drawFrame(xOffsetStatus - 2, y+1, statWidth+4, 11);
}

state midiConfigBoolState;
char * midiConfigBoolItems[2];

void midiConfigBoolActivate() {
  midiConfigBoolState.menuHeader = midiSysConfigState.currentItem;
  bool select;
  if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[0]) == 0) {
    select = midiInOn;
  } else if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[1]) == 0) {
    select = midiOutOn;
  } else if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[2]) == 0) {
    select = midiThroughOn;
  }
  midiConfigBoolState.currentItem = select ? midiConfigBoolItems[0] : midiConfigBoolItems[1];
  externalDisplayRefresh = true;
}

void midiConfigBoolDeactivate() {
  
}

char* midiConfigBoolTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        return "midiSysConfig";
      } else if(menuPins[i] == enterPin) {
        bool select = strcmp(midiConfigBoolState.currentItem, "On") == 0;
        
        if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[0]) == 0) {
          midiInOn = select;
          saveMidiInOn();
        } else if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[1]) == 0) {
          midiOutOn = select;
          saveMidiOutOn();
        } else if(strcmp(midiSysConfigState.currentItem, midiSysConfigItems[2]) == 0) {
          midiThroughOn = select;
          saveMidiThroughOn();
          if(select) {
            midiInst.turnThruOn();
          } else {
            midiInst.turnThruOff();
          }
        }
        return "midiSysConfig";
      }
    }
  }
  return NULL; 
}

state midiInChannelState;
char * midiInChannelTitles[1];
char midiInChannelItems[1][3];
char * midiInChannelItemsState[1];

void midiInChannelActivate() {
  intToStr0(midiInChannel, midiInChannelItemsState[0]);
  externalDisplayRefresh = true;
}

void midiInChannelDeactivate() {
  
}

char * midiInChannelTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        readMidiInChannel();
        return "midiSysConfig";
      } else if(menuPins[i] == enterPin) {
        saveMidiInChannel();
        return "midiSysConfig";
      } else if(menuPins[i] == upPin) {
        if(midiInChannel < 15) {
          midiInChannel++;
          intToStr0(midiInChannel, midiInChannelItemsState[0]);
          externalDisplayRefresh = true;
        } else {
          midiInChannel = 1;
          intToStr0(midiInChannel, midiInChannelItemsState[0]);
          externalDisplayRefresh = true;
        }
      } else if(menuPins[i] == downPin) {
        if(midiInChannel > 1) {
          midiInChannel--; 
          intToStr0(midiInChannel, midiInChannelItemsState[0]);
          externalDisplayRefresh = true;
        } else {
          midiInChannel = 15; 
          intToStr0(midiInChannel, midiInChannelItemsState[0]);
          externalDisplayRefresh = true;
        }
      }
    } else if(menuClockFast[i]) {
      if(menuPins[i] == upPin) {
        if(midiInChannel < 15) {
          midiInChannel++;
          intToStr0(midiInChannel, midiInChannelItemsState[0]);
          externalDisplayRefresh = true;
        } else {
          midiInChannel = 1;
          intToStr0(midiInChannel, midiInChannelItemsState[0]);
          externalDisplayRefresh = true;
        }
      } else if(menuPins[i] == downPin) {
        if(midiInChannel > 1) {
          midiInChannel--; 
          intToStr0(midiInChannel, midiInChannelItemsState[0]);
          externalDisplayRefresh = true;
        } else {
          midiInChannel = 15; 
          intToStr0(midiInChannel, midiInChannelItemsState[0]);
          externalDisplayRefresh = true;
        }
      }
    }
  }
  return NULL;
}

state midiBankCtrlState;
char * midiBankCtrlTitles[1];
char midiBankCtrlItems[1][3];
char * midiBankCtrlItemsState[1];

void midiBankCtrlActivate() {
  intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
  externalDisplayRefresh = true;
}

void midiBankCtrlDeactivate() {
  
}

char * midiBankCtrlTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        readMidiBankCtrl();
        return "midiSysConfig";
      } else if(menuPins[i] == enterPin) {
        saveMidiBankCtrl();
        return "midiSysConfig";
      } else if(menuPins[i] == upPin) {
        if(midiBankCtrl < 127) {
          midiBankCtrl++;
          intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
          externalDisplayRefresh = true;
        } else {
          midiBankCtrl = 1; 
          intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
          externalDisplayRefresh = true;
        }
      } else if(menuPins[i] == downPin) {
        if(midiBankCtrl > 1) {
          midiBankCtrl--; 
          intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
          externalDisplayRefresh = true;
        } else {
          midiBankCtrl = 127; 
          intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
          externalDisplayRefresh = true;
        }
      }
    } else if(menuClockFast[i]) {
      if(menuPins[i] == upPin) {
        if(midiBankCtrl < 127) {
          midiBankCtrl++;
          intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
          externalDisplayRefresh = true;
        } else {
          midiBankCtrl = 1; 
          intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
          externalDisplayRefresh = true;
        }
      } else if(menuPins[i] == downPin) {
        if(midiBankCtrl > 1) {
          midiBankCtrl--; 
          intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
          externalDisplayRefresh = true;
        }  else {
          midiBankCtrl = 127; 
          intToStr0(midiBankCtrl, midiBankCtrlItemsState[0], 3);
          externalDisplayRefresh = true;
        }
      }
    }
  }
  return NULL;
}

state midiState;
char midiItems[noMidiMsg][10];
char* midiItemsState[noMidiMsg];

void midiActivate() {
  SerialMuted("Midi Activate\n");
  externalDisplayRefresh = true;
}

void midiDeactivate() {
  SerialMuted("Midi Deactivate\n");
}

char * midiTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        return "mainMenu";
      } else if(menuPins[i] == enterPin) {
        return "midiConfig";
      }
    }
  }
  return NULL;
}

state midiConfigState;

void midiConfigStateChange() {
  externalDisplayRefresh = true;
  if(midiConfigState.configTitles) {
    delete midiConfigState.configTitles;
  }
  midiConfigState.configTitles = NULL;
  if(midiConfigState.configValues) {
    for(int i=0; i<sizeof(midiConfigState.configValues)/sizeof(char**); i++) {
      delete midiConfigState.configValues[i];
    }
    delete midiConfigState.configValues;
    midiConfigState.configValues = NULL;
  }
  int msgNo = 0;
  for(msgNo = 0; msgNo<noMidiMsg; msgNo++) {
    if(strcmp(midiState.currentItem, midiState.menuItems[msgNo]) == 0)
      break;
  }
  preset_t* curPreset = (preset_t*) presetList->last->item;
  if(curPreset->midiType[msgNo] == midiPC) {
    midiConfigState.noConfig = 4;
    char ** configTitles = new char*[4];
    configTitles[0] = "On/Off";
    configTitles[1] = "PC/CC";
    configTitles[2] = "Ch.";
    configTitles[3] = "No.";
    midiConfigState.configTitles = configTitles;
    char ** configValues = new char*[4];
    midiConfigState.configValues = configValues;
    for(int i = 0; i < 4; i++) {
      configValues[i] = new char[6];
    }
    if(curPreset->midiOn[msgNo]) {
      strcpy(configValues[0], "On");
    } else {
      strcpy(configValues[0], "Off");
    }
    strcpy(configValues[1], "PC");
    intToStr0(curPreset->midiChannel[msgNo], configValues[2]);
    intToStr0(curPreset->midiPC[msgNo], configValues[3], 3);
  } else if(curPreset->midiType[msgNo] == midiCC) {
    midiConfigState.noConfig = 5;
    char ** configTitles = new char*[5];
    configTitles[0] = "On/Off";
    configTitles[1] = "PC/CC";
    configTitles[2] = "Ch.";
    configTitles[3] = "Ctrl.";
    configTitles[4] = "Val.";
    midiConfigState.configTitles = configTitles;
    char ** configValues = new char*[5];
    midiConfigState.configValues = configValues;
    for(int i = 0; i < 5; i++) {
      configValues[i] = new char[6];
    }
    if(curPreset->midiOn[msgNo]) {
      strcpy(configValues[0], "On");
    } else {
      strcpy(configValues[0], "On");
    }
    strcpy(configValues[1], "CC");
    intToStr0(curPreset->midiChannel[msgNo], configValues[2]);
    intToStr0(curPreset->midiCC[msgNo], configValues[3], 3);
    intToStr0(curPreset->midiCCValue[msgNo][0], configValues[4], 3);
  } else if(curPreset->midiType[msgNo] == expr1 || curPreset->midiType[msgNo] == expr2) {
    midiConfigState.noConfig = 6;
    char ** configTitles = new char*[6];
    configTitles[0] = "On/Off";
    configTitles[1] = "PC/CC";
    configTitles[2] = "Ch.";
    configTitles[3] = "Ctrl.";
    configTitles[4] = "Hell";
    configTitles[5] = "Toe";
    midiConfigState.configTitles = configTitles;
    char ** configValues = new char*[6];
    midiConfigState.configValues = configValues;
    for(int i = 0; i < 6; i++) {
      configValues[i] = new char[6];
    }
    if(curPreset->midiOn[msgNo]) {
      strcpy(configValues[0], "On");
    } else {
      strcpy(configValues[0], "Off");
    }
    if(curPreset->midiType[msgNo] == expr1)
      strcpy(configValues[1], "Expr1");
    else
      strcpy(configValues[1], "Expr2");
    intToStr0(curPreset->midiChannel[msgNo], configValues[2]);
    intToStr0(curPreset->midiCC[msgNo], configValues[3], 3);
    intToStr0(curPreset->midiCCValue[msgNo][0], configValues[4], 3);
    intToStr0(curPreset->midiCCValue[msgNo][1], configValues[5], 3);
  }
}

void midiConfigActivate() {
  SerialMuted("Midi Config Activate\n");
  midiConfigStateChange();
  midiConfigState.menuHeader = midiState.currentItem;
  externalDisplayRefresh = true;
}

void midiConfigDeactivate() {
  SerialMuted("Midi Config Deactivate\n");
  midiConfigState.configIdx = 0;
  if(midiConfigState.configTitles) {
    delete midiConfigState.configTitles;
  }
  midiConfigState.configTitles = NULL;
  if(midiConfigState.configValues) {
    for(int i=0; i<sizeof(midiConfigState.configValues)/sizeof(char**); i++) {
      delete midiConfigState.configValues[i];
    }
    delete midiConfigState.configValues;
    midiConfigState.configValues = NULL;
  }
}

void midiConfigUpDown(bool up) {
  preset_t* curPreset = (preset_t*) presetList->last->item;
  int msgNo = 0;
  for(msgNo = 0; msgNo<noMidiMsg; msgNo++) {
    if(strcmp(midiState.currentItem, midiState.menuItems[msgNo]) == 0)
      break;
  }
  if(midiConfigState.configIdx == 0) {
    curPreset->midiOn[msgNo] ^= true;
  } else if (midiConfigState.configIdx == 1) {
    midiTypes selection[4] = {midiPC, midiCC, expr1 , expr2};
    int i;
    for(i=0; i<4; i++) {
      if(curPreset->midiType[msgNo] == selection[i])
        break;
    }
    if(up) {
      if(curPreset->midiType[msgNo] != expr2) {
        curPreset->midiType[msgNo] = selection[i+1];
      } else {
        curPreset->midiType[msgNo] = midiPC;
      }
    } else {
      if(curPreset->midiType[msgNo] != midiPC) {
        curPreset->midiType[msgNo] = selection[i-1];
      } else {
        curPreset->midiType[msgNo] = expr2;
      }
    }
  } else if (midiConfigState.configIdx == 2) {
    if(up) {
      curPreset->midiChannel[msgNo] = curPreset->midiChannel[msgNo] < 15 ? curPreset->midiChannel[msgNo] + 1 : 1;
    } else {
      curPreset->midiChannel[msgNo] = curPreset->midiChannel[msgNo] > 1 ? curPreset->midiChannel[msgNo] - 1 : 15;
    }
  } else if (midiConfigState.configIdx == 3) {
    uint8_t * data;
    if(curPreset->midiType[msgNo] == midiPC){
      data = &curPreset->midiPC[msgNo];
    } else if(curPreset->midiType[msgNo] == midiCC || curPreset->midiType[msgNo] == expr1 || curPreset->midiType[msgNo] == expr2) {
      data = &curPreset->midiCC[msgNo];
    }
    if(up) {
      *data = *data < 127 ? *data + 1 : 0;
    } else {
      *data = *data > 0 ? *data - 1 : 127;
    }
  } else {
    uint8_t * data = &curPreset->midiCCValue[msgNo][midiConfigState.configIdx - 4];
    if(up) {
      *data = *data < 127 ? *data + 1 : 0;
    } else {
      *data = *data > 0 ? *data - 1 : 127;
    }
  }
  midiConfigStateChange();
}

char* midiConfigTransitions() {
  SerialMuted("Midi Config Transitions\n");
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        returnState = "midi";
        return "discard";
      } else if(menuPins[i] == enterPin) {
        returnState = "midi";
        return "save";
      } else if(menuPins[i] == leftPin) {
        midiConfigState.configIdx = midiConfigState.configIdx > 0 ? midiConfigState.configIdx - 1 : 0;
        externalDisplayRefresh = true;
      } else if(menuPins[i] == rightPin) {
        midiConfigState.configIdx = midiConfigState.configIdx < midiConfigState.noConfig - 1 ? midiConfigState.configIdx + 1 : midiConfigState.noConfig - 1;
        externalDisplayRefresh = true;
      } else if(menuPins[i] == upPin) {
        midiConfigUpDown(true);
      } else if(menuPins[i] == downPin) {
        midiConfigUpDown(false);
      }
    } else if(menuClockFast[i]) {
      if(menuPins[i] == upPin) {
        midiConfigUpDown(true);
      } else if(menuPins[i] == downPin) {
        midiConfigUpDown(false);
      }
    }
  }
  return NULL;
}

void midiDisplay(int y, char* item) {

  SerialMuted("midiDisplay\n");
  SerialMuted(item);
  SerialMuted("\n");

  int xOffsetText = 20;
  int xOffsetStatus = 90;

  preset_t* preset = (preset_t*) presetList->last->item;
  int i;
  for(i=0; i<noMidiMsg; i++) {
    if(strcmp(item, midiItems[i]) == 0)
      break;
  }
  char* stat;
  if(preset->midiOn[i]) {
    stat = "On";
  } else {
    stat = "Off";
  }
  u8g2.drawStr(xOffsetText, y + 2, item);
  int statWidth = u8g2.drawStr(xOffsetStatus, y + 2, stat);
  u8g2.drawFrame(xOffsetStatus - 2, y+1, statWidth+4, 11);
}

state tapTempoConfigState;
char * tapTempoConfigTitles[5];
state tapTempoState;
char tapTempoItems[noTapTempo][10];
char * tapTempoItemsState[noTapTempo];

void tapTempoDisplay(int y, char* item) {

  int xOffsetText = 20;
  int xOffsetStatus = 90;

  preset_t* preset = (preset_t*) presetList->last->item;
  int i;
  for(i=0; i<noTapTempo; i++) {
    if(strcmp(item, tapTempoItems[i]) == 0)
      break;
  }
  char* stat;
  if(preset->tapOn[i]) {
    stat = "On";
  } else {
    stat = "Off";
  }
  u8g2.drawStr(xOffsetText, y + 2, item);
  int statWidth = u8g2.drawStr(xOffsetStatus, y + 2, stat);
  u8g2.drawFrame(xOffsetStatus - 2, y+1, statWidth+4, 11);
}

void tapTempoActivate() {
  SerialMuted("Tap Tempo Activate\n");
  externalDisplayRefresh = true;
}

void tapTempoDeactivate() {
  SerialMuted("Tap Tempo Deactivate\n");
}

char * tapTempoTransitions() {
  SerialMuted("Tap Tempo Transitions\n");
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        return "mainMenu";
      } else if(menuPins[i] == enterPin) {
        returnState = "tapTempoConfig";
        return "tapTempoConfig";
      }
    }
  }
  return NULL;
}

void tapTempoConfigStateChange() {
  externalDisplayRefresh = true;
  int msgNo = 0;
  for(msgNo = 0; msgNo<noTapTempo; msgNo++) {
    if(strcmp(tapTempoState.currentItem, tapTempoState.menuItems[msgNo]) == 0)
      break;
  }
  preset_t* curPreset = (preset_t*) presetList->last->item;
  char ** configValues = new char*[6];
  tapTempoConfigState.configValues = configValues;
  for(int i = 0; i < 6; i++) {
    configValues[i] = new char[6];
  }
  if(curPreset->tapOn[msgNo]) {
    strcpy(configValues[0], "On");
  } else {
    strcpy(configValues[0], "Off");
  }
  intToStr0(curPreset->tapChannel[msgNo], configValues[1]);
  intToStr0(curPreset->tapCC[msgNo], configValues[2], 3);
  intToStr0(curPreset->tapCCOn[msgNo], configValues[3], 3);
  intToStr0(curPreset->tapCCOff[msgNo], configValues[4], 3);
}

void tapTempoConfigActivate() {
  SerialMuted("Tap Tempo Config Activate\n");
  tapTempoConfigStateChange();
  tapTempoConfigState.menuHeader = tapTempoState.currentItem;
  externalDisplayRefresh = true;
}

void tapTempoConfigDeactivate() {
  SerialMuted("Tap Tempo Config Deactivate\n");
  tapTempoConfigState.configIdx = 0;
  for(int i = 0; i < 6; i++) {
    delete tapTempoConfigState.configValues[i];
  }
  delete tapTempoConfigState.configValues;
}

void tapTempoConfigUpDown(bool up) {
  preset_t* curPreset = (preset_t*) presetList->last->item;
  int msgNo = 0;
  for(msgNo = 0; msgNo<noTapTempo; msgNo++) {
    if(strcmp(tapTempoState.currentItem, tapTempoState.menuItems[msgNo]) == 0)
      break;
  }
  if(tapTempoConfigState.configIdx == 0) {
    curPreset->tapOn[msgNo] ^= true;
  } else if (tapTempoConfigState.configIdx == 1) {
    if(up) {
      curPreset->tapChannel[msgNo] = curPreset->tapChannel[msgNo] < 15 ? curPreset->tapChannel[msgNo] + 1 : 1;
    } else {
      curPreset->tapChannel[msgNo] = curPreset->tapChannel[msgNo] > 1 ? curPreset->tapChannel[msgNo] - 1 : 15;
    }
  } else if (tapTempoConfigState.configIdx == 2) {
    uint8_t * data;
    data = &curPreset->tapCC[msgNo];
    if(up) {
      *data = *data < 127 ? *data + 1 : 0;
    } else {
      *data = *data > 0 ? *data - 1 : 127;
    }
  } else {
    uint8_t * data;
    if(tapTempoConfigState.configIdx == 3) {
      data = &curPreset->tapCCOn[msgNo];
    } else {
      data = &curPreset->tapCCOff[msgNo];
    }
    if(up) {
      *data = *data < 127 ? *data + 1 : 0;
    } else {
      *data = *data > 0 ? *data - 1 : 127;
    }
  }
  tapTempoConfigStateChange();
}

char* tapTempoConfigTransitions() {
  SerialMuted("Tap Tempo Config Transitions\n");
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == backPin) {
        returnState = "tapTempo";
        return "discard";
      } else if(menuPins[i] == enterPin) {
        returnState = "tapTempo";
        return "save";
      } else if(menuPins[i] == leftPin) {
        tapTempoConfigState.configIdx = tapTempoConfigState.configIdx > 0 ? tapTempoConfigState.configIdx - 1 : 0;
        externalDisplayRefresh = true;
      } else if(menuPins[i] == rightPin) {
        tapTempoConfigState.configIdx = tapTempoConfigState.configIdx < tapTempoConfigState.noConfig - 1 ? tapTempoConfigState.configIdx + 1 : tapTempoConfigState.noConfig - 1;
        externalDisplayRefresh = true;
      } else if(menuPins[i] == upPin) {
        tapTempoConfigUpDown(true);
      } else if(menuPins[i] == downPin) {
        tapTempoConfigUpDown(false);
      }
    } else if(menuClockFast[i]) {
      if(menuPins[i] == upPin) {
        tapTempoConfigUpDown(true);
      } else if(menuPins[i] == downPin) {
        tapTempoConfigUpDown(false);
      }
    }
  }
  return NULL;
}

state stereoLoopsState;
char stereoLoopsItems[noLoops/2][15];
char* stereoLoopsItemsState[noLoops/2];

state stereoLoopOnOffState;
char * stereoLoopOnOffItems[2];

void stereoLoopsActivate() {
  SerialMuted("Stereo Loops Activate\n");
  externalDisplayRefresh = true;
}

void stereoLoopsDeactivate() {
  SerialMuted("Stereo Loops Deactivate\n");
}

char * stereoLoopsTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        for(int i=0; i<noLoops/2; i++) {
          if(strcmp(stereoLoopsState.currentItem, stereoLoopsItemsState[i]) == 0) {
            strcpy(header, "Stereo - ");
            stereoLoopOnOffState.menuHeader = strcat(header, stereoLoopsState.currentItem);
            if(stereoLoops[i]) {
              stereoLoopOnOffState.currentItem = stereoLoopOnOffItems[1];
            } else {
              stereoLoopOnOffState.currentItem = stereoLoopOnOffItems[0];
            }
            break;
          }
        }
        return "stereoLoopOnOff";
      } else if(menuPins[i] == backPin) {
        return "sysConfig";
      }
    }
  }
  return NULL;
}

void stereoLoopsDisplay(int y, char* item) {

  SerialMuted("stereoLoopsDisplay\n");
  SerialMuted(item);
  SerialMuted("\n");

  int xOffsetText = 10;
  int xOffsetStatus = 100;
  
  char* stat;
  for(int i=0; i<noLoops/2; i++) {
    if(strcmp(item, stereoLoopsItemsState[i]) == 0) {
      if(stereoLoops[i]) {
        stat = "On";
      } else {
        stat = "Off";
      }
      break;
    }
  }
  
  u8g2.drawStr(xOffsetText, y + 2, item);
  int statWidth = u8g2.drawStr(xOffsetStatus, y + 2, stat);
  u8g2.drawFrame(xOffsetStatus - 2, y+1, statWidth+4, 11);
}

void stereoLoopOnOffActivate() {
  SerialMuted("Stereo Loop On Off Activate\n");
  externalDisplayRefresh = true;
}

void stereoLoopOnOffDeactivate() {
  SerialMuted("Stereo Loop On Off Deactivate\n");
}

char * stereoLoopOnOffTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        for(int i=0; i<noLoops/2; i++) {
          if(strcmp(stereoLoopsState.currentItem, stereoLoopsItemsState[i]) == 0) {
            if(strcmp(stereoLoopOnOffState.currentItem, "On") == 0) {
              stereoLoops[i] = true;
            } else {
              stereoLoops[i] = false;
            }
            saveStereoLoops();
            break;
          }
        }
        return "stereoLoops";        
      } else if(menuPins[i] == backPin) {
        return "stereoLoops";
      }
    }
  }

  return NULL;
}

state phaseReverseState;
char phaseReverseItems[noLoops + noOuts - 1][15];
char* phaseReverseItemsState[noLoops + noOuts - 1];

state phaseReverseOnOffState;
char * phaseReverseOnOffItems[2];

void phaseReverseActivate() {
  SerialMuted("Phase Reverse Activate\n");
  externalDisplayRefresh = true;
}

void phaseReverseDeactivate() {
  SerialMuted("Phase Reverse Deactivate\n");
}

char * phaseReverseTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        for(int i=0; i<noLoops + noOuts - 1; i++) {
          if(strcmp(phaseReverseState.currentItem, phaseReverseItemsState[i]) == 0) {
            strcpy(header, "Phase - ");
            phaseReverseOnOffState.menuHeader = strcat(header, phaseReverseState.currentItem);
            if(phaseReverse[i]) {
              phaseReverseOnOffState.currentItem = phaseReverseOnOffItems[1];
            } else {
              phaseReverseOnOffState.currentItem = phaseReverseOnOffItems[0];
            }
            break;
          }
        }
        return "phaseReverseOnOff";
      } else if(menuPins[i] == backPin) {
        return "sysConfig";
      }
    }
  }
  return NULL;
}

void phaseReverseDisplay(int y, char* item) {

  SerialMuted("phaseReverseDisplay\n");
  SerialMuted(item);
  SerialMuted("\n");

  int xOffsetText = 10;
  int xOffsetStatus = 100;
  
  char* stat;
  for(int i=0; i<noLoops + noOuts - 1; i++) {
    if(strcmp(item, phaseReverseItemsState[i]) == 0) {
      if(phaseReverse[i]) {
        stat = "On";
      } else {
        stat = "Off";
      }
      break;
    }
  }
  
  u8g2.drawStr(xOffsetText, y + 2, item);
  int statWidth = u8g2.drawStr(xOffsetStatus, y + 2, stat);
  u8g2.drawFrame(xOffsetStatus - 2, y+1, statWidth+4, 11);
}

void phaseReverseOnOffActivate() {
  SerialMuted("Phase Reverse On Off Activate\n");
  externalDisplayRefresh = true;
}

void phaseReverseOnOffDeactivate() {
  SerialMuted("Phase Reverse On Off Deactivate\n");
}

char * phaseReverseOnOffTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        for(int i=0; i<noLoops + noOuts - 1; i++) {
          if(strcmp(phaseReverseState.currentItem, phaseReverseItemsState[i]) == 0) {
            if(strcmp(phaseReverseOnOffState.currentItem, "On") == 0) {
              phaseReverse[i] = true;
            } else {
              phaseReverse[i] = false;
            }
            savePhaseReverse();
            break;
          }
        }
        return "phaseReverse";        
      } else if(menuPins[i] == backPin) {
        return "phaseReverse";
      }
    }
  }

  return NULL;
}

state factoryResetState;
char* factoryResetItems[2];

void factoryResetActivate() {
  SerialMuted("Factory Reset Activate\n");
  externalDisplayRefresh = true;
}

void factoryResetDeactivate() {
  SerialMuted("Factory Reset Deactivate\n");
}

char * factoryResetTransitions() {
   for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        if(strcmp(factoryResetState.currentItem, "Ok") == 0) {
          displayPage = factoryResetDisplay;
          display();
          factoryReset();
          delay(storeDelay);
        }
        return "sysConfig";
      } else if(menuPins[i] == backPin) {
        return "sysConfig";
      }
    } 
  }
  return NULL;
}

state stompBoxModeState;
char* stompBoxModeItems[5];

//{offStomp = 0, normalStomp = 1, permanentStomp = 2, toggleStomp = 3, permanentBankStomp = 4}
void stompBoxModeActivate() {
  SerialMuted("Stomp Box Activate\n");
  externalDisplayRefresh = true;
  switch(((preset_t*)presetList->last->item)->stompMode) {
    case offStomp: stompBoxModeState.currentItem = stompBoxModeItems[0];break;
    case normalStomp: stompBoxModeState.currentItem = stompBoxModeItems[1];break;
    case permanentStomp: stompBoxModeState.currentItem = stompBoxModeItems[2];break;
    case toggleStomp: stompBoxModeState.currentItem = stompBoxModeItems[3];break;
    case permanentBankStomp: stompBoxModeState.currentItem = stompBoxModeItems[4];break;
  }
}

void stompBoxModeDeactivate() {
  SerialMuted("Stomp Box Deactivate\n");
}

char * stompBoxModeTransitions() {
  for(int i = 0; i < noMenuPins; i++) {
    if(menuNegEdges[i]) {
      if(menuPins[i] == enterPin) {
        stompModes preMode = ((preset_t*)presetList->last->item)->stompMode;
        if(strcmp(stompBoxModeState.currentItem, stompBoxModeItems[0]) == 0) {
          ((preset_t*)presetList->last->item)->stompMode = offStomp;
        } else if(strcmp(stompBoxModeState.currentItem, stompBoxModeItems[1]) == 0) {
          ((preset_t*)presetList->last->item)->stompMode = normalStomp;
          keepLast = true;
        }
        else if(strcmp(stompBoxModeState.currentItem, stompBoxModeItems[2]) == 0) {
          ((preset_t*)presetList->last->item)->stompMode = permanentStomp;
          keepLast = true;
        } else if(strcmp(stompBoxModeState.currentItem, stompBoxModeItems[3]) == 0) {
          ((preset_t*)presetList->last->item)->stompMode = toggleStomp;
          keepLast = true;
        } else if(strcmp(stompBoxModeState.currentItem, stompBoxModeItems[4]) == 0) {
          ((preset_t*)presetList->last->item)->stompMode = permanentBankStomp;
          keepLast = true;
        }
        returnState = "mainMenu";
        return "save";
      } else if(menuPins[i] == backPin) {
        return "mainMenu";
      }
    } 
  }
  return NULL;
}

void setupFSM() {

  loadPresetState.name = "loadPreset";
  loadPresetState.activate = loadPresetActivate;
  loadPresetState.deactivate = loadPresetDeactivate;
  loadPresetState.transitions = loadPresetTransitions;
  loadPresetState.isMenu = false;
  loadPresetState.menuItems = NULL;
  loadPresetState.menuDisp = NULL;
  loadPresetState.currentItem = NULL;
  loadPresetState.noItems = 0;
  loadPresetState.isConfig = false;
  
  runState.name = "run";
  runState.activate = runActivate;
  runState.deactivate = runDeactivate;
  runState.transitions = runTransitions;
  runState.isMenu = false;
  runState.menuItems = NULL;
  runState.menuDisp = NULL;
  runState.currentItem = NULL;
  runState.noItems = 0;
  runState.isConfig = false;

  loopProgramState.name = "loopProgram";
  loopProgramState.activate = loopProgramActivate;
  loopProgramState.deactivate = loopProgramDeactivate;
  loopProgramState.transitions = loopProgramTransitions;
  loopProgramState.isMenu = false;
  loopProgramState.menuItems = NULL;
  loopProgramState.menuDisp = NULL;
  loopProgramState.currentItem = NULL;
  loopProgramState.noItems = 0;
  loopProgramState.isConfig = false;

  secondaryProgramState.name = "secondaryProgram";
  secondaryProgramState.activate = secondaryProgramActivate;
  secondaryProgramState.deactivate = secondaryProgramDeactivate;
  secondaryProgramState.transitions = secondaryProgramTransitions;
  secondaryProgramState.isMenu = false;
  secondaryProgramState.menuItems = NULL;
  secondaryProgramState.menuDisp = NULL;
  secondaryProgramState.currentItem = NULL;
  secondaryProgramState.noItems = 0;
  secondaryProgramState.isConfig = false;

  saveState.name = "save";
  saveState.activate = saveActivate;
  saveState.deactivate = saveDeactivate;
  saveState.transitions = saveTransitions;
  saveState.isMenu = false;
  saveState.menuItems = NULL;
  saveState.menuDisp = NULL;
  saveState.currentItem = NULL;
  saveState.noItems = 0;
  saveState.isConfig = false;
  
  discardState.name = "discard";
  discardState.activate = discardActivate;
  discardState.deactivate = discardDeactivate;
  discardState.transitions = discardTransitions;
  discardState.isMenu = false;
  discardState.menuItems = NULL;
  discardState.menuDisp = NULL;
  discardState.currentItem = NULL;
  discardState.noItems = 0;
  discardState.isConfig = false;

  bankSelectState.name = "bankSelect";
  bankSelectState.activate = bankSelectActivate;
  bankSelectState.deactivate = bankSelectDeactivate;
  bankSelectState.transitions = bankSelectTransitions;
  bankSelectState.isMenu = false;
  bankSelectState.menuItems = NULL;
  bankSelectState.menuDisp = NULL;
  bankSelectState.currentItem = NULL;
  bankSelectState.noItems = 0;
  bankSelectState.isConfig = false;

  mainMenuItems[0] = "Loop Order";
  mainMenuItems[1] = "Stomp Box Mode";
  mainMenuItems[2] = "Midi";
  mainMenuItems[3] = "Tap Tempo";
  mainMenuItems[4] = "Copy";
  mainMenuItems[5] = "Reset";
  mainMenuItems[6] = "System Config";

  mainMenuState.name = "mainMenu";
  mainMenuState.activate = mainMenuActivate;
  mainMenuState.deactivate = mainMenuDeactivate;
  mainMenuState.transitions = mainMenuTransitions;
  mainMenuState.isMenu = true;
  mainMenuState.menuItems = mainMenuItems;
  mainMenuState.menuDisp = NULL;
  mainMenuState.currentItem = mainMenuItems[0];
  mainMenuState.noItems = 7;
  mainMenuState.menuHeader = "banknpreset";
  mainMenuState.menuStart = 0;
  mainMenuState.isConfig = false;

  sysConfigItems[0] = "Stereo Loops";
  sysConfigItems[1] = "Phase Reverse";
  sysConfigItems[2] = "Bank Config";
  sysConfigItems[3] = "Midi Config";
  sysConfigItems[4] = "Startup Preset";
  sysConfigItems[5] = "Brightness";
  sysConfigItems[6] = "Factory Reset";
  
  sysConfigState.name = "sysConfig";
  sysConfigState.activate = sysConfigActivate;
  sysConfigState.deactivate = sysConfigDeactivate;
  sysConfigState.transitions = sysConfigTransitions;
  sysConfigState.isMenu = true;
  sysConfigState.menuItems = sysConfigItems;
  sysConfigState.menuDisp = NULL;
  sysConfigState.currentItem = sysConfigItems[0];
  sysConfigState.noItems = 7;
  sysConfigState.menuHeader = "System Config";
  sysConfigState.menuStart = 0;
  sysConfigState.isConfig = false;

  for(int i=0; i<noTapTempo; i++) {
    sprintf(tapTempoItems[i], "Tap %d", i + 1);
    tapTempoItemsState[i] = tapTempoItems[i];
  }

  tapTempoState.name = "tapTempo";
  tapTempoState.activate = tapTempoActivate;
  tapTempoState.deactivate = tapTempoDeactivate;
  tapTempoState.transitions = tapTempoTransitions;
  tapTempoState.isMenu = true;
  tapTempoState.menuItems = tapTempoItemsState;
  tapTempoState.menuDisp = tapTempoDisplay;
  tapTempoState.currentItem = tapTempoItemsState[0];
  tapTempoState.noItems = noTapTempo;
  tapTempoState.menuHeader = "banknpreset";
  tapTempoState.menuDispHeight = 13;
  tapTempoState.menuStart = 0;
  tapTempoState.isConfig = false;

  for(int i=0; i<noMidiMsg; i++) {
    sprintf(midiItems[i], "Msg. %d", i + 1);
    midiItemsState[i] = midiItems[i];
  }

  midiState.name = "midi";
  midiState.activate = midiActivate;
  midiState.deactivate = midiDeactivate;
  midiState.transitions = midiTransitions;
  midiState.isMenu = true;
  midiState.menuItems = midiItemsState;
  midiState.menuDisp = midiDisplay;
  midiState.currentItem = midiItemsState[0];
  midiState.noItems = noMidiMsg;
  midiState.menuHeader = "banknpreset";
  midiState.menuDispHeight = 13;
  midiState.menuStart = 0;
  midiState.isConfig = false;

  midiConfigState.name = "midiConfig";
  midiConfigState.activate = midiConfigActivate;
  midiConfigState.deactivate = midiConfigDeactivate;
  midiConfigState.transitions = midiConfigTransitions;
  midiConfigState.isMenu = false;
  midiConfigState.isConfig = true;
  midiConfigState.noConfig = 0;
  midiConfigState.configTitles = NULL;
  midiConfigState.configValues = NULL;
  midiConfigState.configIdx = 0;

  midiSysConfigItems[0] = "In";
  midiSysConfigItems[1] = "Out";
  midiSysConfigItems[2] = "Through";
  midiSysConfigItems[3] = "In Ch.";
  midiSysConfigItems[4] = "Bank Ctrl.";

  midiSysConfigState.name = "midiSysConfig";
  midiSysConfigState.activate = midiSysConfigActivate;
  midiSysConfigState.deactivate = midiSysConfigDeactivate;
  midiSysConfigState.transitions = midiSysConfigTransitions;
  midiSysConfigState.isMenu = true;
  midiSysConfigState.menuItems = midiSysConfigItems;
  midiSysConfigState.menuDisp = midiSysConfigDisplay;
  midiSysConfigState.currentItem = midiSysConfigItems[0];
  midiSysConfigState.noItems = 5;
  midiSysConfigState.menuHeader = "Midi Sys Config";
  midiSysConfigState.menuDispHeight = 13;
  midiSysConfigState.menuStart = 0;
  midiSysConfigState.isConfig = false;

  midiConfigBoolItems[0] = "On";
  midiConfigBoolItems[1] = "Off";

  midiConfigBoolState.name = "midiSysBool";
  midiConfigBoolState.activate = midiConfigBoolActivate;
  midiConfigBoolState.deactivate = midiConfigBoolDeactivate;
  midiConfigBoolState.transitions = midiConfigBoolTransitions;
  midiConfigBoolState.isMenu = true;
  midiConfigBoolState.menuItems = midiConfigBoolItems;
  midiConfigBoolState.menuDisp = NULL;
  midiConfigBoolState.currentItem = midiConfigBoolItems[0];
  midiConfigBoolState.noItems = 2;
  midiConfigBoolState.menuStart = 0;
  midiConfigBoolState.isConfig = false;
  
  tapTempoConfigTitles[0] = "On/Off";
  tapTempoConfigTitles[1] = "Ch.";
  tapTempoConfigTitles[2] = "Ctrl.";
  tapTempoConfigTitles[3] = "On Val.";
  tapTempoConfigTitles[4] = "Off Val.";

  tapTempoConfigState.name = "tapTempoConfig";
  tapTempoConfigState.activate = tapTempoConfigActivate;
  tapTempoConfigState.deactivate = tapTempoConfigDeactivate;
  tapTempoConfigState.transitions = tapTempoConfigTransitions;
  tapTempoConfigState.isMenu = false;
  tapTempoConfigState.isConfig = true;
  tapTempoConfigState.noConfig = 5;
  tapTempoConfigState.configTitles = tapTempoConfigTitles;
  tapTempoConfigState.configValues = NULL;
  tapTempoConfigState.configIdx = 0;

  for(int i=0; i<noLoops/2; i++) {
    sprintf(stereoLoopsItems[i], "Loops %d & %d", (i*2) + 1, (i*2) + 2);
    stereoLoopsItemsState[i] = stereoLoopsItems[i];
  }

  midiInChannelItemsState[0] = midiInChannelItems[0];
  midiInChannelTitles[0] = "Channel";

  midiInChannelState.name = "midiSysInChannel";
  midiInChannelState.activate = midiInChannelActivate;
  midiInChannelState.deactivate = midiInChannelDeactivate;
  midiInChannelState.transitions = midiInChannelTransitions;
  midiInChannelState.isMenu = false;
  midiInChannelState.isConfig = true;
  midiInChannelState.noConfig = 1;
  midiInChannelState.menuHeader = "Midi In Channel";
  midiInChannelState.configTitles = midiInChannelTitles;
  midiInChannelState.configValues = midiInChannelItemsState;
  midiInChannelState.configIdx = 0;

  midiBankCtrlItemsState[0] = midiBankCtrlItems[0];
  midiBankCtrlTitles[0] = "Ctrl. No.";

  midiBankCtrlState.name = "midiSysBankCtrl";
  midiBankCtrlState.activate = midiBankCtrlActivate;
  midiBankCtrlState.deactivate = midiBankCtrlDeactivate;
  midiBankCtrlState.transitions = midiBankCtrlTransitions;
  midiBankCtrlState.isMenu = false;
  midiBankCtrlState.isConfig = true;
  midiBankCtrlState.noConfig = 1;
  midiBankCtrlState.menuHeader = "Midi Bank Ctrl";
  midiBankCtrlState.configTitles = midiBankCtrlTitles;
  midiBankCtrlState.configValues = midiBankCtrlItemsState;
  midiBankCtrlState.configIdx = 0;

  stereoLoopsState.name = "stereoLoops";
  stereoLoopsState.activate = stereoLoopsActivate;
  stereoLoopsState.deactivate = stereoLoopsDeactivate;
  stereoLoopsState.transitions = stereoLoopsTransitions;
  stereoLoopsState.isMenu = true;
  stereoLoopsState.menuItems = stereoLoopsItemsState;
  stereoLoopsState.menuDisp = stereoLoopsDisplay;
  stereoLoopsState.currentItem = stereoLoopsItemsState[0];
  stereoLoopsState.noItems = noLoops/2;
  stereoLoopsState.menuHeader = "Stereo Loops";
  stereoLoopsState.menuDispHeight = 13;
  stereoLoopsState.menuStart = 0;
  stereoLoopsState.isConfig = false;

  stereoLoopOnOffItems[0] = "Off";
  stereoLoopOnOffItems[1] = "On";

  stereoLoopOnOffState.name = "stereoLoopOnOff";
  stereoLoopOnOffState.activate = stereoLoopOnOffActivate;
  stereoLoopOnOffState.deactivate = stereoLoopOnOffDeactivate;
  stereoLoopOnOffState.transitions = stereoLoopOnOffTransitions;
  stereoLoopOnOffState.isMenu = true;
  stereoLoopOnOffState.menuItems = stereoLoopOnOffItems;
  stereoLoopOnOffState.menuDisp = NULL;
  stereoLoopOnOffState.noItems = 2;
  stereoLoopOnOffState.menuStart = 0;
  stereoLoopOnOffState.isConfig = false;

  for(int i=0; i<noLoops + noOuts - 1; i++) {
    if(i<noLoops) {
      sprintf(phaseReverseItems[i], "Loop %d", i+1);  
    } else {
      sprintf(phaseReverseItems[i], "Output %d", i-noLoops+2);
    }
    
    phaseReverseItemsState[i] = phaseReverseItems[i];
  }

  phaseReverseState.name = "phaseReverse";
  phaseReverseState.activate = phaseReverseActivate;
  phaseReverseState.deactivate = phaseReverseDeactivate;
  phaseReverseState.transitions = phaseReverseTransitions;
  phaseReverseState.isMenu = true;
  phaseReverseState.menuItems = phaseReverseItemsState;
  phaseReverseState.menuDisp = phaseReverseDisplay;
  phaseReverseState.currentItem = phaseReverseItemsState[0];
  phaseReverseState.noItems = noLoops + noOuts - 1;
  phaseReverseState.menuHeader = "Phase Reverse";
  phaseReverseState.menuDispHeight = 13;
  phaseReverseState.menuStart = 0;
  phaseReverseState.isConfig = false;

  phaseReverseOnOffItems[0] = "Off";
  phaseReverseOnOffItems[1] = "On";

  phaseReverseOnOffState.name = "phaseReverseOnOff";
  phaseReverseOnOffState.activate = phaseReverseOnOffActivate;
  phaseReverseOnOffState.deactivate = phaseReverseOnOffDeactivate;
  phaseReverseOnOffState.transitions = phaseReverseOnOffTransitions;
  phaseReverseOnOffState.isMenu = true;
  phaseReverseOnOffState.menuItems = phaseReverseOnOffItems;
  phaseReverseOnOffState.menuDisp = NULL;
  phaseReverseOnOffState.noItems = 2;
  phaseReverseOnOffState.menuStart = 0;
  phaseReverseOnOffState.isConfig = false;

  factoryResetItems[0] = "Ok";
  factoryResetItems[1] = "Cancel";

  factoryResetState.name = "factoryReset";
  factoryResetState.activate = factoryResetActivate;
  factoryResetState.deactivate = factoryResetDeactivate;
  factoryResetState.transitions = factoryResetTransitions;
  factoryResetState.isMenu = true;
  factoryResetState.menuItems = factoryResetItems;
  factoryResetState.menuDisp = NULL;
  factoryResetState.noItems = 2;
  factoryResetState.menuStart = 0;
  factoryResetState.menuHeader = "Factory Reset";
  factoryResetState.currentItem = factoryResetItems[0];
  factoryResetState.isConfig = false;

  stompBoxModeItems[0] = "Off";
  stompBoxModeItems[1] = "Normal";
  stompBoxModeItems[2] = "Permanent";
  stompBoxModeItems[3] = "Toggle";
  stompBoxModeItems[4] = "Bank Permanent";
  
  stompBoxModeState.name = "stompBoxMode";
  stompBoxModeState.activate = stompBoxModeActivate;
  stompBoxModeState.deactivate = stompBoxModeDeactivate;
  stompBoxModeState.transitions = stompBoxModeTransitions;
  stompBoxModeState.isMenu = true;
  stompBoxModeState.menuItems = stompBoxModeItems;
  stompBoxModeState.menuDisp = NULL;
  stompBoxModeState.noItems = 5;
  stompBoxModeState.menuStart = 0;
  stompBoxModeState.menuHeader = "Stomp Box Mode";
  stompBoxModeState.currentItem = NULL;
  stompBoxModeState.isConfig = false;

  resetPresetItems[0] = "Ok";
  resetPresetItems[1] = "Cancel";

  resetPresetState.name = "resetPreset";
  resetPresetState.activate = resetPresetActivate;
  resetPresetState.deactivate = resetPresetDeactivate;
  resetPresetState.transitions = resetPresetTransitions;
  resetPresetState.isMenu = true;
  resetPresetState.menuItems = resetPresetItems;
  resetPresetState.menuDisp = NULL;
  resetPresetState.noItems = 2;
  resetPresetState.menuStart = 0;
  resetPresetState.menuHeader = "Reset";
  resetPresetState.currentItem = resetPresetItems[0];
  resetPresetState.isConfig = false;
  
  states[0] = &loadPresetState;
  states[1] = &runState;
  states[2] = &loopProgramState;
  states[3] = &secondaryProgramState;
  states[4] = &saveState;
  states[5] = &bankSelectState;
  states[6] = &discardState;
  states[7] = &mainMenuState;
  states[8] = &sysConfigState;
  states[9] = &midiState;
  states[10] = &stereoLoopsState;
  states[11] = &stereoLoopOnOffState;
  states[12] = &phaseReverseState;
  states[13] = &phaseReverseOnOffState;
  states[14] = &factoryResetState;
  states[15] = &stompBoxModeState;
  states[16] = &resetPresetState;
  states[17] = &midiConfigState;
  states[18] = &tapTempoState;
  states[19] = &tapTempoConfigState;
  states[20] = &midiSysConfigState;
  states[21] = &midiConfigBoolState;
  states[22] = &midiInChannelState;
  states[23] = &midiBankCtrlState;
  
  curState = &loadPresetState;
}

void programChangeHandle(uint8_t channel, uint8_t data) {
  if(!midiInOn)
    return;
  SerialMuted("Midi In Channel: ");
  SerialMuted(channel);
  SerialMuted(", Data: ");
  SerialMuted(data);
  SerialMuted("\n");
  if(channel == midiInChannel && data < noPresetPins && curState == &runState) {
    activatePreset(bank, data+1);
    externalDisplayRefresh = true;
  }
}

void controlChangeHandle(uint8_t channel, uint8_t controller, uint8_t data) {
  if(!midiInOn)
    return;
  SerialMuted("Midi In Channel: ");
  SerialMuted(channel);
  SerialMuted(", Controller: ");
  SerialMuted(controller);
  SerialMuted(", Data: ");
  SerialMuted(data);
  SerialMuted("\n");
  if(channel == midiInChannel && controller == midiBankCtrl && data <= noBanks && data > 0 && curState == &runState) {
    bank = data;
    saveBank();
    externalDisplayRefresh = true;
  }
}

void matrixIsr() {
  readMatrixFlag = true;
}

void readLoopConn() {
  Adafruit_MCP23X08 * curMcp = NULL;
  for(int i = 0; i < 3*8; i++) {
    if(i == 0) {
      curMcp = &mcp_matrix1;
    } else if(i == 8) {
      curMcp = &mcp_matrix2;
    } else if(i == 16) {
      curMcp = &mcp_matrix3;
    }
    bool state = !curMcp->digitalRead(i%8);
    if(i < 20) {
      if(i % 2 == 0) {
        loopConnsSend[i/2] = state;
      } else {
        loopConnsReturn[i/2] = state;
      }
    } else if(i < 22) {
      outConns[i-20] = state;
    } else {
      exprConns[i-22] = state;
    }
  }
}

void setupMCP() {
  pinMode(INT_MATRIX, INPUT);
  pinMode(INT_STATUS, INPUT);

  if(!mcp_matrix1.begin_I2C(0x20))
    SerialMuted("Matrix 1 offline\n");
  if(!mcp_matrix2.begin_I2C(0x21))
    SerialMuted("Matrix 2 offline\n");
  if(!mcp_matrix3.begin_I2C(0x22))
    SerialMuted("Matrix 3 offline\n");
  if(!mcp_status1.begin_I2C(0x23))
    SerialMuted("Status 1 offline\n");
  if(!mcp_status2.begin_I2C(0x24))
    SerialMuted("Status 2 offline\n");
  if(!mcp_status3.begin_I2C(0x25))
    SerialMuted("Status 3 offline\n");

  Adafruit_MCP23X08 * curMcp = NULL;
  for(int i = 0; i < 3*8; i++) {
    if(i == 0) {
      curMcp = &mcp_matrix1;
      curMcp->setupInterrupts(true, true, CHANGE);
    } else if(i == 8) {
      curMcp = &mcp_matrix2;
      curMcp->setupInterrupts(true, true, CHANGE);
    } else if(i == 16) {
      curMcp = &mcp_matrix3;
      curMcp->setupInterrupts(true, true, CHANGE);
    }
    curMcp->pinMode(i%8, INPUT_PULLUP);
    curMcp->setupInterruptPin(i%8, LOW);
  }

  for(int i = 0; i < 3*8; i++) {
    if(i == 0) {
      curMcp = &mcp_status1;
      curMcp->setupInterrupts(true, true, LOW);
    } else if(i == 8) {
      curMcp = &mcp_status2;
      curMcp->setupInterrupts(true, true, LOW);
    } else if(i == 16) {
      curMcp = &mcp_status3;
      curMcp->setupInterrupts(true, true, LOW);
    }

    if(i < 18) {
      curMcp->pinMode(i%8, OUTPUT);
    } else {
      curMcp->pinMode(i%8, INPUT_PULLUP);
      curMcp->setupInterruptPin(i%8, CHANGE);
    }
  }
}

void setup() {
  // ToDO: Replace with PWM
  pinMode(REF_BLUE, OUTPUT);
  pinMode(REF_GREEN, OUTPUT);
  pinMode(REF_STATUS, OUTPUT);
  digitalWrite(REF_BLUE, HIGH);
  digitalWrite(REF_GREEN, HIGH);
  digitalWrite(REF_STATUS, HIGH);

  pinMode(I2C_RESET, OUTPUT);
  digitalWrite(I2C_RESET, LOW);
  delay(200);
  digitalWrite(I2C_RESET, HIGH);
  
  // put your setup code here, to run once:
  exprCount = 0;
  for(int i = 0; i < noExpr; i++) {
    exprVals[i] = 0;
  }
  Serial.begin(9600);
  while (!Serial) delay(10);

  setupMCP();
  for(int i = 0; i < noPresetPins; i++) {
    pinMode(presetPins[i], INPUT);
  }

  fram.begin(addrSizeInBytes);
  readMidiInChannel();
  readMidiBankCtrl();
  readMidiInOn();
  readMidiOutOn();
  readMidiThroughOn();
  u8g2.begin();
  u8g2.setContrast(225);
  midiInst.setHandleProgramChange(programChangeHandle);
  midiInst.setHandleControlChange(controlChangeHandle);
  midiInst.begin();
  midiInst.setInputChannel(MIDI_CHANNEL_OMNI);
  if(midiThroughOn)
    midiInst.turnThruOn();
  else
    midiInst.turnThruOff();
  display();
  delay(5000);
  attachInterrupt(digitalPinToInterrupt(INT_MATRIX), matrixIsr, FALLING);
  readLoopConn();
  //displayPage = mainDisplay;
  readBank();
  readStereoLoops();
  readPhaseReverse();
  readActivePresets();
  setupFSM();
  curState->activate();
  for(int i=0; i<noStates; i++) {
    SerialMuted(states[i]->name);
    SerialMuted("\n");
  }
  printFreeMemory();
}

void loop() {
  display();
  if(!digitalRead(INT_STATUS)) {
    SerialMuted("Interrupt\n");
    for(int i = 0; i < noMenuPins; i++) {
      menuMcpVals[i] = mcp_status3.digitalRead(menuPins[i]);
    }
  }
  bool pressDetect = false;
  detectMenuEdges();
  for(int i = 0; i < noMenuPins; i++) {
    if(menuEdges[i]) {
      SerialMuted("Menu Edge: ");
      SerialMuted(i);
      SerialMuted("\n");
      pressDetect = true;
    }
    if(menuNegEdges[i] || menuClock[i]) {
      SerialMuted("Menu Negative Edge: ");
      SerialMuted(i);
      SerialMuted("\n");
      if(curState->isMenu) {
        int menuMove = 0;
        if(menuPins[i] == upPin)
          menuMove = -1;
        else if(menuPins[i] == downPin)
          menuMove = 1;
        if(menuMove != 0) {
          int curSelect;  
          for(curSelect = 0; curSelect < curState->noItems; curSelect++) {
            if(strcmp(curState->currentItem, curState->menuItems[curSelect]) == 0)
              break;
          }
          curSelect += menuMove;
          if(curSelect >= 0 && curSelect < curState->noItems) {
            curState->currentItem = curState->menuItems[curSelect];
            externalDisplayRefresh = true;
          }
        }
      }
      pressDetect = true;
    }
    if(menuLongPress[i] || menuClockFast[i]) {
      SerialMuted("Menu Long Press: ");
      SerialMuted(i);
      SerialMuted("\n");
      pressDetect = true;
    }
  }
  detectPresetEdges();
  for(int i = 0; i< noPresetPins; i++) {
    if(presetEdges[i]) {
      SerialMuted("Preset Edge: ");
      SerialMuted(i);
      SerialMuted("\n");
      pressDetect = true;
    }
  }

  if(pressDetect || externalStateChange) {
    printFreeMemory();
    externalStateChange = false;
    char * nextState = curState->transitions();
    if(nextState) {
      bool foundState = false;
      for(int i = 0; i<sizeof(states)/sizeof(state*); i++) {
        state* posState = states[i];
        if(strcmp(posState->name, nextState) == 0) {
          curState->deactivate();
          curState = posState;
          curState->activate();
          if(curState->isMenu) {
            displayPage = menuDisplay;
          } else if(curState->isConfig) {
            displayPage = configDisplay;
          }
          foundState = true;
          break;
        }
      }
      if(!foundState) {
        SerialMuted("State ");
        SerialMuted(nextState);
        SerialMuted(" not found\n");
      }
    }
  }
  if(exprCount == exprDelay) {
    for(int i = 0; i < noExpr; i++) {
      uint16_t exprVal = analogRead(exprPins[i]);
      exprVal >>= 2;
      if(exprVal > exprVals[i] + exprDelta || exprVal < exprVals[i] - exprDelta) {
        exprVals[i] = exprVal;
        checkExpr(i);
        SerialMuted("Expr. Value: ");
        SerialMuted(exprVal);
        SerialMuted("\n");
      }
    }
  }

  for(int i = 0; i< noPresetPins; i++) {
    if(presetEdges[i]) {
      presetEdges[i] = false;
    }
  }
  for(int i = 0; i < noMenuPins; i++) {
    menuEdges[i] = false;
    menuNegEdges[i] = false;
    menuLongPress[i] = false;
    menuClock[i] = false;
    menuClockFast[i] = false;
  }
  if(exprCount < exprDelay)
    exprCount++;
  else 
    exprCount = 0;
  if(tapCheckOn) {
    checkTapTempo();
  }
  midiInst.read();

  if(readMatrixFlag && readMatrixCount < readMatrixDelay) {
    readMatrixCount += 1;
  } else if(readMatrixFlag) {
    readMatrixFlag = false;
    readMatrixCount = 0;
    readLoopConn();  
  }
  delay(loopDelay);
}
