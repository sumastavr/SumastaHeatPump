
// *** MAIN SETTINGS ***
// Replace this block with correct template settings.
// You can find it for every template here:
//
//   https://blynk.cloud/dashboard/templates

#define BLYNK_TEMPLATE_ID     "TMPL5Ep0MFoNy"
#define BLYNK_TEMPLATE_NAME   "LED ESP32"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

#define HEATING_CTRL_PIN 26
#define COOLING_CTRL_PIN 32

#include "BlynkEdgent.h"

#define LED_PIN 2  // Use pin 2 for LED (change it, if your board uses another pin)

// Parsed temperature variables (in °C)
float outdoorTemp = 0;
float waterOutTemp = 0;
float waterInTemp = 0;
float tapWaterTemp = 0;

long windowPushTimer=millis();
long pushTimeout=1200;

int heatPumpState=0;

// Standard CRC-16-MODBUS calculation function (re-used from previous examples).
// This function is kept for reference or if you later decide to calculate CRCs
// for values outside the lookup table.
// Polynomial: 0x8005 (or 0xA001 when reflected)
// Initial value: 0xFFFF
// Input data reflected: true
// Output CRC reflected: true
// Final XOR value: 0x0000
// @param data An array of bytes for which to calculate the CRC.
// @param length The number of bytes in the data array.
// @return The calculated 16-bit CRC checksum.
unsigned int calculateCRC16Modbus(const byte *data, int length) {
  unsigned int crc = 0xFFFF; // Initial CRC value

  for (int i = 0; i < length; i++) {
    crc ^= data[i]; // XOR current byte with CRC

    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) { // Check if the LSB is 1
        crc >>= 1;       // Shift right
        crc ^= 0xA001;   // XOR with polynomial (reflected 0x8005)
      } else {
        crc >>= 1;       // Shift right
      }
    }
  }
  return crc;
}

// Define a structure to hold each complete temperature message.
// This structure now stores the integer temperature value for easy lookup,
// and the 8-byte message (6 data bytes + 2 CRC bytes).
struct TemperatureMessage {
  int tempC;        // Temperature in Celsius (e.g., 40, 41, etc.)
  byte message[8];  // The full 8-byte message including data and CRC.
};

// --- Pre-calculated Lookup Table ---
// This const array stores all the messages from 40°C to 60°C,
// with a step of 1°C.
// Each entry includes:
// { Temperature in °C, {Byte1, Byte2, ..., Byte6 (data), CRC_LSB, CRC_MSB} }
// The temperature values are scaled by 10 (e.g., 40C -> 400 decimal).
// The format is 0x01 0x06 0x00 0x05 [Temp_MSB] [Temp_LSB] [CRC_LSB] [CRC_MSB]
const TemperatureMessage temperatureLookupTable[] PROGMEM = {
  {40, {0x01, 0x06, 0x00, 0x05, 0x01, 0x90, 0x30, 0xC5}}, // 400 dec -> 01 90, CRC: 0xC530 -> 30 C5
  {41, {0x01, 0x06, 0x00, 0x05, 0x01, 0x9A, 0x7A, 0xF3}}, // 410 dec -> 01 9A, CRC: 0xF37A -> 7A F3
  {42, {0x01, 0x06, 0x00, 0x05, 0x01, 0xA4, 0xBC, 0xED}}, // 420 dec -> 01 A4, CRC: 0xEDBC -> BC ED
  {43, {0x01, 0x06, 0x00, 0x05, 0x01, 0xAE, 0xFD, 0x85}}, // 430 dec -> 01 AE, CRC: 0x85FD -> FD 85
  {44, {0x01, 0x06, 0x00, 0x05, 0x01, 0xB8, 0x49, 0x07}}, // 440 dec -> 01 B8, CRC: 0x0749 -> 49 07
  {45, {0x01, 0x06, 0x00, 0x05, 0x01, 0xC2, 0x6B, 0x4A}}, // 450 dec -> 01 C2, CRC: 0x4A6B -> 6B 4A
  {46, {0x01, 0x06, 0x00, 0x05, 0x01, 0xCC, 0x2D, 0x89}}, // 460 dec -> 01 CC, CRC: 0x892D -> 2D 89
  {47, {0x01, 0x06, 0x00, 0x05, 0x01, 0xD6, 0xCB, 0x2F}}, // 470 dec -> 01 D6, CRC: 0x2FCB -> CB 2F
  {48, {0x01, 0x06, 0x00, 0x05, 0x01, 0xE0, 0xB3, 0x47}}, // 480 dec -> 01 E0, CRC: 0x47B3 -> B3 47
  {49, {0x01, 0x06, 0x00, 0x05, 0x01, 0xEA, 0x75, 0x84}}, // 490 dec -> 01 EA, CRC: 0x8475 -> 75 84
  {50, {0x01, 0x06, 0x00, 0x05, 0x01, 0xF4, 0xB7, 0x06}}, // 500 dec -> 01 F4, CRC: 0x06B7 -> B7 06
  {51, {0x01, 0x06, 0x00, 0x05, 0x01, 0xFE, 0x71, 0xC5}}, // 510 dec -> 01 FE, CRC: 0xC571 -> 71 C5
  {52, {0x01, 0x06, 0x00, 0x05, 0x02, 0x08, 0x99, 0x6D}}, // 520 dec -> 02 08, CRC: 0x6D99 -> 99 6D
  {53, {0x01, 0x06, 0x00, 0x05, 0x02, 0x12, 0x18, 0xA6}}, // 530 dec -> 02 12, CRC: 0x18A6 -> 18 A6
  {54, {0x01, 0x06, 0x00, 0x05, 0x02, 0x1C, 0x99, 0x62}}, // 540 dec -> 02 1C, CRC: 0x6299 -> 99 62
  {55, {0x01, 0x06, 0x00, 0x05, 0x02, 0x26, 0x19, 0x71}}, // 550 dec -> 02 26, CRC: 0x7119 -> 19 71
  {56, {0x01, 0x06, 0x00, 0x05, 0x02, 0x30, 0xDF, 0xB2}}, // 560 dec -> 02 30, CRC: 0xB2DF -> DF B2
  {57, {0x01, 0x06, 0x00, 0x05, 0x02, 0x3A, 0x71, 0xD8}}, // 570 dec -> 02 3A, CRC: 0xD871 -> 71 D8
  {58, {0x01, 0x06, 0x00, 0x05, 0x02, 0x44, 0xC6, 0x24}}, // 580 dec -> 02 44, CRC: 0x24C6 -> C6 24
  {59, {0x01, 0x06, 0x00, 0x05, 0x02, 0x4E, 0x0E, 0xE6}}, // 590 dec -> 02 4E, CRC: 0xE60E -> 0E E6
  {60, {0x01, 0x06, 0x00, 0x05, 0x02, 0x58, 0x32, 0xA5}}  // 600 dec -> 02 58, CRC: 0xA532 -> 32 A5
};

// Calculate the number of entries in the lookup table.
const int NUM_TEMP_ENTRIES = sizeof(temperatureLookupTable) / sizeof(temperatureLookupTable[0]);

// Function to get the full 8-byte message for a given integer temperature.
// @param tempC The temperature in Celsius (e.g., 40, 52).
// @param outputBuffer A pointer to a byte array of size 8, where the
//                     resulting message will be copied.
// @return true if the temperature was found in the table, false otherwise.
bool getTemperatureMessage(int tempC, byte* outputBuffer) {
  // Calculate the index for the lookup table.
  // The table starts at 40C and increments by 1C per entry.
  int index = tempC - 40;

  // Check if the calculated index is within the valid range of the table.
  if (index >= 0 && index < NUM_TEMP_ENTRIES) {
    // Copy the pre-calculated message from the lookup table to the output buffer.
    // Use `memcpy_P` to read from PROGMEM (program memory/flash).
    memcpy_P(outputBuffer, temperatureLookupTable[index].message, 8);
    return true; // Message found and copied.
  }
  return false; // Temperature not found in the defined range.
}

// Define a structure to hold each complete heating temperature message.
// This structure stores the integer temperature value for easy lookup,
// and the 8-byte message (6 data bytes + 2 CRC bytes).
struct HeatingTemperatureMessage {
  int tempC;        // Heating temperature in Celsius (e.g., 20, 21, etc.)
  byte message[8];  // The full 8-byte message including data and CRC.
};

// --- Pre-calculated Lookup Table for Heating Water Temperature ---
// This const array stores all the messages from 20°C to 40°C,
// with a step of 1°C.
// Each entry includes:
// { Temperature in °C, {Byte1, Byte2, ..., Byte6 (data), CRC_LSB, CRC_MSB} }
// The temperature values are scaled by 10 (e.g., 20C -> 200 decimal).
// The format is 0x01 0x06 0x00 0x03 [Temp_MSB] [Temp_LSB] [CRC_LSB] [CRC_MSB]
const HeatingTemperatureMessage heatingTemperatureLookupTable[] PROGMEM = {
  {20, {0x01, 0x06, 0x00, 0x03, 0x00, 0xC8, 0x49, 0xC8}}, // 200 dec -> 00 C8, CRC: 0xC849 -> 49 C8
  {21, {0x01, 0x06, 0x00, 0x03, 0x00, 0xD2, 0x88, 0x89}}, // 210 dec -> 00 D2, CRC: 0x8988 -> 88 89
  {22, {0x01, 0x06, 0x00, 0x03, 0x00, 0xDC, 0x6B, 0x4A}}, // 220 dec -> 00 DC, CRC: 0x4A6B -> 6B 4A
  {23, {0x01, 0x06, 0x00, 0x03, 0x00, 0xE6, 0x2D, 0x0B}}, // 230 dec -> 00 E6, CRC: 0x0B2D -> 2D 0B
  {24, {0x01, 0x06, 0x00, 0x03, 0x00, 0xF0, 0x2F, 0xCC}}, // 240 dec -> 00 F0, CRC: 0xCC2F -> 2F CC
  {25, {0x01, 0x06, 0x00, 0x03, 0x00, 0xFA, 0x6E, 0x0D}}, // 250 dec -> 00 FA, CRC: 0x0D6E -> 6E 0D
  {26, {0x01, 0x06, 0x00, 0x03, 0x01, 0x04, 0x82, 0x94}}, // 260 dec -> 01 04, CRC: 0x9482 -> 82 94
  {27, {0x01, 0x06, 0x00, 0x03, 0x01, 0x0E, 0xC3, 0x55}}, // 270 dec -> 01 0E, CRC: 0x55C3 -> C3 55
  {28, {0x01, 0x06, 0x00, 0x03, 0x01, 0x18, 0x78, 0x50}}, // 280 dec -> 01 18, CRC: 0x5078 -> 78 50 (Verified with your example)
  {29, {0x01, 0x06, 0x00, 0x03, 0x01, 0x22, 0xF8, 0x43}}, // 290 dec -> 01 22, CRC: 0x43F8 -> F8 43 (Verified with your example)
  {30, {0x01, 0x06, 0x00, 0x03, 0x01, 0x2C, 0x79, 0x87}}, // 300 dec -> 01 2C, CRC: 0x8779 -> 79 87 (Verified with your example)
  {31, {0x01, 0x06, 0x00, 0x03, 0x01, 0x36, 0xF8, 0x4C}}, // 310 dec -> 01 36, CRC: 0x4CF8 -> F8 4C (Verified with your example)
  {32, {0x01, 0x06, 0x00, 0x03, 0x01, 0x40, 0x79, 0xAA}}, // 320 dec -> 01 40, CRC: 0xAA79 -> 79 AA (Verified with your example)
  {33, {0x01, 0x06, 0x00, 0x03, 0x01, 0x4A, 0xAE, 0x6E}}, // 330 dec -> 01 4A, CRC: 0x6EAE -> AE 6E
  {34, {0x01, 0x06, 0x00, 0x03, 0x01, 0x54, 0xD3, 0x2F}}, // 340 dec -> 01 54, CRC: 0x2FD3 -> D3 2F
  {35, {0x01, 0x06, 0x00, 0x03, 0x01, 0x5E, 0xCB, 0xEF}}, // 350 dec -> 01 5E, CRC: 0xEFDC -> CB EF
  {36, {0x01, 0x06, 0x00, 0x03, 0x01, 0x68, 0xC8, 0xA7}}, // 360 dec -> 01 68, CRC: 0xA7C8 -> C8 A7
  {37, {0x01, 0x06, 0x00, 0x03, 0x01, 0x72, 0x0B, 0x66}}, // 370 dec -> 01 72, CRC: 0x660B -> 0B 66
  {38, {0x01, 0x06, 0x00, 0x03, 0x01, 0x7C, 0x06, 0x25}}, // 380 dec -> 01 7C, CRC: 0x2506 -> 06 25
  {39, {0x01, 0x06, 0x00, 0x03, 0x01, 0x86, 0x40, 0xE4}}, // 390 dec -> 01 86, CRC: 0xE440 -> 40 E4
  {40, {0x01, 0x06, 0x00, 0x03, 0x01, 0x90, 0x83, 0xA7}}  // 400 dec -> 01 90, CRC: 0xA783 -> 83 A7
};

// Calculate the number of entries in the lookup table.
const int NUM_HEATING_TEMP_ENTRIES = sizeof(heatingTemperatureLookupTable) / sizeof(heatingTemperatureLookupTable[0]);

// Function to get the full 8-byte heating message for a given integer temperature.
// @param tempC The heating temperature in Celsius (e.g., 20, 35).
// @param outputBuffer A pointer to a byte array of size 8, where the
//                     resulting message will be copied.
// @return true if the temperature was found in the table, false otherwise.
bool getHeatingTemperatureMessage(int tempC, byte* outputBuffer) {
  // Calculate the index for the lookup table.
  // The table starts at 20C and increments by 1C per entry.
  int index = tempC - 20;

  // Check if the calculated index is within the valid range of the table.
  if (index >= 0 && index < NUM_HEATING_TEMP_ENTRIES) {
    // Copy the pre-calculated message from the lookup table to the output buffer.
    // Use `memcpy_P` to read from PROGMEM (program memory/flash).
    memcpy_P(outputBuffer, heatingTemperatureLookupTable[index].message, 8);
    return true; // Message found and copied.
  }
  return false; // Temperature not found in the defined range.
}

// Define a structure to hold each complete cooling temperature message.
// This structure stores the integer temperature value for easy lookup,
// and the 8-byte message (6 data bytes + 2 CRC bytes).
struct CoolingTemperatureMessage {
  int tempC;        // Cooling temperature in Celsius (e.g., 7, 8, etc.)
  byte message[8];  // The full 8-byte message including data and CRC.
};

// --- Pre-calculated Lookup Table for Cooling Water Temperature ---
// This const array stores all the messages from 7°C to 22°C,
// with a step of 1°C.
// Each entry includes:
// { Temperature in °C, {Byte1, Byte2, ..., Byte6 (data), CRC_LSB, CRC_MSB} }
// The temperature values are scaled by 10 (e.g., 7C -> 70 decimal).
// The format is 0x01 0x06 0x00 0x01 [Temp_MSB] [Temp_LSB] [CRC_LSB] [CRC_MSB]
const CoolingTemperatureMessage coolingTemperatureLookupTable[] PROGMEM = {
  {7, {0x01, 0x06, 0x00, 0x01, 0x00, 0x46, 0x98, 0x75}}, // 70 dec -> 00 46, CRC: 0x7598 -> 98 75
  {8, {0x01, 0x06, 0x00, 0x01, 0x00, 0x50, 0x89, 0xB6}}, // 80 dec -> 00 50, CRC: 0xB689 -> 89 B6
  {9, {0x01, 0x06, 0x00, 0x01, 0x00, 0x5A, 0x48, 0xF7}}, // 90 dec -> 00 5A, CRC: 0xF748 -> 48 F7
  {10, {0x01, 0x06, 0x00, 0x01, 0x00, 0x64, 0x4C, 0xD4}}, // 100 dec -> 00 64, CRC: 0xD44C -> 4C D4
  {11, {0x01, 0x06, 0x00, 0x01, 0x00, 0x6E, 0x0D, 0x95}}, // 110 dec -> 00 6E, CRC: 0x950D -> 0D 95
  {12, {0x01, 0x06, 0x00, 0x01, 0x00, 0x78, 0x2A, 0x56}}, // 120 dec -> 00 78, CRC: 0x562A -> 2A 56
  {13, {0x01, 0x06, 0x00, 0x01, 0x00, 0x82, 0xEA, 0x17}}, // 130 dec -> 00 82, CRC: 0x17EA -> EA 17
  {14, {0x01, 0x06, 0x00, 0x01, 0x00, 0x8C, 0x6B, 0xD0}}, // 140 dec -> 00 8C, CRC: 0xD06B -> 6B D0
  {15, {0x01, 0x06, 0x00, 0x01, 0x00, 0x96, 0x2A, 0x91}}, // 150 dec -> 00 96, CRC: 0x912A -> 2A 91
  {16, {0x01, 0x06, 0x00, 0x01, 0x00, 0xA0, 0xD8, 0x72}}, // 160 dec -> 00 A0, CRC: 0x72D8 -> D8 72 (Verified with your example)
  {17, {0x01, 0x06, 0x00, 0x01, 0x00, 0xAA, 0x58, 0x75}}, // 170 dec -> 00 AA, CRC: 0x7558 -> 58 75 (Verified with your example)
  {18, {0x01, 0x06, 0x00, 0x01, 0x00, 0xB4, 0xD8, 0x7D}}, // 180 dec -> 00 B4, CRC: 0x7DD8 -> D8 7D (Verified with your example)
  {19, {0x01, 0x06, 0x00, 0x01, 0x00, 0xBE, 0x58, 0x7A}}, // 190 dec -> 00 BE, CRC: 0x7A58 -> 58 7A (Verified with your example)
  {20, {0x01, 0x06, 0x00, 0x01, 0x00, 0xC8, 0xD9, 0x9C}}, // 200 dec -> 00 C8, CRC: 0x9CD9 -> D9 9C (Verified with your example)
  {21, {0x01, 0x06, 0x00, 0x01, 0x00, 0xD2, 0x18, 0x5D}}, // 210 dec -> 00 D2, CRC: 0x5D18 -> 18 5D
  {22, {0x01, 0x06, 0x00, 0x01, 0x00, 0xDC, 0xDB, 0x1E}}  // 220 dec -> 00 DC, CRC: 0x1EDB -> DB 1E
};

// Calculate the number of entries in the lookup table.
const int NUM_COOLING_TEMP_ENTRIES = sizeof(coolingTemperatureLookupTable) / sizeof(coolingTemperatureLookupTable[0]);

// Function to get the full 8-byte cooling message for a given integer temperature.
// @param tempC The cooling temperature in Celsius (e.g., 7, 15).
// @param outputBuffer A pointer to a byte array of size 8, where the
//                     resulting message will be copied.
// @return true if the temperature was found in the table, false otherwise.
bool getCoolingTemperatureMessage(int tempC, byte* outputBuffer) {
  // Calculate the index for the lookup table.
  // The table starts at 7C and increments by 1C per entry.
  int index = tempC - 7;

  // Check if the calculated index is within the valid range of the table.
  if (index >= 0 && index < NUM_COOLING_TEMP_ENTRIES) {
    // Copy the pre-calculated message from the lookup table to the output buffer.
    // Use `memcpy_P` to read from PROGMEM (program memory/flash).
    memcpy_P(outputBuffer, coolingTemperatureLookupTable[index].message, 8);
    return true; // Message found and copied.
  }
  return false; // Temperature not found in the defined range.
}

void getWaterTemps() {
  uint8_t wakeCmd[] = { 0x01, 0x04, 0x00, 0x0E, 0x00, 0x03, 0xD1, 0xC8 };
  Serial2.write(wakeCmd, sizeof(wakeCmd));
  Serial.println("Get Water Temps");
}

void getOutdoorTemps() {
  // 01 04 00 3A 00 01 11 C7
  uint8_t wakeCmd[] = { 0x01, 0x04, 0x00, 0x3A, 0x00, 0x01, 0x11, 0xC7 };
  Serial2.write(wakeCmd, sizeof(wakeCmd));
  Serial.print(millis());
  Serial.print(" : ");
  Serial.println("Get Water Temps");
}

// 01 06 00 83 00 00 78 22
void disableDHW(){
  uint8_t disableDHWCommand[] = { 0x01, 0x06, 0x00, 0x83, 0x00, 0x00, 0x78, 0x22 };
  Serial2.write(disableDHWCommand, sizeof(disableDHWCommand));
  Serial.println("Disable DHW");
  delay(500);
  Serial2.write(disableDHWCommand, sizeof(disableDHWCommand));
}

void enableDHW(){
  uint8_t enableDHWCommand[] = { 0x01, 0x06, 0x00, 0x83, 0x00, 0x01, 0xB9, 0xE2 };
  Serial2.write(enableDHWCommand, sizeof(enableDHWCommand));
  Serial.println("Enable DHW");
  delay(500);
  Serial2.write(enableDHWCommand, sizeof(enableDHWCommand));
}


// V0 is a datastream used to transfer and store LED switch state.
// Evey time you use the LED switch in the app, this function
// will listen and update the state on device
BLYNK_WRITE(V0)
{
  // Local variable `value` stores the incoming LED switch state (1 or 0)
  // Based on this value, the physical LED on the board will be on or off:
  int value = param.asInt();

  if (value == 1) {
    Serial2.flush();
    getWaterTemps();
    delay(200);
    getOutdoorTemps();
    windowPushTimer=millis();
  } 
}

BLYNK_WRITE(V10)
{
  // Local variable `value` stores the incoming LED switch state (1 or 0)
  // Based on this value, the physical LED on the board will be on or off:
  int value = param.asInt();

  if (value == 1) {
    uint8_t wakeCmd[] = { 0x01, 0x05, 0x00, 0x6C, 0xFF, 0x00, 0x4C, 0x27 };
    Serial2.write(wakeCmd, sizeof(wakeCmd));
    Serial.println("Reset E101 Temps");
  } 
}

// Air conditioning Mode 
BLYNK_WRITE(V5){

  int value = param.asInt();
  heatPumpState=value;

  if (value == 0) {

    Serial.println("AC OFF");
    digitalWrite(HEATING_CTRL_PIN,LOW);
    digitalWrite(COOLING_CTRL_PIN,LOW);

  } else if (value == 1) {
 
    Serial.println("Heating Mode");
    digitalWrite(HEATING_CTRL_PIN,HIGH);
    digitalWrite(COOLING_CTRL_PIN,LOW);

  } else if (value == 2) {
 
    Serial.println("Cooling Mode");
    digitalWrite(HEATING_CTRL_PIN,LOW);
    digitalWrite(COOLING_CTRL_PIN,HIGH);
  }
}

// Setpoint Temperature
BLYNK_WRITE(V7){

  int value = param.asInt();
  float setpointTemp=value/10.0;
  Blynk.virtualWrite(V8, setpointTemp);
  
}

// Setpoint Heating Water Temperature
BLYNK_WRITE(V9){

  int value = param.asInt();
  Serial.print("New Setpoint Heating Temp: ");
  Serial.println(value); 

  int requestedTemp1 = value;
  byte messageBuffer1[8]; // Buffer to store the retrieved message
  if (getHeatingTemperatureMessage(requestedTemp1, messageBuffer1)) {
    Serial.print("Message for ");
    Serial.print(requestedTemp1);
    Serial.print("C: ");
    for (int i = 0; i < 8; i++) {
      if (messageBuffer1[i] < 0x10) Serial.print("0");
      Serial.print(messageBuffer1[i], HEX);
      Serial.print(" ");
    }
    for(int i=0;i<1;i++){
      Serial2.write(messageBuffer1,sizeof(messageBuffer1));
      delay(250);
    }
    Serial.println();
  }

}

// Setpoint Cooling Water Temperature
BLYNK_WRITE(V11){

  int value = param.asInt();
  Serial.print("New Setpoint Cooling Temp: ");
  Serial.println(value); 

  int requestedTemp1 = value;
  byte messageBuffer1[8]; // Buffer to store the retrieved message
  if (getCoolingTemperatureMessage(requestedTemp1, messageBuffer1)) {
    Serial.print("Message for ");
    Serial.print(requestedTemp1);
    Serial.print("C: ");
    for (int i = 0; i < 8; i++) {
      if (messageBuffer1[i] < 0x10) Serial.print("0");
      Serial.print(messageBuffer1[i], HEX);
      Serial.print(" ");
    }
    for(int i=0;i<1;i++){
      Serial2.write(messageBuffer1,sizeof(messageBuffer1));
      delay(250);
    }
    Serial.println();
  }
}

// Setpoint DHW Water Temperature
BLYNK_WRITE(V12){

  int value = param.asInt();
  Serial.print("New Setpoint DHW Temp: ");
  Serial.println(value); 

  int requestedTemp1 = value;
  byte messageBuffer1[8]; // Buffer to store the retrieved message
  if (getTemperatureMessage(requestedTemp1, messageBuffer1)) {
    Serial.print("Message for ");
    Serial.print(requestedTemp1);
    Serial.print("C: ");
    for (int i = 0; i < 8; i++) {
      if (messageBuffer1[i] < 0x10) Serial.print("0");
      Serial.print(messageBuffer1[i], HEX);
      Serial.print(" ");
    }

    for(int i=0;i<1;i++){
      Serial2.write(messageBuffer1,sizeof(messageBuffer1));
      delay(250);
    }

    Serial.println();
  }
  //Serial2.write(wakeCmd, sizeof(wakeCmd));
}

// DHW Heating Toggle
BLYNK_WRITE(V13){
  int value = param.asInt();
  Serial.print("DHW Heating: ");
  Serial.println(value);

  if (value==0){
    uint8_t disableDHWCommand[] = { 0x01, 0x06, 0x00, 0x83, 0x00, 0x00, 0x78, 0x22 };
    //uint8_t disableDHWCommand[] = { 0x01, 0x06, 0x00, 0x83, 0x00, 0x01, 0xB9, 0xE2 };
    Serial2.write(disableDHWCommand, sizeof(disableDHWCommand));
    Serial.println("Disable DHW");
  }else{
    uint8_t enableDHWCommand[] = { 0x01, 0x06, 0x00, 0x83, 0x00, 0x01, 0xB9, 0xE2 };
    /*
    uint8_t enableDHWCommand[] = {
      0x01, 0x03, 0x00, 0xE3, 0x00, 0x23, 0xF5, 0xE5,
      0x01, 0x03, 0x46, 0x00, 0x50, 0x00, 0x3C, 0x05,
      0xDC, 0x00, 0x00, 0x00, 0xC8, 0x02, 0x58, 0x03,
      0xAC, 0x03, 0xD4, 0x03, 0xFC, 0x04, 0x24, 0x04,
      0xE2, 0x00, 0x3C, 0x03, 0x84, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x07, 0x03, 0x55, 0x00, 0x55, 0x02,
      0x8A, 0x02, 0x26, 0x02, 0x8A, 0x02, 0x26, 0x00,
      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x04, 0x00, 0x04, 0x00, 0x01, 0x00, 0x01, 0x00,
      0x00, 0x00, 0xC8, 0x00, 0x00, 0x01, 0xF4, 0x00,
      0x00, 0x0C, 0x37
    };
    */
    for(int i=0;i<2;i++){
      Serial2.write(enableDHWCommand, sizeof(enableDHWCommand));
      delay(500);
    }
    Serial.println("Enable DHW");
  }
}


#define FRAME_TIMEOUT_MS 20
#define MAX_FRAME_SIZE 64

// Frame buffer
uint8_t frameBuffer[MAX_FRAME_SIZE];
size_t framePos = 0;
unsigned long lastByteTime = 0;

void processBuffer(uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len - 4; i++) {
    // Look for Modbus response header: [0x01][0x04][byteCount]
    if (buf[i] == 0x01 && buf[i + 1] == 0x04) {
      uint8_t byteCount = buf[i + 2];
      size_t frameLen = 3 + byteCount + 2;  // header + data + CRC

      if (i + frameLen <= len) {
        // Extract and parse frame
        parseFrame(buf + i, frameLen);
        i += frameLen - 1; // skip to end of this frame
      }
    }
  }
}

void parseFrame(uint8_t *buf, size_t len) {
  uint8_t byteCount = buf[2];

  // Outdoor Temp Frame: 2 data bytes
  if (byteCount == 2 && len >= 7) {
    int16_t rawOutdoor = (buf[3] << 8) | buf[4];
    outdoorTemp = rawOutdoor / 10.0;
    Serial.printf("🌤️  Outdoor Temp: %.1f °C\n", outdoorTemp);
    float temperatureIndoor = temperatureRead();

    if (millis()-windowPushTimer<pushTimeout){
      Blynk.virtualWrite(V1, outdoorTemp);
      Blynk.virtualWrite(V6, temperatureIndoor);
    }

  }

  // Water Frame: 3 registers (6 data bytes)
  else if (byteCount == 6 && len >= 11) {
    int16_t rawWaterIn  = (buf[3] << 8) | buf[4];
    int16_t rawWaterOut = (buf[5] << 8) | buf[6];
    int16_t rawTapWater = (buf[7] << 8) | buf[8];

    waterInTemp   = rawWaterIn / 10.0;
    waterOutTemp  = rawWaterOut / 10.0;
    tapWaterTemp  = rawTapWater / 10.0;

    Serial.println("💧 Water Temperatures:");
    Serial.printf("   IN   : %.1f °C\n", waterInTemp);
    Serial.printf("   OUT  : %.1f °C\n", waterOutTemp);
    Serial.printf("   TAP  : %.1f °C\n", tapWaterTemp);

    if (millis()-windowPushTimer<pushTimeout){
      Blynk.virtualWrite(V2, waterOutTemp);
      Blynk.virtualWrite(V3, waterInTemp);
      Blynk.virtualWrite(V4, tapWaterTemp);
    }
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  // Debug console. Make sure you have the same baud rate selected in your serial monitor
  Serial.begin(115200);
  delay(100);

  BlynkEdgent.begin();

  Serial2.begin(19200, SERIAL_8N1, 22, 19);
  pinMode(39,INPUT);

  pinMode(HEATING_CTRL_PIN, OUTPUT);
  pinMode(COOLING_CTRL_PIN, OUTPUT);

  digitalWrite(HEATING_CTRL_PIN,LOW);
  digitalWrite(COOLING_CTRL_PIN,LOW);

}

long requestTimer=millis();

bool debug=false;

long timerToggleDHW=millis();
long toggleDHWInterval=3600000;

void loop() {

  BlynkEdgent.run();

  if (debug==true){
    while (Serial2.available()){
      Serial.write(Serial2.read());
    }

  }else{

    if (millis()-requestTimer>1800000){
      Serial2.flush();
      getWaterTemps();
      delay(200);
      getOutdoorTemps();
      requestTimer=millis();
      windowPushTimer=millis();
    }

    while (Serial2.available()) {
      uint8_t b = Serial2.read();
      if (framePos < MAX_FRAME_SIZE) {
        frameBuffer[framePos++] = b;
        lastByteTime = millis();
      }
    }

    if (framePos > 0 && millis() - lastByteTime > FRAME_TIMEOUT_MS) {
      processBuffer(frameBuffer, framePos);
      framePos = 0;
    }

    if (heatPumpState==0){
      if(millis()-timerToggleDHW>toggleDHWInterval){
        disableDHW();
        long tagTime=millis();
        Serial.println();
        Serial.println("Restarting DHW");
        Serial.println();
        while(millis()-tagTime<40000){
          BlynkEdgent.run();
          delay(10);
        }
        enableDHW();
        delay(1000);
        Serial2.flush();
        timerToggleDHW=millis();
      }
    }

  }

  if (digitalRead(39)==LOW){
    delay(300);
    Serial.println("Debug Toggled");
    debug=!debug;
  }

  delay(10);
}

