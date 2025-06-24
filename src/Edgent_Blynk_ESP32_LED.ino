
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

void loop() {

  BlynkEdgent.run();

  if (debug==true){
    while (Serial2.available()){
      Serial.write(Serial2.read());
    }
  }else{

    if (millis()-requestTimer>60000*30){
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

  }

  if (digitalRead(39)==LOW){
    delay(300);
    Serial.println("Debug Toggled");
    debug=!debug;
  }

  delay(10);
}

