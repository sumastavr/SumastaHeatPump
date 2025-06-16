
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

#include "BlynkEdgent.h"

#define LED_PIN 2  // Use pin 2 for LED (change it, if your board uses another pin)

// Parsed temperature variables (in °C)
float outdoorTemp = 0;
float waterOutTemp = 0;
float waterInTemp = 0;
float tapWaterTemp = 0;


// V0 is a datastream used to transfer and store LED switch state.
// Evey time you use the LED switch in the app, this function
// will listen and update the state on device
BLYNK_WRITE(V0)
{
  // Local variable `value` stores the incoming LED switch state (1 or 0)
  // Based on this value, the physical LED on the board will be on or off:
  int value = param.asInt();

  if (value == 1) {
    digitalWrite(LED_PIN, HIGH);
    Serial.print("value =");
    Serial.println(value);
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.print("value = ");
    Serial.println(value);
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
  }
}

void wakeUpHeatPump() {
  uint8_t wakeCmd[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x3C, 0xC4, 0x0E };
  Serial2.write(wakeCmd, sizeof(wakeCmd));
  Serial.println("Sent wake-up poll to heat pump");
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  // Debug console. Make sure you have the same baud rate selected in your serial monitor
  Serial.begin(115200);
  delay(100);

  BlynkEdgent.begin();

  Serial2.begin(19200, SERIAL_8N1, 22, 19);

}

long requestTimer=millis();

void loop() {
  BlynkEdgent.run();

  //while (Serial2.available()){
  //  Serial.write(Serial2.read());
  //}

  if (millis()-requestTimer>5000){
    wakeUpHeatPump();
    requestTimer=millis();
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



  delay(10);
}

