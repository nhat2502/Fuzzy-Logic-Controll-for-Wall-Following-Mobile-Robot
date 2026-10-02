#include <Arduino.h>
#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

float SP = 1.5;  // Setpoint (mặc định)
float PV = 0.0;  // Giá trị đo hiện tại

void setup() {
  Serial.begin(9600);          // UART với Arduino
  SerialBT.begin("ESP32_BT");  // Bluetooth Serial
  Serial.println("ESP32 PV/SP gateway started");
}

// Chuyển chuỗi sang float an toàn, giới hạn 0.0 -> 3.0
float parseFloatSafe(String str) {
  str.trim();
  if (str.length() == 0) return NAN;
  char buf[16];
  str.toCharArray(buf, 16);
  float val = atof(buf);
  if (val < 0.0) val = 0.0;
  if (val > 3.0) val = 3.0;
  return val;
}

void loop() {
  // 1️⃣ Arduino gửi PV qua UART → ESP32 → WinForms
  if (Serial.available()) {
    String pvStr = Serial.readStringUntil('\n');
    float pvVal = parseFloatSafe(pvStr);
    if (!isnan(pvVal)) {
      PV = pvVal;

      // Gửi PV qua Bluetooth Serial
      SerialBT.println(PV, 2);
    }
  }

  // 2️⃣ WinForms gửi SP qua Bluetooth → ESP32 → Arduino
  if (SerialBT.available()) {
    String spStr = SerialBT.readStringUntil('\n');
    float spVal = parseFloatSafe(spStr);
    if (!isnan(spVal)) {
      SP = spVal;

      // Gửi SP về Arduino
      Serial.println(SP, 2);
    }
  }

  delay(10);
}
