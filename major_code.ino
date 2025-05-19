#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11
#define MQ2_PIN A0
#define LDR_PIN A1
#define RELAY1_PIN 7  // Fan
#define RELAY2_PIN 8  // Bulb

#define TEMP_THRESHOLD 30.0
#define PRESSURE_THRESHOLD 1000.0
#define GAS_THRESHOLD 150
#define LDR_THRESHOLD 200

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);
  dht.begin();

  if (!bmp.begin(0x76)) {
    Serial.println("Could not find BMP280 sensor!");
    while (1);
  }

  lcd.begin(16, 2);
  lcd.backlight();

  pinMode(MQ2_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, LOW);
}

void loop() {
  static unsigned long lastReadTime = 0;
  if (millis() - lastReadTime >= 2000) {
    lastReadTime = millis();

    float temperature = dht.readTemperature();
    float pressure = bmp.readPressure() / 100.0;
    int gasLevel = analogRead(MQ2_PIN);
    int ldrValue = analogRead(LDR_PIN);

    Serial.println("=== Sensor Readings ===");
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("Gas: "); Serial.println(gasLevel);
    Serial.print("LDR: "); Serial.println(ldrValue);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:"); lcd.print(temperature); lcd.print("C ");
    lcd.print("G:"); lcd.print(gasLevel);
    lcd.setCursor(0, 1);
    lcd.print("P:"); lcd.print(pressure);

    bool airBad = (pressure > PRESSURE_THRESHOLD);
    bool gasBad = (gasLevel > GAS_THRESHOLD);

    if (temperature > TEMP_THRESHOLD || airBad || gasBad) {
      digitalWrite(RELAY1_PIN, LOW);   // Fan ON
      digitalWrite(RELAY2_PIN, HIGH);  // Bulb OFF
      Serial.println("ALERT: Fan ON, Bulb OFF");

      // ===== Air Quality Display =====
      unsigned long airStart = millis();
      while (millis() - airStart < 10000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Air Quality:");

        if (!gasBad && !airBad) {
          lcd.setCursor(0, 1); lcd.print("Good");
          Serial.println("Air Quality is Good");
        } else if (gasBad != airBad) {
          lcd.setCursor(0, 1); lcd.print("Manageable");
          Serial.println("Air Quality is Manageable");
        } else {
          lcd.setCursor(0, 1); lcd.print("Worst");
          Serial.println("Air Quality is Worst");
        }
        delay(2000);
      }

      // ===== Radiation Quality Display =====
      bool tempBad = (temperature > TEMP_THRESHOLD);
      bool lightBad = (ldrValue > LDR_THRESHOLD);

      unsigned long radStart = millis();
      while (millis() - radStart < 10000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Radiation:");

        if (!tempBad && !lightBad) {
          lcd.setCursor(0, 1); lcd.print("Good");
          Serial.println("Radiation is Good");
        } else if (tempBad != lightBad) {
          lcd.setCursor(0, 1); lcd.print("Manageable");
          Serial.println("Radiation is Manageable");
        } else {
          lcd.setCursor(0, 1); lcd.print("Worst");
          Serial.println("Radiation is Worst");
        }
        delay(2000);
      }

    } else {
      digitalWrite(RELAY1_PIN, HIGH);  // Fan OFF
      digitalWrite(RELAY2_PIN, LOW);   // Bulb ON
      Serial.println("NORMAL: Bulb ON, Fan OFF");
    }

    // Light Level Indicator
    lcd.setCursor(12, 1);
    if (ldrValue > LDR_THRESHOLD) {
      lcd.print("Sun");
      Serial.println("Bright (Sun Detected)");
    } else {
      lcd.print("Dark");
      Serial.println("Dark (No Sun)");
    }
  }

  delay(100);
}
