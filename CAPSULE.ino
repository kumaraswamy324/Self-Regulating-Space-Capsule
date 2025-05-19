#include <DHT.h>

#define DHTPIN 2        // DHT11 data pin connected to Pin 2
#define DHTTYPE DHT11   // Define the type of DHT sensor
#define LDR_PIN 3       // LDR sensor connected to digital pin 3
#define RELAY1_PIN 7    // Relay 1 connected to pin 7
#define RELAY2_PIN 8    // Relay 2 connected to pin 8

DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor

void setup() {
  Serial.begin(9600);       // Start serial communication
  dht.begin();              // Start DHT sensor

  pinMode(LDR_PIN, INPUT);      // Set LDR pin as input
  pinMode(RELAY1_PIN, OUTPUT);  // Set Relay 1 as output
  pinMode(RELAY2_PIN, OUTPUT);  // Set Relay 2 as output

  digitalWrite(RELAY1_PIN, HIGH); // Initially turn off Relay 1
  digitalWrite(RELAY2_PIN, HIGH); // Initially turn off Relay 2
}

void loop() {
  float temperature = dht.readTemperature();  // Read temperature

  if (isnan(temperature)) {  // Check if reading is valid
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  if (temperature > 30) {
    // Turn on Relay 1 if temperature is above 30°C
    digitalWrite(RELAY1_PIN, LOW);
    Serial.println("Relay 1 ON (Temperature above 30°C)");
  } 
  else if (temperature < 30) {
    // Check LDR if temperature is below 28°C
    int ldrValue = digitalRead(LDR_PIN);  // Read LDR value
    Serial.print("LDR Value: ");
    Serial.println(ldrValue);

    if (ldrValue == 1) {  // No sunlight
      digitalWrite(RELAY2_PIN, LOW);  // Turn on Relay 2
      Serial.println("Relay 2 ON (No sunlight)");
    } else {
      digitalWrite(RELAY2_PIN, HIGH);  // Sunlight detected, Relay 2 OFF
      Serial.println("Relay 2 OFF (Sunlight detected)");
    }
  } else {
    // Turn off both relays if no conditions are met
    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, HIGH);
    Serial.println("Both relays OFF");
  }

  delay(2000);  // Delay for 2 seconds before next reading
}
