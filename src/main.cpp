#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag(12345);

#define CALIBRATION_BLINK_FREQUENCY 5
#define CALIBRATION_TIME_MILLISECONDS 20000
#define MAIN_LOOP_FREQUENCY 50
float max_value[3];
float min_value[3];
bool is_pointing_north = false;
bool is_calibrating = true;

void initialize_values() {
  for(int i = 0; i<3;i++) {
    max_value[i] = -INFINITY;
    min_value[i] = INFINITY;
  }
}

void check_HMC5883_connection() {
  Serial.println("Checking HMC5883 I2C connection");
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {  // ACK -> device present
      Serial.printf("Found an I2C Device at: 0x%02X, ", a);
      if(a == 0x1E) {
        Serial.printf("found the HMC5883!\n");
        return;
      }
      else
        Serial.printf("this is not the HMC5883 (it should be at 0x1E), continuing search...\n");
    }
  }
  while(1) { Serial.printf("Could not find the HMC5883\n"); delay(500); }
}



void read_HMC5883_data() {
  sensors_event_t event;
  mag.getEvent(&event);                  // microTesla
  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;
  if(is_calibrating) {
    Serial.printf("Calibrating, %.0fs ", (CALIBRATION_TIME_MILLISECONDS - millis())/(float)1000);

    max_value[0] = max(x, max_value[0]);
    max_value[1] = max(y, max_value[1]);
    max_value[2] = max(z, max_value[2]);

    min_value[0] = min(x, min_value[0]);
    min_value[1] = min(y, min_value[1]);
    min_value[2] = min(z, min_value[2]);
  }

  float normalized_x = (x - min_value[0]) / (max_value[0] - min_value[0]) * 2 - 1; 
  float normalized_y = (y - min_value[1]) / (max_value[1] - min_value[1]) * 2 - 1; 
  float normalized_z = (z - min_value[2]) / (max_value[2] - min_value[2]) * 2 - 1; 

  float heading = atan2(normalized_z,
                        normalized_x) * 180.0 / PI;
  if (heading < 0) heading += 360.0;

  is_pointing_north = heading < 10 or heading > 350;

  Serial.printf("X:%.2f  Y:%.2f  Z:%.2f  Heading:%.1f°\n",
                normalized_x,
                normalized_y,
                normalized_z,
                heading);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  check_HMC5883_connection();
  initialize_values();

  if (!mag.begin()) {
    while (1) { Serial.println("Library Error: HMC5883L not found"); delay(500); }
  }

  // Setup LED
  pinMode(LED_PIN, OUTPUT);
}

void led_logic() {
  if(is_calibrating) {
    digitalWrite(LED_PIN, ((CALIBRATION_BLINK_FREQUENCY * millis()/1000) % 2)? HIGH:LOW);    // LED off
  }
  else {
    digitalWrite(LED_PIN, is_pointing_north? HIGH : LOW);    // LED off
  }
}



void loop() {
  is_calibrating = millis() < CALIBRATION_TIME_MILLISECONDS;
  read_HMC5883_data();
  led_logic();

  delay(1000/MAIN_LOOP_FREQUENCY);
}