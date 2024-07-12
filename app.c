#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <DFRobot_Heartrate.h>
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi credentials
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

// Server endpoint for sending data
const char* serverEndpoint = "http://your-server-endpoint.com/api/data";

// MPU6050 instance for motion detection
MPU6050 mpu;

// ADS1115 instance for microphone (analog to digital converter)
Adafruit_ADS1115 ads;

// Heart rate sensor instance
DFRobot_Heartrate heartrateSensor(A0);

// Function to connect to WiFi
void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

// Function to initialize MPU6050
void initMPU6050() {
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

// Function to initialize ADS1115
void initADS1115() {
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115");
  } else {
    Serial.println("ADS1115 initialized");
  }
}

// Function to initialize heart rate sensor
void initHeartRateSensor() {
  heartrateSensor.begin();
}

// Function to send data to server
void sendDataToServer(String data) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverEndpoint);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(data);
    if (httpResponseCode > 0) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Error sending data");
    }
    http.end();
  } else {
    Serial.println("Not connected to WiFi");
  }
}

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  initMPU6050();
  initADS1115();
  initHeartRateSensor();
}

void loop() {
  // Read MPU6050 data for motion detection
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("Accel: "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.println(az);
  Serial.print("Gyro: "); Serial.print(gx); Serial.print(", "); Serial.print(gy); Serial.print(", "); Serial.println(gz);

  // Read heart rate data
  int heartRate = heartrateSensor.getValue();
  Serial.print("Heart Rate: "); Serial.println(heartRate);

  // Read microphone data
  int16_t adc0 = ads.readADC_SingleEnded(0);
  Serial.print("Microphone ADC: "); Serial.println(adc0);

  // Prepare data in JSON format
  String jsonData = "{";
  jsonData += "\"accel_x\":" + String(ax) + ",";
  jsonData += "\"accel_y\":" + String(ay) + ",";
  jsonData += "\"accel_z\":" + String(az) + ",";
  jsonData += "\"gyro_x\":" + String(gx) + ",";
  jsonData += "\"gyro_y\":" + String(gy) + ",";
  jsonData += "\"gyro_z\":" + String(gz) + ",";
  jsonData += "\"heart_rate\":" + String(heartRate) + ",";
  jsonData += "\"microphone\":" + String(adc0);
  jsonData += "}";

  // Send data to server
  sendDataToServer(jsonData);

  delay(1000); // Delay for 1 second before next reading
}
