// Include required libraries
#include <Wire.h>            // For I2C communication
#include <Adafruit_Sensor.h> // For sensor abstraction
#include <Adafruit_ADXL345_U.h> // For ADXL345 accelerometer
#include <SoftwareSerial.h>  // For communication with GSM module

// Create instances for modules
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
SoftwareSerial gsm(7, 8); // RX, TX pins for GSM module

// Define GPS data pins
#define GPS_RX 9
#define GPS_TX 10
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// Variables for accident detection
const float threshold = 2.5; // Acceleration threshold for accident detection
boolean accidentDetected = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  gsm.begin(9600);
  gpsSerial.begin(9600);

  // Initialize accelerometer
  if (!accel.begin()) {
    Serial.println("Accelerometer not detected!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  
  // Setup complete message
  Serial.println("System initialized and ready.");
}

void loop() {
  sensors_event_t event;
  accel.getEvent(&event);

  // Check if the acceleration exceeds the threshold
  float totalAcceleration = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
  
  if (totalAcceleration > threshold) {
    accidentDetected = true;
    Serial.println("Accident detected!");

    // Get GPS data
    String gpsData = getGPSData();

    // Send alert
    if (accidentDetected) {
      sendAlert(gpsData);
      delay(10000); // Delay to prevent multiple alerts
      accidentDetected = false;
    }
  }
  delay(500); // Delay between readings
}

String getGPSData() {
  String gpsInfo = "";
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gpsInfo += c;
  }
  // Process GPS data as needed
  return gpsInfo;
}

void sendAlert(String gpsData) {
  gsm.println("AT+CMGF=1"); // Set SMS mode
  delay(100);
  gsm.println("AT+CMGS=\"+1234567890\""); // Replace with emergency contact number
  delay(100);
  gsm.print("Accident detected! Location: ");
  gsm.print(gpsData); // Send GPS data
  delay(100);
  gsm.write(26); // End SMS with Ctrl+Z
  delay(5000);
}
