#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
//Servo myservo;

const int servoPin = 5; // Define the pin connected to the servo signal

// Constants for the servo pulse width range in microseconds
const int minPulseWidth = 500;   // Corresponds to 0 degrees (500 microseconds)
const int maxPulseWidth = 2500;  // Corresponds to 270 degrees (2500 microseconds)
const int delayFactor = 1;       // Factor to slow down the movement

ESP8266WebServer server(80); // Create a web server on port 80

int currentAngle = 0; // Variable to keep track of the current servo position
//int targetAngle = 0; 
int lastState = 0;    // 0 means Closed, 1 means Open

// Replace these with your network credentials
const char* ssid = "Mravcho";
const char* password = "bulsatcom";

// EEPROM address where the servo position is stored
const int EEPROMAddress = 0;
const int EEPROMStateAddress = 4;

void setup() {
  Serial.begin(115200);
  delay(500);



  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Start the web server
  server.on("/", handleRoot);  // Define the root URL handling function
  server.on("/open", handleOpen);  // Define the /open URL handling function
  server.on("/close", handleClose);  // Define the /close URL handling function
  server.begin();  // Start the server
  Serial.println("Server started");


//  myservo.attach(servoPin);
pinMode(servoPin, OUTPUT);
  // Do not move the servo upon initialization

  // Retrieve the last known angle and state from EEPROM
Serial.println("Setup started");
EEPROM.begin(512);
Serial.println("EEPROM initialized");

currentAngle = EEPROM.read(EEPROMAddress) + (EEPROM.read(EEPROMAddress + 1) << 8);
Serial.println("Angle read from EEPROM");

//targetAngle = currentAngle;

lastState = EEPROM.read(EEPROMStateAddress);
Serial.println("State read from EEPROM");

// Existing debug statements
Serial.print("Restored angle from EEPROM: ");
Serial.println(currentAngle);
Serial.print("Restored state from EEPROM: ");
Serial.println(lastState);
  // Validate the retrieved angle and state
  if (currentAngle < 0 || currentAngle > 270) {
    currentAngle = 0; // Default to 0 degrees if invalid
  }

  if (lastState != 0 && lastState != 1) {
    lastState = 0; // Default to Closed state if invalid
  }

  Serial.print("Restored angle from EEPROM: ");
  Serial.println(currentAngle);
  Serial.print("Restored state from EEPROM: ");
  Serial.println(lastState);


}

void loop() {
  server.handleClient();  // Handle incoming client requests
 // Serial.println(millis());
//  moveServo(targetAngle);
}

// Function to handle the root URL
void handleRoot() {
  String html = "<html><body>";
  html += "<h1>Telescope Lid Control</h1>";
  html += "<form action=\"/open\" method=\"GET\">";
  html += "<input type=\"submit\" value=\"Open\">";
  html += "</form>";
  html += "<form action=\"/close\" method=\"GET\">";
  html += "<input type=\"submit\" value=\"Close\">";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Function to handle the /open URL
void handleOpen() {
  if (lastState != 1) { // Only move if the current state is not Open
    moveServo(270);  // Move servo to 270 degrees (Open)
   // targetAngle = 270;
    lastState = 1;  // Update the state to Open
    EEPROM.write(EEPROMStateAddress, lastState); // Store the state in EEPROM
    EEPROM.commit();
  }
  server.send(200, "text/html", "<html><body><h1>Lid Opened</h1><a href=\"/\">Go Back</a></body></html>");
}

void handleClose() {
  if (lastState != 0) { // Only move if the current state is not Closed
    moveServo(0);  // Move servo to 0 degrees (Close)
  //  targetAngle = 0;
    lastState = 0;  // Update the state to Closed
    EEPROM.write(EEPROMStateAddress, lastState); // Store the state in EEPROM
    EEPROM.commit();
  }
  server.send(200, "text/html", "<html><body><h1>Lid Closed</h1><a href=\"/\">Go Back</a></body></html>");
}

// Function to move the servo to a specified angle slowly
void moveServo(int targetAngle) {
//  Serial.print("Moving from ");
 // Serial.print(currentAngle);
 // Serial.print(" to ");
 // Serial.println(targetAngle);

  int stepDelay = 20 * delayFactor;  // Delay between steps
  unsigned long lastStepTime = 0;

  if (currentAngle < targetAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      
        int pulseWidth = map(pos, 0, 270, minPulseWidth, maxPulseWidth);
        writeServoPulse(servoPin, pulseWidth);
   //     delay(20);
        lastStepTime = millis(); 
        while (millis() - lastStepTime < stepDelay) {
      }
    }
  } else if (currentAngle > targetAngle) {
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      
        int pulseWidth = map(pos, 0, 270, minPulseWidth, maxPulseWidth);
        writeServoPulse(servoPin, pulseWidth);
   //     delay(20);
        lastStepTime = millis();  // Reset the step time
        while (millis() - lastStepTime < stepDelay) {
      }
    }
  }
  
  // Update the current position to the target position
  currentAngle = targetAngle;

  // Store the current position in EEPROM
  EEPROM.write(EEPROMAddress, currentAngle & 0xFF); // Store lower byte
  EEPROM.write(EEPROMAddress + 1, (currentAngle >> 8) & 0xFF); // Store upper byte
  EEPROM.commit();
  
  Serial.print("Stored angle to EEPROM: ");
  Serial.println(currentAngle);
}

// Function to generate the servo pulse
void writeServoPulse(int pin, int pulseWidth) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
  // int waiting = millis();
  //while ((millis() - waiting) == 20) {
  //}
  delay(20 - pulseWidth / 1000);  // Wait for the remainder of the 20ms frame time (50Hz frequency)
}