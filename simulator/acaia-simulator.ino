/*
  AcaiaSimulatorBLE.ino - Arduino sketch to simulate an Acaia-like BLE peripheral.
  Created by ChatGPT, April 27, 2024.
  Released into the public domain.

  This sketch simulates an Acaia scale by advertising BLE services and characteristics,
  handling incoming commands, and sending simulated weight data.
*/

#include <ArduinoBLE.h>

// ====== Configuration ======

// Define UUIDs for the custom service and characteristics.
// Replace these UUIDs with the actual ones used by your Acaia scale if different.
// #define ACAIA_SERVICE_UUID           "12345678-1234-5678-1234-56789abcdef0" // Example UUID
// #define ACAIA_WRITE_CHAR_UUID        "12345678-1234-5678-1234-56789abcdef1" // Example UUID
// #define ACAIA_READ_NOTIFY_CHAR_UUID  "12345678-1234-5678-1234-56789abcdef2" // Example UUID

#define ACAIA_SERVICE_UUID           "A8D45EF-F4B3-2D6F-EC17-5D17BD1D272F" // Example UUID
#define ACAIA_WRITE_CHAR_UUID        "49535343-8841-43f4-a8d4-ecbe34729bb3"
#define ACAIA_READ_NOTIFY_CHAR_UUID  "49535343-1e4d-4bd9-ba61-23c647249616"

// Define command byte arrays (must match the client's expected commands)
const byte IDENTIFY[20]             = { 0xef, 0xdd, 0x0b, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x9a, 0x6d };
const byte HEARTBEAT[7]             = { 0xef, 0xdd, 0x00, 0x02, 0x00, 0x02, 0x00 };
const byte NOTIFICATION_REQUEST[14] = { 0xef, 0xdd, 0x0c, 0x09, 0x00, 0x01, 0x01, 0x02, 0x02, 0x05, 0x03, 0x04, 0x15, 0x06 };
const byte START_TIMER[7]           = { 0xef, 0xdd, 0x0d, 0x00, 0x00, 0x00, 0x00 };
const byte STOP_TIMER[7]            = { 0xef, 0xdd, 0x0d, 0x00, 0x02, 0x00, 0x02 };
const byte RESET_TIMER[7]           = { 0xef, 0xdd, 0x0d, 0x00, 0x01, 0x00, 0x01 };
const byte TARE_ACAIA[6]            = { 0xef, 0xdd, 0x04, 0x00, 0x00, 0x00 };
const byte TARE_GENERIC[6]          = { 0x03, 0x0a, 0x01, 0x00, 0x00, 0x08 };
const byte START_TIMER_GENERIC[6]   = { 0x03, 0x0a, 0x04, 0x00, 0x00, 0x0a };
const byte STOP_TIMER_GENERIC[6]    = { 0x03, 0x0a, 0x05, 0x00, 0x00, 0x0d };
const byte RESET_TIMER_GENERIC[6]   = { 0x03, 0x0a, 0x06, 0x00, 0x00, 0x0c };

// Define packet structures
const int HEARTBEAT_PERIOD_MS = 30000; // Example heartbeat period

// ====== BLE Characteristics ======

// Create the BLE Service
BLEService acaiaService(ACAIA_SERVICE_UUID);

// Create the Write Characteristic
BLECharacteristic acaiaWriteCharacteristic(
  ACAIA_WRITE_CHAR_UUID,
  BLEWrite | BLEWriteWithoutResponse,
  20 // Maximum length
);

// Create the Read/Notify Characteristic
BLECharacteristic acaiaReadNotifyCharacteristic(
  ACAIA_READ_NOTIFY_CHAR_UUID,
  BLEWrite | BLENotify,
  20 // Maximum length
);

// ====== Variables ======

unsigned long lastHeartbeatTime = 0;
float simulatedWeight = 0.0; // Starting weight
bool timerRunning = false;

// ====== Setup ======

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; } // Wait for Serial to be ready

  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Set device name and local name
  BLE.setLocalName("LUNAR_SIM");
  BLE.setDeviceName("LUNAR_SIM");
  BLE.setAppearance(0x0840); // Example appearance value for a scale

  // Add characteristics to service first
  acaiaService.addCharacteristic(acaiaWriteCharacteristic);
  Serial.println("Write Characteristic added.");

  acaiaService.addCharacteristic(acaiaReadNotifyCharacteristic);
  Serial.println("Read/Notify Characteristic added.");

  // Now add the service
  BLE.addService(acaiaService);
  Serial.println("Service added.");
  
  // Set initial value for notify characteristic
  acaiaReadNotifyCharacteristic.writeValue((const byte*)IDENTIFY, sizeof(IDENTIFY));

  // Start advertising
  BLE.advertise();
  Serial.println("Acaia Simulator BLE Peripheral is now advertising.");
}

// ====== Loop ======

void loop() {
  // Listen for BLE events
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Check for incoming writes
      if (acaiaWriteCharacteristic.written()) {
        handleWriteCommand(acaiaWriteCharacteristic.value(), acaiaWriteCharacteristic.valueLength());
      }

      if (acaiaReadNotifyCharacteristic.written()) {
        handleReadCommand(acaiaReadNotifyCharacteristic.value(), acaiaReadNotifyCharacteristic.valueLength());
      }

      // Send simulated weight data periodically
      if (timerRunning) {
        unsigned long currentTime = millis();
        if (currentTime - lastHeartbeatTime > 1000) { // Update weight every second
          lastHeartbeatTime = currentTime;
          updateSimulatedWeight();
          sendWeightData();
        }
      }

      delay(10); // Small delay to prevent overwhelming the loop
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

// ====== Helper Functions ======

void handleReadCommand(const byte* data, int length) {
  Serial.print("Received read command (");
  Serial.print(length);
  Serial.print(" bytes): ");
  printData(data, length);
  Serial.println();
}
// Handle incoming write commands
void handleWriteCommand(const byte* data, int length) {
  Serial.print("Received write command (");
  Serial.print(length);
  Serial.print(" bytes): ");
  printData(data, length);
  Serial.println();

  // Compare incoming data with known commands
  if (length == sizeof(IDENTIFY) && memcmp(data, IDENTIFY, sizeof(IDENTIFY)) == 0) {
    Serial.println("IDENTIFY command received.");
    // Respond to IDENTIFY if necessary
    // For simulation, we can send back IDENTIFY or another response
    acaiaReadNotifyCharacteristic.writeValue((const byte*)IDENTIFY, sizeof(IDENTIFY));
    return;
  }

  if (length == sizeof(HEARTBEAT) && memcmp(data, HEARTBEAT, sizeof(HEARTBEAT)) == 0) {
    Serial.println("HEARTBEAT command received.");
    sendHeartbeat();
    return;
  }

  if (length == sizeof(NOTIFICATION_REQUEST) && memcmp(data, NOTIFICATION_REQUEST, sizeof(NOTIFICATION_REQUEST)) == 0) {
    Serial.println("NOTIFICATION_REQUEST command received.");
    // Enable notifications
    acaiaReadNotifyCharacteristic.setValue((const byte*)IDENTIFY, sizeof(IDENTIFY));
    return;
  }

  if (length == sizeof(START_TIMER) && memcmp(data, START_TIMER, sizeof(START_TIMER)) == 0) {
    Serial.println("START_TIMER command received.");
    timerRunning = true;
    return;
  }

  if (length == sizeof(STOP_TIMER) && memcmp(data, STOP_TIMER, sizeof(STOP_TIMER)) == 0) {
    Serial.println("STOP_TIMER command received.");
    timerRunning = false;
    return;
  }

  if (length == sizeof(RESET_TIMER) && memcmp(data, RESET_TIMER, sizeof(RESET_TIMER)) == 0) {
    Serial.println("RESET_TIMER command received.");
    // Reset timer logic (if any)
    simulatedWeight = 0.0;
    sendWeightData();
    return;
  }

  if (length == sizeof(TARE_ACAIA) && memcmp(data, TARE_ACAIA, sizeof(TARE_ACAIA)) == 0) {
    Serial.println("TARE_ACAIA command received.");
    // Reset weight to zero
    simulatedWeight = 0.0;
    sendWeightData();
    return;
  }

  // Add handling for generic commands if necessary
  // ...

  Serial.println("Unknown command received.");
}

// Send simulated weight data via notify characteristic
void sendWeightData() {
  // Construct a data packet similar to Acaia's format
  // Example for GENERIC type (20 bytes)
  byte weightPacket[20] = {0};

  // Fill with example data
  weightPacket[0] = 0xef;
  weightPacket[1] = 0xdd;
  weightPacket[2] = 0x0c; // Example command identifier
  weightPacket[3] = 0x09;
  
  // Insert weight value (simulate)
  long weightValue = (long)(simulatedWeight * 100); // e.g., weight in grams
  weightPacket[6] = (weightValue >> 16) & 0xFF;
  weightPacket[7] = (weightValue >> 8) & 0xFF;
  weightPacket[8] = weightValue & 0xFF;

  // Sign byte
  if (simulatedWeight < 0) {
    weightPacket[6] = 0x2D; // ASCII '-' sign
    weightPacket[9] = 0x45; // Example sign indicator
  } else {
    weightPacket[9] = 0x00; // Positive
  }

  // Scale factor (example)
  weightPacket[10] = 2; // e.g., divide by 100 to get weight in kg

  // Add any other necessary bytes to complete the packet
  // ...

  // Send the weight packet
  acaiaReadNotifyCharacteristic.writeValue(weightPacket, sizeof(weightPacket));
  Serial.print("Sent weight data: ");
  printData(weightPacket, sizeof(weightPacket));
  Serial.println();
}

// Update simulated weight (simple increment/decrement for demo)
void updateSimulatedWeight() {
  // Simple simulation: fluctuate weight between 0 and 100 kg
  simulatedWeight += 0.1;
  if (simulatedWeight > 100.0) {
    simulatedWeight = 0.0;
  }
}

// Utility function to print byte array in hex
void printData(const byte* data, int length) {
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
}

// ====== End of Sketch ======

