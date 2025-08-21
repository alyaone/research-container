#include <SPI.h>       // For LoRa module communication
#include <LoRa.h>      // LoRa library
#include <TinyGPSPlus.h> // GPS parsing library
#include "DHT.h"       // DHT sensor library

// --- LoRa Module Definitions ---
#define ss 13    // SS (Slave Select) pin for LoRa module
#define rst 12   // RST (Reset) pin for LoRa module
#define dio0 4   // DIO0 pin for LoRa module (Interrupt pin)


// --- GPS Module Definitions ---
#define RXD2 16    // RX pin for GPS module (connected to TX of GPS)
#define TXD2 17    // TX pin for GPS module (connected to RX of GPS)
#define GPS_BAUD 9600 // Baud rate for GPS communication
// Note: 'gpsSerial' is defined as a HardwareSerial object below,
// not as a simple #define.

// --- DHT22 Sensor Definitions ---
#define DHTPIN 5     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // Type of DHT sensor (DHT11, DHT22, DHT21)

// --- Global Objects ---
TinyGPSPlus gps; // GPS object to parse NMEA sentences
HardwareSerial gpsSerial(2); // Use Serial2 for GPS communication on ESP32 (pins 16 and 17)
DHT dht(DHTPIN, DHTTYPE); // DHT sensor object, initialized with pin and type

// --- Global Variables for Sensor Data and Timing ---
int packetCounter = 0; // Counter for LoRa packets sent

// Magnetic sensor variables
bool magneticValue;
const char* magneticStatus;

// Timers for non-blocking operations using millis()
unsigned long lastDhtReadMillis = 0;
const long DHT_READ_INTERVAL = 2000; // Read DHT every 2 seconds (2000 ms)

unsigned long lastLoRaSendMillis = 0;
const long LORA_SEND_INTERVAL = 10000; // Send LoRa packet every 10 seconds (10000 ms)

// Variables to store current sensor readings
float currentHumidity = 0.0;
float currentTemperatureC = 0.0;
double currentLatitude = 0.0;
double currentLongitude = 0.0;
float currentAltitude = 0.0; // In meters
float currentSpeed = 0.0;    // In km/h
int currentSatellites = 0;
String currentTimeUTC = "N/A"; // Store GPS time as a string
