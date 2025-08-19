#include <Arduino.h> 
#include "defines.h" 


void readDHTSensor();
void readGPSData();
void sendLoRaPacket();


//DHT-function 
void readDHTSensor() {
    float h = dht.readHumidity();
    float t = dht.readTemperature(); 

    //if fault, check the wiring
    if (isnan(h) || isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return; 
    }

    //update humidity and temp
    currentHumidity = h;
    currentTemperatureC = t;

    Serial.print(F("Humidity: "));
    Serial.print(currentHumidity);
    Serial.print(F("%  Temperature: "));
    Serial.print(currentTemperatureC);
    Serial.println(F("°C"));
}

// GPS-function
void readGPSData() {
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isUpdated()) {
                currentLatitude = gps.location.lat(); // Latitude in degrees
                currentLongitude = gps.location.lng(); // Longitude in degrees
                currentAltitude = gps.altitude.meters(); // Altitude in meters
                currentSpeed = gps.speed.kmph(); // Speed in kilometers per hour
                currentSatellites = gps.satellites.value(); //Number of satellites in view  

                // Format GPS time (UTC) into a readable string.
                char timeStr[20]; // Buffer to hold the formatted time string
                sprintf(timeStr, "%04d/%02d/%02d,%02d:%02d:%02d",
                        gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second());
                currentTimeUTC = String(timeStr);

                // Print updated GPS data to the Serial Monitor.
                Serial.println("GPS Data Updated:");
                Serial.print("  LAT: "); Serial.println(currentLatitude, 6);
                Serial.print("  LONG: "); Serial.println(currentLongitude, 6);
                Serial.print("  ALT (m): "); Serial.println(currentAltitude, 2);
                Serial.print("  SPEED (km/h): "); Serial.println(currentSpeed, 2);
                Serial.print("  Satellites: "); Serial.println(currentSatellites);
                Serial.print("  Time UTC: "); Serial.println(currentTimeUTC);
            }
        }
    }

    // --- GPS No Data Warning ---
    static unsigned long lastNoDataWarningTime = 0;
    const unsigned long NO_DATA_WARNING_INTERVAL_MS = 1000; 
    unsigned long currentMillis = millis(); 

    // After an initial boot-up delay and if very few characters have been processed,
    // print a warning about no GPS data.
    if (currentMillis > NO_DATA_WARNING_INTERVAL_MS && gps.charsProcessed() < 10) {
        if (currentMillis - lastNoDataWarningTime >= NO_DATA_WARNING_INTERVAL_MS) {
            Serial.println("No GPS data detected! Ensure the module is properly connected.");
            lastNoDataWarningTime = currentMillis; // Reset the timer for the next warning
        }
    }
}

// LoRa sending packets function
void sendLoRaPacket() {
    packetCounter++; // Increment the global packet counter for each sent packet

    String message = "Packet #";
    message += packetCounter;
    message += "\nHumidity: ";
    message += String(currentHumidity, 2); 
    message += "%\nTemperature: ";
    message += String(currentTemperatureC, 2); 
    message += "C\nLatitude: ";
    message += String(currentLatitude, 6); 
    message += "\nLongitude: ";
    message += String(currentLongitude, 6); 
    message += "\nAltitude: ";
    message += String(currentAltitude, 2); 
    message += "m\nSpeed: ";
    message += String(currentSpeed, 2); 
    message += "km/h\nSatellites: ";
    message += String(currentSatellites); 
    message += "\nTime UTC: ";
    message += currentTimeUTC; 

    // Print the message to the Serial Monitor before sending via LoRa.
    Serial.print("LoRa packet..... (");
    Serial.print(message.length()); // Display packet size
    Serial.print(" bytes): \n");
    Serial.println(message);

    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();

    Serial.println("LoRa packet sent!");
    Serial.println();
}


void setup() {
    Serial.begin(115200); 
    while (!Serial); 
    Serial.println("\n Starting Integrated LoRa-GPS-DHT Sensor Node...");

    // dht setup
    dht.begin();
    Serial.println("✅ DHT Sensor Initialized.");

    // gps setup
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
    Serial.println("✅ GPS Serial Port Initialized.");

    // LoRa setup
    LoRa.setPins(ss, rst, dio0);
    
    while (!LoRa.begin(865.0625E6)) { //frequency
        Serial.println("LoRa initialization failed. Retrying...");
        delay(10); 
    }
    LoRa.setSyncWord(0xF3); 
    Serial.println("LoRa Initialized OK!");
    Serial.println("Ready for monitoring and transmitting data...");
}


void loop() {
    unsigned long currentMillis = millis(); // Get the current time for non-blocking operations

    //dht reading
    if (currentMillis - lastDhtReadMillis >= DHT_READ_INTERVAL) {
        lastDhtReadMillis = currentMillis; // Update the last read time
        readDHTSensor(); // Call the function to read DHT data
    }

    //gps reading
    readGPSData();

    // ssend lora packet
    if (currentMillis - lastLoRaSendMillis >= LORA_SEND_INTERVAL) {
        lastLoRaSendMillis = currentMillis; // Update the last send time
        sendLoRaPacket(); // Call the function to construct and send the LoRa packet
    }

    delay (10);
}
