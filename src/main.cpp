#include <Arduino.h>
#include <magnetic_read.h>
#include <AHT_read.h>
#include <MQ7_read.h>
#include <MQ135_read.h>

// Global variable to store the previous magnetic sensor value
int lastMagneticValue = -1; // initialize invalid value
//for AHT
const unsigned long AHT_READ_INTERVAL = 10; // Read AHT sensor every 5 seconds (adjust as needed)
unsigned long lastAhtReadMillis = 0;          // Stores the last time AHT sensor was read

// void readDHTSensor();
void readGPSData();
void sendLoRaPacket();
void readMagneticSensor();

//DHT-function
void readDHTSensor() {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    //update humidity and temp
    currentHumidity = h;
    currentTemperatureC = t;

    // Serial.print(F("Humidity: "));
    // Serial.print(currentHumidity);
    // Serial.print(F("%  Temperature: "));
    // Serial.print(currentTemperatureC);
    // Serial.println(F("°C"));
}

// GPS-function
void readGPSData() {
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isUpdated()) {
                currentLatitude = gps.location.lat(); // Latitude 
                currentLongitude = gps.location.lng(); // Longitude
                currentAltitude = gps.altitude.meters(); // Altitude (m)
                currentSpeed = gps.speed.kmph(); // Speed (km/h)
                currentSatellites = gps.satellites.value(); //Number of satellites 

                
                char timeStr[20]; // Buffer to hold the formatted time string
                sprintf(timeStr, "%04d/%02d/%02d,%02d:%02d:%02d",
                        gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second());
                currentTimeUTC = String(timeStr);

                // Print updated GPS data to the Serial Monitor.
                Serial.println("GPS Data Updated:");
                Serial.print("   LAT: "); Serial.print(currentLatitude, 6); Serial.println("\t");
                Serial.print("   LONG: "); Serial.println(currentLongitude, 6);
                Serial.print("   ALT (m): "); Serial.print(currentAltitude, 2); Serial.println("\t");
                Serial.print("   SPEED (km/h): "); Serial.println(currentSpeed, 2);
                Serial.print("   Satellites: "); Serial.print(currentSatellites); Serial.println("\t");
                Serial.print("   Time UTC: "); Serial.println(currentTimeUTC);
                Serial.println();
            }
        }
    }

    // --- GPS No Data Warning ---
    static unsigned long lastNoDataWarningTime = 0;
    const unsigned long NO_DATA_WARNING_INTERVAL_MS = 10;
    unsigned long currentMillis = millis();


    if (currentMillis > NO_DATA_WARNING_INTERVAL_MS && gps.charsProcessed() < 10) {
        if (currentMillis - lastNoDataWarningTime >= NO_DATA_WARNING_INTERVAL_MS) {
            Serial.println("No GPS data detected! Ensure the module is properly connected.");
            lastNoDataWarningTime = currentMillis; 
        } 
    } delay (10);
}
void readMagneticSensor() {
    magneticValue = digitalRead(Sensor);

    // Only print the status if it has changed
    if (magneticValue != lastMagneticValue) {
        if (magneticValue == 0) {
            magneticStatus = "Magnet Detected!";
        } else {
            magneticStatus = "No Magnet Detected";
        }

        Serial.print("Magnetic Sensor Status: ");
        Serial.println(magneticStatus);
        Serial.println();

        lastMagneticValue = magneticValue;
    }
}

// //function to read MQ-7 sensor data
void readMQ7Sensor() {
    coPPM = mq7.readPpm();
    // Serial.print("CO PPM: ");
    // Serial.println(coPPM);
}

// function to read MQ-135 sensor data
void readMQ135Sensor() {
    MQ135.update();
    
    // Read and store CO2 concentration (add 400 PPM offset)
    MQ135.setA(110.47); 
    MQ135.setB(-2.862); 
    aqiCO2 = MQ135.readSensor(false, 0) + 400;

    // Read and store other gases
    MQ135.setA(77.255); MQ135.setB(-3.18);
    alcoholPPM = MQ135.readSensor(false, 0);

    MQ135.setA(44.947); MQ135.setB(-3.445);
    toluenPPM = MQ135.readSensor(false, 0);

    MQ135.setA(102.2); MQ135.setB(-2.473);
    nh4PPM = MQ135.readSensor(false, 0);

    MQ135.setA(34.668); MQ135.setB(-3.369);
    acetonPPM = MQ135.readSensor(false, 0);

    // Serial.print("CO2 (PPM): "); Serial.print(aqiCO2); Serial.print("\t");
    // Serial.print("Alcohol (PPM): "); Serial.println(alcoholPPM);
    // Serial.print("Toluen (PPM): "); Serial.print(toluenPPM); Serial.print("\t");
    // Serial.print("NH4 (PPM): "); Serial.print(nh4PPM); Serial.print("\t");
    // Serial.print("Aceton (PPM): "); Serial.println(acetonPPM);
    // Serial.println();
    // Serial.println();
}

// LoRa sending packets function
void sendLoRaPacket() {
    packetCounter++; 

    String message = "Packet #";
    message += packetCounter;
    //from AHT
    message += "\nHumidity: ";
    message += String(currentHumidity, 2);
    message += "%\nTemperature: ";
    message += String(currentTemperatureC, 2);
    message += "C\nLatitude: ";
    //from GPS
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
    //from magnetic sensor
    message += "\nMagnetic Sensor: ";
    message += magneticStatus;
    //from MQ sensors
    message += "\nCO PPM: ";
    message += String(coPPM);
    message += "\nCO2 (PPM): ";
    message += String(aqiCO2);
    message += "\nAlcohol (PPM): ";
    message += String(alcoholPPM);      
    message += "\nToluen (PPM): ";
    message += String(toluenPPM);
    message += "\nNH4 (PPM): ";
    message += String(nh4PPM);
    message += "\nAceton (PPM): ";
    message += String(acetonPPM);

    // Print the message to the Serial Monitor before sending via LoRa.
    Serial.print("LoRa packet..... (");
    Serial.print(message.length()); 
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
    Serial.println("DHT Sensor Initialized.");

    // gps setup
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
    Serial.println("GPS Serial Port Initialized.");

    // // Initialize AHT Sensor
    // setupAHTSensor(); // Call the AHT sensor setup function
    // Serial.println("AHT Sensor Initialized.");

    // MQ-7 setup
    mq7.calibrate();
    Serial.println(" MQ-7 Sensor Initialized and calibrated.");

    // MQ-135 setup
    MQ135.init(); 
    Serial.print("Calibrating MQ-135, please wait.");
    float calcR0 = 0;
    for(int i = 1; i <= 10; i++) {
        MQ135.update();
        calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
        Serial.print(".");
    }
    MQ135.setR0(calcR0/10);
    Serial.println(" done!.");
    if(isinf(calcR0)) {
        Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); 
        while(1);
    }
    if(calcR0 == 0) {
        Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); 
        while(1);
    }
    Serial.println("MQ-135 Sensor Initialized.");

    // LoRa setup
    LoRa.setPins(ss, rst, dio0);

    while (!LoRa.begin(865.0625E6)) { //frequency
        Serial.println("LoRa initialization failed. Retrying...");
        delay(10);
    }
    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Initialized OK!");
    Serial.println("Ready for monitoring and transmitting data...");

    // magnetic sensor setup
    pinMode(Sensor, INPUT);
    Serial.println("Magnetic Sensor Initialized.");
}


void loop() {
    unsigned long currentMillis = millis(); // Get the current time for non-blocking operations

    //dht reading
    if (currentMillis - lastDhtReadMillis >= DHT_READ_INTERVAL) {
        lastDhtReadMillis = currentMillis; // Update the last read time
        readDHTSensor(); // Call the function to read DHT data
    }

    //AHT reading
    // // AHT Sensor Reading
    // if (currentMillis - lastAhtReadMillis >= AHT_READ_INTERVAL) {
    //     lastAhtReadMillis = currentMillis; // Update the last read time
    //     readAHTSensor();                   // Call the function to read AHT data
    // }
    //gps reading
    readGPSData();

    // // Read MQ-7 and MQ-135 data
    readMQ7Sensor();
    readMQ135Sensor();

    // ssend lora packet
    if (currentMillis - lastLoRaSendMillis >= LORA_SEND_INTERVAL) {
        lastLoRaSendMillis = currentMillis; 
        sendLoRaPacket(); 
    }
    // magnetic sensor reading
    readMagneticSensor(); 
}