#include <SPI.h>
#include <LoRa.h>

// --- LoRa Module Definitions ---
#define ss 13
#define rst 12
#define dio0 4

int counter = 0;

void setup_lora() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  while (!LoRa.begin(865.0625E6)) {
    Serial.println(".");
    delay(10);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop_lora() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print("hello Electronicsinnovation follower");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(10);
}
