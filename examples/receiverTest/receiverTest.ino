/*!
 * @file receiverTest.ino
 * @brief SX1278 receive data
 * @n [Get the module here]
 * @n This example Set the volume size and play music snippet.
 * @n [Connection and Diagram]
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yangyang]
 * @version  V1.0
 * @date  2017-04-10
 */
#include "DFRobot_SX1278.h"

DFRobot_SX1278 lora;

void setup() {
  Serial.begin(115200);

  Serial.println("Receiver Test");

  if (!lora.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = lora.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (lora.available()) {
      Serial.print((char)lora.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(lora.packetRssi());
  }
}