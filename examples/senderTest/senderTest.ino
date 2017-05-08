/*!
 * @file senderTest.ino
 * @brief SX1278 send data
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

uint8_t counter = 0;
uint8_t arr[] = "HelloWorld!";

void setup() {
  Serial.begin(115200);

  Serial.println("Sender Test");

  if (!lora.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  lora.beginPacket();
  lora.write(arr, 11);
  lora.print(counter);//You can use the print() function to send
  lora.endPacket();

  counter++;
  
  if(counter%10 == 0){
    lora.sleep();
    delay (5000);
    }

  delay(100);
}
