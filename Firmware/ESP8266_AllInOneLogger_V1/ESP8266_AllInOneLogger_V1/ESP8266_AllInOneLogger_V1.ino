/*
  updates UART received data from STM32F1 to thinsspeak.


  TO:DO
  *Change message structure. after start byte a commandTypeByte, to enable cofiguration commands ect...
  *Change message to enable floats 
  *Write thingspeak library, instead of the included one to support floats. or find a solution
  *ESP8266 also visible as hotspot. disable this

  -Add CRC 
  -Send ACK to STM32F1
  -Add configuration message response: to configure SSID; Pass; ChannelNumber; myWritekey.
  -Add message response to get the Time From the internet and send it back to the STM32F1
 
  Optional:
  -Get ESP8266 to program the STM32F over the air. (set bootloader pins and reset STM32F1 and program over UART)

*/

#include "ThingSpeak.h"
#include <ESP8266WiFi.h>

#define bufferSize 13 //bufferSize in bytes

char ssid[] = "13A1-online3";    //  your network SSID (name) 
char pass[] = "kutpolen";   // your network password
int status = WL_IDLE_STATUS;
WiFiClient  client;

unsigned long myChannelNumber = 113979;
char * myWriteAPIKey = "DDACRDYIJWVA15Z3";

uint8_t getCheckSum(uint8_t * packet);
void updateDataThingspeak(uint8_t * messageBuffer);


void setup() {
	WiFi.begin(ssid, pass);
	ThingSpeak.begin(client);

	Serial.begin(115200);
	Serial.swap(); //swaps serial pins to  GPIO15=D8 (TX) and GPIO13=D7 (RX)
}


uint8_t messageBuffer[bufferSize] = {};

void loop() {

	if (Serial.available() > 0) { //When there is som data inside the RX UART buffer
		///make room in buffer for new element
		uint8_t i = 0;
		for (i = 1; i < bufferSize; i++) {
			messageBuffer[i-1] = messageBuffer[i];
		}
		messageBuffer[bufferSize-1] = Serial.read(); //store new byte in last position of the buffer
		
		///Check for start of message, indication that a full message has been received
		if ((messageBuffer[0] == 0xFF) && (messageBuffer[1] == 0xFF) && (messageBuffer[2] == 0xFF)) {
			//CRC checking here
			if (getCheckSum(messageBuffer) == messageBuffer[12]) { //message is correct
				switch (messageBuffer[3]) //check cmd
				{
				case 0x00: //Thingsspeak data update
					updateDataThingspeak(messageBuffer);
					break;
				default:
					break;
				}
			}
		}
	}
}

/* Thingsspeak data update cmd 0x00

+---------------------------------------------------------------------------------+
|Start | cmd  | LightLevel | AirTemp | Humidity | CO2 | WaterTemp | PH | EC | CRC |
+---------------------------------------------------------------------------------+
| 0-2  |  3   |     4      |    5    |    6     | 7-8 |    9      | 10 | 11 | 12  |
+---------------------------------------------------------------------------------+

*/
void updateDataThingspeak(uint8_t * messageBuffer)
{
	///Extract data from messageBuffer
	uint8_t lightLevel = messageBuffer[4];
	uint8_t airTemp = messageBuffer[5];
	uint8_t humidity = messageBuffer[6];
	uint16_t co2 = (((uint16_t)messageBuffer[7] << 8) | ((uint16_t)messageBuffer[8]));//combine and store 2 bytes as 16bit number
	uint8_t waterTemp = messageBuffer[9];
	uint8_t ph = messageBuffer[10];
	uint8_t ec = messageBuffer[11];


	unsigned long currentMillis = millis();
	static unsigned long prevMillis = 0;

	if ((currentMillis - prevMillis) > 20000) { //minimum update interval is 20 sec
		prevMillis = currentMillis;

		//TO DO: only set fields thatare active, use new byte for this
		ThingSpeak.setField(1, lightLevel);
		ThingSpeak.setField(2, airTemp);
		ThingSpeak.setField(3, humidity);
		ThingSpeak.setField(4, co2);
		ThingSpeak.setField(5, waterTemp);
		ThingSpeak.setField(6, ph);
		ThingSpeak.setField(7, ec);
		ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);  // Write the fields at once. NOTE! Thingspeak write interval must be >15 sec.
	}
}

uint8_t
getCheckSum(uint8_t * packet)
{
	/* The checksum = (invert (byte 3 +... + 11)) + 1 */
	uint8_t i;
	uint8_t checksum = 0;

	for (i = 3; i < 12; i++)
	{
		checksum += packet[i];
	}
	checksum = (0xff - checksum);
	checksum += 1;
	return checksum;
}