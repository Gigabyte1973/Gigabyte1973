#include <Wire.h>
#include <MCP342x.h>

/* Demonstrate the use of convertAndRead().
 */


// 0x68 is the default address for all MCP342x devices
uint8_t address = 0x6B;
MCP342x adc = MCP342x(address);

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  // Enable power for MCP342x (needed for FL100 shield only)
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);
  
  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
  
  // Check device present
  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1)
      ;
  }

}

void loop(void)
{
  long value = 0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel4, MCP342x::oneShot,
				   MCP342x::resolution12, MCP342x::gain1,
				   1000000, value, status);
  if (err) {
    Serial.print("Convert error: ");
    Serial.println(err);
  }
  else {
    Serial.print("Value: ");
    Serial.println(value);
  }
  
  delay(100);
}
