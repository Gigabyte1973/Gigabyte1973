#include "RA8876.h"

#define RA8876_CS        15
#define RA8876_RESET     17
#define RA8876_BACKLIGHT 20


RA8876 tft = RA8876(RA8876_CS, RA8876_RESET);

void setup()
{
  Serial.begin(9600);
  SPI.setSCK(14);
  delay(1000);

  while (!Serial);

  Serial.println("Starting up...");

  pinMode(RA8876_BACKLIGHT, OUTPUT);  // Set backlight pin to OUTPUT mode
  digitalWrite(RA8876_BACKLIGHT, HIGH);  // Turn on backlight
  
  if (!tft.init())
  {
    Serial.println("Could not initialize RA8876");
  }

  Serial.println("Startup complete...");

  //tft.clearScreen(0);
  
  tft.colorBarTest(true);
 // delay(1000);
 // tft.colorBarTest(false);

}
void loop()
{
  

}
