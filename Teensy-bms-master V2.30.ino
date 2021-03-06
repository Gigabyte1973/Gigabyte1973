#pragma GCC diagnostic ignored "-Wwrite-strings"

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_MODEM_ESP8266
#define RA8876_CS 15
#define RA8876_RESET 17
#define RA8876_BACKLIGHT 20
#define SD_SELECT BUILTIN_SDCARD
#define NUM_VALUES 29
#define BLYNK_PRINT Serial // Comment this out to disable prints and save space
#define BUFFER_SIZE 60
#define SerialMon Serial
#define SerialAT Serial4
#define DOUBLE_BUFFER_1  PAGE1_START_ADDR
#define DOUBLE_BUFFER_2  PAGE2_START_ADDR

#define CHARGER_CONNECTED 26

#define CHARGE_WIRE_ENABLE_1 35
#define CHARGE_WIRE_ENABLE_2 36
#define CHARGE_WIRE_ENABLE_3 37
#define CHARGE_SOLAR_ENABLE_1 51

#define CHARGE_PIN_OUTPUT 50

#define EEPROM_ADDRESS 0

#define DEBUG_1
//#define DEBUG_2

#define FUEL_FILLER_CAP_STATUS 26

#include "RA8876.h"
#include <DS1307RTC.h>
#include <IniFile.h>
#include <SensorModbusMaster.h>
#include <SPI.h>
#include <SD.h>
#include <DS18B20.h>
#include <BME280I2C.h> //Bosch BME 280Temp, Humidity, Barometric Presure.
#include <Wire.h>
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <Adafruit_SleepyDog.h>
#include <EEPROM.h>
#include <TimeLib.h>



enum DISPLAY_TYPE {ADVANCED, SIMPLE};

const char apn[] = "data.tre.dk";
const char user[] = "";
const char pass[] = "";
const char wifiSSID[] = "Gigabyte_2.4Ghz";
const char wifiPass[] = "Spirill1111";
const char auth[] = "dd6a78ac747e4f99aedadd96d9862792";

BME280I2C bme;
DS18B20 ds(22);        // Pin used for DS18B20 sensor(s)
TinyGsm modem(SerialAT);
HardwareSerial modbusSerial = Serial1; //RS485 using UART1
modbusMaster modbus;
RA8876 tft = RA8876(RA8876_CS, RA8876_RESET);
IntervalTimer Timer1Second;
IntervalTimer Timer10Second;
IntervalTimer Timer60Second;
IntervalTimer Timer360Second;
tmElements_t RTC_Clock_Data;
BlynkTimer timer;

int ACTIVE_FRONT_BUFFER = DOUBLE_BUFFER_1;
int ACTIVE_BACK_BUFFER  = DOUBLE_BUFFER_2;

DISPLAY_TYPE ACTIVE_DISPLAY_TYPE = ADVANCED;

float Cabin_temp = 0;           //Bosch BME 280 Temperature
float Cabin_humidity = 0;       //Bosch BME 280 Humidity
float Cabin_barometric_presure; // Bosch barometric presure
int Rear_battery_temp = 0;      //Rear battery pack DS18B20 temp sensor.

int MainSoc = 25;
int FrontBattTemp = 0;
int RearBattTemp = 0;
int MotorTemp = 0;
int ContTemp = 0;
int DcDcTemp = 0;
float OutsideTemp = 0;
int DriveCurrent = 255;
int MainBatvoltage = 86;
int Charge1Current = 19;
float Charge2Current = 6.1;
float LowVoltVoltage = 13.6;
float AuxCurrent = 3.2;
int topPos = 0;
int MaxHeight = 285;
int TopLinePos = 200;
int GraphWidth = 22;
int ColumBegin = 98;
int ColumWidth = 10;
int ColumSpacing = 28;
int ColumBottom = 560;
int LastDrawnValues = 0;
float TotalVolatageTracktionBattery = 0;
int txtLine1 = 10;
int txtLine2 = 45;
int txtLine3 = 80;
int txtLine4 = 115;
int txtLine5 = 150;
int txtLine6 = 185;
int txtLine7 = 220;
int txtLine8 = 255;
int txtColumn1 = 10;
int txtColumn2 = 350;
int txtColumn3 = 690;

bool ledState = false;

char ClockBuffer[100];  

int Voltage_divider = 194; //269
int ADC_ChannelEnable1[] = {0, 0, 0, 0};
int ADC_ChannelEnable2[] = {0, 0, 0, 0};
int ADC_Sensitivity[] = {0, 0, 0, 0};
float ADC_Sensitivity_Final[] = {0.0, 0.0, 0.0, 0.0};
int ADC_ZERO_Volt[] = {0, 0, 0, 0};
double Watts[] = {0.0, 0.0, 0.0, 0.0};
uint16_t AmpRawValue[] = {0, 0, 0, 0};
double AmpCompensatedValue[] = {0.0, 0.0, 0.0, 0.0};
double AmpHours[] = {0.0, 0.0, 0.0, 0.0};
double CurrentTotalAmpHours = 0.0;
double AverageAmps[] = {0.0, 0.0, 0.0, 0.0};
double WattHours[] = {0.0, 0.0, 0.0, 0.0};
float WattHoursTotal = 0.0;
int NumberOfSampels = 0;
double CurrentTotalAmpage = 0;
double InstantaniousAmpHours = 0;
bool ChargerConnected = false;
bool ChargerPreviouslyConnected = false;
int TimeSinceLastChargingStatusChange = 0;
double BatteryMaxAmpHours = 0;
int numberOfChargersEnabled = 0;
int chargersDutyCyclePercent = 0;
int ChargingPWMIndex = 0;
bool DoFullCharge = false;

char endTimeDay = 0;
char endTimeHour = 0;
char endTimeMinute = 0;

int CellActive[NUM_VALUES + 1];
float CellValue[NUM_VALUES + 1];
float CellVoltage[NUM_VALUES + 1];
uint16_t BleedValues[NUM_VALUES + 1];

float Power_supply_12V;

float Current[4];

double RUNTIME = 0.0;
double LAST_RUNTIME = 0.0;

int Device64Active = 0;
int Device65Active = 0;

float totalcellvoltage = 0;
int Next_delay = 0; //RS485 SLAVE ID read delay

int graph6 = 0;

int CellBeginL = 100;
int TxtBeginL = 10;
int CellSpacing = 28;
int TotalVoltage = 0;

bool summerTime = false;
bool fuel_filler_cap_status = false;

float AUX_end_charge_voltage;
float AUX_start_charge_voltage;

BLYNK_WRITE(V3) {  //Reset battery state command from blynk
  Serial.println("Setting charging finish time");

  DoFullCharge = false;

  TimeInputParam t(param);
  if (t.hasStartTime())
  {
    endTimeHour = t.getStartHour();
    endTimeMinute = t.getStartMinute();

    for(int i = 0; i <7 ; i++){
      if( t.isWeekdaySelected(i) ){
        Serial.println("Weekday");
        Serial.println(RTC_Clock_Data.Wday);
        endTimeDay = i;        
      }
    }

    EEPROM.write(EEPROM_ADDRESS + 4, endTimeDay);
    EEPROM.write(EEPROM_ADDRESS + 5, endTimeHour);
    EEPROM.write(EEPROM_ADDRESS + 6, endTimeMinute);

    Serial.println(String("Start: ") +
                     t.getStartHour() + ":" +
                     t.getStartMinute() + ":" +
                     t.getStartSecond());
    }
    else if (t.isStartSunrise())
    {
      Serial.println("Start at sunrise");
    }
    else if (t.isStartSunset())
    {
      Serial.println("Start at sunset");
    }
    else
    {
      // Do nothing
    }
}

BLYNK_WRITE(V10) {  //Reset battery state command from blynk
  Serial.println("Resetting battery level");
 for (int i = 0; i < 4; i++)
  {
    if (ADC_ChannelEnable1[i])
    {
      AmpHours[i] = 0;
      WattHours[i] = 0;
    }
  }
}

BLYNK_WRITE(V6){
  if( (summerTime && !param.asInt()) || (!summerTime && param.asInt())){

    summerTime = param.asInt();
   
    EEPROM.write(EEPROM_ADDRESS + 7, summerTime);  

    RTC.read(RTC_Clock_Data);
    if( summerTime ){
      RTC_Clock_Data.Hour += 1;
    }
    else{
      RTC_Clock_Data.Hour -= 1;
    }
    RTC.write(RTC_Clock_Data);
  }
}

void update_ADC_Values(int modbus_ID){

  modbus.begin(modbus_ID, modbusSerial, 2);

  CurrentTotalAmpage = 0;

  for (int i = 0; i < 4; i++)
  {
    if (ADC_ChannelEnable1[i])
    {
      AmpRawValue[i] = modbus.uint16FromRegister(0x03, i, bigEndian);
      ADC_Sensitivity_Final[i] = ADC_Sensitivity[i] / 100.0;
      AmpCompensatedValue[i] = AmpRawValue[i] - ADC_ZERO_Volt[i];
      AmpCompensatedValue[i] /= ADC_Sensitivity_Final[i];

      //BOSS: TESTCODE - fjern denne linei til produktion
      AmpCompensatedValue[i] = 4.0;

      Watts[i] = AmpCompensatedValue[i] * TotalVolatageTracktionBattery;
      CurrentTotalAmpage += AmpCompensatedValue[i];

      WattHours[i] = TotalVolatageTracktionBattery * AmpHours[i];
      WattHoursTotal += WattHours[i];
#ifdef DEBUG_1  
      Serial.print("=");
      Serial.println(TotalVolatageTracktionBattery);
      Serial.println(AmpHours[i]);
      Serial.println(RUNTIME);
      Serial.println(AmpCompensatedValue[i], 1);  
      Serial.println(CurrentTotalAmpHours);
#endif      
    }
  }

  InstantaniousAmpHours = CurrentTotalAmpage * (RUNTIME - LAST_RUNTIME);
  CurrentTotalAmpHours += InstantaniousAmpHours;

  LAST_RUNTIME = RUNTIME;
}

void set_BLEED_Values(){
  for (int i = 1; i <= NUM_VALUES; i++)
  {
    if(CellActive[i] ){
      modbus.begin(CellActive[i], modbusSerial, 2);
      modbus.uint16ToRegister(20, BleedValues[i], bigEndian);
    }
  }
}

void printErrorMessage(uint8_t e, bool eol = true){
  switch (e)
  {
  case IniFile::errorNoError:
    Serial.print("no error");
    break;
  case IniFile::errorFileNotFound:
    Serial.print("file not found");
    break;
  case IniFile::errorFileNotOpen:
    Serial.print("file not open");
    break;
  case IniFile::errorBufferTooSmall:
    Serial.print("buffer too small");
    break;
  case IniFile::errorSeekError:
    Serial.print("seek error");
    break;
  case IniFile::errorSectionNotFound:
    Serial.print("section not found");
    break;
  case IniFile::errorKeyNotFound:
    Serial.print("key not found");
    break;
  case IniFile::errorEndOfFile:
    Serial.print("end of file");
    break;
  case IniFile::errorUnknownError:
    Serial.print("unknown error");
    break;
  default:
    Serial.print("unknown error value");
    break;
  }
  if (eol)
    Serial.println();
}

void myTimerEvent(){
  Blynk.virtualWrite(V5, TotalVolatageTracktionBattery);
}

void layout_advanced(){
  tft.canvasImageStartAddress(ACTIVE_BACK_BUFFER);
  tft.clearScreen(ACTIVE_BACK_BUFFER, COLOR65K_BLACK);
  tft.selectExternalFont(RA8876_FONT_FAMILY_ARIAL, RA8876_FONT_SIZE_24, RA8876_FONT_ENCODING_ASCII);
  tft.setTextScale(1);
  tft.drawRect(0, 599, 1023, 0, COLOR65K_WHITE);
  tft.drawLine(0, 300, 1023, 300, COLOR65K_WHITE);
  tft.drawLine(341, 0, 341, 300, COLOR65K_WHITE);
  tft.drawLine(341 * 2, 0, 341 * 2, 300, COLOR65K_WHITE);
  tft.textColor(COLOR65K_RED, COLOR65K_BLACK);
  tft.putString(10, 310, "4,0 V");
  tft.putString(10, 335, "3,8 V");
  tft.textColor(COLOR65K_GREEN, COLOR65K_BLACK);
  tft.putString(10, 360, "3,6 V");
  tft.putString(10, 385, "3,4 V");
  tft.putString(10, 410, "3,2 V");
  tft.putString(10, 435, "3,0 V");
  tft.putString(10, 460, "2,8 V");
  tft.putString(10, 485, "2,6 V");
  tft.textColor(COLOR65K_RED, COLOR65K_BLACK);
  tft.putString(10, 510, "2,4 V");
  tft.putString(10, 535, "2,2 V");
  tft.textColor(COLOR65K_RED, COLOR65K_BLACK);
  tft.putString(958, 310, "4,0 V");
  tft.putString(958, 335, "3,8 V");
  tft.textColor(COLOR65K_GREEN, COLOR65K_BLACK);
  tft.putString(958, 360, "3,6 V");
  tft.putString(958, 385, "3,4 V");
  tft.putString(958, 410, "3,2 V");
  tft.putString(958, 435, "3,0 V");
  tft.putString(958, 460, "2,8 V");
  tft.putString(958, 485, "2,6 V");
  tft.textColor(COLOR65K_RED, COLOR65K_BLACK);
  tft.putString(958, 510, "2,4 V");
  tft.putString(958, 535, "2,2 V");

  tft.selectExternalFont(RA8876_FONT_FAMILY_ARIAL, RA8876_FONT_SIZE_32, RA8876_FONT_ENCODING_ASCII);
  tft.textColor(COLOR65K_GREEN, COLOR65K_BLACK);

  tft.putString(txtColumn1, txtLine1, "Solar Charger");
  tft.putString(txtColumn1, txtLine2, "Charger 1");
  tft.putString(txtColumn1, txtLine3, "Charger 2");
  tft.putString(txtColumn1, txtLine4, "Charger 3");

  tft.putString(txtColumn2, txtLine1, "Main Batt %");
  tft.putString(txtColumn2, txtLine2, "#Act. Chargers");
  tft.putString(txtColumn2, txtLine3, "Main batt Ah");
  tft.putString(txtColumn2, txtLine4, "Main batt V");
  tft.putString(txtColumn2, txtLine5, "12V Batt A");
  tft.putString(txtColumn2, txtLine6, "12V Batt V");
  tft.putString(txtColumn2, txtLine7, "");
  tft.putString(txtColumn2, txtLine8, "");

  tft.putString(txtColumn3, txtLine1, "Outdoor");
  tft.putString(txtColumn3, txtLine2, "Cabin");
  tft.putString(txtColumn3, txtLine3, "Motor cont");
  tft.putString(txtColumn3, txtLine4, "Front Batt");
  tft.putString(txtColumn3, txtLine5, "Bottom Batt");
  tft.putString(txtColumn3, txtLine6, "Rear Batt");
  tft.putString(txtColumn3, txtLine7, "Charger");
  tft.putString(txtColumn3, txtLine8, "DC-DC conv.");

  for (int i = 0; i < NUM_VALUES; i++)
  {
    if (i < 9)
    {
      tft.setCursor(CellBeginL + i * CellSpacing, 575);
    }
    else
    {
      tft.setCursor(CellBeginL - 4 + i * CellSpacing, 575);
    }
    tft.textColor(COLOR65K_WHITE, COLOR65K_BLACK);
    tft.selectExternalFont(RA8876_FONT_FAMILY_ARIAL, RA8876_FONT_SIZE_16, RA8876_FONT_ENCODING_ASCII);
    tft.print(i + 1);
  }
}

void layout_simple(){
  //Draw charging graphics
  tft.canvasImageStartAddress(ACTIVE_BACK_BUFFER); // Start PAGE 1
  tft.clearScreen(ACTIVE_BACK_BUFFER, COLOR65K_BLACK);
  tft.drawRect(256, 150,512, 450, COLOR65K_WHITE);
}

void update_advanced(){
  layout_advanced();
  for (int i = 1; i <= NUM_VALUES; i++)
  {
    //if (CellActive[i] == 1)
    {
      modbus.begin(i, modbusSerial, 2);
      //BOSS: Test CODE CellValue[i] = modbus.uint16FromRegister(0x03, 0, bigEndian);
      CellVoltage[i] = CellValue[i] / Voltage_divider;

      //BOSS: Test CODE
      CellVoltage[i] = 3.6;

     
#ifdef DEBUG_2      
      Serial.print("Reading Cell ");
      Serial.println(i);
      Serial.print("CellValue  = ");
      Serial.println(CellValue[i]);
      Serial.print("CellVoltage  = ");
      Serial.println(CellVoltage[i]);
#endif      
    }
  }
  //tft.fillRect(txtColumn1 + 220, 10, 340, txtLine8 + 32, COLOR65K_BLACK);
  tft.selectExternalFont(RA8876_FONT_FAMILY_ARIAL, RA8876_FONT_SIZE_32, RA8876_FONT_ENCODING_ASCII);
  tft.textColor(COLOR65K_WHITE, COLOR65K_BLACK);
 
  if (ADC_ChannelEnable1[0]){    
    //Serial.print("Update Solar Charger ");
    //Serial.print(AmpCompensatedValue[0]);
    //Serial.print(" ");
    //Serial.print(WattHours[0]);
    //Serial.print(" ");
    tft.setCursor(txtColumn1 + 240, txtLine1);
    tft.print(AmpCompensatedValue[0], 1);
    //tft.putFloat(txtColumn1 + 240, txtLine1, (float)AmpCompensatedValue[0], 5, 2, "n");
    //tft.putFloat(txtColumn1 + 240, txtLine5, (float)WattHours[0], 5, 2, "n");
  }
  if (ADC_ChannelEnable1[1]){
    //Serial.print("Update Charger 1 ");
    //Serial.print(AmpCompensatedValue[1]);
    //Serial.print(" ");
    //Serial.print(WattHours[1]);
    //Serial.print(" ");
    tft.setCursor(txtColumn1 + 240, txtLine2);
    tft.print(AmpCompensatedValue[1], 1);
    //tft.putFloat(txtColumn1 + 240, txtLine2, (float)AmpCompensatedValue[1], 5, 2, "n");
    //tft.putFloat(txtColumn1 + 240, txtLine6, (float)WattHours[1], 5, 2, "n");
  }  
  if (ADC_ChannelEnable1[2]){
    //Serial.print("Update Charger 2 ");
    //Serial.print(AmpCompensatedValue[2]);
    //Serial.print(" ");
    //Serial.print(WattHours[2]);
    //Serial.print(" ");
    tft.setCursor(txtColumn1 + 240, txtLine3);
    tft.print(AmpCompensatedValue[2], 1);
    //tft.putFloat(txtColumn1 + 240, txtLine3, (float)AmpCompensatedValue[2], 5, 2, "n");  
    //tft.putFloat(txtColumn1 + 240, txtLine7, (float)WattHours[2], 5, 2, "n");
  }
  if (ADC_ChannelEnable1[3]){
    //Serial.print("Update Charger 3 ");
    //Serial.print(AmpCompensatedValue[3]);
    //Serial.print(" ");
    //Serial.println(WattHours[3]);
    tft.setCursor(txtColumn1 + 240, txtLine4);
    tft.print(AmpCompensatedValue[3], 1);
    //tft.putFloat(txtColumn1 + 240, txtLine4, (float)AmpCompensatedValue[3], 5, 2, "n");
    //tft.putFloat(txtColumn1 + 240, txtLine8, (float)WattHours[3], 5, 2, "n");
  }  

  tft.selectExternalFont(RA8876_FONT_FAMILY_ARIAL, RA8876_FONT_SIZE_32, RA8876_FONT_ENCODING_ASCII);
  tft.setCursor(txtColumn2 + 250, txtLine4);
  tft.textColor(COLOR65K_GREEN, COLOR65K_BLACK);
  tft.print(TotalVolatageTracktionBattery, 1);

  tft.setCursor(txtColumn2 + 250, txtLine2);
  tft.print(numberOfChargersEnabled, 1);

  tft.setCursor(txtColumn2 + 250, txtLine3);
  tft.print(CurrentTotalAmpHours, 1);

  tft.setCursor(txtColumn2 + 250, txtLine6);
  tft.print( (Power_supply_12V*3.3/4096), 1 );

  int color;
  for (int i = 0; i < NUM_VALUES; i++)
  {
    int cell = (int)(CellVoltage[i + 1] * 100);
    topPos = map(cell, 200, 400, 560, 320);

    if (cell < 250 || cell > 370)
    {
      color = COLOR65K_RED;
    }
    else
    {
      color = COLOR65K_GREEN;
    }
   
    if( cell > 250 ){
      tft.fillRect(ColumBegin + (i * ColumSpacing), topPos, ColumBegin + ColumWidth + (i * ColumSpacing), ColumBottom, color);
    }
    else{
      tft.fillRect(ColumBegin + (i * ColumSpacing), ColumBottom, ColumBegin + ColumWidth + (i * ColumSpacing), ColumBottom - 2, color);
    }
  }

  if( OutsideTemp < 2 ){
    tft.textColor(COLOR65K_WHITE, COLOR65K_BLUE);
  }
  else{
    tft.textColor(COLOR65K_WHITE, COLOR65K_BLACK);
  }
  tft.setCursor(txtColumn3 + 200, txtLine1);
  tft.print(OutsideTemp, 1);

  tft.textColor(COLOR65K_WHITE, COLOR65K_BLACK);
  tft.setCursor(txtColumn3 + 200, txtLine2);
  tft.print(Cabin_temp, 1);

  Serial.println("TEST---CurrentTotalAmpHours----BatteryMaxAmpHours");
  Serial.println(CurrentTotalAmpHours);
  Serial.println(BatteryMaxAmpHours);

  sprintf(ClockBuffer, "%i%%", (int)(100*CurrentTotalAmpHours/BatteryMaxAmpHours));
  tft.putString(txtColumn2 + 250, txtLine1, ClockBuffer);

  sprintf(ClockBuffer, "Time: %02i:%02i:%02i", RTC_Clock_Data.Hour, RTC_Clock_Data.Minute, RTC_Clock_Data.Second);
  tft.putString(txtColumn2 + 50, txtLine7, ClockBuffer);

  bool timeSet = true;

  if( endTimeDay == 1){
    sprintf(ClockBuffer, "End Chg: MON:", endTimeHour, endTimeMinute);
  }
  else if( endTimeDay == 2){
    sprintf(ClockBuffer, "End Chg: TUE:", endTimeHour, endTimeMinute);
  }
  else if( endTimeDay == 3){
    sprintf(ClockBuffer, "End Chg: WED:", endTimeHour, endTimeMinute);
  }
  else if( endTimeDay == 4){
    sprintf(ClockBuffer, "End Chg: THU:", endTimeHour, endTimeMinute);
  }
  else if( endTimeDay == 5){
    sprintf(ClockBuffer, "End Chg: FRI:", endTimeHour, endTimeMinute);
  }
  else if( endTimeDay == 6){
    sprintf(ClockBuffer, "End Chg: SAT:", endTimeHour, endTimeMinute);
  }
  else if( endTimeDay == 7){
    sprintf(ClockBuffer, "End Chg: SUN:", endTimeHour, endTimeMinute);
  }
  else
  {
    timeSet = false;
    sprintf(ClockBuffer, "End Chg: NOT SET");
  }

  tft.putString(txtColumn2, txtLine8, ClockBuffer);

  if(timeSet){
    sprintf(ClockBuffer, "%02i:%02i", endTimeHour, endTimeMinute);
    tft.putString(txtColumn2 + 200, txtLine8, ClockBuffer);
  }

  if( fuel_filler_cap_status ){
    sprintf(ClockBuffer, "CHARGE CAP OPEN!");
    tft.putString(txtColumn1, txtLine8, ClockBuffer);
  }

  swap_display();
}

void update_simple(){
  layout_simple();
  tft.fillRect(256, 150, (unsigned long)RUNTIME % 512, 450, COLOR65K_GREEN);
  swap_display();
}

void swap_display(){
  if( ACTIVE_FRONT_BUFFER == DOUBLE_BUFFER_1 ){        
    ACTIVE_FRONT_BUFFER = DOUBLE_BUFFER_2;
    ACTIVE_BACK_BUFFER  = DOUBLE_BUFFER_1;
    tft.displayImageStartAddress(DOUBLE_BUFFER_2);
    tft.canvasImageStartAddress(DOUBLE_BUFFER_1);
  }
  else{
     
    ACTIVE_FRONT_BUFFER = DOUBLE_BUFFER_1;
    ACTIVE_BACK_BUFFER  = DOUBLE_BUFFER_2;
    tft.displayImageStartAddress(DOUBLE_BUFFER_1);
    tft.canvasImageStartAddress(DOUBLE_BUFFER_2);  
  }
}

int CalculateTimeDifferneceInSeconds(){
  //RTC_Clock_Data.Day
  int daysInSeconds = 0;
  int hoursInSeconds = 0;
  int minsInSeconds = 0;

  if( RTC_Clock_Data.Hour > endTimeHour ){
    hoursInSeconds = (endTimeHour + 24 - RTC_Clock_Data.Hour)*60*60;
  }
  else{
    hoursInSeconds = (endTimeHour - RTC_Clock_Data.Hour)*60*60;
  }
   
  if(  RTC_Clock_Data.Minute > endTimeMinute ){
    minsInSeconds = (endTimeMinute + 60 - RTC_Clock_Data.Minute)*60;
  }
  else{
    minsInSeconds = (endTimeMinute - RTC_Clock_Data.Minute)*60;
  }

  Serial.print("Time Difference: ");
  Serial.println(daysInSeconds + hoursInSeconds + minsInSeconds);
 
  return daysInSeconds + hoursInSeconds + minsInSeconds;
}

void SetChargerAndDutyCycle( int secondsToFinish, int missingAmpHours, int maxChargeingCapability){

  Serial.println("secondsToFinish : missingAmpHours : maxChargeingCapability");
  Serial.print(secondsToFinish);
  Serial.print(" ");
  Serial.print(missingAmpHours);
  Serial.print(" ");
  Serial.println(maxChargeingCapability);
 
  double charging_fraction = ((double)missingAmpHours/(double)secondsToFinish )/(double)maxChargeingCapability;

  tft.println("charging_fraction");
  tft.println(charging_fraction);
 
  if( charging_fraction < 0.33){
    numberOfChargersEnabled = 1;
    chargersDutyCyclePercent = 100*charging_fraction * 3.0;
  }
  else if(  charging_fraction < 0.66){
    numberOfChargersEnabled = 2;
    chargersDutyCyclePercent = 100*charging_fraction * 3.0/2.0;
  }
  else{
    numberOfChargersEnabled = 3;
    chargersDutyCyclePercent = 100*charging_fraction;
  }
}

void check_battery_state(){
    if( Power_supply_12V > AUX_end_charge_voltage){
      digitalWrite(CHARGE_PIN_OUTPUT, false);
    }
    if( Power_supply_12V < AUX_start_charge_voltage){
      digitalWrite(CHARGE_PIN_OUTPUT, true);
    }
}

void EverySecond(){

  update_RUNTIME();

  //Turn on LED while in service routine
  digitalWrite(13, 1);

  if (Device64Active == 1)
  {
    update_ADC_Values(64);  
  }

  if (Device65Active == 1)
  {    
    update_ADC_Values(65);      
  }

  check_battery_state();
 
  if (RTC.read(RTC_Clock_Data)) {
    //BOSS: Update the time in the display    
  }
  else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
  }
 
  TotalVolatageTracktionBattery = 0;
  for (int i = 1; i <= NUM_VALUES; i++)
  {
    TotalVolatageTracktionBattery += CellVoltage[i];
  }

  Blynk.virtualWrite(V5, TotalVolatageTracktionBattery);

  ChargerPreviouslyConnected = ChargerConnected;

  if( digitalRead(CHARGER_CONNECTED) && ChargerPreviouslyConnected == 0){
    DoFullCharge = true;
  }  
  ChargerConnected = digitalRead(CHARGER_CONNECTED);


  #ifdef DEBUG_2
  Serial.println("CurrentTotalAmpHours < BatteryMaxAmpHours: ");
  Serial.print(CurrentTotalAmpHours);
  Serial.print( " " );
  Serial.print(BatteryMaxAmpHours);
  Serial.print( " " );
  Serial.println(CurrentTotalAmpHours < BatteryMaxAmpHours);

  Serial.println("ChargingPWMIndex : chargersDutyCyclePercent : numberOfChargersEnabled : DoFullCharge");
  Serial.print(ChargingPWMIndex);
  Serial.print( " " );
  Serial.print(chargersDutyCyclePercent);
  Serial.print( " " );
  Serial.print(numberOfChargersEnabled);
  Serial.print( " " );
  Serial.println(DoFullCharge);  
  #endif

  if( ChargerConnected ){
    Serial.print( " ChargerConnected " );
    if( CurrentTotalAmpHours < BatteryMaxAmpHours){  
      Serial.println( " CurrentTotalAmpHours < BatteryMaxAmpHours " );  
      if( DoFullCharge ){
        Serial.println( " DoFullCharge " );  
        numberOfChargersEnabled = 4;
        chargersDutyCyclePercent = 100;
      }
      else{
        Serial.println( " !DoFullCharge  -> SetChargerAndDutyCycle" );  
        SetChargerAndDutyCycle( CalculateTimeDifferneceInSeconds(), BatteryMaxAmpHours - CurrentTotalAmpHours, AmpCompensatedValue[0] + AmpCompensatedValue[1] + AmpCompensatedValue[2] + AmpCompensatedValue[3]  );
      }
    }
    else{
      Serial.println( " All chargers OFF!!!!!!!---------------------- " );
      numberOfChargersEnabled = 0;      
      digitalWrite( CHARGE_WIRE_ENABLE_1, LOW );
      digitalWrite( CHARGE_WIRE_ENABLE_2, LOW );
      digitalWrite( CHARGE_WIRE_ENABLE_3, LOW );
      digitalWrite( CHARGE_SOLAR_ENABLE_1, LOW );
    }
  }
  else{
    Serial.println( " All chargers OFF!!!!!!!---------------------- " );
      numberOfChargersEnabled = 0;
      digitalWrite( CHARGE_WIRE_ENABLE_1, LOW );
      digitalWrite( CHARGE_WIRE_ENABLE_2, LOW );
      digitalWrite( CHARGE_WIRE_ENABLE_3, LOW );
      digitalWrite( CHARGE_SOLAR_ENABLE_1, LOW );
  }

  Blynk.virtualWrite(V11, (int)(100*CurrentTotalAmpHours/BatteryMaxAmpHours));

  fuel_filler_cap_status = digitalRead(FUEL_FILLER_CAP_STATUS);

  Power_supply_12V = analogRead(A9);

  Watchdog.reset();
  update_advanced();
}

void Every10Seconds(){
  //Turn on LED while in service routine
  digitalWrite(4, 1);
  digitalWrite(4, 0);
 
}

void EveryMinute(){
  //Enviromental Sensor code.
  OutsideTemp = (ds.getTempC());
  bme.read(Cabin_barometric_presure, Cabin_temp, Cabin_humidity);

  Blynk.virtualWrite(V0, Cabin_temp);
  Blynk.virtualWrite(V1, Cabin_barometric_presure);
  Blynk.virtualWrite(V2, Cabin_humidity);
  Blynk.virtualWrite(V7, (float)((int)(10*OutsideTemp))/10.0f);

  if( CurrentTotalAmpHours < BatteryMaxAmpHours && (ChargingPWMIndex < chargersDutyCyclePercent || DoFullCharge)  ){

    if( DoFullCharge ){
      digitalWrite(CHARGE_WIRE_ENABLE_1, HIGH);
      digitalWrite(CHARGE_WIRE_ENABLE_2, HIGH);
      digitalWrite(CHARGE_WIRE_ENABLE_3, HIGH);
      digitalWrite(CHARGE_SOLAR_ENABLE_1, HIGH);    
    }
    else{    
      if( numberOfChargersEnabled == 1 ){
        Serial.println( " 1 chargers ON---------------------- " );
        digitalWrite(CHARGE_WIRE_ENABLE_1, HIGH);
        digitalWrite(CHARGE_SOLAR_ENABLE_1, HIGH);
      }
      else if( numberOfChargersEnabled == 2){
        Serial.println( " 2 chargers ON---------------------- " );
        digitalWrite(CHARGE_WIRE_ENABLE_1, HIGH);
        digitalWrite(CHARGE_WIRE_ENABLE_2, HIGH);
        digitalWrite(CHARGE_SOLAR_ENABLE_1, HIGH);
      }
      else if( numberOfChargersEnabled == 3){
        Serial.println( " 3 chargers ON---------------------- " );
        digitalWrite(CHARGE_WIRE_ENABLE_1, HIGH);
        digitalWrite(CHARGE_WIRE_ENABLE_2, HIGH);
        digitalWrite(CHARGE_WIRE_ENABLE_3, HIGH);
        digitalWrite(CHARGE_SOLAR_ENABLE_1, HIGH);
      }  
      else{
        Serial.println( " All chargers OFF!!!!!!!---------------------- " );
        digitalWrite(CHARGE_WIRE_ENABLE_1, LOW);
        digitalWrite(CHARGE_WIRE_ENABLE_2, LOW);
        digitalWrite(CHARGE_WIRE_ENABLE_3, LOW);
        digitalWrite(CHARGE_SOLAR_ENABLE_1, LOW);
      }        
    }
  }
  else{
    Serial.println( " All chargers OFF!!!!!!!---------------------- " );
      digitalWrite(CHARGE_WIRE_ENABLE_1, LOW);
      digitalWrite(CHARGE_WIRE_ENABLE_2, LOW);
      digitalWrite(CHARGE_WIRE_ENABLE_3, LOW);
      digitalWrite(CHARGE_SOLAR_ENABLE_1, LOW);
  }

  ChargingPWMIndex +=10;
  if( ChargingPWMIndex >= 100 ){
    ChargingPWMIndex = 0;
  }
}

void EveryHour(){
  EEPROM.write(EEPROM_ADDRESS + 0, ((char*)&CurrentTotalAmpHours)[0]);
  EEPROM.write(EEPROM_ADDRESS + 1, ((char*)&CurrentTotalAmpHours)[1]);
  EEPROM.write(EEPROM_ADDRESS + 2, ((char*)&CurrentTotalAmpHours)[2]);
  EEPROM.write(EEPROM_ADDRESS + 3, ((char*)&CurrentTotalAmpHours)[3]);  
}

void update_RUNTIME(){
  unsigned long msec = millis();

  //Serial.print("Milis: ");
  //Serial.println(msec);
 
  double minutes = ((double)msec / 1000.0) / 60.0;
  RUNTIME = minutes / 60.0;  
}

void setup(){

  analogReadResolution(12);
  analogReadAveraging(100);
 
  OutsideTemp = (ds.getTempC());
 
  ((char*)&CurrentTotalAmpHours)[0] = EEPROM.read(EEPROM_ADDRESS + 0);
  ((char*)&CurrentTotalAmpHours)[1] = EEPROM.read(EEPROM_ADDRESS + 1);
  ((char*)&CurrentTotalAmpHours)[2] = EEPROM.read(EEPROM_ADDRESS + 2);
  ((char*)&CurrentTotalAmpHours)[3] = EEPROM.read(EEPROM_ADDRESS + 3);

  endTimeDay    = EEPROM.read(EEPROM_ADDRESS + 4);
  endTimeHour   = EEPROM.read(EEPROM_ADDRESS + 5);
  endTimeMinute = EEPROM.read(EEPROM_ADDRESS + 6);

  //Skal fjernes hvis der ikke skal understøttes sommertid
  summerTime    = EEPROM.read(EEPROM_ADDRESS + 7);
 
  Wire.begin();   //Startup i2c BUS, Default)(0)
  SPI.setSCK(14); //Set SPI clock to Pin14 insted of Pin13 (default)
  Serial.begin(115200);
  modbusSerial.begin(57600);
  delay(100);

  Serial.print("Amp Hours: ");
  Serial.println(CurrentTotalAmpHours);

  Serial.println("Starting up...");

  pinMode(RA8876_BACKLIGHT, OUTPUT);    // Set backlight pin to OUTPUT mode
  digitalWrite(RA8876_BACKLIGHT, HIGH); // Turn on backlight

  if (!tft.init())
  {
    Serial.println("Could not initialize RA8876");
  }

  tft.initExternalFontRom(0, RA8876_FONT_ROM_GT30L32S4W); // GT30L16U2W connected to SPI interface 0
  tft.clearScreen(DOUBLE_BUFFER_1, COLOR65K_BLACK);
  tft.selectExternalFont(RA8876_FONT_FAMILY_ARIAL, RA8876_FONT_SIZE_32, RA8876_FONT_ENCODING_ASCII);
  tft.setTextScale(1);
  tft.textColor(COLOR65K_WHITE, COLOR65K_BLACK);
  tft.canvasImageStartAddress(DOUBLE_BUFFER_1); //Start PAGE 1  
  Serial.println("Clearing double buffers");
 
  Serial.println("RA8876 Startup complete...");
  tft.putString(200, 36, "RA8876 Startup complete...");

  Serial.println("Reading SD card init file");
  tft.putString(200, 72, "Reading SD card init file");
  pinMode(SD_SELECT, OUTPUT);
  digitalWrite(SD_SELECT, HIGH); // disable SD card
  const size_t bufferLen = 127;
  char buffer[bufferLen];
  const char *filename = "/BMS.ini";
  SPI.begin();
  if (!SD.begin(SD_SELECT)){
    while (1) Serial.println("SD.begin() failed");      
  }
  IniFile ini(filename);
  if (!ini.open())
  {
    Serial.print("Ini file ");
    Serial.print(filename);
    Serial.println(" does not exist");
    tft.putString(200, 108, "Inifile does not exist");
    // Cannot do anything else
    while (1){}
  }
  Serial.println("SD Ini file exists");

  char SlaveString[18];

  for (int i = 1; i <= NUM_VALUES; i++)
  {
    if (i < 10)
    {
      sprintf(SlaveString, "Battery Slave #0%i", i);
    }
    else
    {
      sprintf(SlaveString, "Battery Slave #%i", i);
    }
    if (ini.getValue("Slave Units", SlaveString, buffer, bufferLen))
    {
      Serial.print("section 'Slave Units' has an entry 'Battery Slave ");
      Serial.print(i);
      Serial.print(" with value ");
      CellActive[i] = atoi(buffer);
      Serial.println(CellActive[i]);
    }
    else
    {
      Serial.print("Could not read 'Battery Slave #");
      Serial.print(i);
      Serial.print("' from section 'Slave Units', error was ");
      printErrorMessage(ini.getError());
    }

    if (i < 10)
    {
      sprintf(SlaveString, "Bleed #0%i", i);
    }
    else
    {
      sprintf(SlaveString, "Bleed #%i", i);
    }
    if (ini.getValue("Battery Bleed Values", SlaveString, buffer, bufferLen))
    {
      Serial.print("section 'Battery Bleed Values' has an entry 'Bleed ");
      Serial.print(i);
      Serial.print(" with value ");
      BleedValues[i] = atoi(buffer);
      Serial.println(BleedValues[i]);
    }
    else
    {
      Serial.print("Could not read 'Bleed #");
      Serial.print(i);
      Serial.print("' from section 'Battery Bleed Values', error was ");
      printErrorMessage(ini.getError());
    }
  }

  ini.getValue("Charge Limits", "AUX Start Charge Voltage", buffer, bufferLen);
  AUX_start_charge_voltage = atoi(buffer)/1000;
 
  ini.getValue("Charge Limits", "AUX Stop Charge Voltage", buffer, bufferLen);
  AUX_end_charge_voltage = atoi(buffer)/1000;

  tft.putString(200, 144, "Battery status read");

  if( ini.getValue("Battery Info", "Battery Max Charge Capacity mAmp Hours", buffer, bufferLen)){
    BatteryMaxAmpHours = atoi(buffer);
  }
  else
  {
    Serial.print("Could not read 'Battery Max Charge Capacity' from section 'Battery Info', error was ");
    printErrorMessage(ini.getError());
  }

  if (ini.getValue("Slave Units", "Current Slave #65", buffer, bufferLen))
  {
    Serial.print("section 'Slave Units' has an entry 'Current Slave #65' with value ");
    Device65Active = atoi(buffer);
    Serial.println(Device65Active);

    char enableBuffer[100];
    char sensitivityBuffer[100];
    char zeroBuffer[100];

    //Denne sektion (1) skal duplikeres for den anden ADC (#65 -> #64 og ADC_ChannelEnable1[i] -> ADC_ChannelEnable2[i] ) ved til kopling and det andet ADC board
    for (int i = 0; i < 4; i++)
    {
      sprintf(enableBuffer, "ADC%i channel enable", i + 1);
      ini.getValue("Current Slave #65", sensitivityBuffer, buffer, bufferLen);
      Serial.print("section 'Current sensor data' has an entry 'ADC Enable");
      Serial.print(i + 1);
      Serial.print(" sensitivity' with value ");
      ADC_ChannelEnable1[i] = atoi(buffer);
      Serial.println(ADC_ChannelEnable1[i]);

      sprintf(sensitivityBuffer, "ADC%i sensitivity", i + 1);
      ini.getValue("Current Slave #65", sensitivityBuffer, buffer, bufferLen);
      Serial.print("section 'Current sensor data' has an entry 'ADC");
      Serial.print(i + 1);
      Serial.print(" sensitivity' with value ");
      ADC_Sensitivity[i] = atoi(buffer);
      Serial.println(ADC_Sensitivity[i]);

      sprintf(zeroBuffer, "ADC%i ZERO Volt", i + 1);
      ini.getValue("Current Slave #65", zeroBuffer, buffer, bufferLen);
      Serial.print("section 'Current sensor data' has an entry 'ADC");
      Serial.print(i + 1);
      Serial.print(" ZERO Volt' with value ");
      ADC_ZERO_Volt[i] = atoi(buffer);
      Serial.println(ADC_ZERO_Volt[i]);
    }
  }
  else
  {
    Serial.print("Could not read 'Current Slave #65' from section 'Slave Units', error was ");
    printErrorMessage(ini.getError());
  }
  //Sektionen (1) ender her

  tft.putString(200, 180, "Current slave status read");

  SerialMon.begin(115200);
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(115200);
  delay(3000);
  SerialMon.println("Initializing modem...");
  modem.restart();
  String temp = "Modem: ";
  String modemInfo = temp + modem.getModemInfo();
  SerialMon.println(modemInfo);
  tft.putString(200, 216, modemInfo.c_str());
  //modem.simUnlock("1066");
  modem.setPreferredMode(3);

  char tempBuffer[100];
  sprintf(tempBuffer, "AmpHours on battery %i ", (int)CurrentTotalAmpHours  );
  tft.putString(200, 252, tempBuffer);

  sprintf(tempBuffer, "Total possible AmpHours %i", (int)BatteryMaxAmpHours );
  tft.putString(200, 288, tempBuffer);

  Blynk.begin(auth, modem, apn, user, pass); //Startup BLYNK
 
  tft.putString(200, 324, "Blynk initialized");
 
  Timer1Second.begin(EverySecond, 1000000);
  Timer10Second.begin(Every10Seconds, 10000000);
  Timer60Second.begin(EveryMinute,  60000000);
  Timer360Second.begin(EveryHour, 3600000000.0);

  pinMode(CHARGER_CONNECTED, INPUT);

  pinMode(CHARGE_WIRE_ENABLE_1, OUTPUT);
  pinMode(CHARGE_WIRE_ENABLE_2, OUTPUT);
  pinMode(CHARGE_WIRE_ENABLE_3, OUTPUT);
  pinMode(CHARGE_SOLAR_ENABLE_1, OUTPUT);

  set_BLEED_Values();
 
  Watchdog.enable(10000);
  Serial.print("Enabled the watchdog");
}

void loop(){  
  Blynk.run();
  timer.run();  
  Watchdog.reset();  
}
