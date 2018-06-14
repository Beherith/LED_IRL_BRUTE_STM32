//lean printf: http://www.stm32duino.com/viewtopic.php?t=1014


/*
  Function  Description Number of bytes Decimal places  UNIT  Read/Write  Register address
  U-SET Voltage setting 2 2 V R/W 0000H
  I-SET Current setting 2 2 A R/W 0001H
  UOUT  Output voltage display value  2 2 V R 0002H
  IOUT  Output current display value  2 2 A R 0003H
  POWER Output power display value  2 1 or 2  W R 0004H
  UIN Input voltage display value 2 2 V R 0005H
  LOCK  Key lock  2 0 - R/W 0006H
  PROTECT Protection status 2 0 - R 0007H
  CV/CC Constant Voltage / Constant Current status  2 0 - R 0008H
  ONOFF Switch output state 2 0 - R/W 0009H
  B_LED Backlight brightness level  2 0 - R/W 000AH
  MODEL Product model 2 0 - R 000BH
  VERSON  Firmware Version  2 0 - R 000CH
  EXTRACT_M Shortcut to bring up the required data set  2 0 - R/W 0023H
  U-SET Voltage settings  2 2 V R/W 0050H
  I-SET Current setting 2 2 A R/W 0051H
  S-OVP Over-voltage protection value 2 2 V R/W 0052H
  S-OCP Over-current protection value 2 2 A R/W 0053H
  S-OPP Over power protection 2 1   W R/W 0054H
  B-LED Backlight brightness levels 2 0 - R/W 0055H
  M-PRE Memory Preset Number  2 0 - R/W 0056H
  S-INI Power output switch 2 2 - R/W 0057H
*/


/* MODBUS test:

  Host sends	Number of bytes	Information sent	Notes
  Slave address	1	01	From host to slave address 01H
  Function code	1	03	Read holding register(s)
  Register starting address	2	0002H	Register starting address
  Number of registers to read	2	0002H	A total of 2 registers (4 byte = 2 words)
  CRC Checksum:	2	65CBH	CRC Checksum from Host:

*/
#define DEBUG

//LCD STUFF from https://github.com/Nkawu/TFT_22_ILI9225/blob/master/examples/GFX_Font_Demo/GFX_Font_Demo.ino
#include "SPI.h"
#include "TFT_22_ILI9225.h"
// Include font definition file
// NOTE: These files may not have all characters defined! Check the GFXfont def
// params 3 + 4, e.g. 0x20 = 32 = space to 0x7E = 126 = ~
#include <../fonts/FreeSans9pt7b.h> //not really 9 pts
//#include <../fonts/Picopixel.h>
extern uint8_t Terminal6x8[];
#define TFT_RS  PB11
#define TFT_CS  PB12 // SS
#define TFT_SDI PB15 // MOSI
#define TFT_CLK PB13 // SCK
#define TFT_LED 0 //but tied to PB15 really // 0 if wired to +5V directly
#define TFT_RST PB10

#define TFT_BRIGHTNESS 200

// Use hardware SPI (faster - on Uno: 13-SCK, 12-MISO, 11-MOSI)
//!!!! REMEMBER THAT SPI.CPP HAS BEEN CHANGED FOR SPI1!!!!!!!!!!!!!!!!!
//!!!! IN STM32DUINO LIBS
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, TFT_BRIGHTNESS);
//Software SPI:
//TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_SDI, TFT_CLK, TFT_LED, TFT_BRIGHTNESS);
int16_t x = 0, y = 0, w, h;

#define BAUDRATE 9600 //for modbus

const uint32_t minimum_vin = 3900;
const float thermal_shutdown_temperature = 70.0; // above this temperature, the firmware will shutdown all lights, while running all fans at full blast.
#define DPS_OVERPOWERPROTECTION 4500
#define DPS_OVERCURRENTPROTECTION 1250
#define DPS_OVERVOLTAGEPROTECTION 3800


float TargetTemperature = 55.0;

const float minimum_fan_temperature_threshold = 40.0;

#define FANTIMER_MAX 2880
HardwareTimer fanpwmtimer1(3); //uses timer3
uint32_t last_fan_sanity_check = 0;

enum CMD {
  USET = 0,
  ISET = 1,
  UOUT = 2,
  IOUT = 3,
  POWER = 4,
  UIN = 5,
  LOCK = 6, //Key lock function reading and writing value are 0 and 1, 0 represents not lock, 1 represents lock
  PROTECT = 7, // Protection status reading value are 0-3, 0 represents good running, 1 represents OVP, 2 represents OCP, 3 represents OPP.
  CVCC = 8, //Constant voltage and constant current reading value are 0-1, 0 represents CV, 1 represents CV
  ONOFF = 9,
  BLED = 10, //Level of backlight rank reading and writing value is 0-5, 0 represents the darkest, 5 represents the brightest.
  MODEL = 11,
  VERSION = 12,
  EXTRACT_M = 0x23,
  S_OVP = 0x52,
  S_OCP = 0x53,
  S_OPP = 0x54,
  S_INI = 0x57, //Control output function reading and writing value are 0-1, 0 represents close, 1 represents open.
};

uint32_t ADC(int adcpin, int precision = 16) { //Returns values from 0-4095 (12 bits)
  analogRead(adcpin);
  analogRead(adcpin);
  uint32_t result = 0 ;
  for (int i = 0; i < precision; i++) {
    result += analogRead(adcpin);
  }
  result = result / precision;
#ifdef DEBUG
  Serial.printf("ADC on pin %d is %d, precision = %d\n", adcpin, result, precision);
#endif
  return result;
}


float Thermistor(int thermpin, int multisample = 16) { //returns the celsius value in a float
  float RawADC = ADC(thermpin, multisample);
#define PullupR 4700.0
  long Resistance = PullupR * (RawADC / 4096.0) / (1 - RawADC / 4096.0); //changed cause i use thermistors the other way around
  float Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later

  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius
  Temp = Temp;
#ifdef DEBUG
  Serial.printf("ThermADC = %d, R = %d, T = %d", int(RawADC), int(Resistance), int(Temp));
#endif
  return Temp;
}

class DPS {
  public:

    //stuff to move in:
    //pointers to fan speeds
    volatile uint32_t * fan1rpm;
    volatile uint32_t * fan2rpm;
    volatile uint32_t * fan3rpm;
    uint32_t fantimerch;// e.g. TIMER_CH3;
    uint32_t fanpowerpin = 0;
    uint32_t fanpwmpin = 0;
    int fanspeed = 0;

    //temperature stuff:
    uint32_t temperaturepin = -1;
    float targettemperature = 60.0;
    float lasttemperatures[10];
    float currenttemperature = 0;

    //DPS management stuff:
    uint16_t targetcurrent = 0; // 2 digit precision, max 1250
    uint32_t potpin = -1;
    uint32_t potvalue = 0;
    uint32_t newpotvalue = 0;
    uint16_t vset = 0;
    uint16_t iset = 0;
    uint16_t vout = 0;
    uint16_t iout = 0;
    uint16_t power = 0;
    uint16_t vin = 0;
    uint16_t lock = 0;
    uint16_t protect = 0;
    uint16_t cvcc = 0;
    uint16_t onoff = 0;

    uint16_t powerout_calc = 0; // 1 or 2 digit precision

    int isOK = 0; //Value of 0 means all is ok, all others are errors
    HardwareSerial * myserial;
    const uint16_t DPStimeout = 1000;
    static const int MBUFSIZE = 32;
    uint8_t mbuf[MBUFSIZE];
    uint16_t readresults[10];
    uint32_t responsedelay = 0;
    char * name;
    //DPS(){};
    float getVin();
    int drawInfo(int x);
    DPS(HardwareSerial * serial);
    int init( uint32_t _fanpowerpin, uint32_t _fantimerch, uint32_t _temperaturepin, uint32_t _potpin, uint32_t _fanpwmpin, char * _name);
    void setfanspeedvars(volatile uint32_t * f1, volatile uint32_t * f2, volatile uint32_t * f3);
    void emergency_shutdown(char * reason);
    int update();
    int write( uint16_t command, uint16_t value); //returns 1 if command was OK
    uint16_t read(uint16_t command, int len); //returns 0xFFFF if command failed
    float readinputs();
    uint16_t CRC(uint8_t * buf, int len);
    int checkCRC(int len); //returns 0 if crc ok, -1 if crc failed
    void clrbuf();
    void printpacketHEX(int len);

};

DPS::DPS(HardwareSerial * serial) {
  myserial = serial;
}

int DPS::init( uint32_t _fanpowerpin, uint32_t _fantimerch, uint32_t _temperaturepin, uint32_t _potpin, uint32_t _fanpwmpin, char * _name) {
  fanpowerpin = _fanpowerpin;
  fantimerch = _fantimerch;
  fanpwmpin = _fanpwmpin;
  temperaturepin = _temperaturepin;
  potpin = _potpin;
  name = _name;
  for (int i = 0; i < 10; i++) lasttemperatures[i] = 0.0;

  pinMode(potpin, INPUT_ANALOG);
  pinMode(temperaturepin, INPUT_ANALOG);
  pinMode(fanpowerpin, OUTPUT);
  //TODO: init the fanpwnpins to a sane value?
  digitalWrite(fanpowerpin, HIGH);
  delay(100);
  readinputs();
#ifndef DEBUG
  isOK += write(LOCK, 1);
#endif
  isOK += write(BLED, 5);
  isOK += write(S_OPP, DPS_OVERPOWERPROTECTION); //300.0W max - for now :D
  isOK += write(S_OVP, DPS_OVERVOLTAGEPROTECTION); //38.00V max
  isOK += write(S_OCP, DPS_OVERPOWERPROTECTION); //12.50A max
  isOK += write(ISET, 0);
  isOK += write(USET, min(DPS_OVERVOLTAGEPROTECTION, vin - 101));
  isOK += write(ONOFF, 1);
  if (isOK != 0) Serial.printf("DPS%s Failed to initialize, isOK = %d\n", name, isOK);
  else Serial.printf("DPS%s OK.\n", name);
  return isOK;
}

float DPS::readinputs() {
  if (read(0, 10) == 0) { //means read was successful
    vset = readresults[USET];
    iset = readresults[ISET];
    iset = readresults[ISET];
    vout = readresults[UOUT];
    iout = readresults[IOUT];
    power = readresults[POWER];
    vin = readresults[UIN];
    lock = readresults[LOCK];
    protect = readresults[PROTECT];
    cvcc = readresults[CVCC];
    onoff = readresults[ONOFF];
    powerout_calc = ((iout / 100) * (vout / 100));
  }
  newpotvalue = ADC(potpin);
  currenttemperature = Thermistor(temperaturepin);
}

int DPS::update() { //Should be called every second!
  readinputs();
#ifndef DEBUG
  //if we are in emergency_shutdown, dont do anything
  if (isOK < -10) {
    emergency_shutdown("LOST DPS CONNECTION");
  }
  if (isOK != 0) {
    return;
  }

#endif

  //if fans should have been on, we should have seen an update for them by now.
  //nah, well check for them globally

  if (fanspeed > 0) {
    //if( *fan1rpm == 0) emergency_shutdown("Fan 1 off");
    //if( *fan2rpm == 0) emergency_shutdown("Fan 2 off");
    //if( *fan3rpm == 0) emergency_shutdown("Fan 3 off");
  }

  //unstable fans
  if ((fanspeed > 0 ) && (fanSpeedUnstable(*fan1rpm, *fan2rpm, *fan3rpm))) {
    //emergency_shutdown("Unstable fans");
  }

  if (currenttemperature > thermal_shutdown_temperature) { //Overtemp
    Serial.printf("ERROR: %s overtemperature %d, shutting down\n", name, int(currenttemperature));
    emergency_shutdown("Overtemperature");
  }

  //undertemp, e.g. if thermistor gets disconnected
  if (currenttemperature < -20) {
    Serial.printf("ERROR: %s undertemperature %d, shutting down\n", name, int(currenttemperature));
    emergency_shutdown("Undertemp");
  }

  if (vin < minimum_vin) {   //input voltage instability
    Serial.printf("ERROR: Undervoltage on %s input", name);
    Serial.printf("VinLeft = %dmV, Minimum = %dmV\n", vin * 10, minimum_vin * 10);
    //emergency_shutdown("Undervoltage");
  }

  if (powerout_calc > 450) { //overpower
    emergency_shutdown("Overpower");
  }

  //push temperature table for PID:
  for (int i = 9; i > 0; i--) {
    lasttemperatures[i] = lasttemperatures[i - 1];
  }
  lasttemperatures[0] = currenttemperature;

  //ok, we can finally actually do logic
  if (currenttemperature < minimum_fan_temperature_threshold) {
    fanspeed = 0;
    //digitalWrite(fanpowerpin,LOW);
  } else {
    if (currenttemperature < targettemperature) {
      fanspeed = max(1, fanspeed - 1);
      //digitalWrite(fanpowerpin,HIGH);
    } else {
      fanspeed = min(100, fanspeed + (int(currenttemperature - targettemperature) * int(currenttemperature - targettemperature)));
    }
  }
  fanpwmtimer1.setCompare(fantimerch, map(fanspeed, 0, 100, 0, FANTIMER_MAX - 1)); //init at a low speed
  if (fanspeed > 0) {
    digitalWrite(fanpowerpin, HIGH);
  } else {
    digitalWrite(fanpowerpin, LOW);
  }

  //now we can check if potmeter changed:
  if ((newpotvalue < 16) && (potvalue >= 16)) {
    potvalue = newpotvalue;
    targetcurrent = pot_to_target_current(potvalue);
    write(ISET, targetcurrent);
    write(ONOFF, 0);
    Serial.printf("%s Turning light off\n", name);
  } else if ((newpotvalue > 16) && (potvalue <= 16)) {
    potvalue = newpotvalue;
    targetcurrent = pot_to_target_current(potvalue);
    write(ISET, targetcurrent);
    write(ONOFF, 1);
    Serial.printf("%s Turning light on\n", name);
  } else if (abs(newpotvalue - potvalue) > 16) { // A knob has been twiddled
    //calculate new target power.
    Serial.printf("%s pot changed from %d to %d\n", name, potvalue, newpotvalue);
    potvalue = newpotvalue;
    targetcurrent = pot_to_target_current(potvalue);
    write(ISET, targetcurrent);
  }
  return 0;
}

void DPS::emergency_shutdown(char * reason = "?") {
  Serial.print(name);
  Serial.print(": EMERGENCY SHUTDOWN reason: ");
  Serial.println(reason);
  fanpwmtimer1.setCompare(fantimerch, FANTIMER_MAX - 1); //go full blast

  tft.setFont(Terminal6x8);

  int ystep = 10;
  char strbuf[32];
  //tft is 220*170 px size
  //y position in increments of 10
  x = 0;
  y = 150;

  int rx = 220;

  sprintf(strbuf, "%s EMERGENCY STOP", name);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_RED);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "%s", reason);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_RED);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;



  digitalWrite(fanpowerpin, HIGH);
  write(ONOFF, 0);
  isOK--;
  /*
    //  tft.clear();
    tft.setGFXFont(&FreeSans9pt7b); // Set current font
    //tft.getGFXTextExtent("EMERGENCY STOP", x, y, &w, &h); // Get string extents
    tft.drawGFXText(0,40,name,COLOR_RED);
    tft.drawGFXText(0,80,"EMERGENCY STOP",COLOR_RED);
    tft.drawGFXText(0,120,reason,COLOR_WHITE);



    //while(1){
  */

  //delay(1000);
  //}
}

void DPS::setfanspeedvars(volatile uint32_t * f1, volatile uint32_t * f2, volatile uint32_t * f3) {
  fan1rpm = f1;
  fan2rpm = f2;
  fan3rpm = f3;
}
int DPS::drawInfo(int x) {

  tft.setFont(Terminal6x8);
  int ystep = 10;
  char strbuf[32];
  int rx = 110;
  //tft is 220*170 px size
  //y position in increments of 10
  y = 0;
  //-------------------LEFT-------------------
  sprintf(strbuf, "%s isOK: %d", name, isOK);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Temperature %dC", int(currenttemperature));
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;


  sprintf(strbuf, "Vin %d.%02dV", vin / 100, vin % 100);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Vout %d.%02dV", vout / 100, vout % 100);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Uout %d.%02dA", iout / 100, iout % 100);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Fan1 %drpm", *fan1rpm);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Fan2 %drpm", *fan2rpm);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Fan3 %drpm", *fan3rpm);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Fan target %d", fanspeed);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Pot %d", potvalue);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Power %dW", (uint32_t)((iout/10)*(vout/10))/100);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Delay %dms", responsedelay);
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;


  return y;

}

void DPS::clrbuf() {
  for (int i = 0; i < MBUFSIZE; i++) mbuf[i] = 0;
}

void DPS::printpacketHEX(int len) {
  for (int i = 0; i < len; i++) {
    Serial.print(mbuf[i], HEX);
    Serial.print(':');
  }
  Serial.write('\n');
}

int DPS::checkCRC(int len) { //len is the length of the whole message
  uint16_t calccrc = CRC(mbuf, len - 2);
  uint16_t recvcrc = (mbuf[len - 2] << 8) + mbuf[len - 1];
  if (calccrc != recvcrc) {
    Serial.print(name);
    Serial.print(": CRC failed: recieved ");
    Serial.print(recvcrc, HEX);
    Serial.print(" instead of: ");
    Serial.println(calccrc, HEX);
    isOK--;
    return -1;
  } else {
#ifdef DEBUG
    Serial.println("CRC OK");
#endif
    return 0;
  }

}

int DPS::write( uint16_t command, uint16_t value) { //returns 0 if command was OK, -1 if command failed
  clrbuf();
  mbuf[0] = 1; //Address of unit, always 1
  mbuf[1] = 6; //Write command
  mbuf[3] = command;
  mbuf[4] = value >> 8;
  mbuf[5] = value & 0xFF;
  uint16_t crc = CRC(mbuf, 6); //might need to swap this...
  mbuf[6] = crc >> 8;
  mbuf[7] = crc & 0xFF;

#ifdef DEBUG
  Serial.printf("%s: Sending packet: ", name);
  printpacketHEX(8);
#endif
  while (myserial->available()) myserial->read(); //clear serial receive buffer
  myserial->write(mbuf, 8);

  clrbuf();
  int buffptr = 0;
  uint32_t listenstarttime = millis();
  //single writes will always be 8 chars long messages both ways
  while ( ((millis() - listenstarttime) < DPStimeout) && buffptr < 8) {
    if (myserial->available()) {
      mbuf[buffptr] = myserial->read();
      buffptr++;
    }
  }

  delay(50);
#ifdef DEBUG
  Serial.printf("Recieved message %d bytes", buffptr);
  printpacketHEX(8);
#endif
  return checkCRC(buffptr);
}
uint16_t DPS::CRC(uint8_t * buf, int len) {
  // From: http://www.ccontrolsys.com/w/How_to_Compute_the_Modbus_RTU_Message_CRC
  // CRC should be DD18
  //char *t = (char *)"DEADBEEF";
  // Compute the MODBUS RTU CRC
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  crc = (crc >> 8) | (crc << 8); //yeah had to swap them...

  return crc;
}
uint16_t DPS::read(uint16_t command, int nfields) { //returns 0xFFFF if command failed
  //queries n fields at a time
  uint16_t value = 0;

  clrbuf();
  mbuf[0] = 1; //Address of unit, always 1
  mbuf[1] = 3; //read command
  mbuf[3] = command;
  mbuf[5] = nfields; //read a single register
  uint16_t crc = CRC(mbuf, 6); //might need to swap this...
  mbuf[6] = crc >> 8;
  mbuf[7] = crc & 0xFF;
#ifdef DEBUG
  Serial.printf("%s: Sending packet: ", name);
  printpacketHEX(8);
#endif

  while (myserial->available()) myserial->read(); //clear serial receive buffer
  myserial->write(mbuf, 8);

  //expected response size is 5+2*nfields?
  int expectedpacketsize = 5 + 2 * nfields;
  unsigned long listenstarttime = millis();

  clrbuf();
  int buffptr = 0;
  while ( (millis() - listenstarttime < 1000) && buffptr < expectedpacketsize) {
    if (myserial->available()) {
      mbuf[buffptr] = myserial->read();
      buffptr++;
    }
  }
  responsedelay = millis() - listenstarttime;
#ifdef DEBUG
  Serial.printf("%s: Recieved message %d bytes", name, buffptr);
  printpacketHEX(buffptr);
#endif
  delay(50);
  if (checkCRC(buffptr) == 0) {
    //CRC was correct, check that we got the expected number of values too
    if ((mbuf[2] == 2 * nfields) && (buffptr == expectedpacketsize)) {
      for (int i = 0; i < nfields; i++) {
        readresults[i] = (mbuf[3 + 2 * i] << 8) + mbuf[4 + 2 * i];
      }
      isOK = min(isOK + 1, 0);
      return 0;
    }
    Serial.printf("%s: CRC was ok but packet is malformed\n", name);
  }
  return -1;
}


//Serial1 TX = PA9
//Serial1 RX = PA10
//Serial2 TX = PA2
//Serial2 RX = PA3
//Serial is used for debugging, fan speeds, etc...

//Analog reads:
//PA0 -- ADC0 Internal temperature, 10k pullup
const int internalTempPin = PA0;
//PA4 -- ADC4 LEFT side thermistor, 10k pullup
const int leftTempPin = PA4;
//PA5 -- ADC5 RIGHT side thermistor, 10k pullup
const int rightTempPin = PA5;
//PA6 -- ADC6 LEFT side potentiometer
const int leftPotPin = PA6;
//PA7 -- ADC7 RIGHT side potentiometer
const int rightPotPin = PA7;
float internalTemperature = 0.0;


//Display pins on SPI2:
//SS2 PB12
//SCK2 PB13
//MISO PB14
//MOSI PB15
//RS PB11
//RST PB10

//Digital input pins:
// fans produce 2 pulses per revolution, so at an RPM range of 300-3000, they should come each at 20-200ms. Store their values in millis().
// open-collector type pins

//Digital outputs:
//fan enable pins
//
const int fanLeftEnablePin = PB9;
const int fanRightEnablePin = PA8;

//fan speed pins, using external interrupts:
//http://docs.leaflabs.com/static.leaflabs.com/pub/leaflabs/maple-docs/0.0.12/external-interrupts.html#external-interrupts-exti-line
//Thus all pins should be on one port, since PA4 and PB4 cant both have interrupts
//
const int speedL1pin = PB3;
const int speedL2pin = PB4;
const int speedL3pin = PB5;
const int speedR1pin = PB6;
const int speedR2pin = PB7;
const int speedR3pin = PB8;

const int pwmLpin = PB0;
const int pwmRpin = PB1;

#define BOARD_LED_PIN PC13

volatile uint32_t speedL1 = 0, speedL2 = 0, speedL3 = 0, speedR1 = 0, speedR2 = 0, speedR3 = 0;
volatile uint32_t lastL1time = 0, lastL2time = 0, lastL3time = 0, lastR1time = 0, lastR2time = 0, lastR3time = 0;

void speedL1int() {
  speedL1 = 30000 / max(1, (millis() - lastL1time));
  lastL1time = millis();
}
void speedL2int() {
  speedL2 = 30000 / max(1, (millis() - lastL2time));
  lastL2time = millis();
}
void speedL3int() {
  speedL3 = 30000 / max(1, (millis() - lastL3time));
  lastL3time = millis();
}
void speedR1int() {
  speedR1 = 30000 / max(1, (millis() - lastR1time));
  lastR1time = millis();
}
void speedR2int() {
  speedR2 = 30000 / max(1, (millis() - lastR2time));
  lastR2time = millis();
}
void speedR3int() {
  speedR3 = 30000 / max(1, (millis() - lastR3time));
  lastR3time = millis();
}

//Safety checks

const int fan_sanity_check_interval = 2000; //ms

uint32_t housekeepingtime = 0;
uint32_t displayupdatetime = 0;

DPS DPSLEFT( &Serial1);
DPS DPSRIGHT( &Serial2);
char leftname[] = "LEFT";
char rightname[] = "RIGHT";
int dpsLeftOK = 0;
int dpsRightOK = 0;

void tftinit() {
  tft.begin();
  tft.setOrientation(3); //left rotated
  tft.clear();
  // Draw first string in big font
  String s1 = "LED_IRL";
  tft.setGFXFont(&FreeSans9pt7b); // Set current font
  tft.getGFXTextExtent(s1, x, y, &w, &h); // Get string extents
  y = h; // Set y position to string height
  x = 110 - w / 2;
  tft.drawGFXText(x, y, s1, COLOR_RED); // Print string
  s1 = "MINI BRUTE";
  tft.setGFXFont(&FreeSans9pt7b); // Set current font
  tft.getGFXTextExtent(s1, x, y, &w, &h); // Get string extents
  y = 70; // Set y position to string height
  x = 110 - w / 2;
  tft.drawGFXText(x, y, s1, COLOR_GREEN); // Print string

  s1 = "mysterme@gmail.com";
  tft.setGFXFont(&FreeSans9pt7b); // Set current font
  tft.getGFXTextExtent(s1, x, y, &w, &h); // Get string extents
  y = 120; // Set y position to string height
  x = 110 - w / 2;
  tft.drawGFXText(x, y, s1, COLOR_BLUE); // Print string

  s1 = "BAG HODLERS INC.";
  tft.setGFXFont(&FreeSans9pt7b); // Set current font
  tft.getGFXTextExtent(s1, x, y, &w, &h); // Get string extents
  y = 170; // Set y position to string height
  x = 110 - w / 2;
  tft.drawGFXText(x, y, s1, COLOR_WHITE); // Print string

  /*
    // Draw second string in smaller font
    tft.setGFXFont(&TomThumb);  // Change font
    String s2 = "Hello"; // Create string object
    tft.getGFXTextExtent(s2, x, y, &w, &h); // Get string extents
    y += h + 10; // Set y position to string height plus shift down 10 pixels
    tft.drawGFXText(x, y, s2, COLOR_BLUE); // Print string


    tft.setFont(Terminal6x8);  // Change font
    String s3 = "World! LOL IMA RAEP U"; // Create string object
    y += h + 12; // Set y position to previous string height plus shift down 2 pixels
    tft.drawText(x, y, s3, COLOR_WHITE); // Print string
  */
}


void updateDisplay() {
  //tft.clear();
  tft.setFont(Terminal6x8);

  int ystep = 10;
  char strbuf[32];
  //tft is 220*170 px size
  //y position in increments of 10
  x = 0;
  y = ystep;
  //-------------------LEFT-------------------
  DPSLEFT.drawInfo(0);
  y = 0;
  y = DPSRIGHT.drawInfo(110);

  //------CENTER-------------------
  int rx = 150;
  x = 50;
  sprintf(strbuf, "Internals %dC", int(internalTemperature));
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  sprintf(strbuf, "Updated in %dms", millis() - displayupdatetime);
  displayupdatetime = millis();
  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, strbuf, COLOR_WHITE);
  y = y + ystep;

  tft.fillRectangle(x, y, x + rx, y + ystep - 1, COLOR_BLACK);
  tft.drawText(x, y, "BAG HODLERS INC.", COLOR_BLUE);
  y = y + ystep;


}


void setup() {
  // put your setup code here, to run once:
  //Init fans
  tftinit();
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) {
    Serial.printf("Initializing LED IRL BRUTE in %ds\n", 10 - i);
    delay(1000);
  }
  pinMode(speedL1pin, INPUT_PULLUP);
  pinMode(speedL2pin, INPUT_PULLUP);
  pinMode(speedL3pin, INPUT_PULLUP);
  pinMode(speedR1pin, INPUT_PULLUP);
  pinMode(speedR2pin, INPUT_PULLUP);
  pinMode(speedR3pin, INPUT_PULLUP);

  attachInterrupt(speedL1pin, speedL1int, FALLING);
  attachInterrupt(speedL2pin, speedL2int, FALLING);
  attachInterrupt(speedL3pin, speedL3int, FALLING);
  attachInterrupt(speedR1pin, speedR1int, FALLING);
  attachInterrupt(speedR2pin, speedR2int, FALLING);
  attachInterrupt(speedR3pin, speedR3int, FALLING);

  pinMode(pwmLpin, PWM);
  pinMode(pwmRpin, PWM);
  fanpwmtimer1.pause();
  fanpwmtimer1.setPrescaleFactor(1); //divides by 1, e.g. 72mhz
  fanpwmtimer1.setOverflow(FANTIMER_MAX);//for a pwm frequency of 25khz
  fanpwmtimer1.setCompare(TIMER_CH3, 1000); //init at a low speed
  fanpwmtimer1.setCompare(TIMER_CH4, 1000); //init at a low speed
  fanpwmtimer1.refresh();
  fanpwmtimer1.resume();

  //Set up thermistors
  pinMode(internalTempPin, INPUT_ANALOG);

  pinMode(BOARD_LED_PIN, OUTPUT);

  //INIT DPS modules
  //uint32_t _fanpowerpin, uint32_t _fantimerch, uint32_t _temperaturepin, uint32_t _potpin, String _name

  //lock the keypads, init the device
  DPSLEFT.myserial->begin(BAUDRATE);
  DPSRIGHT.myserial->begin(BAUDRATE);
  DPSLEFT.init(fanLeftEnablePin, TIMER_CH3, leftTempPin, leftPotPin, pwmLpin, leftname);
  DPSRIGHT.init(fanRightEnablePin, TIMER_CH4, rightTempPin, rightPotPin, pwmRpin, rightname);
  DPSLEFT.setfanspeedvars(&speedL1, &speedL2, &speedL3);
  DPSRIGHT.setfanspeedvars(&speedR1, &speedR2, &speedR3);
  DPSLEFT.targettemperature = TargetTemperature;
  DPSRIGHT.targettemperature = TargetTemperature;

  tft.clear();
}



int fanStopped(uint32_t lastfantime, char * fanid) {
  if (lastfantime < last_fan_sanity_check) {
    Serial.printf("FAN ERROR %s: Fan should be on, but has not responded in %d ms!\n", fanid, millis() - lastL1time);
    return 1;
  }
  else return 0;
}


int fanSpeedUnstable(int speed1, int speed2, int speed3) {
  if    (speed1 * 2 > speed2 * 3 \
         || speed2 * 2 > speed1 * 3 \
         || speed1 * 2 > speed3 * 3 \
         || speed3 * 2 > speed1 * 3 \
         || speed3 * 2 > speed2 * 3 \
         || speed2 * 2 > speed3 * 3 ) {

    Serial.printf("FAN ERROR, SPEED MISMATCH: 1 = %d, 2 = %d, 3 = %d\n", speed1, speed2, speed3);
    return 1;
  } else return 0;
}


uint16_t pot_to_target_current(uint32_t inpotvalue) {
  //use a squared value for it
  // ADC is 12 bits
  // so our max value is 2^16
  inpotvalue = (inpotvalue * inpotvalue) >> 8; //
  //since the nominal voltage is 36 volts, and the max power is 450wpc, the max current is 12.5A
  //we can set the current to a precision of 2 decimal places:
  uint16_t out_pot = map(inpotvalue, 0, 65305, 0, 1250);
  return out_pot;
}

void loop() {
  //Start off by reading all 3 temperature sensors and the pots
  digitalWrite(BOARD_LED_PIN, HIGH);
  housekeepingtime = millis();

  internalTemperature = Thermistor(internalTempPin);

  delay(10);
  DPSLEFT.update();
  DPSRIGHT.update();

  updateDisplay();

  housekeepingtime = millis() - housekeepingtime;
  Serial.printf("%dms spent housekeeping\n", housekeepingtime);
  Serial.printf("Internal Temperature = %d C\n", int(internalTemperature));


  delay(10);
  digitalWrite(BOARD_LED_PIN, LOW);

  // internal overtemperature
  if (internalTemperature > thermal_shutdown_temperature) {
    Serial.printf("ERROR: Internal overtemperature %d, shutting down\n", int(internalTemperature));
    DPSLEFT.emergency_shutdown("Internal overheat");
    DPSRIGHT.emergency_shutdown("Internal overheat");
  }


  //DO THIS LAST:

  uint32_t now = millis();
  if (now - lastL1time > 2000) {
    speedL1 = 0;
    lastL1time = now;
  }
  if (now - lastL2time > 2000) {
    speedL2 = 0;
    lastL2time = now;
  }
  if (now - lastL3time > 2000) {
    speedL3 = 0;
    lastL3time = now;
  }
  if (now - lastR1time > 2000) {
    speedR1 = 0;
    lastR1time = now;
  }
  if (now - lastR2time > 2000) {
    speedR2 = 0;
    lastR2time = now;
  }
  if (now - lastR3time > 2000) {
    speedR3 = 0;
    lastR3time = now;
  }


}
