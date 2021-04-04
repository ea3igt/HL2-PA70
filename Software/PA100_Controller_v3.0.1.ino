//***************************************************************************************
//*      PA70 Controller - Band, and Fan Control by Temperature for Arduino Nano        *
//*                                 by Cesc Gudayol (EA3IGT)                            *
//*                 Version 3.0.1                             01/04/2021                *
//*                                                                                     *
//*                  More info at: https://github.com/ea3igt/HL2-PA70                   *
//*                                                                                     * 
//***************************************************************************************
//*           THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND            *
//***************************************************************************************
//* v2.0   * 24/10/2020 * Initial version                                               *
//* v2.1   * 27/10/2020 * Convert sendOled to a function to refresh the OLED display    *
//* V2.2   * 25/11/2020 * Store last band selected in EPROM                             *
//*        * 12/12/2020 * Get 6 least significative I2C bus bits to select the band     *
//* V2.3   * 14/12/2020 * Use EXTTR to control PTT status                               *
//* V3.0   * 20/02/2021 * First version to test new Control Board v2.1                  *
//* V3.0.1 * 01/04/2021 * Correct OLED refresh problems                                 *
//***************************************************************************************
//
// Pin Configuration:
//  - PIN A00 OUTPUT: SCL for I2C Master (OLED display)
//  - PIN A01 OUTPUT: SDA for I2C Master (OLED display)
//  - PIN A04 INPUT: SDA for I2C Slave  (Hermes Lite 2 Control bus)
//  - PIN A05 INPUT: SCL for I2C Slave  (Hermes Lite 2 Control bus)
//  - PIN A03 INPUT: NTC divider voltage
//  - PIN D02 INPUT: EXTTR (PTT) control from Hermes Lite 2
//  - PIN D03 OUTPUT: LPF Band 1 Control (80m)
//  - PIN D04 OUTPUT: LPF Band 2 Control (40/60m)
//  - PIN D05 OUTPUT: LPF Band 3 Control (30/20m)
//  - PIN D06 OUTPUT: LPF Band 4 Control (17/15/12/10m)
//  - PIN D07 LPF Spare
//  - PIN D08 LPF Spare
//  - PIN D09 LPF Spare
//  - PIN D10 LPF Spare
//  - PIN D11 OUTPUT: PWM for Fan control f(Temp)
//  - PIN D12 OUTPUT: PTT Control to the Power Amp

#include <Wire.h>                     //For I2C control
#include <U8g2lib.h>                  //For SSD1306 Chip Control (OLED)
#include <EEPROM.h>                   //To store values from PA70-Controller
#include <util/delay.h>               //Library to delay inside one Interrupt

//#define TEST                        //Uncomment if Testing (Serial Monitor traces)

// Declaration for an SH1106 display connected to I2C 
//   U8G2_RX: Rotation (R0 = 0º | R1 = 90º | R2 = 180º | R3 = 270º | MIRROR)
//   SW SCL pin A0
//   SW SDA pin A1
//   Use U8X8_PIN_NONE if the reset pin is not connected (All Boards without Reset of the Display)
U8G2_SH1106_128X64_NONAME_2_SW_I2C u8g2(U8G2_R0, A0, A1, U8X8_PIN_NONE);  

// Declaration for the Hermes Lites v2 I2C (Listen to Adress 0x20)
//   HW SDA pin A4
//   HW SCL pin A5

// Thermometer definitions
int ThermistorPin = 3;                            //Pin A3 for NTC voltage divider reading
int Vo;                                           //To store thermistor bridge readings
float R1 = 10000;                                 //Adjust to the R value for the divider (About 10K Ohm)
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

// PWM definitions
const byte PWM_Pin = 11;                          //PWM Pin

// Output Pins (Bands and PTT)
#define LPF_band1 3                               //Pin D3
#define LPF_band2 4                               //Pin D4
#define LPF_band3 5                               //Pin D5
#define LPF_band4 6                               //Pin D6

// PTT Control
#define EXTTR 2                                   //Pin D2
#define PTTpin 12                                 //Pin D12

// Other Configurations
float initialTempFanActivation = 27;              //Minimum temperature for Fan Activation
float tempFanActivation;                          //to implement the histeresis cycle
float previousTemp_fanActivation;                 //to implement the histeresis cycle
bool fanConnected = false;                        //to store if Fan is activated to show on OLED
String myBand="---";                              //to store myBand String
float myTemp;                                     //to store Temperature read from NTC
byte selectedBand;                                //to store Selected Band
byte previousSelectedBand;                        //to store previously Selected Band
byte previousI2CBand;                             //to store previously I2C Band decoded
byte PTTStatus = 0;                               //to store global PTT Status 
bool PTTChanged = false;                          //to store global PTT Status Changed 

// SetUp function initializations
void setup() {
  #ifdef TEST
  Serial.begin(9600);
  #endif

  // U8G2 Display initialization
  u8g2.begin();  
  u8g2.setFont(u8g2_font_helvB12_tr);
  u8g2.firstPage();
  do {
    u8g2.drawStr(5,25,"PA70 Ctrl v3.0.1");        //Software version
    u8g2.drawStr(5,53,"      EA3IGT");            //Change to whatever you want
  } while ( u8g2.nextPage() );

  // I2C Slave Setup
  Wire.begin(0x20);                               // join i2c bus with address to listen
  Wire.onReceive(receiveEvent);                   // register event

  // Fan Setup
  tempFanActivation = initialTempFanActivation;
  previousTemp_fanActivation = tempFanActivation;

  // LPF Setup
  pinMode(LPF_band1, OUTPUT);         
  pinMode(LPF_band2, OUTPUT);         
  pinMode(LPF_band3, OUTPUT);         
  pinMode(LPF_band4, OUTPUT);         
  
  // OLED Setup
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);

  // PTT Setup
  pinMode(PTTpin,OUTPUT);  
  pinMode(EXTTR,INPUT);  
  
  // PWM Setup
  pinMode(PWM_Pin,OUTPUT);
  
  delay(200);

  //Listen to EXTTR status change 
  attachInterrupt(digitalPinToInterrupt(EXTTR),changePTT,CHANGE);
    
  //Get stored Band from EEPROM for initial filter selection
  previousSelectedBand=EEPROM.read(0);              //Read LPF filter Band from EEPROM byte 0
  previousI2CBand=EEPROM.read(1);                   //Read I2C byte Band from EEPROM byte 1

  #ifdef TEST
  Serial.print(F("*** Previous I2C Band: "));
  Serial.println(previousI2CBand);
  Serial.print(F("*** Previous Selected Band: "));
  Serial.println(previousSelectedBand);
  Serial.println(F("Initializing..."));
  Serial.println();
  #endif 

  decodeBand(previousI2CBand);
  setLpfBand(previousSelectedBand);                   //Select LPF filter

  delay(2000);                                        //Delay to show the Logo & Version
  sendOled();                                         //Refresh OLED
}

// Main loop program
void loop() 
{
  int PWMDuty;
  int Times = 100;                                    //Average N times to be more stable
  float Temperature = 0;                              //All thermometer code from: https://bit.ly/3pOUjVo
  for(int i = 0; i < Times; i++) {
    Vo = analogRead(ThermistorPin);
    R2 = R1 * (1024.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
    T = T - 273.15 + 1;                               //+1 is my personal adjustment to be more precise
    Temperature = Temperature + T;
    if (PTTChanged)
    {
        digitalWrite(PTTpin,PTTStatus);               //Toggle PTT
        PTTChanged=false;                             //Reset PTTChanged
    }
  }
  myTemp = int(Temperature/Times*10)/(float)10.0;     //Rounded to get 1 decimal

  PWMDuty = int((myTemp - tempFanActivation)*20);     //Calculate PWMDuty (increase to get more aggresive fan)
  if (PWMDuty < 0) 
  {
    PWMDuty = 0;
  }
  if (PWMDuty > 254) 
  {
    PWMDuty = 254;
  }

  #ifdef TEST
  Serial.print(F("Temperature: "));
  Serial.print(myTemp,1);
  Serial.print(F(" - PWMDuty: "));
  Serial.println(PWMDuty);
  #endif

  analogWrite(PWM_Pin,PWMDuty);                       //Select the appropriate PWM f(temperature)

  if (PWMDuty > 0 and not(fanConnected))              //Implement histeresis cycle
  {
    tempFanActivation = initialTempFanActivation - 1;
  }
  if (PWMDuty == 0 and fanConnected)
  {
    tempFanActivation = initialTempFanActivation;
  }

  if (PWMDuty > 0)
  {
    fanConnected=true;
  }
  else
  {
    fanConnected=false;
  }

  #ifdef TEST
  if (fanConnected)
  {
    Serial.println(F("Fan Connected"));
    Serial.print(F("Minimum Temperature Fan Deactivation: "));
    Serial.println(tempFanActivation,1);
  }
  else
  {
    Serial.println(F("Fan Disconnected")); 
    Serial.print(F("Minimum Temperature Fan Activation: "));
    Serial.println(tempFanActivation,1);
  }
  Serial.print(F("BAND:"));
  Serial.println(myBand); 
  Serial.println();
  #endif

  sendOled();                                         //Refresh OLED
}

//Interrupt function to handle any PTT change at HL2 (EXTTR signal)
void changePTT()
{
  int myEXTTR = digitalRead(EXTTR);
  PTTStatus = not(myEXTTR);
  PTTChanged = true;
  
  #ifdef TEST
  Serial.println(F("*** EXTTR Changed ***"));
  Serial.print(F("Value="));
  Serial.println(myEXTTR);
  Serial.print(F("PTTStatus="));
  Serial.println(PTTStatus);
  #endif
}

void receiveEvent(int howMany) {      //Triggered when receive I2C information from HL2
  byte c[10];
  int i= 0;
  while (0 < Wire.available()) {      // loop through all but the last
    c[i] = Wire.read();               // receive byte as a character
    i++;
  }

  #ifdef TEST
  Serial.print(F("I2C BUS DATA:"));
  Serial.println(c[1]);               // write to usb monitor
  #endif

  byte myC=c[1] & B00111111;          //Get 6 least significative bits
  decodeBand(myC);                    //Decode Band

  // write to LPF "Binary Bus" (Band Selection)
  setLpfBand(selectedBand);
  if ((selectedBand!=previousSelectedBand) and (PTTStatus==1))
  {
    #ifdef TEST
    // write to usb monitor
    Serial.println(F("*** Change Filter: Delay 50 ms ***"));
    #endif
    _delay_ms(50);                    //Delay 50ms to settle LPF filters (if change) before PTT   
  }
  previousSelectedBand = selectedBand;

  // Toggle PTT
  digitalWrite(PTTpin,PTTStatus);
  
  //Store Last Selected Band into EEPROM
  if (PTTStatus)
  {
    EEPROM.write(0,selectedBand);     //Byte 0 for selectedBand
    EEPROM.write(1,myC);              //Byte 1 for I2C decoded
  }                 

  #ifdef TEST
  // write to usb monitor
  Serial.print(F("LPF BAND CONTROL (Binary):"));
  Serial.println(selectedBand); 
  Serial.print(F("PTT CONTROL:"));
  Serial.println(PTTStatus);    
  #endif
}

//Function to activate the appropriate filter band at LPF
//***************** Change accordingly ******************
void setLpfBand(int lpfBand)
{
  // write to LPF "Binary Bus" (Band Selection)
  digitalWrite(LPF_band1, lpfBand & B0001);
  digitalWrite(LPF_band2, lpfBand & B0010);
  digitalWrite(LPF_band3, lpfBand & B0100);
  digitalWrite(LPF_band4, lpfBand & B1000);
}

//Function to decode HL2 band and select/map the filter
//**************** Change accordingly *****************
void decodeBand(byte c)
{ 
  switch (c) {
    case 0x1:     //160m
      selectedBand = B0001;
      myBand = F("160m");
      break;
    case 0x2:     //80m
      selectedBand = B0001;
      myBand = F("80m");
      break;
    case 0x4:     //60m & 40m
      selectedBand = B0010;
      myBand = F("60/40m");
      break;
    case 0x8:     //30m & 20m
      selectedBand = B0100;
      myBand = F("30/20m");
      break;
    case 0x10:     //17m & 15m
      selectedBand = B1000;
      myBand = F("17/15m");
      break;
    case 0x20:     //12m & 10m
      selectedBand = B1000;
      myBand = F("12/10m");
      break;           
    default:
      selectedBand=selectedBand;  //No Change
  }
}

//Function to send all the information to the OLED
void sendOled(void)
{
  // write to OLED
  //u8g2.clearBuffer();
  u8g2.firstPage();
  do {
    // u8g2.drawFrame(0,0,128,64);
    u8g2.setFont(u8g2_font_t0_17_tr);
    u8g2.drawStr(5,17,"BAND:");
    
    u8g2.drawStr(55,17,myBand.c_str());
    u8g2.drawStr(5,37,"TEMP:");
    String myTempStr=String(myTemp,1);
    u8g2.drawStr(55,37,myTempStr.c_str());
    if (fanConnected)
    {
      //u8g2.drawStr(97,37,"#");
      u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
      u8g2.drawStr(96,39,"\xcd");
    }
    else
    {
      u8g2.drawStr(97,36,"_");
    }
    //u8g2.setFont(u8g2_font_t0_17_tr);
    //u8g2.drawStr(5,57,"TRANSMITING");  //Space for future uses
    delay(2);
  } while ( u8g2.nextPage() );
  u8g2.sendBuffer();  
}
