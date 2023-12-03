//***************************************************************************************
//*      PA70 Controller - Band, and Fan Control by Temperature for Arduino Nano        *
//*        To be used together with Hermes Lite v2 (http://www.hermeslite.com/)         *
//*                                 by Cesc Gudayol (EA3IGT)                            *
//*                 Version 4.0.0                             07/05/2021                *
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
//* V4.0   * 07/05/2021 * First Beta version for Nextion Display                        *
//* V4.5   * 23/05/2021 * Adapt Control Board to new LPF-100 Filter                     *
//***************************************************************************************
//
// Pin Configuration:
//  - PIN A00 OUTPUT: SCL for I2C Master (OLED display)
//  - PIN A01 OUTPUT: SDA for I2C Master (OLED display)
//  - PIN A04 INPUT: SDA for I2C Slave  (Hermes Lite 2 Control bus)
//  - PIN A05 INPUT: SCL for I2C Slave  (Hermes Lite 2 Control bus)
//  - PIN A03 INPUT: NTC divider voltage
//  - PIN D02 INPUT: EXTTR (PTT) control from Hermes Lite 2
//  - PIN D03 OUTPUT: LPF Band 1 Control (160m)
//  - PIN D04 OUTPUT: LPF Band 2 Control (80m)
//  - PIN D05 OUTPUT: LPF Band 3 Control (40/60m)
//  - PIN D06 OUTPUT: LPF Band 4 Control (30/20m)
//  - PIN D07 OUTPUT: LPF Band 5 Control (17/15)
//  - PIN D08 OUTPUT: LPF Band 6 Control (12/10m)
//  - PIN D09 LPF Spare
//  - PIN D10 LPF Spare
//  - PIN D11 OUTPUT: PWM for Fan control f(Temp)
//  - PIN D12 OUTPUT: PTT Control to the Power Amp

//#define TEST                        //Uncomment if Testing (Serial Monitor traces)

/*
#include <doxygen.h>
#include <NexCheckbox.h>
#include <NexConfig.h>
#include <NexCrop.h>
#include <NexDualStateButton.h>
#include <NexGauge.h>
#include <NexGpio.h>
#include <NexHardware.h>
#include <NexHotspot.h>
#include <NexObject.h>
#include <NexPicture.h>
#include <NexProgressBar.h>
#include <NexRadio.h>
#include <NexRtc.h>
#include <NexScrolltext.h>
#include <NexSlider.h>
#include <NexText.h>
#include <NexTimer.h>
#include <Nextion.h>
#include <NexTouch.h>
#include <NexUpload.h>
#include <NexVariable.h>
#include <NexWaveform.h>
*/

#include <Wire.h>                                 //For I2C control
#include <EEPROM.h>                               //To store values from PA70-Controller
#include <SoftwareSerial.h>                       //To use simulated serial port (not HW)
#include <NexButton.h>                            // 
#include <NexNumber.h>                            // Required Nextion Elements
#include <NexPage.h>                              //

#define RxD A0                                    //SW Serial RX pin
#define TxD A1                                    //SW Serial TX pin
#define ThermistorPin A3                          //Pin A3 for NTC voltage divider reading
#define PWM_Pin 11                                //PWM Pin

// LPF Output Pins (D3..D10 pins can be used)
#define LPF_band1 3                               //Pin D3 (160 m)
#define LPF_band2 4                               //Pin D4 (80 m)
#define LPF_band3 5                               //Pin D5 (40 m)
#define LPF_band4 6                               //Pin D6 (30-20 m)
#define LPF_band5 7                               //Pin D7 (17-15 m)
#define LPF_band6 8                               //Pin D8 (12-10 m)

// PTT Control
#define EXTTR 2                                   //Pin D2
#define PTTpin 12                                 //Pin D12

// Declaration for the Hermes Lites v2 I2C (Listen to Adress 0x20)
//   HW SDA pin A4
//   HW SCL pin A5

String myBand = "---";                            //to store myBand String
byte selectedBand;                                 //to store Selected Band
byte previousSelectedBand;                         //to store previously Selected Band
byte previousI2CBand;                             //to store previously I2C Band decoded
bool biasStatus = false;                          //to store bias switch state (initially OFF)
byte PTTStatus = 0;                               //to store global PTT Status 
bool PTTChanged = false;                          //to store global PTT Status Changed 
bool BandChanged = false;                         //to store global Band Changed 
byte ampProtected = 0;                            //to store global Protection indicator 
                                                  // 0 - No protection
                                                  // 1 - Max Temperature protection
                                                  // 2 - Max SWR protection
                                                  // 3 - Max PWR protection

SoftwareSerial HMISerial(RxD,TxD);

NexPage page0 = NexPage(0, 0, "page0");  
NexPage page1 = NexPage(1, 0, "page1");  

//Page 0 Buttons & Actiond
NexButton bias = NexButton(0,14,"b1");
NexButton t13 = NexButton(0,19,"t13");

//Page 1 Buttons & Actiond
NexButton b1 = NexButton(1,14,"b1");
NexButton b2 = NexButton(1,15,"b2");
NexButton bn0 = NexButton(1,7,"n0");
NexNumber va0 = NexNumber(1,30,"va0");

byte selectedValue = 0;
byte myValue[12];
byte currentPage = 0;

NexTouch *nex_listen_list[] = 
{
    &page0,
    &page1,
    &b1,
    &b2,
    &bn0,
    &bias,
    &t13,
    NULL
};

//***************************************************************************************
//*                                         SETUP                                       *
//***************************************************************************************

void setup(void)
{    
  byte i;
  nexInit();
  HMISerial.begin(115200);
  Serial.begin(9600);
  
  page0.attachPop(page0PushCallback);                   // Page press event
  page1.attachPop(page1PushCallback);                   // Page press event
  bias.attachPop(biasPopCallback, &bias);
  t13.attachPop(t13PopCallback, &t13);
  b1.attachPop(b1PopCallback, &b1);
  b2.attachPop(b2PopCallback, &b2);
  bn0.attachPop(bn0PopCallback, &bn0);

  delay(100);

  HMISerial.print(F("cle 1,255\xFF\xFF\xFF"));

  int byte1=EEPROM.read(0);                             //Read LPF70 Signature from EEPROM byte 0
  int byte2=EEPROM.read(1);                             //Read LPF70 Signature from EEPROM byte 1
  int signature = byte1 | byte2 << 8;                   //Convert "signature" to Integer

  if (signature == 1963) {
    #ifdef TEST      
    Serial.println("EPROM Signature found!");
    #endif
 
    for(i = 0; i < 12; i++) {
      myValue[i] = EEPROM.read(5+i);                    //Retrieve previously stored myValues[] from EEPROM
    }
  } else {                                              //First time store all values to EPROM
    //First the "signature"
    #ifdef TEST      
    Serial.println("EPROM Signature NOT found. First time use!");
    #endif
    
    signature = 1963;
    byte1 = (byte) (signature & 0xFF);
    byte2 = (byte) ((signature >> 8) & 0xFF);
    EEPROM.write(0,byte1);
    EEPROM.write(1,byte2);   
    
    //And then the initial values
    myValue[0]=20;    //Min Temperature
    myValue[1]=50;    //Max Temperature
    myValue[2]=35;    //Yellow Temperature
    myValue[3]=45;    //Red Temperature
    myValue[4]=28;    //Fan Activation Temperature
    myValue[5]=50;    //Protect Temperature
    myValue[6]=2;     //Yellow SWR
    myValue[7]=3;     //Red SWR
    myValue[8]=4;     //Protect SWR
    myValue[9]=60;    //Yellow PWR
    myValue[10]=80;   //Red PWR
    myValue[11]=100;  //Protect PWR

    for(i = 0; i < 12; i++) {
      EEPROM.write(5+i,(byte)(myValue[i] & 0xFF));                    //Store myValues[] into EEPROM
    }
  }

  //Get stored Band from EEPROM for initial filter selection
  previousSelectedBand=EEPROM.read(2);                                //Read LPF filter Band from EEPROM byte 2
  previousI2CBand=EEPROM.read(3);                                     //Read I2C byte Band from EEPROM byte 3
  decodeBand(previousI2CBand);
  setLpfBand(previousSelectedBand);                                   //Select LPF filter

  #ifdef TEST     
  Serial.print(F("Previous I2C DATA:"));
  Serial.println(previousI2CBand); 
  Serial.print(F("Previous LPF BAND CONTROL (Binary):"));
  Serial.println(previousSelectedBand); 
  Serial.print(F("Previous myBand:"));
  Serial.println(myBand);   
  #endif

  HMISerial.print(F("page 0\xFF\xFF\xFF"));                           //Switch Nextion to Page 0
  mySendNextion("t8",49151,0,myBand);                                 //Update Band display                       

  // I2C Slave Setup
  Wire.begin(0x20);                                                   // join i2c bus with address to listen
  Wire.onReceive(receiveEvent);                                       // register event

  // LPF Setup
  pinMode(LPF_band1, OUTPUT);         
  pinMode(LPF_band2, OUTPUT);         
  pinMode(LPF_band3, OUTPUT);         
  pinMode(LPF_band4, OUTPUT);
  pinMode(LPF_band5, OUTPUT);         
  pinMode(LPF_band6, OUTPUT); 
  
  // PTT Setup
  pinMode(PTTpin,OUTPUT);  
  pinMode(EXTTR,INPUT);  
  //pinMode(EXTTR,INPUT_PULLUP);  
  
  // PWM Setup
  pinMode(PWM_Pin,OUTPUT);
  
  //Listen to EXTTR status change (PTT from HL2)
  attachInterrupt(digitalPinToInterrupt(EXTTR),interruptPTT,CHANGE);
}

//***************************************************************************************
//*                                NEXTION CALLBACKS                                    *
//***************************************************************************************

void page0PushCallback(void *ptr)                                     //page 0 is loaded on the Nextion display
{
  currentPage = 0;                                                    //From now on arduino knows page 1 is loaded on the display
  HMISerial.print(F("page 0\xFF\xFF\xFF"));
  #ifdef TEST      
  Serial.println("Page 0");
  #endif
  mySendNextion("t8",49151,0,myBand);                                 //Update Band display
  if (biasStatus) {
    mySendNextion("b1",63488,0,"ON");                                 //BIAS = ON, Update Bias button
  } else {
    mySendNextion("b1",50712,0,"OFF");                                //BIAS = OFF, Update Bias button
  }
}  

void page1PushCallback(void *ptr)                                     //page 1 is loaded on the Nextion display
{
  #ifdef TEST      
  Serial.println("Page 1");
  #endif      
  currentPage = 1;                                                    //From now on arduino knows page 1 is loaded on the display
  HMISerial.print(F("page 1\xFF\xFF\xFF"));    
  
  //Update Parameters
  String myString;
  for(int i = 0; i < 12; i++) {
    myString = "n"+String(i)+".val="+String(myValue[i])+"\xFF\xFF\xFF";
    HMISerial.print(myString);           
    #ifdef TEST      
    Serial.println(myString);
    #endif
  }
}  

void biasPopCallback(void *ptr)                                       //Pushed page 0 - Bias button 
{
  #ifdef TEST      
  Serial.println("--->biasPopCallback");
  #endif
  
  biasStatus = !biasStatus;
  if (biasStatus) {
    mySendNextion("b1",63488,0,"ON");                                 //BIAS = ON, Update Bias button
  } else {
    mySendNextion("b1",50712,0,"OFF");                                //BIAS = OFF, Update Bias button
    PTTStatus = false;
    PTTChanged = true;  
    changePTT();
  }
}

void t13PopCallback(void *ptr)                                        //Pushed page 0 - RESET button after Protect situation
{
  #ifdef TEST      
  Serial.println("--->t13PopCallback");
  #endif
  
  if (ampProtected>0) {
    mySendNextion("t10",21130,14791,"TEMP");                          //Reset TEMP button
    mySendNextion("t11",21130,14791,"SWR");                           //Reset SWR button
    mySendNextion("t12",21130,14791,"PWR");                           //Reset PWR button
    mySendNextion("t13",50712,65535,"Stand By");                      //Reset Status button
    ampProtected=0;                                                   //Restore protection status after RESET
  }
}

void b1PopCallback(void *ptr)                                         //Pushed page 1 - b1 (+) button 
{
  #ifdef TEST      
  Serial.println("--->b1PopCallback");
  #endif

  myValue[selectedValue] -= 1;
  setValue();  
}

void b2PopCallback(void *ptr)                                         //Pushed page 1 b2 (-) button 
{
  #ifdef TEST      
  Serial.println("--->b2PopCallback");
  #endif
  
  myValue[selectedValue] += 1;
  setValue();
}

void bn0PopCallback(void *ptr)                                        //Pushed page 1 (simulated) n0 button 
{
  #ifdef TEST      
  Serial.println("--->bn0PopCallback");
  #endif  
  
  uint32_t myNumber;
  va0.getValue(&myNumber);                                            //Read variable va0 from page 1 to know  where to change value
  selectedValue=int(myNumber);
  setColor();
}

//***************************************************************************************
//*                                      INTERRUPTS                                     *
//***************************************************************************************

//Interrupt function to handle any PTT change from HL2 (EXTTR signal)
void interruptPTT()
{
  if (biasStatus & (ampProtected==0)) {
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
}

//Interrupt function triggered when we receive I2C information from HL2
void receiveEvent(int howMany) {      
  byte c[10];
  int i= 0;
  while (0 < Wire.available()) {      // loop through all but the last
    c[i] = Wire.read();               // receive byte as a character
    i++;
  }

  #ifdef TEST
  Serial.print(F("I2C BUS DATA:"));
  Serial.println(c[1]);               
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
  changePTT();            

  //Store Last Selected Band into EEPROM if changed
  if (myC!=0)
  {
    EEPROM.write(2,selectedBand);                                         
    EEPROM.write(3,myC);     
    BandChanged = true;
  }

  #ifdef TEST
  // write to usb monitor
  Serial.print(F("LPF BAND CONTROL (Binary):"));
  Serial.println(selectedBand); 
  Serial.print(F("PTT CONTROL:"));
  Serial.println(PTTStatus);    
  #endif
}

//***************************************************************************************
//*                                GENERAL FUNCTIONS                                    *
//***************************************************************************************

void mySendNextion(String button, uint16_t bco, uint16_t pco, String content)
{
  HMISerial.print(button + ".bco=" + String(bco)+ "\xFF\xFF\xFF");    //Set bco
  HMISerial.print(button + ".pco=" + String(pco)+ "\xFF\xFF\xFF");    //Set pco
  HMISerial.print(button + ".txt=\"" + content + "\"\xFF\xFF\xFF");   //Set text
}

void setValue(void)
{
  String myString = "n"+String(selectedValue)+".val="+String(myValue[selectedValue])+"\xFF\xFF\xFF";
  EEPROM.write(selectedValue+5,byte(myValue[selectedValue]));               //Store new value into EPROM
  HMISerial.print(myString);  
}

void setColor(void)
{
  String myString;
  for(int i = 0; i < 12; i++) {
    if (i == selectedValue) {
      myString = "n"+String(i)+".bco=65535"+"\xFF\xFF\xFF";
    } else {
      myString = "n"+String(i)+".bco=48631"+"\xFF\xFF\xFF";
    }
    HMISerial.print(myString);
  }
}

void changePTT()
{
  digitalWrite(PTTpin,PTTStatus);                                       //Toggle PTT
  if (ampProtected==0) {
    if (!PTTStatus) {
      mySendNextion("t13",50712,65535,"Stand By");                      //Power = OFF, Update Status button
    } else {
      mySendNextion("t13",63488,65535,"Power");                         //Power = ON, Update Status button
    }
  }
}

//Function to decode HL2 band and select/map the filter
//**************** Change accordingly *****************
void decodeBand(byte c)
{ 
  switch (c) {
    case 0x1:     //160m
      selectedBand = 1;
      myBand = F("160 m");
      break;
    case 0x2:     //80m
      selectedBand = 2;
      myBand = F("80 m");
      break;
    case 0x4:     //60m & 40m
      selectedBand = 3;
      myBand = F("60 / 40 m");
      break;
    case 0x8:     //30m & 20m
      selectedBand = 4;
      myBand = F("30 / 20 m");
      break;
    case 0x10:     //17m & 15m
      selectedBand = 5;
      myBand = F("17 / 15 m");
      break;
    case 0x20:     //12m & 10m
      selectedBand = 6;
      myBand = F("12 / 10 m");
      break;           
    default:
      selectedBand=selectedBand;  //No Change
  }
}

//Function to activate the appropriate filter band at LPF
//***************** Change accordingly ******************
void setLpfBand(int lpfBand)
{
  int myBinaryBand = int(pow(2,(lpfBand-1))+0.01);              //decode Band into pin (avoid double conversion error)
  // write to LPF "Binary Bus" (Band Selection)
  digitalWrite(LPF_band1, myBinaryBand & B00000001);
  digitalWrite(LPF_band2, myBinaryBand & B00000010);
  digitalWrite(LPF_band3, myBinaryBand & B00000100);
  digitalWrite(LPF_band4, myBinaryBand & B00001000);
  digitalWrite(LPF_band5, myBinaryBand & B00010000);
  digitalWrite(LPF_band6, myBinaryBand & B00100000);
  #ifdef TEST
  // write to usb monitor
  Serial.println(F("*** setLpfBand ***"));
  Serial.print(F("lpfBand="));
  Serial.println(lpfBand); 
  Serial.print(F("myBinaryBand="));
  Serial.println(myBinaryBand);    
  #endif
}

void readTemperature() { 
  // Thermometer definitions
  int Vo;                                                       //To store thermistor bridge readings
  int R1 = 10000;                                               //Adjust to the R value for the divider (About 10K Ohm)
  float logR2, R2, T;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  int Times = 250;                                              //Average N times to be more stable
  int PWMDuty;                                                  //Fan control PWM
  
  float Temperature = 0;                                        //All thermometer code from: https://bit.ly/3pOUjVo
  for(int i = 0; i < Times; i++) {
    Vo = analogRead(ThermistorPin);
    R2 = R1 * (1024.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
    T = T - 273.15 + 1;                                         //+1 is my personal adjustment to be more precise
    Temperature = Temperature + T;
  }
  float myTemp = int(Temperature/Times*100)/(float)100.0;       //Rounded to get 2 decimal
  String myTempStr=String(myTemp,1);

  PWMDuty = constrain(int((myTemp - myValue[4])*20),0,255);     //Calculate PWMDuty (increase to get more aggresive fan)

  if (ampProtected!=1) {
    if (PWMDuty>=1){
      //Update Fan Duty Cycle (if any)
      String myString = "FAN\\r";
      if (PWMDuty<26) {
        myString = myString+"0";
       }
      myString = myString + String(map(PWMDuty,0,254,0,100)) + "%";     
      mySendNextion("t10",1055,65535,myString);                       //FAN On, FAN indicator      
    } else {
      mySendNextion("t10",21130,14791," TEMP");                       //FAN Off, TEMP indicator
    }  
  }

  analogWrite(PWM_Pin,PWMDuty);                                       //Select the appropriate PWM f(temperature)

  //Protect amplifier if Temperature > Protection Temp
  if (myTemp>=myValue[5]) {
    PTTStatus=false;                                                  //Set PTT OFF
    changePTT();
    HMISerial.print(F("t10.bco=63488\xFF\xFF\xFF"));                  //Update lower buttons
    HMISerial.print(F("t10.txt=\" TEMP\\rProtect\"\xFF\xFF\xFF"));
    HMISerial.print(F("t13.bco=65504\xFF\xFF\xFF")); 
    HMISerial.print(F("t13.pco=8484\xFF\xFF\xFF")); 
    HMISerial.print(F("t13.txt=\"RESET\"\xFF\xFF\xFF"));
    ampProtected = 1;                                                 //Max Temperature protection
  }                                  

  if (currentPage==0){
    byte green_value = 255;
    byte blue_value = 0;
    byte yellowTemp = myValue[2];
    byte redTemp = myValue[3];
    byte tempMinValue = myValue[0];
    byte tempMaxValue = myValue[1];
    float middleTemp = yellowTemp + (redTemp-yellowTemp)/2;
    int myValue = constrain(map(myTemp,tempMinValue,tempMaxValue,0,100),0,100);
    
    uint16_t red_565;
    if (myTemp>=yellowTemp) red_565=31; else red_565=0;
    uint16_t green_565 = 63-constrain(map(myTemp, middleTemp, redTemp, 0, 63),0,63);     
    uint16_t blue_565 = map(blue_value, 0, 255, 0, 31);
    uint16_t rgb_565 = (red_565<<11)|(green_565<<5)|blue_565;
    
    String mySend = "j0.pco=" + String(rgb_565) + "\xFF\xFF\xFF";     //Update Temperature Progress-Bar color
    HMISerial.print(mySend);   
    
    HMISerial.print(F("j0.val="));                                    //Update Temperature Value
    HMISerial.print(myValue);                                     
    HMISerial.print(F("\xFF\xFF\xFFt2.txt=\x22"));                
    HMISerial.print(myTempStr);                                   
    HMISerial.print(F("\x20\xBA\x43\x22\xFF\xFF\xFF")); 
  }
}

void loop(void)
{   
    nexLoop(nex_listen_list);
    if (currentPage==0) readTemperature();
    if (PTTChanged)
    {
        changePTT();                                  //Toggle PTT
        PTTChanged=false;                             //Reset PTTChanged
    }
    if (BandChanged)
    {
        mySendNextion("t8",49151,0,myBand);           //Update Band display                       
        BandChanged=false;                            //Reset BandChanged
    }
}
