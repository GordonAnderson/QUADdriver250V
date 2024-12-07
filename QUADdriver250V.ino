#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>

#include <SPI.h>

#include <Button.h>
#include <RingBuffer.h>
#include <commandProcessor.h>
#include <charAllocate.h>
#include <debug.h>
#include <Errors.h>
#include <Devices.h>

#include <arduino-timer.h>
#include <Adafruit_SPIFlash.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
//
#include "QUADdriver250V.h"

//
// QAUDdriver250V
//
//  Adafruit Itsbitsy M0 microcontroller.
//
//  Revision history
//  1.0, July 31, 2024
//    - First release
//  1.1, Dec 6, 2024
//    - Updated for rev 2.0 hardware
//    - Added HV enable
//    - Added V offset
//    - Improved V offset and resolution
//    - Implemented readback testing
//    - Fixed bug in calibration readback
//  


auto timer = timer_create_default(); // create a timer with default settings

const char *Version = "QUAD driver, version 1.0 July 31, 2024";

const char *fault = NULL;

ReadBacks rb    = {0,0,0,0,0,0,0,0,0,0};
State     sdata = {true,0,0,0,0};

commandProcessor cp;
debug dbg(&cp);

// for flashTransport definition
#include "flash_config.h"
Adafruit_SPIFlash flash(&flashTransport);
// file system object from SdFat
FatVolume fatfs;
File32 file;

Data data;
int holdOff = 0;

Data Rev_1_data =
{
    sizeof(Data),"QUAD250V",1,
    false,false,false,20,0,0,
    20,-20,20,-20,
    3,70,250,
    // ADC channels on the processor pins
    {monitor3p3V,9968.19,0},      // 3.3V monitor
    {monitor5V,4652.54,0},         // 5V monitor
    {monitor12V,1818.79,0},        // 12V monitor
    {monitorSupply,953.32,0},      // Supply monitor
    {monitorI,2000,7100},        // Current monitor
    // ADC channels on AD5592 to monitor supplies
    {PHAPMON,245.53,-38.17},
    {PHANMON,-242.68,-19.44},
    {PHBPMON,262,0},
    {PHBNMON,-262,0},
    // DAC channels
    {PHAPCTRL,241.24,27.34},
    {PHANCTRL,-265.50,15.93},
    {PHBPCTRL,262,0},
    {PHBNCTRL,-262,0},
    SIGNATURE
};

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is cleared every time
// the sketch is uploaded on the board.
FlashStorage(flash_data, Data);

Command cmds[] =
{
  // Main application commands
  {"GVER",CMDstr,-1,(void *)Version,NULL,                 "Firmware version"},
  {"?NAME",CMDstr,-1,(void *)&data.Name,NULL,             "Device name"},
  {"SAVE",CMDfunction,0,(void *)SaveSettings,NULL,        "Save the system settings"},
  {"RESTORE",CMDfunction,0,(void *)RestoreSettings,NULL,  "Restore system settings"},
  {"FAULT",CMDfunction,0,(void *)reportFault,NULL,        "Report last system fault, enter CLEAR to erase flult"},
  {"BLOAD", CMDfunction,0,(void *)bootloader,NULL,        "Jump to bootloader"},
// SPI flash file IO function
  {"SAVEF", CMDfunction,0,(void *)saveDefaults,NULL,      "Save current setting to flash FS"},
  {"LOADF", CMDfunction,0,(void *)loadDefaults,NULL,      "Load current setting from flash FS"},
  {"SAVECAL", CMDfunction,0,(void *)saveCalibrations,NULL,"Save current calibration data to flash FS"},
  {"LOADCAL", CMDfunction,0,(void *)loadCalibrations,NULL,"Load current calibration from flash FS"},
  // 
  {"?ENA",CMDbool,-1,(void *)&data.Enable,NULL,           "Set/Return enable status, TRUE or FALSE"},
  {"?HVENA",CMDbool,-1,(void *)&data.HVena,NULL,          "Set/Return HV enable status, TRUE or FALSE"},
  {"?ENATST",CMDbool,-1,(void *)&data.EnaRBtesting,NULL,  "Set/Return voltage readbase testing status, TRUE or FALSE"},
  {"SVop",CMDfunction,1,(void *)setVop,NULL,              "Set voltage base to peak, in volts"},
  {"GVop",CMDfloat,-1,(void *)&data.Vop,NULL,             "Report voltage base to peak, in volts"},
  {"SVres",CMDfunction,1,(void *)setVres,NULL,            "Set resolving voltage, in volts"},
  {"GVres",CMDfloat,-1,(void *)&data.Vres,NULL,           "Report resolving voltage, in volts"},
  {"SVoff",CMDfunction,1,(void *)setVoff,NULL,            "Set offset voltage, in volts"},
  {"GVoff",CMDfloat,-1,(void *)&data.Voff,NULL,           "Report offset voltage, in volts"},
  {"SVpa",CMDfunction,1,(void *)setVpa,NULL,              "Set pole 1 positive supply, in volts"},
  {"GVpa",CMDfloat,-1,(void *)&data.Vpa,NULL,             "Report pole 1 positive supply, in volts"},
  {"SVna",CMDfunction,1,(void *)setVna,NULL,              "Set pole 1 negative supply, in volts"},
  {"GVna",CMDfloat,-1,(void *)&data.Vna,NULL,             "Report pole 1 negative supply, in volts"},
  {"SVpb",CMDfunction,1,(void *)setVpb,NULL,              "Set pole 2 positive supply, in volts"},
  {"GVpb",CMDfloat,-1,(void *)&data.Vpb,NULL,             "Report pole 2 positive supply, in volts"},
  {"SVnb",CMDfunction,1,(void *)setVnb,NULL,              "Set pole 2 negative supply, in volts"},
  {"GVnb",CMDfloat,-1,(void *)&data.Vnb,NULL,             "Report pole 2 negative supply, in volts"},
  // Monitor commands
  {"G3P3V",CMDfloat,-1,(void *)&rb.V3p3,NULL,             "Report 3.3 volt supply, in volts"},
  {"G5V",CMDfloat,-1,(void *)&rb.V5,NULL,                 "Report 5 volt supply, in volts"},
  {"G12V",CMDfloat,-1,(void *)&rb.V12,NULL,               "Report 12 volt supply, in volts"},
  {"GSV",CMDfloat,-1,(void *)&rb.Vsupply,NULL,            "Report source voltage, in volts"},
  {"GCUR",CMDfloat,-1,(void *)&rb.current,NULL,           "Report current, in amps"},
  {"GPWR",CMDfloat,-1,(void *)&rb.power,NULL,             "Report power, in watts"},

  {"GAVpa",CMDfloat,-1,(void *)&rb.Vpa,NULL,              "Report pole 1 positive readback, in volts"},
  {"GAVna",CMDfloat,-1,(void *)&rb.Vna,NULL,              "Report pole 1 negative readback, in volts"},
  {"GAVpb",CMDfloat,-1,(void *)&rb.Vpb,NULL,              "Report pole 2 positive readback, in volts"},
  {"GAVnb",CMDfloat,-1,(void *)&rb.Vnb,NULL,              "Report pole 2 negative readback, in volts"},
  // Calibration commands
  {"CALPOLES",CMDfunction,1,(void *)calibratePoles,NULL,    "Calibrate pole supplies, Vpa | Vna | Vpb | Vnb"},
  {"CALSPLYS",CMDfunction,1,(void *)calibrateSupplies,NULL, "Calibrate power supply monitors"},
  {"CALCUR",CMDfunction,1,(void *)calibrateCurrent,NULL,    "Calibrate current monitor"},

  {NULL}
};
static CommandList cmdList = {cmds, NULL};

void Debug(void)
{
}

// Init the AD5592 (Analog and digital IO chip).
void ARBAD5592init(int8_t addr)
{
   pinMode(addr,OUTPUT);
   digitalWrite(addr,HIGH);
   // General purpose configuration
   AD5592write(addr, 3,0x0100);
   // Set ext reference
   AD5592write(addr, 11,0x0200);
   // Set LDAC mode
   AD5592write(addr, 7,0x0000);
   // Set DO outputs channels
   AD5592write(addr, 8,0x0000);
   // Set DAC outputs channels
   AD5592write(addr, 5,0x0095);
   // Set ADC input channels
   AD5592write(addr, 4,0x006A);
   // Turn off all pulldowns
   AD5592write(addr, 6,0x0000);
   
   // Set default values
   // Init DAC channels 0,1,2 to mid range
   AD5592writeDAC(addr, 0, 0);
   AD5592writeDAC(addr, 2, 0);
   AD5592writeDAC(addr, 4, 0);
   AD5592writeDAC(addr, 7, 0);
}

int readADC(int8_t chan)
{
  int val=0;
  for(int i=0;i<16;i++) val += analogRead(chan);
  return val;
}

int readAD5592(int8_t chan)
{
  return AD5592readADC(AD5592_CS,chan);
}

void writeAD5592(int8_t chan, int counts)
{
  AD5592writeDAC(AD5592_CS, chan, counts);
}

bool UpdateDACvalue(DACchan *dchan, float *value, float *svalue)
{
  if((sdata.update) || (*value != *svalue))
  {
    AnalogOut(writeAD5592, dchan, *value);
    *svalue = *value;
    return true;
  }
  return false;
}

void faultTesting(void)
{
  if(!data.Enable) return;
  // Test for setpoints exceeding voltage limit
  if(abs(data.Vpa) > data.maxVolatge) data.Vpa =  data.maxVolatge;
  if(abs(data.Vna) > data.maxVolatge) data.Vna = -data.maxVolatge;
  if(abs(data.Vpb) > data.maxVolatge) data.Vpb =  data.maxVolatge;
  if(abs(data.Vnb) > data.maxVolatge) data.Vnb = -data.maxVolatge;
  // Test for power limit
  if(rb.power > data.maxPower) 
  {
    data.Enable = false; 
    FAULT_ON;
    fault = "Maximum power limit exceeded!";
  }
  if(rb.current > data.maxCurrent) 
  {
    data.Enable = false; 
    FAULT_ON;
    fault = "Maximum current limit exceeded!";
  }
  // Test for voltage readback error
  if(!data.EnaRBtesting) return;
  if(holdOff > 0) holdOff--;
  if(holdOff > 0) return;
  float error = 0;
  if(abs(data.Vpa-rb.Vpa) > error) error = abs(data.Vpa-rb.Vpa);
  if(abs(data.Vna-rb.Vna) > error) error = abs(data.Vna-rb.Vna);
  if(abs(data.Vpb-rb.Vpb) > error) error = abs(data.Vpb-rb.Vpb);
  if(abs(data.Vnb-rb.Vnb) > error) error = abs(data.Vnb-rb.Vnb);
  if(error > 10.0) 
  {
    data.Enable = false; 
    FAULT_ON;
    fault = "Pole voltage regulator readback error!";
  }
}

bool Update(void *)
{
  static bool lastEnable = false;
  // Read the analog inputs on prcessor and filter
  rb.V3p3    = Filter(rb.V3p3,AnalogIn(readADC,&data.mon3p3V));
  rb.V5      = Filter(rb.V5,AnalogIn(readADC,&data.mon5V));
  rb.V12     = Filter(rb.V12,AnalogIn(readADC,&data.mon12V));
  rb.Vsupply = Filter(rb.Vsupply,AnalogIn(readADC,&data.monSupply));
  rb.current = Filter(rb.current,AnalogIn(readADC,&data.monI));
  rb.power   = Filter(rb.power, rb.Vsupply * rb.current, 0.05);
  // Read the analog inputs fron AD9992 and filter
  rb.Vpa = Filter(rb.Vpa,AnalogIn(readAD5592,&data.phaseAposM));
  rb.Vna = Filter(rb.Vna,AnalogIn(readAD5592,&data.phaseAnegM));
  rb.Vpb = Filter(rb.Vpb,AnalogIn(readAD5592,&data.phaseBposM));
  rb.Vnb = Filter(rb.Vnb,AnalogIn(readAD5592,&data.phaseBnegM));
  // Write the output voltages only when values has changed or update flag is set
  if(data.HVena) digitalWrite(HVON,LOW);
  else digitalWrite(HVON,HIGH);
  if(data.Enable)
  {
    if(lastEnable == false) holdOff = PSHOLDTIME;
    digitalWrite(ENABLE,LOW);
    digitalWrite(ON,LOW);
    digitalWrite(FAULT,HIGH);
    faultTesting();
    UpdateDACvalue(&data.phaseAposS, &data.Vpa, &sdata.Vpa);
    UpdateDACvalue(&data.phaseAnegS, &data.Vna, &sdata.Vna);
    UpdateDACvalue(&data.phaseBposS, &data.Vpb, &sdata.Vpb);
    UpdateDACvalue(&data.phaseBnegS, &data.Vnb, &sdata.Vnb);
    sdata.update = false;
  }
  else
  {
    digitalWrite(ENABLE,HIGH);
    digitalWrite(ON,HIGH);
    AnalogOut(writeAD5592, &data.phaseAposS, 0);
    AnalogOut(writeAD5592, &data.phaseAnegS, 0);
    AnalogOut(writeAD5592, &data.phaseBposS, 0);
    AnalogOut(writeAD5592, &data.phaseBnegS, 0);
    sdata.update = true;
  }
  lastEnable = data.Enable;
  return true;
}

// End of AD5592 routines
void setup() 
{
  delay(100);

  pinMode(ENABLE,OUTPUT);
  pinMode(ON,OUTPUT);
  pinMode(FAULT,OUTPUT);
  digitalWrite(ENABLE,LOW);
  digitalWrite(ON,HIGH);
  digitalWrite(FAULT,HIGH);

  pinMode(HVON,OUTPUT);
  digitalWrite(HVON,HIGH);

  analogReadResolution(12);

  // Read the flash config contents and test the signature
  data = flash_data.read();
  if(data.Signature != SIGNATURE) data = Rev_1_data;

  // Test the supply voltage and hold untill its
  // above threshold
  rb.Vsupply,AnalogIn(readADC,&data.monSupply);
  while((rb.Vsupply=rb.Vsupply,AnalogIn(readADC,&data.monSupply)) < MINSUPPLYV)
  {
    digitalWrite(FAULT,LOW);
    delay(250);
    digitalWrite(FAULT,HIGH);
    delay(250);
  }
 
  Serial.begin(0);
  cp.registerStream(&Serial);
  cp.registerCommands(&cmdList);
  cp.registerCommands(dbg.debugCommands());
  dbg.registerDebugFunction(Debug);

  // Init SPI
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(20);
  SPI.setDataMode(SPI_MODE2);
  ARBAD5592init(AD5592_CS);

  timer.every(100,Update);
  timer.tick();
}

void poll(void)
{
  timer.tick();
}

void loop() 
{
  poll();
  cp.processStreams();
  cp.processCommands();

  // Monitor supply voltage, if it falls below threshold wait for the
  // voltage to return and then reset the processor
  if(rb.Vsupply,AnalogIn(readADC,&data.monSupply) < MINSUPPLYV) 
  {
    if(rb.Vsupply,AnalogIn(readADC,&data.monSupply) > MINSUPPLYV) return;
    data.Enable = false;
    Update(NULL);
    data.Vpa = data.Vna = data.Vpb = data.Vnb = 0;
    Update(NULL);
    delay(1000);
    NVIC_SystemReset();
    while(true);
    //debug::softwareReset();
  }
}

//
// Host command functions
//

void SaveSettings(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  data.Signature = SIGNATURE;
  flash_data.write(data);
  cp.sendACK();
}

void RestoreSettings(void)
{
  static Data td;
  
  if(!cp.checkExpectedArgs(0)) return;
  // Read the flash config contents and test the signature
  td = flash_data.read();
  if(td.Signature == SIGNATURE) data = td;
  else
  {
    cp.sendNAK();
    return;
  }
  cp.sendACK(); 
  data.Enable = false;
  data.HVena = false;
}

void reportFault(void)
{
  if(cp.getNumArgs() == 0)
  {
    cp.sendACK(false);
    if(fault == NULL) cp.println("No detected faults!");
    else cp.println(fault);
  }
  if(cp.getNumArgs() == 1)
  {
    char *arg = (char *)"CLEAR";
    if(cp.getValue(&arg, "CLEAR"))
    {
      cp.sendACK(false);
      if(fault == NULL) cp.println("No detected faults!");
      else cp.println(fault);
      fault = NULL;
    }
    else cp.sendACK();
  }
  else cp.sendACK();
}

void setVop(void)
{
  float v;

  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&v,0,data.maxVolatge)) 
  {
    cp.sendNAK();
    return;
  }
  holdOff = PSHOLDTIME;
  // Set the voltages
  data.Vop = v;
  rb.Vpa = data.Vpa = v;
  rb.Vna = data.Vna = -v;
  rb.Vpb = data.Vpb = v;
  rb.Vnb = data.Vnb = -v;
  // Clear Vres and Voff
  data.Vres = data.Voff = 0;
  cp.sendACK();
}

void setVres(void)
{
  float v;

  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&v,0,data.maxVolatge)) { cp.sendNAK(); return; }
  if((((data.Vpa + v) < 0) || ((data.Vpa + v) > data.maxVolatge)) ||
     (((data.Vna - v) < -data.maxVolatge) || ((data.Vna + v) > 0)) ||
     (((data.Vpb + v) < 0) || ((data.Vpb + v) > data.maxVolatge)) || 
     (((data.Vnb - v) < -data.maxVolatge) || ((data.Vnb + v) > 0)))
     { cp.sendNAK(); return; }
  // Set the voltages
  holdOff = PSHOLDTIME;
  data.Vres = v;
  data.Vpa = data.Vop;
  data.Vna = -data.Vop;
  data.Vpb = data.Vop;
  data.Vnb = -data.Vop;
  // Apply resolution voltage
  data.Vpa += data.Vres;
  data.Vna += data.Vres;
  data.Vpb -= data.Vres;
  data.Vnb -= data.Vres;
  // Apply offset voltage
  data.Vpa += data.Voff;
  data.Vna += data.Voff;
  data.Vpb += data.Voff;
  data.Vnb += data.Voff;

  cp.sendACK();
}

void setVoff(void)
{
  float v;

  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&v,-data.maxVolatge,data.maxVolatge)) { cp.sendNAK(); return; }
  if((((data.Vpa + v) < 0) || ((data.Vpa + v) > data.maxVolatge)) ||
     (((data.Vna + v) < -data.maxVolatge) || ((data.Vna + v) > 0)) ||
     (((data.Vpb + v) < 0) || ((data.Vpb + v) > data.maxVolatge)) || 
     (((data.Vnb + v) < -data.maxVolatge) || ((data.Vnb + v) > 0)))
     { cp.sendNAK(); return; }
  // Set the voltages
  holdOff = PSHOLDTIME;
  data.Voff = v;
  data.Vpa = data.Vop;
  data.Vna = -data.Vop;
  data.Vpb = data.Vop;
  data.Vnb = -data.Vop;
  // Apply resolution voltage
  data.Vpa += data.Vres;
  data.Vna += data.Vres;
  data.Vpb -= data.Vres;
  data.Vnb -= data.Vres;
  // Apply offset voltage
  data.Vpa += data.Voff;
  data.Vna += data.Voff;
  data.Vpb += data.Voff;
  data.Vnb += data.Voff;

  cp.sendACK();
}

void setPoleVoltage(float *v, float ll, float ul)
{
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(v,ll,ul)) 
  {
    cp.sendACK();
    return;
  }
  cp.sendNAK();
}

void setVpa(void) {setPoleVoltage(&data.Vpa,0,data.maxVolatge);}
void setVna(void) {setPoleVoltage(&data.Vna,-data.maxVolatge,0);}
void setVpb(void) {setPoleVoltage(&data.Vpb,0,data.maxVolatge);}
void setVnb(void) {setPoleVoltage(&data.Vnb,-data.maxVolatge,0);}

// Calibration routines

// Calibrate pole regulators
void calibratePole(const char *title, DACchan *dacchan, ADCchan *adcchan, float p1, float p2)
{
  cp.println(title);
  // Set the point 1 voltage, write to DAC and ask for actual voltage
  int dac1 = Value2Counts(p1,dacchan);
  writeAD5592(dacchan->Chan, dac1);
  float P1 = cp.userInputFloat("Enter measured voltage: ", NULL);
  int adc1 = 0;
  for(int i =0;i<100;i++) adc1+= readAD5592(adcchan->Chan);
  adc1 /= 100;
  // Set the point 2 voltage, write to DAC and ask for actual voltage
  int dac2 = Value2Counts(p2,dacchan);
  writeAD5592(dacchan->Chan, dac2);
  float P2 = cp.userInputFloat("Enter measured voltage: ", NULL);
  int adc2 = 0;
  for(int i =0;i<100;i++) adc2 += readAD5592(adcchan->Chan);
  adc2 /= 100;
  // Calculate the calibration parameters
  // counts = m * value + b
  // dac1 = m * P1 + b
  // dac2 = m * P2 + b
  // dac1-dac2 = m * P1 - m * P2
  // m = (dac1-dac2)/(P1 - P2)
  // b = dac1 - m * P1
  dacchan->m = (float)(dac1-dac2)/(P1 - P2);
  dacchan->b = (float)dac1 - dacchan->m * P1;
  cp.println("DAC settings");
  cp.print("m = "); cp.println(dacchan->m);
  cp.print("b = "); cp.println(dacchan->b);
  adcchan->m = (float)(adc1-adc2)/(P1 - P2);
  adcchan->b = (float)adc1 - adcchan->m * P1;
  cp.println("ADC settings");
  cp.print("m = "); cp.println(adcchan->m);
  cp.print("b = "); cp.println(adcchan->b);
}

void calibratePoles(void)
{
  char   *cmd;
  String Cmd;

  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&cmd, "Vpa,Vna,Vpb,Vnb")) {cp.sendNAK(); return;};
  Cmd = cmd;
  Cmd.toUpperCase();
  if(Cmd == "VPA") calibratePole("Calibrate Pole 1 positive supply", &data.phaseAposS, &data.phaseAposM,  50,  200);
  if(Cmd == "VNA") calibratePole("Calibrate Pole 1 negative supply", &data.phaseAnegS, &data.phaseAnegM, -50, -200);
  if(Cmd == "VPB") calibratePole("Calibrate Pole 2 positive supply", &data.phaseBposS, &data.phaseBposM,  50,  200);
  if(Cmd == "VNB") calibratePole("Calibrate Pole 2 negative supply", &data.phaseBnegS, &data.phaseBnegM, -50, -200);
  sdata.update = true;
  cp.sendACK();
}

void calibrateSupply(const char *title, ADCchan *adcchan)
{
  cp.println(title);
  float P1 = cp.userInputFloat("Enter measured voltage: ", NULL);
  int adc1 = 0;
  for(int i=0;i<100;i++) adc1 += readADC(adcchan->Chan);
  adc1 /= 100;
  adcchan->m = (float)(adc1)/P1;
  adcchan->b = 0;
  cp.println("ADC settings");
  cp.print("m = "); cp.println(adcchan->m);
  cp.print("b = "); cp.println(adcchan->b);
}

void calibrateSupplies(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  calibrateSupply("Calibrate 3.3V supply monitor: " ,&data.mon3p3V);
  calibrateSupply("Calibrate 5V supply monitor: "   ,&data.mon5V);
  calibrateSupply("Calibrate 12V supply monitor: "  ,&data.mon12V);
  calibrateSupply("Calibrate power supply monitor: ",&data.monSupply);
  cp.sendACK();
}

// The function assumes the system is enabled and a signal is applied to
// the pulse inputs.
void calibrateCurrent(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  cp.println("Calibrating current monitor.");
  // Set output voltage for the first point
  float v = cp.userInputFloat("Enter Vop: ", NULL);
  data.Vpa = data.Vpb =  v;
  data.Vna = data.Vnb = -v;
  float P1 = cp.userInputFloat("Enter measured current in amps: ", poll);
  int adc1 = 0;
  for(int i =0;i<100;i++) adc1 += readADC(data.monI.Chan);
  adc1 /= 100;
  v = cp.userInputFloat("Enter Vop: ", NULL);
  data.Vpa = data.Vpb =  v;
  data.Vna = data.Vnb = -v;
  float P2 = cp.userInputFloat("Enter measured current in amps: ", poll);
  int adc2 = 0;
  for(int i =0;i<100;i++) adc2 += readADC(data.monI.Chan);
  adc2 /= 100;
 // Calculate the calibration parameters
  data.monI.m = (float)(adc1-adc2)/(P1 - P2);
  data.monI.b = (float)adc1 - data.monI.m * P1;
  cp.println("ADC settings");
  cp.print("m = "); cp.println(data.monI.m);
  cp.print("b = "); cp.println(data.monI.b);
  // Set voltage to 0
  data.Vpa = data.Vpb = 0;
  data.Vna = data.Vnb = 0;
}

// Itsybitsy M0 jump to bootloader function
void bootloader(void)
{
  __disable_irq();
  #define BOOT_DOUBLE_TAP_ADDRESS ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
  #define DBL_TAP_MAGIC 0xf01669ef // Randomly selected, adjusted to have first and last bit set
  #define DBL_TAP_MAGIC_QUICK_BOOT 0xf02669ef
	unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
	*a = DBL_TAP_MAGIC;
	// Reset the device
	NVIC_SystemReset() ;
	while (true);
}

// Functions supporting Circuit Python file system (CPFS). This is used to save
// the configuration and calibration data to the SPI flash filesystem.
// Provides non volitial storage of setup data.
bool FSsetup(void)
{
  // Init external flash
  if (!flash.begin()) cp.println("Error, failed to initialize flash chip!");
  else
  {
    cp.println("Flash chip initalized!");
    if(!fatfs.begin(&flash)) cp.println("Failed to mount filesystem!");
    else
    {
      cp.println("Mounted filesystem!");
      return true;
    }
  }
  return false;
}
void saveDefaults(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("default.dat",O_WRITE | O_CREAT))==0) cp.println("Can't create default.dat!");
  else
  {
    size_t num = file.write((void *)&data,sizeof(Data));
    file.close();
    cp.print("default.dat written, number of bytes = ");
    cp.println((int)num);
  }
}
void loadDefaults(void)
{
  Data h;

  if(!FSsetup()) return;
  if((file = fatfs.open("default.dat",O_READ))==0) cp.println("Can't open default.dat!");
  else
  {
    size_t num = file.read((void *)&h,sizeof(Data));
    file.close();
    if((num != sizeof(Data)) || (h.Signature != SIGNATURE))
    {
      cp.println("Error reading default.dat file!");
      return;
    }
    data = h;
    cp.print("default.dat read, number of bytes = ");
    cp.println((int)num);
  }
}
void saveCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_WRITE | O_CREAT))==0) cp.println("Can't create cal.dat!");
  else
  {
    size_t num = file.write((void *)&data.mon3p3V,sizeof(ADCchan));
    num += file.write((void *)&data.mon5V,sizeof(ADCchan));
    num += file.write((void *)&data.mon12V,sizeof(ADCchan));
    num += file.write((void *)&data.monSupply,sizeof(ADCchan));
    num += file.write((void *)&data.monI,sizeof(ADCchan));
    num += file.write((void *)&data.phaseAposM,sizeof(ADCchan));
    num += file.write((void *)&data.phaseAnegM,sizeof(ADCchan));
    num += file.write((void *)&data.phaseBposM,sizeof(ADCchan));
    num += file.write((void *)&data.phaseBnegM,sizeof(ADCchan));
    num += file.write((void *)&data.phaseAposS,sizeof(DACchan));
    num += file.write((void *)&data.phaseAnegS,sizeof(DACchan));
    num += file.write((void *)&data.phaseBposS,sizeof(DACchan));
    num += file.write((void *)&data.phaseBnegS,sizeof(DACchan));
    file.close();
    cp.print("cal.dat written, number of bytes = ");
    cp.println((int)num);
  }
}

void loadCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_READ))==0) cp.println("Can't open cal.dat!");
  else
  {
    size_t num = file.read((void *)&data.mon3p3V,sizeof(ADCchan));
    num += file.read((void *)&data.mon5V,sizeof(ADCchan));
    num += file.read((void *)&data.mon12V,sizeof(ADCchan));
    num += file.read((void *)&data.monSupply,sizeof(ADCchan));
    num += file.read((void *)&data.monI,sizeof(ADCchan));
    num += file.read((void *)&data.phaseAposM,sizeof(ADCchan));
    num += file.read((void *)&data.phaseAnegM,sizeof(ADCchan));
    num += file.read((void *)&data.phaseBposM,sizeof(ADCchan));
    num += file.read((void *)&data.phaseBnegM,sizeof(ADCchan));
    num += file.read((void *)&data.phaseAposS,sizeof(DACchan));
    num += file.read((void *)&data.phaseAnegS,sizeof(DACchan));
    num += file.read((void *)&data.phaseBposS,sizeof(DACchan));
    num += file.read((void *)&data.phaseBnegS,sizeof(DACchan));
    file.close();
    cp.print("cal.dat read, number of bytes = ");
    cp.println((int)num);
  }
}
// End of CPFS

