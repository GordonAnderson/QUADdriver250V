#ifndef QUADdriver250V_h
#define QUADdriver250V_h

#define SIGNATURE  0xAA55A5A5

#define MINSUPPLYV 20
#define PSHOLDTIME 100

// Analog inputs
#define monitor3p3V     A0    // 1K:1K divider
#define monitor5V       A1    // 10K:1K divider
#define monitor12V      A2    // 10K:1K divider
#define monitorSupply   A5    // 20K:1K divider
#define monitorI        A4

// DIO
#define AD5592_CS       7
#define FAULT           13
#define ON              12
#define ENABLE          11
#define HVON            26

#define FAULT_ON        digitalWrite(FAULT,LOW)

// AD5592 channels, used to control and monitor the 4 regurlators
// 4 output controls and 4 monitors
#define PHAPCTRL       0      // Phase A positive control, DAC
#define PHAPMON        1      // Phase A positive monitor, ADC
#define PHANCTRL       2      // Phase A negative control, DAC
#define PHANMON        3      // Phase A negative monitor, ADC
#define PHBPCTRL       4      // Phase B positive control, DAC
#define PHBPMON        5      // Phase B positive monitor, ADC
#define PHBNMON        6      // Phase B negative monitor, ADC
#define PHBNCTRL       7      // Phase B negative control, DAC

typedef struct
{
  float     V3p3;
  float     V5;
  float     V12;
  float     Vsupply;
  float     current;
  float     power;
  float     Vpa,Vna,Vpb,Vnb;
} ReadBacks;

typedef struct
{
  bool      update;
  float     Vpa,Vna,Vpb,Vnb;
} State;

typedef struct 
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board nam
  int8_t        Rev;                    // Holds the board revision number
  // General parameters
  bool          Enable;
  bool          HVena;
  bool          EnaRBtesting;
  float         Vop;
  float         Vres;
  float         Voff;
  float         Vpa,Vna,Vpb,Vnb;
  // Limits
  float         maxCurrent;
  float         maxPower;
  float         maxVolatge;
  // ADC channels on the processor pins
  ADCchan       mon3p3V;
  ADCchan       mon5V;
  ADCchan       mon12V;
  ADCchan       monSupply;
  ADCchan       monI;
  // ADC channels on AD5592 to monitor supplies
  ADCchan       phaseAposM;
  ADCchan       phaseAnegM;
  ADCchan       phaseBposM;
  ADCchan       phaseBnegM;
  // DAC channels
  DACchan       phaseAposS;
  DACchan       phaseAnegS;
  DACchan       phaseBposS;
  DACchan       phaseBnegS;

  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} Data;

void setVop(void);
void setVres(void);
void setVoff(void);
void setVpa(void);
void setVna(void);
void setVpb(void);
void setVnb(void);

void reportFault(void);

void SaveSettings(void);
void RestoreSettings(void);

void calibratePoles(void);
void calibrateSupplies(void);
void calibrateCurrent(void);
void bootloader(void);

void saveDefaults(void);
void loadDefaults(void);
void saveCalibrations(void);
void loadCalibrations(void);

#endif

