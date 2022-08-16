#include <FlexCAN_T4.h>
#include <Metro.h>

#include <ADC.h>
#include <ADC_util.h>
#include <EEPROM.h>

//CAN Setup
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#define NUM_TX_MAILBOXES 6
#define NUM_RX_MAILBOXES 6
CAN_message_t msg;

signed long loopTime = 0;

void canRX_289(const CAN_message_t &msg); //Inverter RPM, Battery and Torque
void canRX_299(const CAN_message_t &msg); //Inverter Temps
void canRX_351(const CAN_message_t &msg); //BMS Status
void canRX_355(const CAN_message_t &msg); //BMS Status
void canRX_356(const CAN_message_t &msg); //BMS HV Voltage
void canRX_377(const CAN_message_t &msg); //Outlander Charger Low voltage stats
void canRX_389(const CAN_message_t &msg); //Outlander Charger HV stats
void canRX_732(const CAN_message_t &msg); //Inverter Current
void canRX_733(const CAN_message_t &msg); //Inverter Temps
void dashComms();                         //update the dash-board
void bmsComms();                          //Comms to the BMS - Set Key on

void dogFood();
void menu();
void readPins();
void readPedal();
void inverterComms();
void tempCheck();
void showInfo();
void loadDefault();
void saveVarsToEEPROM();
void stateHandler();

//Metro Timers

Metro timer50_1 = Metro(50);     //inverter timer
Metro timer50_2 = Metro(50);     //De-bounce check
Metro timer100_1 = Metro(100);   //2nd inverter timer
Metro timer100_2 = Metro(96);    //longer Debounce
Metro timer100_3 = Metro(110);   //Temp handler
Metro timer500_1 = Metro(50);    //pedal debug timer
Metro timer1000_1 = Metro(1000); //General 1s timer
Metro timer2000_1 = Metro(2000); //Serial update timer
Metro timer30s_1 = Metro(30000); //30Sec Timer to check DC-DC State
Metro timer10_1 = Metro(10);     //Dash coms timer - needs to be fast for stepper motor



//ADC setup

ADC *adc = new ADC();

/*/ Define Outputs
#define OUT1 6    //NEG Contactor
#define OUT2 9    //PRE Charge Contactor
#define OUT3 10   //Drive Contactor
#define OUT4 11   //Brake Lights
#define OUT5 12   //Pump
#define OUT6 24   //FAN
#define OUT7 25   // No connection
#define OUT8 28   //DC -DC Enable
#define OUT9 29   //Temp Gauge
#define OUT10 33  //RED LED
#define OUT11 36  //Green LED
#define OUT12 37  //Reverse Lights
#define LEDpin 13 //Builtin LED

//Define Inputs
#define ISO_IN1 2  //FWD
#define ISO_IN2 3  //START
#define ISO_IN3 4  //Brake
#define ISO_IN4 5  //REV
#define ISO_IN5 26 // MAP 2 ECO
#define ISO_IN6 27 // MAP 3 SPORT
#define ISO_IN7 32 // IGNITION
#define ISO_IN8 21 // PP Detect
*/

#define POT_A 23 //POT A

//Setup Variables

uint8_t brake_pedal; //Brake lights

uint8_t start;
uint8_t ppDetect;    //Prox pilot pin
uint8_t ignition;    //ignition
uint8_t dir_FWD;     //Drive Switch is set to forward
uint8_t dir_REV;     //dRIVE Sitch is to Reverse
uint8_t dir_NEUTRAL; //Set Neutral as the default state
uint8_t BMS_Status;  //BMS Status
uint8_t BMS_SOC;
uint8_t inverterFunction = 0x00;
uint8_t BMS_keyOn = 0;

float BMS_avgtmp; //BMS Battery AVG Temp
float currentact; //BMS Current
float BMS_packvoltage;
int BMS_discurrent;

unsigned long pretimer1;


int chargerTemp1 = 0;
int chargerTemp2 = 0;
int chargerTemp3 = 0;
int chargerTemp4 = 0;
int chargerHVcurrent = 0;
uint8_t chargerStatus;
int avgChargerTemp = 0;

int motorRPM = 0;
int motorTempPeak = 0;
int motorTemp1 = 0;
int motorTemp2 = 0;
uint16_t motorHVbatteryVolts = 0;
int motorTorque = 0;
int motorCurrent1 = 0;
int motorCurrent2 = 0;
int avgMotorTemp = 0;

int inverterTemp1 = 0;
int inverterTemp2 = 0;
int avgInverterTemp = 0;

byte torqueHibyte = 0;
byte torqueLoByte = 0;
int torqueRequest = 0;
int targetTorque = 0;
int curentTorque = 0;
int throttlePosition = 0;
uint8_t pedalDebug = 0;
uint8_t inverterEnable = 1;
int regenTarget = 0;       // target regen torque
uint32_t regenTimer = 0;   // timer for regen delay
uint32_t regenDelay = 250; //delay before regen Starts
uint8_t regenState = 0;    //Are we requesting regen
uint32_t brakeDelay = 0;
int regenRequest = 0;

int incomingByte;
uint8_t menuLoad = 0;
uint8_t showStats = 1;
uint8_t pumpState = 0;
uint8_t fanState = 0;

//Setup the peddal map arrays..

byte idx_j, idx_k; //index of tps,rpm bins
int pedal_offset;

const int num_rpm_bins = 21;
const int tpsbins[21] = {0, 3, 5, 8, 10, 13, 18, 20, 23, 25, 28, 32, 34, 40, 50, 60, 70, 80, 90, 100, 101};

const int rpmbins[num_rpm_bins] = {
    250, 500, 625, 750, 1000, 1250, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 10000};


const int pedal_map_three[21][22] = {  //Sport
    //map 3..
    /*250*/ {0, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*500*/ {-10, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*625*/ {-20, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*750*/ {-30, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*1000*/ {-50, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*1250*/ {-70, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*1500*/ {-90, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*2000*/ {-110, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*2500*/ {-130, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*3000*/ {-150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*3500*/ {-150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*4000*/ {-150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*4500*/ {-150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*5000*/ {-160, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*5500*/ {-180, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*6000*/ {-200, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*6500*/ {-200, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*7000*/ {-225, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*7500*/ {-250, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*8000*/ {-300, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
    /*10000*/ {-300, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
};


//VCU Staus

#define boot 1
#define ready 2
#define driveNeutral 3
#define driveForward 4
#define driveReverse 5


uint8_t VCUstatusChangeCounter = 0;    //used for hysterisys to stop jitter on VCU status
uint8_t VCUstatusChangeThreshold = 60; // n loops

uint8_t VCUstatus = 1;

uint8_t contactorState = 0; //state of contactors


//////timers
Metro coolanttimer = Metro(1000);
Metro chargerEVSE = Metro(100);
Metro charger800 = Metro(800);

//OI inputs
int startbutton = 13;
int brake = 14;
int fwd = 15;
int rev = 16;


//gauges
int rpm = 5;
int motortempgauge = 37;
int fuel = 36;
float rpmraw;
int batterylight = 31;

int dcdcon = 2;
//int dcdccontrol = 23; // 5v signal wire, not used

//coolant temp and engine bay fan

int ThermistorPin = 18;
int enginefan = 17;
int Vo;
int coolanttemp;
float R1 = 10000;
float logR2, R2, T;
float c1 = 0.9818585903e-03, c2 = 1.995199371e-04, c3 = 1.684445298e-07;

//contactors
int maincontactorsignal = 20;
int precharge = 22;
int maincontactor = 21;
int maincontactorsingalvalue = 1;

//Charging
//int cpwm = 24; 12v signal not used
int MG2 = 25;// 12v signal
int negcontactor = 32;
int simpprox = 26;
//int simppilot = 27; grounded input not used
int chargestart = 28; // use for DC-DC pn charger?
//int chargebutton = 12; 12v sinal not used

//HV stuff
float HVbus;
float HVdiff;
float Batvolt;
int Batvoltraw;
int AuxBattVolt;
int Batmax;
float Batmaxraw;

int chargemode;

// car inputs
int Batterysoc;


void setup() {
  Serial.begin(115200); delay(400);
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++) {
    Can0.setMB(i, RX, STD);
  }
  for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++) {
    Can0.setMB(i, TX, STD);
  }
  //  Can0.setMBFilter(REJECT_ALL);
  Can0.enableMBInterrupts();
  //  Can0.setMBFilterProcessing(MB0, 0x3FF, 0xFF);
  //Can0.setMBFilterProcessing(MB1,0x400, 0xFF);
  //Can0.setMBFilterProcessing(MB2,0x0B,0xFF);
  // Can0.enhanceFilter(MB0);
  //Can0.enhanceFilter(MB1);
  Can0.onReceive(MB0, canSniff1);
  //Can0.onReceive(MB1,canSniff2);
  //Can0.onReceive(MB2,canSniff);
  Can0.mailboxStatus();


  //outputs
  pinMode(rpm, OUTPUT);
  pinMode(enginefan, OUTPUT);
  pinMode(motortempgauge, OUTPUT);
  pinMode(precharge, OUTPUT);
  pinMode(maincontactor, OUTPUT);
  // pinMode(dcdccontrol, OUTPUT);
  pinMode(dcdcon, OUTPUT);
  pinMode(chargestart, OUTPUT);
  //pinMode(cpwm, OUTPUT);
  pinMode(MG2, OUTPUT);
  pinMode(negcontactor, OUTPUT);
  // pinMode(startbutton, OUTPUT);
  pinMode(fwd, OUTPUT);
  pinMode(rev, OUTPUT);
  pinMode(brake, OUTPUT);
  pinMode(batterylight, OUTPUT);
  //inputs
  pinMode(simpprox, INPUT_PULLUP);
  //  pinMode(simppilot, INPUT_PULLUP);
  // pinMode(chargebutton, INPUT_PULLUP);
  pinMode(maincontactorsignal, INPUT_PULLUP);

  //gauge pin setup
  analogWrite(rpm, 127);
  analogWrite(motortempgauge, 70);

  //throttle
   pinMode(POT_A, INPUT); //Throtle Pot A


  //Switch off contactors on startup
  digitalWrite (precharge, LOW);
  digitalWrite (maincontactor, LOW);
  digitalWrite (negcontactor, LOW);

  chargemode = 0;

  //Setup ADC

  adc->adc0->setAveraging(16);                                    // set number of averages
  adc->adc0->setResolution(16);                                   // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed

  adc->adc1->setAveraging(16);                                    // set number of averages
  adc->adc1->setResolution(16);                                   // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed

  //EEPROM Stored Vars

uint8_t tempGaugeMin = EEPROM.read(1); //Temp Gauge min PWM
uint8_t tempGaugeMax = EEPROM.read(2); //Temp Gauge Max PWM

uint8_t pumpOnTemp = EEPROM.read(3);
uint8_t pumpoffTemp = EEPROM.read(4);

uint8_t fanOnTemp = EEPROM.read(5);
uint8_t fanOffTemp = EEPROM.read(6);

int maxTorque = EEPROM.read(31) * 255 + EEPROM.read(30);
int minTorque = EEPROM.read(8);                              //Used for creeping
int tpslowOffset = EEPROM.read(21) * 255 + EEPROM.read(20);  //Value when foot off pedal
int tpshighOffset = EEPROM.read(23) * 255 + EEPROM.read(22); //Value when foot on pedal
int torqueIncrement = EEPROM.read(11);                       //used for default rpm based regen
uint8_t setTpsLow = 0;
uint8_t setTpsHigh = 0;

uint8_t active_map = 1;   //Active Pedal map
uint8_t map2;         //Eco Map
uint8_t map3;         //Sport MAp

  delay(3000);


  //-------If charge port plugged in on startup run through charging setup.
  digitalRead (simpprox);
  if (digitalRead(simpprox)) // run normal start up
  {
    // digitalWrite (startbutton, HIGH);
    digitalWrite (negcontactor, HIGH);
    digitalWrite (precharge, HIGH);   //activate prehcharge on start up
    analogWriteFrequency(rpm, 68);//Start rpm at intial high to simulate engine start.Serial.print("normal startup");
    //digitalWrite(csdn, LOW);
    digitalWrite(MG2, HIGH);
    Serial.print("normal startup");
    chargemode = 1;



  }
  else //
  {

    digitalWrite(fwd, HIGH);
    digitalWrite(rev, HIGH);
    delay (1000);
    digitalWrite (negcontactor, HIGH);
    Serial.print("charge port connected");
    chargemode = 2;
  }
  delay(1000);


ADC::Sync_result result;
}

void canSniff1(const CAN_message_t &msg) {
  if (msg.id == 0x3FF)
  {
    HVbus = (( msg.buf[6] << 8) | msg.buf[5]); //Voltage on Prius HVBUS
    HVbus = HVbus / 32;
    rpmraw = (( msg.buf[4] << 8) | msg.buf[3]); //Prius motor rpm
    Batterysoc = msg.buf[7];

  }
  if (msg.id == 0x355)
  {
    Batterysoc = (( msg.buf[1] << 8) | msg.buf[0]);

  }
  if (msg.id == 0x356)//battery voltage from SIMP BMS
  {

    Batvoltraw = (( msg.buf[1] << 8) | msg.buf[0]);
    Batvolt = Batvoltraw / 10;
  }
  if (msg.id == 0x400)
  {
    AuxBattVolt = msg.buf[0]; //Prius inverter aux voltage
  }
  if (msg.id == 0x373) // highest cell voltage from SIMP BMS
  {
    Batmaxraw = (( msg.buf[3] << 8) | msg.buf[2]);
    Batmax = Batmaxraw;
  }

}

void coolant()
{
  if (coolanttimer.check()) {
    //---------Temperature read

    Vo = analogRead(ThermistorPin); /// use 10k resistor
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    T = T - 273.15; //in C
    coolanttemp = T;

    //--------- Activate engine bay fan

    if (coolanttemp > 40)
    {
      digitalWrite(enginefan, LOW);
    }
    else
    {
      digitalWrite(enginefan, HIGH);
    }

  }
}

void closecontactor() { //--------contactor close cycle
  // if hv bus is within a few volts of battery voltage and OI is sending close main contactor, close main contactor and open precharge. Also activate dc-dc
  HVdiff = Batvolt - HVbus; //calculates difference between battery voltage and HV bus

  //digitalRead(maincontactorsignal);
  //maincontactorsingalvalue = digitalRead(maincontactorsignal);
  // Serial.print (maincontactorsingalvalue);
  if (HVbus > 250 &&  HVdiff < 5)
  {
    digitalWrite (maincontactor, HIGH);
    /// digitalWrite (startbutton, HIGH);
    digitalWrite(fwd, HIGH);
    digitalWrite (dcdcon, HIGH);
    digitalWrite (precharge, LOW);
   


  }
  else if (HVbus < 250 or HVdiff > 5)
  {
    digitalWrite (maincontactor, LOW);
    digitalWrite (negcontactor, HIGH);
    digitalWrite (dcdcon, LOW);

  }
}

void gauges() {




  // Battery light
  if (AuxBattVolt < 13)
  {
    digitalWrite(batterylight, HIGH);
  }
  else
  {
    digitalWrite(batterylight, LOW);
  }
  // Battery Soc
  // analogWriteFrequency(fuel, 255);
  int fuelpwm = Batterysoc * 2.43;
  int fuelfreq = fuelpwm + 12.8;

  // send signals

  if (chargerEVSE.check()) { //100ms timer
    {
      // RPM
      //analogWrite(rpm, 127);
      float rpm1 = rpmraw / 32;
      int rpmpulse = rpm1 / 30;
      int rpmsend;
      if (rpmpulse < 30) //power steering is expecting to see engine idle at least.
      {
        rpmsend = 30;
      }
      else
      {
        rpmsend = rpmpulse;
      }
      analogWriteFrequency(rpm, rpmsend);
      analogWrite(fuel, fuelfreq);

    }
  }
  //analogWriteFrequency(motortempgauge, 255);
  //To Do

  // temperature from coolant.


}

void charging() {
  if (chargerEVSE.check()) { //100ms timer to send canbus messages
    if (Batmax < 4100) // as long as max cell is under 4100mV, send signal to EVSE.
    {
      //unsigned char evse[8] = {0x00, 0x00, 0xB6, 0x00, 0x00, 0x00, 0x00, 0x00};
      CAN_message_t msg1;
      msg1.id = (0x285);
      msg1.len = 8;
      msg1.buf[0] = 0;
      msg1.buf[1] = 0;
      msg1.buf[2] = 0xB6;
      msg1.buf[3] = 0;
      msg1.buf[4] = 0;
      msg1.buf[5] = 0;
      msg1.buf[6] = 0;
      msg1.buf[7] = 0;
      Can0.write(msg1);
      digitalWrite (dcdcon, HIGH);
    }
    else
    {
      CAN_message_t msg1;
      msg1.id = (0x285);
      msg1.len = 8;
      msg1.buf[2] = 0x00;
      Can0.write(msg1);
      digitalWrite (dcdcon, LOW);

    }
  }

  if (charger800.check()) {
    unsigned char charger800[8] = {0x28, 0x0F, 0x78, 0x37, 0x00, 0x00, 0x0A, 0x00};
    CAN_message_t msg1;
    msg1.id = (0x286);
    memcpy (msg1.buf, charger800, 8);
    Can0.write(msg1);

  }

}
void inverterComms()
{
  if (timer50_1.check() == 1)
  {

    if (regenState == 0 && VCUstatus == 4 && motorRPM > 2000) // Increment Torque delivery
    {

      if (curentTorque < targetTorque)
      {
        curentTorque += torqueIncrement;
        torqueRequest = curentTorque;
      }

      if (curentTorque >= targetTorque)
      {
        torqueRequest = targetTorque;
        curentTorque = targetTorque;
      }
    }
    else if (regenState == 0 && VCUstatus == 4 && motorRPM < 2000)
    {
      torqueRequest = targetTorque;
      curentTorque = torqueRequest;
    }

    if (active_map == 3)  // No need to increment in 'sport' mode
    {
      torqueRequest = targetTorque;
      curentTorque = torqueRequest;
    }

    if (regenState == 1 && VCUstatus == 4)

    {
      if (regenTarget < regenRequest) // increment Regen
      {
        regenRequest -= 5;
        torqueRequest = regenRequest;
        // Serial.println("Regen inc.");
      }
      else
      regenRequest = regenTarget;
      torqueRequest = regenRequest;
    }

    if (regenState == 2 && VCUstatus == 4)

    {
      if (regenTarget > regenRequest) // increment Regen off
      {
        regenRequest += 30;
        torqueRequest = regenRequest;
        // Serial.println("Regen Dec.");
      }
    }

    if (torqueRequest > (2000))
    {
      torqueRequest = 0;
      Serial.println("--!OVER TOURQUE!--");
    }
    if (torqueRequest < (-1000))
    {
      torqueRequest = 0;
      Serial.println("--!UNDER TOURQUE!--");
    }

    torqueRequest += 10000;

    if (BMS_discurrent < currentact) //Decrese tourque if we are over current - Crude needs work..
    {
      torqueRequest -= 20;
      Serial.println("--!OVER CURRENT!--");
      if (torqueRequest < 0)
      {
        torqueRequest = 0;
      }
    }

    if (pedalDebug == 1)
    {
      Serial.print("Offset: ");
      Serial.print(pedal_offset);
      Serial.print(" Tourque Request: ");
      Serial.println(torqueRequest - 10000);
      Serial.print("Regen Target:");
      Serial.print(regenTarget);
      Serial.print(" Regen Request: ");
      Serial.print(regenRequest);
      Serial.print(" Motor Torque ");
      Serial.println(motorTorque);
    }
    if (inverterEnable != 1)
    {
      inverterFunction = 0x00;
    }
    torqueLoByte = lowByte(torqueRequest);
    torqueHibyte = highByte(torqueRequest);
    msg.id = 0x287;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = torqueHibyte;
    msg.buf[3] = torqueLoByte;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = inverterFunction;
    msg.buf[7] = 0;
    Can0.write(msg);
    torqueRequest = 0;
  }

  if (timer100_1.check() == 1)
  {
    msg.id = 0x371;
    msg.len = 8;
    msg.buf[0] = 48;
    msg.buf[1] = 0;
    msg.buf[2] = 0;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    Can0.write(msg);
    delay(1);
    msg.id = 0x285;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = 20;
    msg.buf[3] = 57;
    msg.buf[4] = 143;
    msg.buf[5] = 254;
    msg.buf[6] = 12;
    msg.buf[7] = 16;
    Can0.write(msg);
    delay(1);

    msg.id = 0x286;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = 0;
    msg.buf[3] = 61;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 33;
    msg.buf[7] = 0;
    Can0.write(msg);
  }
}



void loop() {

  if (chargemode == 1) //normal driving
  {
    Can0.events();
    closecontactor(); //checks precharge level and close contactor
    coolant(); // check coolant temperature and swtich on engine bay fan if needed.
    gauges(); //send information to guages
  }
  else if (chargemode == 2) // charging
  {
    Can0.events();
    charging();
    coolant(); // check coolant temperature and swtich on engine bay fan if needed.
    gauges(); //send information to guages
  }
  /// To Do



}
