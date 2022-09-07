#include <FlexCAN_T4.h>
#include <Metro.h>
#include <ADC.h>
#include <ADC_util.h>
#include <EEPROM.h>
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#define NUM_TX_MAILBOXES 6
#define NUM_RX_MAILBOXES 6
CAN_message_t msg;

//////timers
Metro coolanttimer = Metro(1000);
Metro chargerEVSE = Metro(100);
Metro charger800 = Metro(800);
Metro timer10ms = Metro(10);
Metro timer50_1 = Metro(50); // Inverter timer

//Output to IO
int startbutton = 13;
int fwd = 15;
int rev = 16;


//gauges
int rpm = 5;
int motortempgauge = 37;
int fuel = 36;
float rpmraw;
int batterylight = 31;

//Outlander Inverter control
int motorTorque = 0;
int motorRPM = 0;
int motorTempPeak = 0;
int motorTemp1 = 0;
int motorTemp2 = 0;
int inverterTemp1 = 0;
int inverterTemp2 = 0;
int avgInverterTemp = 0;
byte torqueHibyte = 0;
byte torqueLoByte = 0;
int torqueRequest = 0;
int targetTorque = 0;
int curentTorque = 0;
int throttlePosition = 0;
int Pot_A = 18; // was thermistor pin
int brakeinput = 27; // ground input, orginally simppilot

int dcdcon = 2;
int dcdccontrol = 23; // 5v signal wire, not used

//coolant temp and engine bay fan

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
int RSDN = 25;// 12v signal not used was cdsn. Now inverter shutdown
int negcontactor = 32;
int simpprox = 26;
//int simppilot = 27; grounded input, now used for brake light.
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
int throttlepot; //throttle reading

// car inputs
int Batterysoc;


//// Pedal and Map stuff

int maxTorque = EEPROM.read(31) * 255 + EEPROM.read(30);
int minTorque = EEPROM.read(8);                              //Used for creeping
int tpslowOffset = EEPROM.read(21) * 255 + EEPROM.read(20);  //Value when foot off pedal
int tpshighOffset = EEPROM.read(23) * 255 + EEPROM.read(22); //Value when foot on pedal
int torqueIncrement = EEPROM.read(11);                       //used for default rpm based regen
uint8_t setTpsLow = 0;
uint8_t setTpsHigh = 0;

uint8_t active_map = 3;   //Active Pedal map
uint8_t map2;         //Eco Map
uint8_t map3;         //Sport MAp

//Setup the peddal map arrays..

byte idx_j, idx_k; //index of tps,rpm bins
int pedal_offset;

const int num_rpm_bins = 21;
const int tpsbins[21] = {0, 3, 5, 8, 10, 13, 18, 20, 23, 25, 28, 32, 34, 40, 50, 60, 70, 80, 90, 100, 101};

const int rpmbins[num_rpm_bins] = {
  250, 500, 625, 750, 1000, 1250, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 10000
};

const int pedal_map_one[21][22] = {   //Normal
  //map 1..
  /*250*/ {0, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*500*/ { -10, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*625*/ { -20, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*750*/ { -30, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*1000*/ { -50, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*1250*/ { -70, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*1500*/ { -90, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*2000*/ { -110, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*2500*/ { -130, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*3000*/ { -150, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*3500*/ { -150, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*4000*/ { -150, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*4500*/ { -150, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*5000*/ { -160, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*5500*/ { -180, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*6000*/ { -200, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*6500*/ { -200, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*7000*/ { -225, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*7500*/ { -250, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*8000*/ { -300, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
  /*10000*/ { -300, 0, 6, 6, 6, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5},
};

const int pedal_map_two[21][22] = {   //ECO
  //map 2..
  /*250*/ {0, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*500*/ { -10, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*625*/ { -20, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*750*/ { -30, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*1000*/ { -50, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*1250*/ { -70, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*1500*/ { -90, 0, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*2000*/ { -110, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*2500*/ { -130, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*3000*/ { -150, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*3500*/ { -150, 0, 0, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*4000*/ { -150, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*4500*/ { -150, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*5000*/ { -160, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*5500*/ { -180, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*6000*/ { -200, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*6500*/ { -200, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*7000*/ { -225, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*7500*/ { -250, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*8000*/ { -300, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
  /*10000*/ { -300, 0, 0, 3, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6},
};

const int pedal_map_three[21][22] = {  //Sport
  //map 3..
  /*250*/ {1, 2, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*500*/ { -10, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*625*/ { -20, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*750*/ { -30, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*1000*/ { -50, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*1250*/ { -70, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*1500*/ { -90, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*2000*/ { -110, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*2500*/ { -130, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*3000*/ { -150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*3500*/ { -150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*4000*/ { -150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*4500*/ { -150, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*5000*/ { -160, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*5500*/ { -180, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*6000*/ { -200, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*6500*/ { -200, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*7000*/ { -225, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*7500*/ { -250, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*8000*/ { -300, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
  /*10000*/ { -300, 0, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 6},
};


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

  HVbus = 0;


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
  pinMode(RSDN, OUTPUT);
  pinMode(negcontactor, OUTPUT);
  // pinMode(startbutton, OUTPUT);
  pinMode(fwd, OUTPUT);
  pinMode(rev, OUTPUT);
  // pinMode(brake, OUTPUT);
  pinMode(batterylight, OUTPUT);
  //inputs
  pinMode(simpprox, INPUT_PULLUP);
  pinMode(brakeinput, INPUT_PULLUP);
  // pinMode(chargebutton, INPUT_PULLUP);
  pinMode(maincontactorsignal, INPUT_PULLUP);
  pinMode(Pot_A, INPUT);


  //gauge pin setup
  analogWrite(rpm, 127);
  analogWrite(motortempgauge, 70);


  //Switch off contactors on startup
  digitalWrite (precharge, LOW);
  digitalWrite (maincontactor, LOW);
  digitalWrite (negcontactor, LOW);


  chargemode = 0;

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

    Serial.print("normal startup");
    chargemode = 1;



  }
  else //
  {

    digitalWrite(fwd, HIGH);
    digitalWrite(rev, HIGH);
    delay (1000);
    digitalWrite (negcontactor, HIGH);
    digitalWrite (precharge, HIGH);
    Serial.print("charge port connected");
    chargemode = 2;
  }
  delay(5000);
  closecontactor();
  

}

void canSniff1(const CAN_message_t &msg) {
  if (msg.id == 0x289)
  {

    HVbus = (msg.buf[4] * 256 + msg.buf[5]); //Voltage on Outlander Inverter
    rpmraw = (msg.buf[2] * 256 + msg.buf[3] - 20000); //Outlander inverter RPM
    motorTorque = ((((msg.buf[0] * 256) + msg.buf[1]) - 10000) / 10);


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

  if (msg.id == 0x299)//inverter messages
  {

    //inverter Temps
    motorTempPeak = (msg.buf[0] - 40);
    inverterTemp1 = (msg.buf[1] - 40);
    inverterTemp2 = (msg.buf[4] - 40);

    avgInverterTemp = (inverterTemp1 + inverterTemp2) / 2;
    coolanttemp = avgInverterTemp;
  }

  if (msg.id == 0x733)//motor temps
  {

    motorTemp1 = (msg.buf[0] - 40);
    motorTemp2 = (msg.buf[2] - 40);

  }

  if (msg.id == 0x377) // 12v sense from Charger
  {

    AuxBattVolt = float(((msg.buf[0] * 256) + msg.buf[1]) * 0.01);
  }


  if (msg.id == 0x373) // highest cell voltage from SIMP BMS
  {

    Batmaxraw = (( msg.buf[3] << 8) | msg.buf[2]);
    Batmax = Batmaxraw;
  }

  if (msg.id == 0x389) // not fast enough reporting for contactor control
  {

    //HVbus = (msg.buf[0] * 2);//56 + msg.buf[5]); //Voltage on Outlander Inverter



  }

}

void coolant()
{
  if (coolanttimer.check()) {
    //---------Temperature read
    // use canbus from inverter to get coolant temp
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
  //Serial.println(HVdiff);
  ///Serial.print("bat volt ");
  //Serial.println(Batvolt);

  if (HVbus > 250)// &&  HVdiff < 5)
  {
    digitalWrite (maincontactor, HIGH);
    //Serial.print("close main contactor!");

    //digitalWrite (dcdcon, HIGH);
    digitalWrite (precharge, LOW);
    


  }
  else if (HVbus < 250 or HVdiff > 5)
  {
    digitalWrite (maincontactor, LOW);
    digitalWrite (negcontactor, HIGH);
    digitalWrite (precharge, HIGH);
    digitalWrite (dcdcon, LOW);

    digitalWrite(batterylight, HIGH);
    delay (1000);
    digitalWrite(batterylight, LOW);
    delay (1000);
    digitalWrite(batterylight, HIGH);
    delay (1000);
    digitalWrite(batterylight, LOW);
    delay (1000);
    digitalWrite(batterylight, HIGH);
    delay (1000);
    digitalWrite(batterylight, LOW);
    delay (1000);
    digitalWrite(batterylight, HIGH);
    delay (1000);
    digitalWrite(batterylight, LOW);
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
      int rpmpulse = rpmraw / 30;
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

  if (timer50_1.check()) { //disable inverter for charging
    CAN_message_t msg1;
    msg1.id = (0x287);
    msg1.len = 8;
    msg1.buf[0] = 0;
    msg1.buf[1] = 0;
    msg1.buf[2] = 0;
    msg1.buf[3] = 0;
    msg1.buf[4] = 0;
    msg1.buf[5] = 0;
    msg1.buf[6] = 0x00;
    msg1.buf[7] = 0;
    Can0.write(msg1);

  }
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

void readPedal()


{

  throttlepot = map(analogRead(Pot_A), 520, 880, 0, 100); //change 127 and 950 after in car.
  if (throttlepot < 0) {
    throttlepot = 0;
  }
  // pedal_offset = pedal_map_three[idx_j][idx_k];  // Not needed until you figure out maps
  targetTorque = (throttlepot * 2);//pedal_offset) * 2; Just direct translation from throttle percentage to amount of torque requested.
  if (digitalRead(brakeinput))
  {
  }
  else
  {
    targetTorque = 0; //0 Torque if the brake is presed
  }

}

void inverterComms()
{
  if (timer50_1.check()) {
    readPedal();
    torqueRequest = targetTorque;
    curentTorque = torqueRequest;
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


    torqueLoByte = lowByte(torqueRequest);
    torqueHibyte = highByte(torqueRequest);

    CAN_message_t msg2;
    msg2.id = (0x287);
    msg2.len = 8;
    msg2.buf[0] = 0;
    msg2.buf[1] = 0;
    msg2.buf[2] = torqueHibyte;
    msg2.buf[3] = torqueLoByte;
    msg2.buf[4] = 0;
    msg2.buf[5] = 0;
    msg2.buf[6] = 0x03;
    msg2.buf[7] = 0;
    Can0.write(msg2);
    torqueRequest = 0;
    delay(1);

  }
  
  if (chargerEVSE.check()) {
    CAN_message_t msg3;
    msg3.id = 0x371;
    msg3.len = 8;
    msg3.buf[0] = 48;
    msg3.buf[1] = 0;
    msg3.buf[2] = 0;
    msg3.buf[3] = 0;
    msg3.buf[4] = 0;
    msg3.buf[5] = 0;
    msg3.buf[6] = 0;
    msg3.buf[7] = 0;
    Can0.write(msg3);
    delay(1);
    CAN_message_t msg4;
    msg4.id = 0x285;
    msg4.len = 8;
    msg4.buf[0] = 0;
    msg4.buf[1] = 0;
    msg4.buf[2] = 20;
    msg4.buf[3] = 57;
    msg4.buf[4] = 143;
    msg4.buf[5] = 254;
    msg4.buf[6] = 12;
    msg4.buf[7] = 16;
    Can0.write(msg4);
    delay(1);
    CAN_message_t msg5;
    msg5.id = 0x286;
    msg5.len = 8;
    msg5.buf[0] = 0;
    msg5.buf[1] = 0;
    msg5.buf[2] = 0;
    msg5.buf[3] = 61;
    msg5.buf[4] = 0;
    msg5.buf[5] = 0;
    msg5.buf[6] = 33;
    msg5.buf[7] = 0;
    Can0.write(msg5);
  }



}


void loop() {

  if (chargemode == 1) //normal driving
  {
    Can0.events();
    //closecontactor(); //checks precharge level and close contactor
    coolant(); // check coolant temperature and swtich on engine bay fan if needed.
    gauges(); //send information to guages
    inverterComms();
    delay(50);

  }
  else if (chargemode == 2) // charging
  {
    Can0.events();
   // closecontactor();
    charging();
    coolant(); // check coolant temperature and swtich on engine bay fan if needed.
    //gauges(); //send information to guages
  }
  /// To Do




}
