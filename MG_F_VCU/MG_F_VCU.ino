/*
  Copyright (c) 2022 Somerset EV
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.cur
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  -------
  Outlander inverter code based on repo from AOT93 https://github.com/aot93/Mini-E-VCU

*/


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
Metro gauge100 = Metro(100);
Metro gauge200 = Metro(200);
Metro charger800 = Metro(800);
Metro timer10ms = Metro(10);
Metro timer100ms = Metro(100);
Metro timer50_1 = Metro(50); // Inverter timer


//Output to IO
int startbutton = 13;
int fwd = 15;
//int rev = 16;


//gauges
int rpm = 5;
//int motortempgauge = 37;  No longer used as needed PWM for speedo
int fuel = 36;
float rpmraw;
int batterylight = 31;
int rpmpulse;
int speedopulse;
int speedo = 16;// was rev


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
int16_t torqueRequest = 0;
int16_t torqueRequest1 = 0;
int16_t targetTorque = 0;
int16_t curentTorque = 0;
int16_t throttlePosition = 0;
int16_t Pot_A = 18; // was thermistor pin
int brakeinput = 27; // ground input, orginally simppilot

int dcdcon = 2;
int dcdccontrol = 23; // 5v signal wire, not used

//coolant temp and engine bay fan

int enginefan = 17;
int waterpump = 25;
int Vo;
int coolanttemp;
float R1 = 10000;
float logR2, R2, T;
float c1 = 0.9818585903e-03, c2 = 1.995199371e-04, c3 = 1.684445298e-07;

int chargerTemp1 = 0;
int chargerTemp2 = 0;
int chargerTemp3 = 0;
int chargerTemp4 = 0;
int avgChargerTemp = 0;

//contactors
int maincontactorsignal = 20;
int precharge = 22;
int maincontactor = 21;
int maincontactorsingalvalue = 1;

//Charging
//int cpwm = 24; 12v signal not used
int negcontactor = 32;
int simpprox = 26;
//int simppilot = 27; grounded input, now used for brake light.
int chargestart = 28; // use for DC-DC pn charger?
//int chargebutton = 12; 12v sinal not used

//HV stuff
int HVbus;
float HVdiff;
float Batvolt;
int Batvoltraw;
int AuxBattVolt;
int Batmax;
float Batmaxraw;

int chargemode;
int throttlepot; //throttle reading
int regenmode = 0;

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

  HVbus = 0;

  //outputs
  pinMode(rpm, OUTPUT);
  pinMode(enginefan, OUTPUT);
  // pinMode(motortempgauge, OUTPUT);
  pinMode(waterpump, OUTPUT);
  pinMode(precharge, OUTPUT);
  pinMode(maincontactor, OUTPUT);
  // pinMode(dcdccontrol, OUTPUT);
  pinMode(dcdcon, OUTPUT);
  pinMode(chargestart, OUTPUT);
  //pinMode(cpwm, OUTPUT);
  pinMode(speedo, OUTPUT);
  pinMode(negcontactor, OUTPUT);
  // pinMode(startbutton, OUTPUT);
  pinMode(fwd, OUTPUT);
  //pinMode(rev, OUTPUT);
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
  //analogWrite(motortempgauge, 70);
  analogWrite(speedo, 127);


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
    //digitalWrite(rev, HIGH);
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
    rpmraw *= -1; // until rpm reported correct way by inverter
    rpmpulse = rpmraw / 30;
    motorTorque = ((((msg.buf[0] * 256) + msg.buf[1]) - 10000) / 10);
    speedopulse = rpmraw / 21.156;





  }
  if (msg.id == 0x355)
  {
    Batterysoc = (( msg.buf[1] << 8) | msg.buf[0]);
    //Serial.println("SIMPBMS");
  }
  if (msg.id == 0x356)//battery voltage from SIMP BMS
  {

    Batvoltraw = (( msg.buf[1] << 8) | msg.buf[0]);
    Batvolt = Batvoltraw / 10;
    //Serial.println("SIMPBMS2");
  }

  if (msg.id == 0x299)//inverter messages
  {

    //inverter Temps
    motorTempPeak = (msg.buf[0] - 40);
    inverterTemp1 = (msg.buf[1] - 40);
    inverterTemp2 = (msg.buf[4] - 40);
    //Serial.println("inverter temps");
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
    chargerTemp1 = msg.buf[4] - 40;
    chargerTemp2 = msg.buf[5] - 40;
    chargerTemp3 = msg.buf[6] - 40;
    avgChargerTemp = (chargerTemp1 + chargerTemp2 + chargerTemp3 + chargerTemp4) / 3;
    //Serial.println("Charger Temp");
    //Serial.println(msg.buf[4] - 40);
  }


  if (msg.id == 0x373) // highest cell voltage from SIMP BMS
  {

    Batmaxraw = (( msg.buf[3] << 8) | msg.buf[2]);
    Batmax = Batmaxraw;
  }

  if (msg.id == 0x389) // not fast enough reporting for contactor control
  {
    //Serial.println("Charger amps");
    //Serial.println(msg.buf[2]);
    //int HVbus1 = (msg.buf[0] * 2);//56 + msg.buf[5]); //Voltage on Outlander Inverter



  }

}

void coolant()
{
  if (coolanttimer.check()) {
    //---------Temperature read
    // use canbus from inverter to get coolant temp
    //--------- Activate engine bay fan
    Serial.println("coolant temp");
    Serial.println(coolanttemp);
    if (coolanttemp > 40)
    {
      digitalWrite(enginefan, HIGH);
    }
    else
    {
      digitalWrite(enginefan, LOW);
    }

  }
}

void closecontactor() { //--------contactor close cycle
  // if hv bus is within a few volts of battery voltage and OI is sending close main contactor, close main contactor and open precharge. Also activate dc-dc
  HVdiff = Batvolt - HVbus; //calculates difference between battery voltage and HV bus
  //Serial.println(HVdiff);
  ///Serial.print("bat volt ");
  //Serial.println(Batvolt);

  if ((HVbus > 250) &&  HVdiff < 5)
  {
    digitalWrite (maincontactor, HIGH);
    digitalWrite (waterpump, HIGH);
    //Serial.print("close main contactor!");
    digitalWrite (dcdcon, HIGH);
    digitalWrite (precharge, LOW);



  }
  else if (HVbus < 250 or HVdiff > 5)
  {
    digitalWrite (precharge, LOW);
    digitalWrite (maincontactor, LOW);
    digitalWrite (negcontactor, LOW);
    digitalWrite (dcdcon, LOW);
    Serial.print("OPRECHARGE FAILED");
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

    delay (10000);
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

  if (gauge200.check()) { //200ms timer
    {
      // RPM
      //analogWrite(rpm, 127);

      /*int rpmsend;
        if (rpmpulse < 30 or rpmraw > 10000 ) //power steering is expecting to see engine idle at least.
        {
        rpmsend = 30;
        }
        else
        {
        rpmsend = rpmpulse;
        }
      */
      analogWriteFrequency(rpm,  rpmpulse);
      analogWrite(fuel, fuelfreq);
      analogWriteFrequency(speedo, speedopulse);

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
    }
    else
    {
      CAN_message_t msg1;
      msg1.id = (0x285);
      msg1.len = 8;
      msg1.buf[2] = 0x00;
      Can0.write(msg1);

    }
  }

  if (charger800.check()) {
    unsigned char charger800[8] = {0x28, 0x0F, 0x78, 0x37, 0x00, 0x00, 0x0A, 0x00};
    CAN_message_t msg1;
    msg1.id = (0x286);
    memcpy (msg1.buf, charger800, 8);
    Can0.write(msg1);

    if (chargerTemp1 > 30 or chargerTemp2 > 30 or chargerTemp3 > 30) {//(avgChargerTemp < 30) {
      digitalWrite (waterpump, HIGH);
    }
    else
    {
      digitalWrite (waterpump, LOW);
    }

  }

}

void readPedal()


{

  throttlepot = map(analogRead(Pot_A), 520, 880, 0, 100); //change 127 and 950 after in car.
  if (throttlepot < 0) {
    throttlepot = 0;
  }
  if (throttlepot > 10) {
    regenmode = 1;
  }
  // pedal_offset = pedal_map_three[idx_j][idx_k];  // Not needed until you figure out maps
  targetTorque = (throttlepot * 19);//pedal_offset) * 2; Just direct translation from throttle percentage to amount of torque requested.
  if (digitalRead(brakeinput) )
  {
  }
  else
  {
    if ((rpmraw > 250) and (regenmode == 1))
    {
      targetTorque = -150; //0 Torque if the brake is presed
    }
    else
    {
      targetTorque = 0; //0 Torque if the brake is presed
      regenmode = 0;
    }
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
    torqueRequest *= -1;
    torqueRequest += 10000;
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

  if (timer100ms.check()) {
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
    coolant(); // check coolant temperature and swtich on engine bay fan if needed.
    gauges(); //send information to guages
    inverterComms();


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
