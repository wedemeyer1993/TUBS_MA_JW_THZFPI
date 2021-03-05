#include <Arduino.h>
#include <PololuTB9051FTGSingleBrushedDCDriver.h>
#include <Nextion.h>
//#include <Encoder.h>
#include <TimerOne.h>
#include <PID_v1.h>
//#include <string.h>
//#include <CircularBuffer.h>

//Pinbelegung
#define _LEDINT 13
#define _FWDLIMITSW 8
#define _REVLIMITSW 9
#define _HallsenA 2
#define _HallsenB 3
#define _ENCODEROUTPUT 512

//Objects Page 1
//NexButton bStart = NexButton(1, 1, "bStart");
/* NexButton bMessSpeich = NexButton(1, 2, "bMessSpeich");
NexWaveform wWaveform = NexWaveform(0, 4, "wWaveform");
NexButton bDB = NexButton(1, 5, "bDB");
NexProgressBar pProgress = NexProgressBar(1, 5, "pProgress"); */

//Objects Page 2
NexButton end_s_v = NexButton(2, 1, "end_s_v");
NexButton end_s_h = NexButton(2, 3, "end_s_h");
NexButton bGraph = NexButton(2, 33, "bGraph");
NexButton bStop = NexButton(2, 34, "bStop");
NexButton bPID = NexButton(2, 27, "bPID");
NexSlider hPWM1 = NexSlider(2, 4, "hPWM1");
NexSlider hPWM2 = NexSlider(2, 5, "hPWM2");
//NexText tPos = NexText(2, 30, "tPos");
NexSlider sl_pos = NexSlider(2, 28, "sl_pos");
NexText t0 = NexText(2, 7, "t0");
NexWaveform s0 = NexWaveform(2, 17, "s0");
NexSlider sl_Kp = NexSlider(2, 24, "sl_Kp");
NexSlider sl_Ki = NexSlider(2, 25, "sl_Ki");
NexSlider sl_Kd = NexSlider(2, 26, "sl_Kd");
NexText tKp = NexText(2, 18, "tKp");
NexText tKi = NexText(2, 19, "tKi");
NexText tKd = NexText(2, 20, "tKd");

NexTouch *nex_listen_list[] = {
    &bGraph,
    &bPID,
    &bStop,
    &hPWM1,
    &hPWM2,
    //&tPos,
    &sl_pos,
    &t0,
    &sl_Kp,
    &sl_Ki,
    &sl_Kd,
    NULL};

//*****************************************************************************
//        Variablen
//*****************************************************************************
uint32_t valPWM1 = 0;
uint32_t valPWM2 = 0;
boolean fwdLimitSw = 1;
boolean revLimitSw = 1;
boolean fehler = 0;
//Timer
int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
//Encoder
volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile long oldEncPos = 0;
volatile byte reading = 0;
//PID speed
double pv_speed = 0;
int speedSafeWert = 0;
int speedSafeSend = 0;
double set_speed = 0;
double pwm_pulse = 0; //this value is 0~255
double kp = 0.012, ki = 0.528, kd = 0.001;
char sendString[4];
//PID position
double pv_pos = 0;
uint32_t pv_pos_send = 0;
double set_pos = 0;
double kp1 = 1, ki1 = 0.522, kd1 = 0.990;
//Test
volatile unsigned long pulse[100];
volatile byte flanke = 0;
double pulsAbstand = 0;
double pulsAbstand_MittelW = 0;
boolean ersteMessung = 1;
//Display
boolean showGraph = 0;
boolean posMode = 0;

//Datenspeicher
//*****************************************************************************
//        Variablen
//*****************************************************************************
SoftwareSerial HMISerial(10, 11);
PololuTB9051FTGSingleBrushedDCDriver TLZ825B;
//CircularBuffer<double, 1000> speedBuffer;
//Encoder Z825BEncoder(2, 3);
PID speedPID(&pv_speed, &pwm_pulse, &set_speed, kp, ki, kd, DIRECT);
PID posPID(&pv_pos, &set_speed, &set_pos, kp1, ki1, kd1, DIRECT);

// *****************************
//  Drehzahl Ermittlung Pulsabstand in µs
// *****************************
void PinA()
{
  if (flanke < 100)
  {
    //*(pulsepointer+(flanke*8)) = micros();
    pulse[flanke] = micros();
    flanke++;
  }
  cli();                //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag)
  { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    pv_pos++;
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100)
    bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei();       //restart interrupts
}
void PinB()
{
  cli();                //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag)
  { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    pv_pos--;
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000)
    aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei();       //restart interrupts
}

void motorStop()
{
  TLZ825B.motorStop();
  pulsAbstand_MittelW = 0;
  flanke = 0;
  ersteMessung = 1;
  set_speed = 0;
  speedSafeWert = 0;
  pwm_pulse = 0;
  pv_speed = 0;
  speedPID.SetMode(MANUAL);
  posPID.SetMode(MANUAL);
  sendCommand("motorsteuerung.bStop.bco=63488");
}
void updatePIDwerte()
{
  speedPID.SetTunings(kp, ki, kd);
  posPID.SetTunings(kp1, ki1, kd1);
}
void stopIfFault()
{
  if (TLZ825B.getM1Fault())
  {
    TLZ825B.motorStop();
    fehler = 1;
    Serial.println("!!!MOTORFEHLER!!!");
    Serial.println("moegliche Fehlerquellen");
    Serial.println("under-voltage, VCC over-voltage, over-temperature, or over-current condition");
  }
}
void pruefeFehler()
{
  if (!TLZ825B.getM1Fault())
  {
    Serial.println("Fehler Behoben");
    fehler = 0;
  }
}
void checkLimitSw()
{
  boolean fwdLimitSwalt = fwdLimitSw;
  fwdLimitSw = TLZ825B.getFWDLIMITSW();
  boolean revLimitSwalt = revLimitSw;
  revLimitSw = TLZ825B.getREVLIMITSW();

  if ((!fwdLimitSw && TLZ825B.getFWDstate()) || (!revLimitSw && TLZ825B.getREVstate())) //Enschalter vorne erreicht auf Masse gezogen
  {
    motorStop();
    set_speed = 0;
  }

  if (fwdLimitSwalt != fwdLimitSw || revLimitSwalt != revLimitSw)
  {
    if (!fwdLimitSw)
    {
      sendCommand("motorsteuerung.end_s_v.txt=\"Fwd Limit Sw\"");
      sendCommand("motorsteuerung.end_s_v.bco=63488");
    }
    else
    {
      sendCommand("motorsteuerung.end_s_v.txt=\"--\"");
      sendCommand("motorsteuerung.end_s_v.bco=50712");
    }
    if (!revLimitSw)
    {
      sendCommand("motorsteuerung.end_s_h.txt=\"Rev Limit Sw\"");
      sendCommand("motorsteuerung.end_s_h.bco=63488");
      hPWM1.setValue(0);
      hPWM2.setValue(0);
    }
    else
    {
      sendCommand("motorsteuerung.end_s_h.txt=\"--\"");
      sendCommand("motorsteuerung.end_s_h.bco=50712");
    }
  }
  return;
}
//*****************************************************************************
//        Display Funktionen
//*****************************************************************************
//page 2
void bStopPopCallback(void *ptr)
{
  motorStop();
}
void bGraphPopCallback(void *ptr)
{
  if (showGraph)
  {
    showGraph = 0;
    sendCommand("bGraph.bco=50712");
  }
  else
  {
    showGraph = 1;
    sendCommand("bGraph.bco=1024");
  }
}
void bPIDPopCallback(void *ptr)
{
  if (posMode)
  {
    posMode = 0;
    //sendCommand("tsw hPWM1,1");
    //sendCommand("tsw hPWM2,1");
    //sendCommand("tsw sl_pos,0");
    bPID.setText("SpeedPID");
    speedPID.SetMode(MANUAL);
    posPID.SetMode(MANUAL);
  }
  else
  {
    posMode = 1;
    //sendCommand("tsw hPWM1,0");
    //sendCommand("tsw hPWM2,0");
    //sendCommand("tsw sl_pos,1");
    bPID.setText("Speed und Pos PID");
    speedPID.SetMode(AUTOMATIC);
    posPID.SetMode(AUTOMATIC);
  }
}
void hPWM1PopCallback(void *ptr) //Slider PWM1
{
  if (!fwdLimitSw)
  {
    hPWM1.setValue(0);
    return;
  }
  else
  {
    hPWM1.getValue(&valPWM1);
    hPWM2.setValue(0);
    //TLZ825B.setM1Speed(valPWM1);
    set_speed = valPWM1 * 11;

    if (!speedPID.GetMode())
    {
      speedPID.SetMode(AUTOMATIC);
      sendCommand("motorsteuerung.bStop.bco=50712");
    }

    //itoa(valPWM1, sendString, 10);
    //t0.setText(sendString);
  }
}
void hPWM2PopCallback(void *ptr) //Slider PWM2
{
  if (!revLimitSw)
  {
    hPWM2.setValue(0);
    return;
  }
  else
  {
    hPWM2.getValue(&valPWM2);
    hPWM1.setValue(0);
    //TLZ825B.setM1Speed(-valPWM2);
    set_speed = (double)(valPWM2)*11;
    set_speed = -set_speed;
    if (!speedPID.GetMode())
    {
      speedPID.SetMode(AUTOMATIC);
      sendCommand("motorsteuerung.bStop.bco=50712");
    }

    //itoa(valPWM2, sendString, 10);
  }
}
void sl_posPopCallback(void *ptr)
{
  uint32_t posval;
  sl_pos.getValue(&posval);
  set_pos = posval * 100;
}
void t0PushCallback(void *ptr) //MotorCurrent
{
  itoa(TLZ825B.getM1CurrentMilliamps(), sendString, 10);
  strcat(sendString, " mA");
  t0.setText(sendString);
}
void tPosPushCallback(void *ptr)
{
  pv_pos = 0;
  //tPos.setText("0");
}
void sl_KpPopCallback(void *ptr)
{
  uint32_t wertkp;
  sl_Kp.getValue(&wertkp);
  kp1 = ((double)(wertkp)) / 100;
  String send = String(kp1, 7);
  tKp.setText(send.c_str());
  updatePIDwerte();
}
void sl_KiPopCallback(void *ptr)
{
  uint32_t wertki;
  sl_Ki.getValue(&wertki);
  ki1 = ((double)(wertki)) / 1000;
  String xyz = String(ki1, 3);
  tKi.setText(xyz.c_str());
  updatePIDwerte();
}
void sl_KdPopCallback(void *ptr)
{
  uint32_t wertkd;
  sl_Kd.getValue(&wertkd);
  kd1 = ((double)(wertkd)) / 100;
  String xyz = String(kd1, 3);
  tKd.setText(xyz.c_str());
  updatePIDwerte();
}

void timerInterr()
{
  if (showGraph)
  {
    s0.addValue(0, pv_speed / 12);
    s0.addValue(1, set_speed / 12);
  }
  //pv_pos_send = int (pv_pos/100);
  //sl_pos.setValue(pv_pos_send);
}

void setup()
{
  Serial.begin(115200);

  // Timer Setup
  Timer1.initialize(500000); //X 000 Millisekunden
  Timer1.attachInterrupt(timerInterr);

  //Motor Thorlabs Z825B Initialisieren
  TLZ825B.init();

  //Display Setup
  HMISerial.begin(9600);
  nexInit();

  //PID Setup
  speedPID.SetMode(MANUAL);
  speedPID.SetSampleTime(25);
  speedPID.SetOutputLimits(-255, 255);

  posPID.SetMode(MANUAL);
  speedPID.SetSampleTime(25);
  posPID.SetOutputLimits(-2800, 2800);

  //Pins initialisieren
  pinMode(_LEDINT, OUTPUT);
  pinMode(_HallsenA, INPUT_PULLUP);
  pinMode(_HallsenB, INPUT_PULLUP);

  //AttachInterrupts
  bGraph.attachPop(bGraphPopCallback, &bGraph);
  bStop.attachPop(bStopPopCallback, &bStop);
  bPID.attachPop(bPIDPopCallback, &bPID);
  hPWM1.attachPop(hPWM1PopCallback, &hPWM1);
  hPWM2.attachPop(hPWM2PopCallback, &hPWM2);
  //tPos.attachPop(tPosPushCallback, &tPos);
  sl_pos.attachPop(sl_posPopCallback, &sl_pos);
  t0.attachPush(t0PushCallback, &t0);
  sl_Kp.attachPop(sl_KpPopCallback, &sl_Kp);
  sl_Ki.attachPop(sl_KiPopCallback, &sl_Ki);
  sl_Kd.attachPop(sl_KdPopCallback, &sl_Kd);

  attachInterrupt(digitalPinToInterrupt(_HallsenA), PinA, RISING);
  attachInterrupt(digitalPinToInterrupt(_HallsenB), PinB, RISING);

  previousMillis = millis();

  updatePIDwerte();
  checkLimitSw(); // Endschalter zu Beginn Prüfen
  TLZ825B.enableM1Driver();
  sendCommand("page start"); // Auf Startseite Wechseln
}

void loop()
{
  nexLoop(nex_listen_list); // Loop für Nextion
                            //Serial Output

  if (TLZ825B.getFWDstate() || TLZ825B.getREVstate()) //Bei  Motorbewegung Endschalter überachen und auf Motorfehler Prüfen
  {
    checkLimitSw();
    stopIfFault();
    if (ersteMessung)
    {
      Serial.println("ersteMessung");
      ersteMessung = 0;
      flanke = 0;
    }
    else if (flanke > 25)
    {
      for (int i = 0; i < flanke - 1; i++)
      {
        pulsAbstand += pulse[i + 1] - pulse[i];
      }
      pulsAbstand = pulsAbstand / (flanke - 1);
      flanke = 0;
      pulsAbstand_MittelW = pulsAbstand;
      pulsAbstand = 0;
    }
  }

  if (fehler)
  {
    pruefeFehler(); //Im Fehlerfall, prüfen ob Fehler behoben
  }

  if (pulsAbstand_MittelW != 0 && TLZ825B.getFWDstate())
  {
    pv_speed = (234375 / pulsAbstand_MittelW);
  }
  else if (pulsAbstand_MittelW != 0 && TLZ825B.getREVstate())
  {
    pv_speed = (-234375 / pulsAbstand_MittelW);
  }
  else
  {
    pv_speed = 0;
  }

  posPID.Compute();
  speedPID.Compute();

  currentMillis = millis(); //Timer Mit Millis
  if (currentMillis - previousMillis >= 20)
  {
    pv_pos_send = int(pv_pos/3.4);
    if (set_speed != 0)
    {
      TLZ825B.setM1Speed(pwm_pulse);
    }
    else
    {
      TLZ825B.setM1Speed(0);
    }
    if (Serial.available() <= 0 && (TLZ825B.getFWDstate() || TLZ825B.getREVstate()))
    {
      Serial.print("set_speed: ");
      Serial.print(set_speed);
      Serial.print("   ");
      Serial.print("speed: ");
      Serial.print(pv_speed);
      Serial.print("   ");
      Serial.print("PID: ");
      Serial.print(pwm_pulse);
      Serial.print("   ");
      Serial.print("setpos: ");
      Serial.print(set_pos);
      Serial.print("   ");
      Serial.print("Pos: ");
      Serial.println(pv_pos);
    }
    previousMillis = currentMillis;
  }
}
