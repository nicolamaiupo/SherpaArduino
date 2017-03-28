#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\checksum.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\aslaradio\mavlink.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\checksum.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\def.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\mavlink_conversions.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\mavlink_helpers.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\mavlink_types.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\protocol.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\RX.h"
#include "C:\Users\Nicola\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\types.h"

/*#include <checksum.h>
  #include <def.h>
  #include <mavlink_conversions.h>
  #include <mavlink_helpers.h>
  #include <mavlink_types.h>
  #include <protocol.h>
  #include <RX.h>
  #include <types.h>*/

// #include <LiquidCrystal.h>

// NOTES
// Read PWM from RC receiver (A8-A15) PINreceiver=[1,2,3,4,5,6] connected with PINarduino=[A11,A8,A10,A9,A12,A13]
// Map the data read to a PPM signal (from [1000,2000] to [1000,2000])
// Send mapped RC data to the PixHawk in PPM (Output pin 9)
// Read telemetry Telem1 from PixHawk (Serial 1 -- Arduino pin TX18-RX19) and write it out as it is to ODROID (Serial2 -- Arduino pin TX16 - RX17)
// Read Commands from ODROID (Serial2 -- Arduino pin TX16 - RX17) -- it sends MAVLINK_MSG_RC_CHANNEL_RAW messages.
// Parse the messages and if a button is pushed overwrite RC command with ODROID commands
// Read data from SONAR/DISTANCE SENSOR (Arduino pin 23 = ECHO, 25 = TRIG) and send MAVLINK_MSG_DISTANCE_SENSOR data.
// Read data from ARTVA (Serial3)


// #include "C:\Users\Administrator\Dropbox\Progetti\SHERPA\ARTVA\WASP_Arduino_rNM_20160920\aslaradio\mavlink.h"

// #define ARTVA_LOG
// #define USE_OSD
#define USE_RXSTD     //uncomment if you are using RC
#define USE_SONAR     //uncomment if you are using sonar/distance sensor
#define USE_LASER     // Added by Nicola on 2017/02/23
// #define USE_GPS
#define USE_ARTVA     //uncomment if you are using ARTVA with serial
// #define HIGH_SPEED_SERIAL_ODROID  //uncomment if we want to use high baud rate (500000) with ODROID & PX4
// #define USE_LCD       //uncomment if you are using LCD

#include <NewPing.h> // inserted by Nicola 20/06/2016 
// #include <HC_SR04.h> // inserted by Nicola 21/06/2016
#include <TimerFive.h>
// #include <TinyGPS++.h> // commented by Nicola 20/09/2016

//Mavlink
#ifndef PI
# define PI 3.141592653589793f
#endif
#ifndef M_PI_2
# define M_PI_2 1.570796326794897f

#endif
#ifndef atanf
# define atanf atan
#endif
#ifndef atan2f
# define atan2f atan2
#endif
#ifndef fabsf
# define fabsf abs
#endif

static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

#define SEC 1000000
#define HZ 100

#define P2B(x,y) x = s2foutbuf[y]; x = x <<8; x += s2foutbuf[y+1];
#define P0B(x,y) x = s0foutbuf[y]; x = x <<8; x += s0foutbuf[y+1];

long timer = 0; //general purpuse timer
long timer_old;

#define STATUS_LED 13

int status = LOW;
int counter = 0;
int n = 0;

#define ADC_HW_CHANNELS 16
#define ADC_CHANNELS 8
#define ADC_MAXCOUNT 1500

volatile uint8_t MuxSel = 8;
volatile uint8_t analog_reference;
volatile uint32_t analog_buffer[ADC_HW_CHANNELS];
volatile uint16_t analog_count[ADC_HW_CHANNELS];

//ADC VARIABLES
uint16_t AN[ADC_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0}; //array that store the 3 ADC filtered data (gyros)
uint16_t AN_OFFSET[ADC_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

//POLOLU SERVOS
#define MAXSERVO 8
//#define MINPULSE 2000
//#define MAXPULSE 4000
//#define ZEROPULSE 3000

//------------------------------------------
#define MIN_INPUT_VALUE 0
#define MAX_INPUT_VALUE 4096
#define ZERO_INPUT_VALUE (((MAX_INPUT_VALUE-MIN_INPUT_VALUE)/2) + MIN_INPUT_VALUE)

#define MIN_RC_VALUE 1000
#define MAX_RC_VALUE 2000
#define ZERO_RC_VALUE (((MAX_RC_VALUE-MIN_RC_VALUE)/2) + MIN_RC_VALUE)
#define PERCENT_THRESHOLD_RC_VALUE 15
#define UPPER_THRESHOLD_RC_VALUE 1700
#define LOWER_THRESHOLD_RC_VALUE 1300

#define MIN_PPM_VALUE 1000
#define MAX_PPM_VALUE 2000
#define ZERO_PPM_VALUE (((MAX_PPM_VALUE-MIN_PPM_VALUE)/2) + MIN_PPM_VALUE)

#define MIN_PKSV_VALUE 2000
#define MAX_PKSV_VALUE 4000
#define ZERO_PKSV_VALUE 3000

#define ALPHA_LP_SONAR 0.75      //low pass filter for sonar --> higher value higher band
//------------------------------------------

int degpos[MAXSERVO] = {0, 0, 0, 0, 0, 0, 0, 0};
int servopos = MIN_PKSV_VALUE;
//Vettore per il joystick
double vettore[2] = {0, 0}; //0-360 (angolo), 0-45 (modulo)

int jx = 0;
int jy = 0;
int jz = 0;
int jm = 0;

//#define MINPKSVAL 0 //0-4096
//#define MAXPKSVAL 4096 //0-4096
//#define ZEROPKVAL (MAXPKSVAL/2)
int pksv[MAXSERVO + 1] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int l_pksv[MAXSERVO + 1] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int pksv_artva = 0;
//#define FAILSAFEMAX 50
//int failsafe = 0;
#define FAILSAFEMAXMILLIS 500
long tmrfs = 0;
bool flgFailsafe = false;

//#define SENDCMDMINTIME 50 //-- OK VALUE
#define SENDCMDMINTIME 40
//bool flgSendCommand = false;
long tmrlastcmd = 0;

#define MOTORSAT 3200
#define MOTORGAIN 0.7

#define SENDTELEMETRYMINTIME 120
float AIdT = SENDTELEMETRYMINTIME / 1000.0;

long tmrlasttelemetry = 0;
long tmrcurrtelemetry = 0;

//int cTelemetry = 0;
int cRadio = 0;
int cTelem = 0;

float uz_integ = 0;
float ux_integ = 0;

//Failsafe 1sec (1000ms)
#define FAILSAFE_THR 1000
unsigned long lastRadio = 0;

//TimerOne/TimerThree helper variables
long period = 10000;      //2600;      // the period in microseconds
long period2 = period;   //long period2 = 20000; //5200*6;
int prescale[] = {0, 1, 8, 64, 256, 1024}; // the range of prescale values

//Servo campitch;
//Servo camzoom;
boolean backToHome = false;

uint16_t buttons = 0;
#define BUTTONS_START 30
#define BUTTONS_MAX 12

#define BTN0 30
#define LED0 31
//Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//Read Frame
int status_st = 0;
#define FRAME_LEN 8
uint16_t frame[FRAME_LEN];
uint8_t* buf;

#define MODE_1 1
#define MODE_3 3
int radio_mode_type = MODE_1;

boolean motor_arm = false;
boolean axis_exp = false;
boolean axis_half = false;
float tmpexp = 0;
#define AXIS_EXP 2 //3.4

//Mavlink message
mavlink_gps_raw_int_t msg_pos = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //OK MINIMOSD
mavlink_heartbeat_t msg_hb = {0, 2, 0, 0, 4, 3}; //3rd pos=3 for arducopter autopilot type  //OK MINIMOSD
mavlink_vfr_hud_t msg_hud = {0, 0, 0, 0, 0, 0}; //OK MINIMOSD
mavlink_rc_channels_raw_t msg_radio = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //OK MINIMOSD
mavlink_sys_status_t msg_status = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //OK MINIMOSD
//mavlink_statustext_t msg_txtstatus; //NO MINIMOSD - teniamolo cmq per reference manipolazione stringhe

mavlink_distance_sensor_t msg_sonar = {0, 20, 300, 0, 1, 1, 0, 0}; //SONAR     350 is the max distance considered valid
mavlink_debug_vect_t msg_artva = {0, 0, 0, 0, 0, 0, 0, 0}; //ARTVA-ABUSATO
mavlink_message_t toSend;
uint8_t txbuf[1024];
char sonar_buf[4];
uint8_t index_sonar_buf = 0;
int psize = 0;
int sendsize = 0;

#ifdef USE_GPS  // commented by Nicola 2016/09/20
TinyGPSPlus gps;// commented by Nicola 2016/09/20
#endif          // commented by Nicola 2016/09/20
#define SerialGPS   Serial
#define SerialDEBUG Serial
#define SerialARTVA Serial3

//---------------- Telem1 ------------------
#define SerialTELEM_AP Serial1
#define SerialTELEM_ODROID Serial2
//#define SerialPC Serial3                  // Used for simulating the arrive of a MAVLINK_MSG_RC_CHANNEL_RAW. The message is created using simulink and the PixHawk COM utility
#define FRAMEODROID_LENGTH 8
//------------------------------------------

boolean artvaDownFlag = false;
boolean artvaUpFlag = false;
long lastArtva = 0;
#define ARTVAZERO ZERO_PKSV_VALUE+46
boolean lastUp = true;

//RX e PPM
int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little

#ifdef USE_RXSTD
#include "RX.h"
static uint16_t rcTime  = 0;
static int16_t initialThrottleHold;
int16_t rc;
int32_t prop = 0;
#endif

// ------------------------- Control from ODROID ------------------------
mavlink_rc_channels_override_t cmdsFromOdroid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
mavlink_rc_channels_raw_t cmdsFromOdroidRaw = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
mavlink_message_t rcvmsgFromOdroid;
uint16_t frameOdroid[FRAMEODROID_LENGTH];
bool button_ODROID_pushed = false;
bool button_ODROID_bridge = false;
unsigned long lastOdroidReceived = 0;
// ----------------------------------------------------------------------

#ifdef USE_LCD
// include the library code:
// #include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(27, 4, 3, 29, 2, 31); // lcd(12, 11, 5, 4, 3, 2);
#endif

#ifdef ARTVA_LOG
  //SD Library includes
  #include <SPI.h>
  #include <SD.h>
  const int chipSelect = 4;
  #define USE_ARTVA
#endif

#ifdef USE_ARTVA
  //Artva Read Frame
  boolean startframe = false;
  boolean fillframe = false;
  int framepos = 0;
  //Artva Variables
  int framebytes = 0;
  byte framebuffer[32];
  byte artvaframe[32];
  boolean artvavalid = 0;
  int _distanceOne = 0;
  int _distanceTwo = 0;
  int _distanceThree = 0;
  int _distanceFour = 0;
  int _angleOne = 0;
  int _angleTwo = 0;
  int _angleThree = 0;
  int _angleFour = 0;
  int _transmitterDetected = 0;
  int _frameCounter = 0;
  boolean flagArtvaInterval = false;
  // FILTER // added by Nicola on 2016/11/01
  float _distanceOne_fltrd = -1;
  float _angleOne_fltrd = 0;
  int n_FIFO = 10;
  int ARTVA_FIFO_d[10];
  int ARTVA_FIFO_a[10];
  float _distanceOne_mean;
  float _angleOne_mean;
  float _distanceOne_temp;
  float _angleOne_temp;
  boolean FIFO_OK = false;
  float _distanceOne_std;
  float _angleOne_std;
  float _distance_th = 500;
  float _angle_th = 45;
  int _distanceOne_old = -1;
  int _angleOne_old = 0;
  // END FILTER
#endif

#ifdef USE_SONAR // added by Nicola 16/06/2016
// setup sonar // added by Nicola 16/06/2016
#define echoPin 23 // Echo Pin // added by Nicola 20/09/2016
#define trigPin 25 // Trigger Pin // added by Nicola 20/09/2016
// #define ECHO_INT 0 // added by Nicola 21/09/2016
int maximumRange = 300; // Maximum range needed // added by Nicola 16/06/2016
int minimumRange = 20; // Minimum range needed // added by Nicola 16/06/2016
long duration; // added by Nicola 16/06/2016
long distance; // Duration used to calculate distance // added by Nicola 16/06/2016
// unsigned long CurrentTime = millis();  // added by Nicola 16/06/2016
// unsigned long LastTime = millis(); // added by Nicola 16/06/2016
// NewPing sonar(trigPin, echoPin, maximumRange); // NewPing setup of pins and maximum distance. // added by Nicola 20/09/2016
// HC_SR04 sensor(trigPin, echoPin, ECHO_INT);
// unsigned int pingSpeed = 199; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second. // added by Nicola 20/09/2016
// unsigned long pingTimer;     // Holds the next ping time. // added by Nicola 20/09/2016
// boolean new_distance = false; // added by Nicola 20/09/2016
// ADDED BY NICOLA 20/09/2016
//void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
//  // Don't do anything here!
//  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
//    // Here's where you can add code.
//    distance = sonar.ping_result / US_ROUNDTRIP_CM;
//    // new_distance = true;
//    // Serial.print("Ping: ");
//    // Serial.print(sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
//    // Serial.println("cm");
//  }
//  // Don't do anything here!
//}
#endif // added by Nicola 16/06/2016

#ifdef USE_LASER // added by Nicola on 2017/02/22
long pausa_stampa_laser = 0;
//  bool flag_aggiornamento_laser = false;
int pwm_value = 0;
//  volatile int prev_time = 0;
//  const byte interruptPin = 3;
long d_laser;
long d_laser_max = 4000; // Maximum range needed
long d_laser_min = 20; // Minimum range needed
mavlink_distance_sensor_t msg_laser = {0, 20, 4000, 0, 0, 2, 0, 0}; //LASER     4000 is the max distance considered valid
#endif

void setup()
{
#ifdef USE_LASER // added by Nicola on 2017/02/22
  pinMode(4, OUTPUT); // Set pin 4 as trigger pin
  digitalWrite(4, LOW); // Set trigger LOW for continuous read
  pinMode(3, INPUT); // Set pin 3 as monitor pin
  // attachInterrupt(digitalPinToInterrupt(interruptPin), rising, RISING);
#endif

#ifdef USE_LCD
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
#endif
#ifdef USE_OSD
  SerialOSD.begin(57600);
#endif
#ifdef USE_GPS                                   // commented by Nicola 2016/09/20
  SerialGPS.begin(57600); //GPS a 57600 direi    // commented by Nicola 2016/09/20
#endif                                           // commented by Nicola 2016/09/20
#ifdef USE_SONAR
  // SerialSONAR.begin(9600); //Sonar a 9600 // commented by Nicola 20/09/2016
  pinMode(trigPin, OUTPUT); // added by Nicola 16/06/2016
  pinMode(echoPin, INPUT); // added by Nicola 16/06/2016
  // pingTimer = millis(); // Start now. // added by Nicola 20/09/2016
  // sensor.begin();
  // sensor.start();
#endif
#ifdef USE_ARTVA
  SerialARTVA.begin(9600); //ARTVA a 38400 //new @ 9600
  SerialDEBUG.begin(38400);
#else
  SerialDEBUG.begin(38400); //Debug
  SerialDEBUG.print("STARTING...");
  SerialDEBUG.println();
#endif

  //---------------- Telem1 ------------------
#ifdef HIGH_SPEED_SERIAL_ODROID
  SerialTELEM_AP.begin(500000);
  SerialTELEM_ODROID.begin(500000);
#else
  SerialTELEM_AP.begin(57600);
  SerialTELEM_ODROID.begin(57600);
#endif
  //SerialPC.begin(57600);
  //------------------------------------------

  Analog_Reference(DEFAULT);     //Setting 5V of voltage reference for the ADC conversion
  Analog_Init();                 //Setting ADC

  delay(100);
  //ads.begin();
  buf = (uint8_t*)&frame[0]; //Importantissimo!!

  delay(200);
  //digitalWrite(STATUS_LED,HIGH);
  Read_adc_raw();     // ADC initialization
  timer = micros();
  delay(1);
  tmrlastcmd = millis();
  tmrlasttelemetry = millis();
  lastRadio = 0;
  //while (SerialXBEE.available()>0){
  //  uint8_t fl = SerialXBEE.read();
  //}
#ifdef USE_RXSTD
  //RX tradizionale
  configureReceiver();
  rcTime = 0;
#endif
  setupPPMGen();

  //  SerialDEBUG.print("STARTED"); // Commented by Nicola 20/09/2016
  //  SerialDEBUG.println();        // Commented by Nicola 20/09/2016
  lastArtva = millis();
  lastOdroidReceived = millis();
}

mavlink_message_t rcvmsg;
mavlink_status_t rcvmstatus;
mavlink_commands_t cmds = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t c = 0;

uint16_t sonardistance = 0;


/*this array holds the servo values for the ppm signal
  change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[MAXSERVO];
int16_t tmpval = 0;

int debugCount = 0;
int odroidDebugCounter = 0;


//#ifdef USE_LASER // added by Nicola on 2017/02/22
//void rising() {
//  attachInterrupt(digitalPinToInterrupt(interruptPin), falling, FALLING);
//  // salvo istante del fronte di salita
//  prev_time = micros();
//  // SerialDEBUG.println("rising");
//}
//
//void falling() {
//  attachInterrupt(digitalPinToInterrupt(interruptPin), rising, RISING);
//  // setto il flag di aggiornamento
//  flag_aggiornamento_laser = true;
//  // salvo il periodo del fronte positivo
//  pwm_value = micros() - prev_time;
//  // il resto dell'elaborazione viene effettuato all'interno del loop(), in modo temporizzato...
//}
//#endif


void loop() //Main Loop Loop Loop Loop Loop Loop Loop Loop Loop Loop Loop
{

  //------------------------------ GPS --------------------------------------
  // commented by Nicola 2016/09/20
#ifdef USE_GPS              //NOT USED ON WASP
  //Read GPS DATA
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    gps.encode(c);
    //SerialDEBUG.print(rb);
  }
#endif
  //--------------------------------------------------------------------------


  //------------------------------ ARTVA --------------------------------------
#ifdef USE_ARTVA
  ArtvaGPSCheckNewData();
  if (flagArtvaInterval) {
    SerialDEBUG.print("t [ms]:\t");SerialDEBUG.print(millis());SerialDEBUG.print("\t ARTVA read 1 - DIST:\t");SerialDEBUG.print(_distanceOne);  SerialDEBUG.print("\t ANG:\t"); SerialDEBUG.println(_angleOne);
    SerialDEBUG.print("t [ms]:\t");SerialDEBUG.print(millis());SerialDEBUG.print("\t ARTVA read 2 - DIST:\t");SerialDEBUG.print(_distanceTwo);  SerialDEBUG.print("\t ANG:\t"); SerialDEBUG.println(_angleTwo);
    SerialDEBUG.print("t [ms]:\t");SerialDEBUG.print(millis());SerialDEBUG.print("\t ARTVA read 3 - DIST:\t");SerialDEBUG.print(_distanceThree);SerialDEBUG.print("\t ANG:\t"); SerialDEBUG.println(_angleThree);
    SerialDEBUG.print("t [ms]:\t");SerialDEBUG.print(millis());SerialDEBUG.print("\t ARTVA read 4 - DIST:\t");SerialDEBUG.print(_distanceFour); SerialDEBUG.print("\t ANG:\t"); SerialDEBUG.println(_angleFour);
    msg_artva.name[0] = 'a';
    if (_distanceOne > 0) {
      msg_artva.time_usec = (unsigned int)_distanceOne;
      msg_artva.x = (float)_angleOne;
      //SerialDEBUG.print("ARTVA read 1: distance ");SerialDEBUG.print((int)msg_artva.time_usec);
      //SerialDEBUG.print("m, direction ");SerialDEBUG.println(msg_artva.x);
    } else {
      msg_artva.time_usec = 0;
      msg_artva.x = 0;
    }
    if (_distanceTwo > 0) {
      msg_artva.y = (float)_distanceTwo;
      msg_artva.z = (float)_angleTwo;
      //SerialDEBUG.print("ARTVA read 2: distance ");SerialDEBUG.print(msg_artva.y);
      //SerialDEBUG.print("m, direction ");SerialDEBUG.println(msg_artva.z);
    } else {
      msg_artva.y = 0;
      msg_artva.z = 0;
    }
    psize = mavlink_msg_debug_vect_encode(1, 1, &toSend, &msg_artva);
    sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
    for (int i = 0; i < sendsize; i++) {
      SerialTELEM_ODROID.write(txbuf[i]);
    }
    msg_artva.name[0] = 'b';
    if (_distanceThree > 0) {
      msg_artva.time_usec = (unsigned int)_distanceThree;
      msg_artva.x = (float)_angleThree;
      //SerialDEBUG.print("ARTVA read 3: distance ");SerialDEBUG.print((int)msg_artva.time_usec);
      //SerialDEBUG.print("m, direction ");SerialDEBUG.println(msg_artva.x);
    } else {
      msg_artva.time_usec = 0;
      msg_artva.x = 0;
    }
    if (_distanceFour > 0) {
      msg_artva.y = (float)_distanceFour;
      msg_artva.z = (float)_angleFour;
      //SerialDEBUG.print("ARTVA read 4: distance ");SerialDEBUG.print(msg_artva.y);
      //SerialDEBUG.print("m, direction ");SerialDEBUG.println(msg_artva.z);
    } else {
      msg_artva.y = 0;
      msg_artva.z = 0;
    }
    psize = mavlink_msg_debug_vect_encode(1, 1, &toSend, &msg_artva);
    sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
    for (int i = 0; i < sendsize; i++) {
      SerialTELEM_ODROID.write(txbuf[i]);
    }

    //// START FILTER ////
    // Manage FIFO
    if ( (_distanceOne > 300 and (_distanceOne != _distanceOne_old) and (_angleOne != _angleOne_old)) or (_distanceOne <= 300 and (_distanceOne != _distanceOne_old)) )
      // if ((_distanceOne != _distanceOne_old) or (_angleOne != _angleOne_old))
    {
      _distanceOne_old = _distanceOne;
      _angleOne_old = _angleOne;

      for (int i = 0; i < n_FIFO - 1; i++)
      {
        ARTVA_FIFO_d[n_FIFO - 1 - i] = ARTVA_FIFO_d[n_FIFO - i - 2];
        ARTVA_FIFO_a[n_FIFO - 1 - i] = ARTVA_FIFO_a[n_FIFO - i - 2];
      }
      ARTVA_FIFO_d[0] = _distanceOne;
      ARTVA_FIFO_a[0] = _angleOne;
      // FIFO Check
      FIFO_OK = true;
      for (int i = 0; i < n_FIFO; i++)
      {
        if (ARTVA_FIFO_d[i] > 0)
        {
          FIFO_OK = FIFO_OK and true;
        }
        else
        {
          FIFO_OK = FIFO_OK and false;
        }
      }
      //SerialDEBUG.println("FIFO a");
      //for (int i=0; i < n_FIFO; i++)
      //{
      //SerialDEBUG.println(ARTVA_FIFO_a[i]);
      //}
      // SerialDEBUG.print("IS FIFO OK ?");SerialDEBUG.println(FIFO_OK);
      if (FIFO_OK)
      {
        // Mean
        _distanceOne_mean = 0.0;
        _angleOne_mean = 0.0;
        for (int i = 0; i < n_FIFO; i++)
        {
          _distanceOne_mean = _distanceOne_mean + ARTVA_FIFO_d[i];
          _angleOne_mean = _angleOne_mean + ARTVA_FIFO_a[i];
        }
        _distanceOne_mean = _distanceOne_mean / (float) n_FIFO;
        _angleOne_mean = _angleOne_mean / (float) n_FIFO ;
        // STD deviation
        _distanceOne_temp = 0;
        _angleOne_temp = 0;
        for (int i = 0; i < n_FIFO; i++)
        {
          _distanceOne_temp = _distanceOne_temp + pow(ARTVA_FIFO_d[i] - _distanceOne_mean, 2);
          _angleOne_temp    = _angleOne_temp   + pow(ARTVA_FIFO_a[i] - _angleOne_mean   , 2);
        }
        _distanceOne_std = sqrt(_distanceOne_temp / (float) (n_FIFO - 1));
        _angleOne_std = sqrt(_angleOne_temp / (float) (n_FIFO - 1));
        // STD Check
        if (_distanceOne_std < _distance_th and _angleOne_std < _angle_th)
        { // STD_check_OK = true;
          _distanceOne_fltrd = _distanceOne_mean;
          _angleOne_fltrd = _angleOne_mean;
        }
        else
        { // STD_check_OK = false;
          _distanceOne_fltrd = -1;
          _angleOne_fltrd = 0;
        }
      }
      else
      {
        _distanceOne_fltrd = -1;
        _angleOne_fltrd = 0;
      }
    }
    //// END FILTER ////

    // LCD PRINT
#ifdef USE_LCD
    lcd.clear();
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    // As it is by the sensor
    lcd.setCursor(0, 0);
    lcd.print("AS IT IS");
    lcd.setCursor(0, 1);
    lcd.print("R1:");
    lcd.print(_distanceOne);
    lcd.setCursor(20, 0);
    lcd.print("d1:");
    lcd.print(_angleOne);
    // Filtered
    lcd.setCursor(11, 0);
    lcd.print("FILTERED");
    lcd.setCursor(11, 1);
    lcd.print("R1:");
    lcd.print(_distanceOne_fltrd);
    lcd.setCursor(31, 0);
    lcd.print("d1:");
    lcd.print(_angleOne_fltrd);
    lcd.setCursor(20, 1);
    /*lcd.print("Rm:");
      lcd.print(_distanceOne_mean);
      lcd.setCursor(31, 1);
      lcd.print("Rv:");
      lcd.print(_distanceOne_std);*/
    lcd.setCursor(20, 1);
    lcd.print("dm:");
    lcd.print(_angleOne_mean);
    lcd.setCursor(31, 1);
    lcd.print("dv:");
    lcd.print(_angleOne_std);
    /*        lcd.setCursor(0, 0);
              lcd.print("R1:");
              lcd.print(_distanceOne);
              lcd.setCursor(0, 1);
              lcd.print("d1:");
              lcd.print(_angleOne);
              lcd.setCursor(11, 0);
              lcd.print("R2:");
              lcd.print(_distanceTwo);
              lcd.setCursor(11, 1);
              lcd.print("d2:");
              lcd.print(_angleTwo);
              lcd.setCursor(20, 0);
              lcd.print("R3:");
              lcd.print(_distanceThree);
              lcd.setCursor(20, 1);
              lcd.print("d3:");
              lcd.print(_angleThree);
              lcd.setCursor(31, 0);
              lcd.print("R4:");
              lcd.print(_distanceFour);
              lcd.setCursor(31, 1);
              lcd.print("d4:");
              lcd.print(_angleFour); */
    // END LCD PRINT
#endif
    flagArtvaInterval = false;
  }
#endif
  // ------------------------------------------------------------

  // --------------------- SONAR --------------------------------
  // commented by Nicola 20/09/2016
  /*
    #ifdef USE_SONAR
    while (SerialSONAR.available()>0)
    {
      //Bridge ARTVA->XBEE
      c = SerialSONAR.read();
      if (c == 'R')
      {
        index_sonar_buf = 0;
      }
      else
      {
        if (index_sonar_buf == 3)
        {                  //new sonar data
          sonar_buf[index_sonar_buf] = '\0';        //finished sonar string
          sonardistance = atoi(sonar_buf)*(1-ALPHA_LP_SONAR)+ALPHA_LP_SONAR*sonardistance;          //low pass filter  input*(1-alpha)+alpha*old_measure
          //SerialDEBUG.print("Sonar: ");
          //SerialDEBUG.println(sonardistance);
          msg_sonar.time_boot_ms = millis();
          msg_sonar.current_distance = sonardistance;
          psize = mavlink_msg_distance_sensor_encode(1,1,&toSend,&msg_sonar);
          sendsize = mavlink_msg_to_send_buffer(txbuf,&toSend);
          for(int i = 0; i<sendsize;i++)
          {
            SerialTELEM_ODROID.write(txbuf[i]);
          }
        }
        else
        {
          sonar_buf[index_sonar_buf] = c;
          index_sonar_buf++;
        }
      }
    }
    #endif
  */
  // HC-S04 // added by Nicola 16/06/2016
#ifdef USE_SONAR
  // BLOCKING VERSION, Commented by Nicola 20/09/2016
  //  CurrentTime = millis();
  //  if (CurrentTime - LastTime > 100) // to force the execution at 10 Hz maximum
  //   {
  //       SerialDEBUG.println("");
  //       SerialDEBUG.print(" DT: ");
  //       SerialDEBUG.print(CurrentTime - LastTime);
  //       LastTime = CurrentTime;
        /* The following trigPin/echoPin cycle is used to determine the
         distance of the nearest object by bouncing soundwaves off of it. */
         digitalWrite(trigPin, LOW);
         delayMicroseconds(2);
  
         digitalWrite(trigPin, HIGH);
         delayMicroseconds(10);
  
         digitalWrite(trigPin, LOW);
         duration = pulseIn(echoPin, HIGH, 10000);
  
         //Calculate the distance (in cm) based on the speed of sound.
         distance = duration/58.2;
  //
  //       if (distance >= maximumRange || distance <= minimumRange){
  //       /* Out of Range */
  //       SerialDEBUG.print(" DX: OutOfRange");
  //       }
  //       else {
  //       /* In range */
  //       SerialDEBUG.print(" DX: ");
  //       SerialDEBUG.print(distance);
  //       SerialDEBUG.println(" -------- ");
  //       }
  //   }
  // NEW NON BLOCKING VERSION, Added by Nicola 20/09/2016
  //  if(sensor.isFinished()){
  //      distance = sensor.getRange();
//  if (millis() >= pingTimer)
//  { // pingSpeed milliseconds since last ping, do another ping.
//    pingTimer += pingSpeed;      // Set the next ping time.
//    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
    if (distance >= maximumRange || distance <= minimumRange)
    {
      /* Out of Range */
      // SerialDEBUG.print("SONAR: OutOfRange");
      msg_sonar.time_boot_ms = millis();
      msg_sonar.current_distance = -1;
      psize = mavlink_msg_distance_sensor_encode(1, 1, &toSend, &msg_sonar);
      sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
      for (int i = 0; i < sendsize; i++)
      {
        SerialTELEM_ODROID.write(txbuf[i]);
      }
    }
    else
    {
      /* In range */
      // SerialDEBUG.print("t [ms]:\t");SerialDEBUG.print(millis());SerialDEBUG.print("\t R [cm]:\t");SerialDEBUG.println(distance);
      msg_sonar.time_boot_ms = millis();
      msg_sonar.current_distance = distance;
      psize = mavlink_msg_distance_sensor_encode(1, 1, &toSend, &msg_sonar);
      sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
      for (int i = 0; i < sendsize; i++)
      {
        SerialTELEM_ODROID.write(txbuf[i]);
      }
    }
//  }
#endif
  // ------------------------------------------------------------

  // -------------- TELEMETRY PIXHAWK-->ODROID ------------------
  // Read Telem1 from PIXHAWK and send to ODROID
  while (SerialTELEM_AP.available() > 0) {
    int inTelem1 = SerialTELEM_AP.read();
    //SerialDEBUG.write(inTelem1);
    SerialTELEM_ODROID.write(inTelem1);
  }
  // ------------------------------------------------------------



  // -------------------- Read RC Commands ------------------------
#ifdef USE_RXSTD
  //RC tradizionale
  currentTime = micros();
  if ((int16_t)(currentTime - rcTime) > 0 ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();
    // ------------------ STICKS COMMAND HANDLER --------------------

    // checking sticks positions
    uint8_t stTmp = 0;
    for (int i = 0; i < 4; i++) {
      stTmp >>= 2;
      if (rcData[i] > MINCHECK) stTmp |= 0x80;     // check for MIN
      if (rcData[i] < MAXCHECK) stTmp |= 0x40;     // check for MAX
    }
    /*
      //Inverto pitch e throttle
      tmpval = rcData[0];
      rcData[0] = rcData[2];
      rcData[2] = tmpval;
    */
    /*
      //Qui dovrei giÃ  avere rcData da spedire fuori in PPM
      for(int i=0;i<RC_CHANS-4;i++){
      rcData[i] = constrain(rcData[i], MIN_RC_VALUE, MAX_RC_VALUE);    // Necessary because rcData is a uint variable. Using the map function I cannot obtain a negative number but a nonsense number
      frame[i] = map(rcData[i], MIN_RC_VALUE, MAX_RC_VALUE, MIN_INPUT_VALUE, MAX_INPUT_VALUE);
      }
      frame[4] = ZERO_INPUT_VALUE;
      frame[5] = ZERO_INPUT_VALUE;
      //costruisco bottoni su frame[6] usando rcData[4] e rcData[5] e rcData[6] e rcData[7]
      //valore neutro = nessun bottone, valore alto/basso bottone x/y premuto
      frame[6] = 0;
      if (rcData[4]< int(MIN_RC_VALUE+(MAX_RC_VALUE*(PERCENT_THRESHOLD_RC_VALUE/100)))){
      frame[6] = frame[6] | 0x08;
      }
      if (rcData[4]> int(MAX_INPUT_VALUE-(MAX_INPUT_VALUE*(PERCENT_THRESHOLD_RC_VALUE/100)))){
      frame[6] = frame[6] | 0x10;
      }
      if (rcData[5]< int(MIN_RC_VALUE+(MAX_RC_VALUE*(PERCENT_THRESHOLD_RC_VALUE/100)))){
      frame[6] = frame[6] | 0x20;
      }
      if (rcData[5]> int(MAX_INPUT_VALUE-(MAX_INPUT_VALUE*(PERCENT_THRESHOLD_RC_VALUE/100)))){
      frame[6] = frame[6] | 0x40;
      }
      if (rcData[6]< int(MIN_RC_VALUE+(MAX_RC_VALUE*(PERCENT_THRESHOLD_RC_VALUE/100)))){
      frame[6] = frame[6] | 0x80;
      }
      if (rcData[6]> int(MAX_INPUT_VALUE-(MAX_INPUT_VALUE*(PERCENT_THRESHOLD_RC_VALUE/100)))){
      frame[6] = frame[6] | 0x100;
      }
      if (rcData[7]< int(MIN_RC_VALUE+(MAX_RC_VALUE*(PERCENT_THRESHOLD_RC_VALUE/100)))){
      frame[6] = frame[6] | 0x200;
      }
      if (rcData[7]> int(MAX_INPUT_VALUE-(MAX_INPUT_VALUE*(PERCENT_THRESHOLD_RC_VALUE/100)))){
      frame[6] = frame[6] | 0x400;
      }
    */

    for (int i = 0; i < RC_CHANS - 2; i++) {
      rcData[i] = constrain(rcData[i], MIN_RC_VALUE, MAX_RC_VALUE);    // Necessary because rcData is a uint variable. Using the map function I cannot obtain a negative number but a nonsense number
      frame[i] = map(rcData[i], MIN_RC_VALUE, MAX_RC_VALUE, MIN_INPUT_VALUE, MAX_INPUT_VALUE);
    }
    //frame[RC_CHANS-2] = ZERO_INPUT_VALUE;
    //frame[RC_CHANS-1] = ZERO_INPUT_VALUE;

    //costruisco bottoni su frame[6] (FRAME_LEN-2) usando rcData[4] e rcData[5] e rcData[6] e rcData[7]
    //valore neutro = nessun bottone, valore alto/basso bottone x/y premuto
    frame[FRAME_LEN - 2] = 0;
    if (rcData[4] < int(MIN_RC_VALUE + (MAX_RC_VALUE * (PERCENT_THRESHOLD_RC_VALUE / 100)))) {
      frame[FRAME_LEN - 2] = frame[FRAME_LEN - 2] | 0x08;
    }
    if (rcData[4] > int(MAX_INPUT_VALUE - (MAX_INPUT_VALUE * (PERCENT_THRESHOLD_RC_VALUE / 100)))) {
      frame[FRAME_LEN - 2] = frame[FRAME_LEN - 2] | 0x10;
    }
    if (rcData[5] < int(MIN_RC_VALUE + (MAX_RC_VALUE * (PERCENT_THRESHOLD_RC_VALUE / 100)))) {
      frame[FRAME_LEN - 2] = frame[FRAME_LEN - 2] | 0x20;
    }
    if (rcData[5] > int(MAX_INPUT_VALUE - (MAX_INPUT_VALUE * (PERCENT_THRESHOLD_RC_VALUE / 100)))) {
      frame[FRAME_LEN - 2] = frame[FRAME_LEN - 2] | 0x40;
    }
    if (rcData[6] < int(MIN_RC_VALUE + (MAX_RC_VALUE * (PERCENT_THRESHOLD_RC_VALUE / 100)))) {
      frame[FRAME_LEN - 2] = frame[FRAME_LEN - 2] | 0x80;
    }
    if (rcData[6] > int(MAX_INPUT_VALUE - (MAX_INPUT_VALUE * (PERCENT_THRESHOLD_RC_VALUE / 100)))) {
      frame[FRAME_LEN - 2] = frame[FRAME_LEN - 2] | 0x100;
    }
    if (rcData[7] < int(MIN_RC_VALUE + (MAX_RC_VALUE * (PERCENT_THRESHOLD_RC_VALUE / 100)))) {
      frame[FRAME_LEN - 2] = frame[FRAME_LEN - 2] | 0x200;
    }
    if (rcData[7] > int(MAX_INPUT_VALUE - (MAX_INPUT_VALUE * (PERCENT_THRESHOLD_RC_VALUE / 100)))) {
      frame[FRAME_LEN - 2] = frame[FRAME_LEN - 2] | 0x400;
    }

    debugCount++;
    if (debugCount & 0x10) {
      /*SerialDEBUG.print("rcData: ");
        SerialDEBUG.print("0: ");
        SerialDEBUG.print(rcData[0]);
        SerialDEBUG.print(" 1: ");
        SerialDEBUG.print(rcData[1]);
        SerialDEBUG.print(" 2: ");
        SerialDEBUG.print(rcData[2]);
        SerialDEBUG.print(" 3: ");
        SerialDEBUG.print(rcData[3]);
        SerialDEBUG.print(" 4: ");
        SerialDEBUG.print(rcData[4]);
        SerialDEBUG.print(" 5: ");
        SerialDEBUG.print(rcData[5]);
        SerialDEBUG.print(" 6: ");
        SerialDEBUG.print(rcData[6]);
        SerialDEBUG.print(" 7: ");
        SerialDEBUG.println(rcData[7]);
        SerialDEBUG.println("-------");*/
      debugCount = 0;
    }
    /* //Serial.println();
      SerialDEBUG.print("frame: ");
      SerialDEBUG.print("0: ");
      SerialDEBUG.print(frame[0]);
      SerialDEBUG.print(" 1: ");
      SerialDEBUG.print(frame[1]);
      SerialDEBUG.print(" 2: ");
      SerialDEBUG.print(frame[2]);
      SerialDEBUG.print(" 3: ");
      SerialDEBUG.print(frame[3]);
      SerialDEBUG.print(" 4: ");
      SerialDEBUG.println(frame[4]);
      SerialDEBUG.println("-------");
    */
  } //END IF RC!
#else
  //RC simulated from XBEE
  /*while (SerialXBEE.available()>0){
    c = SerialXBEE.read();
    //Serial.print(c, HEX);
    //Serial.print(" ");
    //    Serial
    //Serial.print("PARSE_START: ");
    //Serial.println(millis());
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &rcvmsg, &rcvmstatus)) {
      // Handle message
      //Serial.print("PACKET_RCV: ");
      //Serial.println(millis());
      switch(rcvmsg.msgid) {
        case MAVLINK_MSG_ID_COMMANDS:
          //Decode mavlink message
          //Serial.print("DECODE_START: ");
          //Serial.println(millis());
          mavlink_msg_commands_decode(&rcvmsg, &cmds);
          //Serial.print("DECODE_END: ");
          //Serial.println(millis());
          frame[0] = cmds.LSX;
          frame[1] = cmds.LSY;
          frame[2] = cmds.RSX;
          frame[3] = cmds.RSY;
          frame[4] = cmds.BSX;
          frame[5] = cmds.BSY;
          frame[6] = cmds.Buttons; //frame[FRAME_LEN-2];
          //Activate FLAGS
          flgSendCommand = true;
          lastRadio = millis();
          digitalWrite(STATUS_LED,HIGH);
          break;
        default:
        SerialDebug.println(rcvmsg.msgid);
          //Do nothing
          break;
      }
    }
    //Serial.print("PARSE_END: ");
    //Serial.println(millis());
    }*/
#endif
  //-------------------------------------------------------------------


  // -------------------------- FAILSAFE ------------------------------
  // CACCIA: safety - if arduino does not receive commands from the receiver - to be tested
  if ((millis() - lastRCCommand > FAILSAFE_THR)) { //&&(rcData[0]+rcData[1]+rcData[2]+rcData[3] == 0)){
    //SerialDEBUG.println("Entrato: Perdita RC");
    //SerialDEBUG.print(millis());
    //SerialDEBUG.print(" - ");
    //SerialDEBUG.print(lastRCCommand);
    //SerialDEBUG.print(" - ");
    //SerialDEBUG.println("FAILSAFE!");
    for (int i = 0; i < FRAME_LEN - 2; i++) {
      frame[i] = 2048; //Joystick center!  No roll/pitch,yaw. Flight mode: loiter.
      //TODO: Gas basso???ug
    }
    frame[2] = 1500; //Throttle less than 50%
    //frame[FRAME_LEN-2] = 128+1024; //19; //Back-to-Home + GPSHold + Motori attivi
    digitalWrite(STATUS_LED, LOW);
    //Serial.print("FS:");
    //Serial.print(frame[3]);
    flgSendCommand = true;
  }

  /*
    //if (lastRadio > 0 && millis()-lastRadio > FAILSAFE_THR && millis()-tmrlastcmd > SENDCMDMINTIME){ //NORMAL BEHAVIOUR!
    if (millis()-lastRadio > FAILSAFE_THR && millis()-tmrlastcmd > SENDCMDMINTIME){ //DEBUG ONLY!
    //TODO: FAILSAFE!
    //SerialDEBUG.print(millis());
    //SerialDEBUG.print(" - ");
    //SerialDEBUG.println("FAILSAFE!");
    for (int i = 0; i<FRAME_LEN-2; i++){
      frame[i] = 2048; //Joystick centrati!
      //TODO: Gas basso???ug
    }
    frame[3] = 1500; //Scendi pianino
    frame[FRAME_LEN-2] = 128+1024; //19; //Back-to-Home + GPSHold + Motori attivi
    digitalWrite(STATUS_LED,LOW);
    //Serial.print("FS:");
    //Serial.print(frame[3]);
    flgSendCommand = true;
    }
  */
  // -------------------------------------------------------------------------

  // --------------------------- Parsing data from RC --------------------
  //flgSendCommand = true; // CACCIA: uncomment to bypass failsafe
  //Invio valori
  if (flgSendCommand) {
    //Ho un pacchetto valido, parso i comandi, e imposto le uscite
    uint16_t buttons = frame[FRAME_LEN - 2];

    /*
      for(int i=0;i<RC_CHANS-4;i++){
      pksv[i] = map(frame[i],MIN_INPUT_VALUE,MAX_INPUT_VALUE,MIN_PKSV_VALUE,MAX_PKSV_VALUE);
      pksv[i] = constrain(pksv[i],MIN_PKSV_VALUE,MAX_PKSV_VALUE);
      }

      pksv[RC_CHANS-3] = ZERO_PKSV_VALUE;
      pksv[RC_CHANS-2] = ZERO_PKSV_VALUE;
      pksv[RC_CHANS-1] = ZERO_PKSV_VALUE;


      if (buttons & 0x2){
      pksv[4] = MAX_PKSV_VALUE; //CHN6 - pin 8
      }else{
      pksv[4] = MIN_PKSV_VALUE; //CHN6 - pin 8
      }
    */


    for (int i = 0; i < RC_CHANS - 2; i++) {
      pksv[i] = map(frame[i], MIN_INPUT_VALUE, MAX_INPUT_VALUE, MIN_PKSV_VALUE, MAX_PKSV_VALUE);
      pksv[i] = constrain(pksv[i], MIN_PKSV_VALUE, MAX_PKSV_VALUE);
    }
    //pksv[RC_CHANS-2] = ZERO_PKSV_VALUE;
    //pksv[RC_CHANS-1] = ZERO_PKSV_VALUE;


    /*if (rcData[4]< LOWER_THRESHOLD_RC_VALUE){
      button_ODROID_pushed = true;
      }
      else{
      button_ODROID_pushed = false;
      }*/

    // If the channel is greater than the upper threshold ODROID ovewrite RC commands
    //SerialDEBUG.print(rcData[4]);
    //SerialDEBUG.println(rcData[5]);
    if (rcData[4] > UPPER_THRESHOLD_RC_VALUE) {
      button_ODROID_bridge = false;
      if (rcData[5] > 1300 && rcData[5] < 1700) {
        //     ODROID ON   AND    LOITER
        button_ODROID_pushed = true;
        //SerialDEBUG.print("Set ODROID true: ");
      } else {
        button_ODROID_pushed = false;
      }
    }
    else {
      button_ODROID_pushed = false;
      if (rcData[4] > LOWER_THRESHOLD_RC_VALUE) {
        button_ODROID_bridge = true;
        //SerialDEBUG.print("Set BRIDGE true: ");
      } else {
        button_ODROID_bridge = false;
      }
    }

    //SerialDEBUG.print(" button: ");
    //SerialDEBUG.print(button_ODROID_bridge);
    //SerialDEBUG.print(" - ");
    //SerialDEBUG.println(button_ODROID_pushed);

    // ---------------------------------------------------------------------

    // --------------------------- Send command to PX4 ---------------------
    // for(int i=0;i<MAXSERVO;i++){
    //  ppm[i] = pksv[i]/2; //Output PPM Values!
    //}
    // ---------------------------------------------------------------------


    /* COMMENTED by CACCIA
       //Debug Radio
       cRadio++;
       if (cRadio & 0x10){
         for (int i = 0; i<8; i++){
           SerialDEBUG.print(pksv[i]);
           SerialDEBUG.print(" ");
         }
         SerialDEBUG.print(buttons);
         SerialDEBUG.print(" ");
         SerialDEBUG.print(pksv_artva);
         SerialDEBUG.println(" ");

         cRadio = 0;
       }
    */

#ifdef USE_OSD
    cTelem++;
    if (cTelem < 0x08) {
      long mls = millis();
      //Invio pacchetti telemetria
      switch (cTelem) {
        case 1:
          //Heart-beat (pre-initialized above)
          psize = mavlink_msg_heartbeat_encode(1, 1, &toSend, &msg_hb);
          sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
          for (int i = 0; i < sendsize; i++) {
            SerialOSD.write(txbuf[i]);
          }
          break;
        case 2:
          //Posizione GPS
          msg_pos.time_usec = mls * 1000;
          msg_pos.lat = gps.location.lat() * 10000000; //44.512864 * 10000000;
          msg_pos.lon = gps.location.lng() * 10000000; //11.341410 * 10000000;
          msg_pos.alt = gps.altitude.meters() * 1000; //1234*1000;
          //msg_pos.relative_alt = 3456*1000;
          msg_pos.eph = 2000; //incertezza? orizzontale (in cm)
          msg_pos.epv = 3000; //incertezza? verticale (in cm)
          msg_pos.vel = 5.63 * 100; //m/sec * 100
          msg_pos.cog = 35 * 100; //gradi sessagesimali * 100
          msg_pos.fix_type = 3;
          msg_pos.satellites_visible = gps.satellites.value();
          //msg_pos.vx = 0;
          //msg_pos.vy = 0;
          //msg_pos.vz = 0;
          //msg_pos.hdg = 0;
          //Override lat/lon artva x debug
          //msg_pos.lat = _distanceOne/100.0;
          //msg_pos.lon = _distanceTwo/100.0;
          //psize = mavlink_msg_global_position_int_encode(1,1,&toSend,&msg_pos);
          psize = mavlink_msg_gps_raw_int_encode(1, 1, &toSend, &msg_pos);
          sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
          for (int i = 0; i < sendsize; i++) {
            SerialOSD.write(txbuf[i]);
          }
          //SerialDEBUG.println(gps.satellites.value());
          //SerialDEBUG.println(gps.satellites.isValid());
          break;
        case 3:
          //Teniamo questo codice per reference manipolazione stringhe
          //strcpy(msg_txtstatus.text, "ARTVA MACHINE!                                   "); //stringa di 50 caratteri
          //msg_txtstatus.severity = 2;
          //psize = mavlink_msg_statustext_encode(1,1,&toSend,&msg_txtstatus);
          msg_hud.airspeed = 12.3; //m/sec float --> rappresentazione in KM/H
          //msg_hud.groundspeed = _distanceTwo / 100.0; //m/sec fload --> rappresentazione in KM/H
          //msg_hud.alt = _distanceOne / 100.0; //metri float --> rappresentazione in metri arrotondato
          msg_hud.climb = 1.2; //m/sec float
          msg_hud.heading = 37; //gradi sessagesimali --> intero normale
          msg_hud.throttle = 89; //0-100 intero --> intero normale
          psize = mavlink_msg_vfr_hud_encode(1, 1, &toSend, &msg_hud);
          sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
          for (int i = 0; i < sendsize; i++) {
            SerialOSD.write(txbuf[i]);
          }
          break;
      }
    } else {
      //SerialDEBUG.print("TELEM!");
      //SerialDEBUG.println(" ");
      cTelem = 0;
    }
#endif


    flgSendCommand = false;
    tmrlastcmd = millis();

    /*
      //Serial.println();
      SerialDEBUG.print("0: ");
      SerialDEBUG.print(pksv[0]);
      SerialDEBUG.print(" 1: ");
      SerialDEBUG.print(pksv[1]);
      SerialDEBUG.print(" 2: ");
      SerialDEBUG.print(pksv[2]);
      SerialDEBUG.print(" 3: ");
      SerialDEBUG.print(pksv[3]);
      SerialDEBUG.print(" 4: ");
      SerialDEBUG.print(pksv[4]);
      SerialDEBUG.print(" 5: ");
      SerialDEBUG.print(pksv[5]);
      SerialDEBUG.print(" 6: ");
      SerialDEBUG.print(pksv[6]);
      SerialDEBUG.print(" 7: ");
      SerialDEBUG.println(pksv[7]);
      SerialDEBUG.println("-------");
    */
  } // --> END flgSendCommand IF



  // ------------------------- Control from ODROID ------------------------
  // Read MAVLINK message MAVLINK_MSG_ID_RC_CHANNELS_RAW From ODROID
  //if (buttons & 0x2){  // Quale bottone? Terzo?
  //SerialDEBUG.println("ODR/BRIDGE");
  while (SerialTELEM_ODROID.available() > 0) {
    uint8_t inchar = SerialTELEM_ODROID.read();
    //SerialDEBUG.write(inchar);
    if (button_ODROID_pushed || button_ODROID_bridge) {
      SerialTELEM_AP.write(inchar); //Comunque PONTE!
      //SerialDEBUG.write(inchar);
    }
    if (mavlink_parse_char(MAVLINK_COMM_0, inchar, &rcvmsgFromOdroid, &rcvmstatus)) {
      // Handle message
      switch (rcvmsgFromOdroid.msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
          //Decode mavlink message
          mavlink_msg_rc_channels_override_decode(&rcvmsgFromOdroid, &cmdsFromOdroid);      //NOT USED ANYMORE-->RC_CHANNELS_RAW used instead
          frameOdroid[0] = cmdsFromOdroid.chan1_raw;        // LSX;
          frameOdroid[1] = cmdsFromOdroid.chan2_raw;        // LSY;
          frameOdroid[2] = cmdsFromOdroid.chan3_raw;        // RSX;
          frameOdroid[3] = cmdsFromOdroid.chan4_raw;        // RSY;
          frameOdroid[4] = cmdsFromOdroid.chan5_raw;        // BSX;
          frameOdroid[5] = cmdsFromOdroid.chan6_raw;        // BSY;
          frameOdroid[6] = cmdsFromOdroid.chan7_raw;        //
          frameOdroid[7] = cmdsFromOdroid.chan8_raw;        //
          lastOdroidReceived = millis(); //Resetto watchdog x ricezione odroide
          //SerialDEBUG.println(rcvmsgFromOdroid.msgid);
          break;
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
          //Decode mavlink message
          mavlink_msg_rc_channels_raw_decode(&rcvmsgFromOdroid, &cmdsFromOdroidRaw);
          frameOdroid[0] = cmdsFromOdroidRaw.chan1_raw;        // LSX;
          frameOdroid[1] = cmdsFromOdroidRaw.chan2_raw;        // LSY;
          frameOdroid[2] = cmdsFromOdroidRaw.chan3_raw;        // RSX;
          frameOdroid[3] = cmdsFromOdroidRaw.chan4_raw;        // RSY;
          frameOdroid[4] = cmdsFromOdroidRaw.chan5_raw;        // BSX;
          frameOdroid[5] = cmdsFromOdroidRaw.chan6_raw;        // BSY;
          frameOdroid[6] = cmdsFromOdroidRaw.chan7_raw;        //
          frameOdroid[7] = cmdsFromOdroidRaw.chan8_raw;        //
          lastOdroidReceived = millis(); //Resetto watchdog x ricezione odroide
          //SerialDEBUG.println(rcvmsgFromOdroid.msgid);
          break;
        default:
          //SerialDEBUG.println(rcvmsgFromOdroid.msgid);
          /*
            //Faccio da ponte su tutti i messaggi non RC_OVERRIDE verso PixHawk
            //psize = mavlink_msg_gps_raw_int_encode(1,1,&toSend,&msg_pos);
            //mavlink_finalize_message(&rcvmsgFromOdroid,);
            sendsize = mavlink_msg_to_send_buffer(txbuf,&rcvmsgFromOdroid);
            for(int is = 0; is<sendsize;is++){
            SerialTELEM_AP.write(txbuf[is]);
            }
          */
          break;
      }
      //SerialDEBUG.println("ODROID!");
    }
  }

  // Overwrite first 4 RC commands (se ho il bottone odroide attivato e ho un dato recente
  if (button_ODROID_pushed && millis() - lastOdroidReceived < 400 && frameOdroid[0] != 0 && frameOdroid[1] != 0) {
    for (int i = 0; i < RC_CHANS - 4; i++) {
      pksv[i] = frameOdroid[i] * 2;
    }
    odroidDebugCounter++;
    if (odroidDebugCounter & 0x100) {
      SerialDEBUG.print("RC OVERRIDE MESSAGE: ");
      SerialDEBUG.print(" 1= ");
      SerialDEBUG.print(frameOdroid[0]);
      SerialDEBUG.print(" 2= ");
      SerialDEBUG.print(frameOdroid[1]);
      SerialDEBUG.print(" 3= ");
      SerialDEBUG.print(frameOdroid[2]);
      SerialDEBUG.print(" 4= ");
      SerialDEBUG.print(frameOdroid[3]);
      SerialDEBUG.print(" 5= ");
      SerialDEBUG.print(frameOdroid[4]);
      SerialDEBUG.print(" 6= ");
      SerialDEBUG.println(frameOdroid[5]);
      odroidDebugCounter = 0;
    }
  }

  // ------------------------------------------------------------

  // -------------------------- Send PPM ------------------------------
  // Send PPM to PX4 overwriting the one of the RC
  for (int i = 0; i < 4; i++) {
    ppm[i] = map(pksv[i], MIN_PKSV_VALUE, MAX_PKSV_VALUE, MIN_PPM_VALUE, MAX_PPM_VALUE);
    ppm[i] = constrain(ppm[i], MIN_PPM_VALUE, MAX_PPM_VALUE);
  }
  ppm[4] = map(pksv[5], MIN_PKSV_VALUE, MAX_PKSV_VALUE, MIN_PPM_VALUE, MAX_PPM_VALUE);
  ppm[4] = constrain(ppm[4], MIN_PPM_VALUE, MAX_PPM_VALUE);
  ppm[5] = map(pksv[4], MIN_PKSV_VALUE, MAX_PKSV_VALUE, MIN_PPM_VALUE, MAX_PPM_VALUE);
  ppm[5] = constrain(ppm[5], MIN_PPM_VALUE, MAX_PPM_VALUE);

  /*
    //Serial.println();
    SerialDEBUG.print("0: ");
    SerialDEBUG.print(ppm[0]);
    SerialDEBUG.print(" 1: ");
    SerialDEBUG.print(ppm[1]);
    SerialDEBUG.print(" 2: ");
    SerialDEBUG.print(ppm[2]);
    SerialDEBUG.print(" 3: ");
    SerialDEBUG.print(ppm[3]);
    SerialDEBUG.print(" 4: ");
    SerialDEBUG.print(ppm[4]);
    SerialDEBUG.print(" 5: ");
    SerialDEBUG.print(ppm[5]);
    SerialDEBUG.print(" 6: ");
    SerialDEBUG.print(ppm[6]);
    SerialDEBUG.print(" 7: ");
    SerialDEBUG.println(ppm[7]);
    SerialDEBUG.println("-------");*/
  // ------------------------------------------------------------------

  //// Codice di elaborazione e invio dei dati del LASER - Flavio 23/2/2017
  // INIZIO
#ifdef USE_LASER
  pwm_value = pulseIn(3, HIGH,40000); // Count how long the pulse is high in microseconds
  if(pwm_value != 0){ // If we get a reading that isn't zero, let's print it
//  // se è passato il tempo di pausa e ho un nuovo valore dal laser, lo elaboro...
//  if (millis() - pausa_stampa_laser > 199 && flag_aggiornamento_laser == true)
//  {
    pausa_stampa_laser = millis();
//    flag_aggiornamento_laser = false;
    d_laser = ((long) pwm_value) / 10.0; // distance in cm
    if (d_laser >= d_laser_max || d_laser <= d_laser_min)
    {
      /* Out of Range */
      // SerialDEBUG.println("LASER: OutOfRange");
      // SerialDEBUG.print("t [ms]:\t"); Serial.print(millis()); Serial.print("\t R [cm]:\t"); Serial.println(-1);
      msg_laser.time_boot_ms = millis();
      msg_laser.current_distance = -1;
      psize = mavlink_msg_distance_sensor_encode(1, 2, &toSend, &msg_laser);
      sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
      for (int i = 0; i < sendsize; i++)
      {
        SerialTELEM_ODROID.write(txbuf[i]);
      }
    }
    else
    {
      /* In range */
      // SerialDEBUG.print("t [ms]:\t"); Serial.print(millis()); Serial.print("\t R [cm]:\t"); Serial.println(d_laser);
      msg_laser.time_boot_ms = millis();
      msg_laser.current_distance = d_laser;
      psize = mavlink_msg_distance_sensor_encode(1, 2, &toSend, &msg_laser);
      sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
      for (int i = 0; i < sendsize; i++)
      {
        SerialTELEM_ODROID.write(txbuf[i]);
      }
    }
  }
//    // se il tempo è passato ma non ho un nuovo valore significa che il laser è morto...
//    else if (millis() - pausa_stampa_laser > 1000 && flag_aggiornamento_laser == false) {
//      SerialDEBUG.print("t [ms]:\t"); Serial.print(millis()); Serial.print("\t R [cm]:\t"); Serial.println(-1);
//      flag_aggiornamento_laser = false;
//      msg_laser.time_boot_ms = millis();
//      msg_laser.current_distance = -1;
//      psize = mavlink_msg_distance_sensor_encode(1, 2, &toSend, &msg_laser);
//      sendsize = mavlink_msg_to_send_buffer(txbuf, &toSend);
//      for (int i = 0; i < sendsize; i++)
//      {
//        SerialTELEM_ODROID.write(txbuf[i]);
//      }
//    }
#endif
  // FINE

} // END LOOP

float UpdateDiff(int lastVal, long lastDelta, int* sValues, long* dTimes) {
  float retval = AverageDifferentiator(lastVal, sValues[0], sValues[1], sValues[2], lastDelta, dTimes[0], dTimes[1]);
  sValues[2] = sValues[1];
  sValues[1] = sValues[0];
  sValues[0] = lastVal;
  dTimes[2] = dTimes[1];
  dTimes[1] = dTimes[0];
  dTimes[0] = lastDelta;
  return retval;
}
