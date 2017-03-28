#ifndef DEF_H_
#define DEF_H_

#define RC_CHANS 8
//Definizione input porta D Promini!
#define THROTTLEPIN                0  //PIN 62 =  PIN A8
#define ROLLPIN                    1  //PIN 63 =  PIN A9
#define PITCHPIN                   2  //PIN 64 =  PIN A10
#define YAWPIN                     3  //PIN 65 =  PIN A11
#define AUX1PIN                    4  //PIN 66 =  PIN A12
#define AUX2PIN                    5  //PIN 67 =  PIN A13
#define AUX3PIN                    6  //PIN 68 =  PIN A14
#define AUX4PIN                    7  //PIN 69 =  PIN A15
//Configurazione porte e pin accessori
#define PCINT_PIN_COUNT            8
#define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7),(1<<0),(1<<1),(1<<3)
#define PCINT_RX_PORT              PORTK
#define PCINT_RX_MASK              PCMSK2
#define PCIR_PORT_BIT              (1<<2)
#define RX_PC_INTERRUPT            PCINT2_vect
#define RX_PCINT_PIN_PORT          PINK
#define V_BATPIN                   A0    // Analog PIN 0
#define PSENSORPIN                 A2    // Analog PIN 2

#define VBAT_CELLS_NUM            3

#define MINCHECK 1000
#define MAXCHECK 2000
#define MEGA
#define STANDARD_RX

extern int16_t rcData[RC_CHANS];
extern int16_t rcSerial[8];
extern int16_t rcCommand[4];
extern uint8_t rcSerialCount;
extern unsigned long lastRCCommand;  // CACCIA: safety - if arduino does not receive commands from the receiver
extern bool flgSendCommand;          // CACCIA: it allows Arduino to send different commands to Pixhawk

#endif



