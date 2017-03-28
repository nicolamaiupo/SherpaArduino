#ifndef RX_H_
#define RX_H_

//#include "config.h"
#include "def.h"
#include "types.h"
//#include "Serial.h"
//#include "Protocol.h"
//#include "MultiWii.h"
//#include "Alarms.h"


void configureReceiver();
void computeRC();
uint16_t readRawRC(uint8_t chan);
void readSerial_RX(void);
#if defined(OPENLRSv2MULTI)
  void initOpenLRS(void);
  void Read_OpenLRS_RC(void);
#endif
#if defined(SPEK_BIND)  // Bind Support
  void spekBind(void);
#endif

#endif /* RX_H_ */

