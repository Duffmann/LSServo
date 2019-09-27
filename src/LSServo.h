/*
 * LSServo.h
 * v0.1: 2017.10.29
 * Basic functions for communication with
 * LewanSoul LX-16A serial bus servos
 * Author: DUffman71
 */

#ifndef _LSSERVO_H
#define _LSSERVO_H

#include "Arduino.h"
#include "LSSProtocol.h"

class LSServo : public LSSProtocol
{
public:
	LSServo(void);
	virtual int writeLSS(unsigned char *nDat, int nLen);
	virtual int readLSS(unsigned char *nDat, int nLen);
	virtual void flushLSS();
public:
	unsigned long int IOTimeOut;
	HardwareSerial *pSerial;
};

#endif
