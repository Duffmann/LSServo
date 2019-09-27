/*
 * LSServo.cpp
 * v0.1: 2017.10.29
 * Basic functions for communication with
 * LewanSoul LX-16A serial bus servos
 * Author: DUffman71
 */

#include "LSServo.h"

LSServo::LSServo()
{
	IOTimeOut = 2;
	pSerial = NULL;
}

int LSServo::readLSS(unsigned char *nDat, int nLen) 	//NEEDS UPDATE/MERGE/ALIGNMENT w LewanSoul code
{
	int Size = 0;
	int ComData;
	unsigned long t_begin = millis();
	unsigned long t_user;
	while(1){
		ComData = pSerial->read();
		if(ComData!=-1){
			if(nDat){
				nDat[Size] = ComData;
			}
			Size++;
			t_begin = millis();
		}
		if(Size>=nLen){
			break;
		}
		t_user = millis() - t_begin;
		if(t_user>IOTimeOut){
			break;
		}
	}
	return Size;
}


int LSServo::writeLSS(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}

void LSServo::flushLSS()
{
	while(pSerial->read()!=-1);
}
