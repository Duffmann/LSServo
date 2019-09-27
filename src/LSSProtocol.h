/*
 * LSSProtocol.h
 * v0.2: 2017.11.01
 * Implementation of Command Protocol for
 * LewanSoul LX-16A serial bus servos
 * Author: DUffman71
 */

#include "Arduino.h"

#ifndef _LSSPROTOCOL_H
#define _LSSPROTOCOL_H

typedef		signed char	s8;
typedef		unsigned char	u8;	
typedef		unsigned short	u16;	
typedef		signed short	s16;
typedef		unsigned long	u32;	
typedef		signed long	s32;

#define LSS_FRAME_HEADER	0x55
#define LSS_Broadcast_ID	0xFE	

#define GET_LOW_BYTE(A) (u8)((A)) 			// Macro to extract lower 8 bits from A
#define GET_HIGH_BYTE(A) (u8)((A) >> 8)			// Macro to extract high 8 bits from A
#define BYTE_TO_HW(A, B) ((((u16)(A)) << 8) | (u8)(B))	// Macro to combine A & B into u16 (little endian)

class LSSProtocol{
public:
	LSSProtocol();
	// Write-based functions
	int SetPos(u8 ID, u16 pos, u16 time); // move to pos 0~1000 (=0~240°) in time = 0~30000ms
	int SetRegPos(u8 ID, u16 pos, u16 time); // same as above but wait until ActionStart cmd
	int SetActionStart(u8 ID); // start registered movement, use with 0xfe to start all servos
	int SetActionStop(u8 ID); // stop ongoing action for servo 'ID', use 0xfe to stop all servos
	int SetMode(u8 ID, u8 mode, s16 speed); // 0=Servo (240°), 1 =heel (360°), Speed -1000~1000 (wheel)
	int SetTorque(u8 ID, u8 enable); // Enable=1 --> Torque on, Enable=0 for teach-in (default)
	int SetAngleOffset(u8 ID, s8 offset); // temp adjust servo offset to -125~+125 (-30°~+30°)

	int FlashServoID(u8 ID, u8 newID); // HANDLE with care to avoid conflicting IDs, stored to Flash
	int FlashAngleOffset(u8 ID); // Store offset written via 'WriteAngleOffset()' to Flash 
	int FlashAngleLimits(u8 ID, u16 angmin, u16 angmax); // min/max angle 0~1000 (max > min) to Flash
	int FlashVinLimits(u8 ID, u16 vinmin, u16 vinmax); // min/max Vin 4500mV~12000mV (max > min) to Flash
	int FlashTempLimit(u8 ID, u8 tempmax); // Tmax to 50~100°C, alarm & torque off if T > Tmax, to Flash 
	int FlashLEDState(u8 ID, u8 ledstate); // Set state of LED, 0 = off, 1 = on, stored to Flash
	int FlashLEDErrCode(u8 ID, u8 lederrcode); // Error cond. to flash LED: b0/Tmax, b1/Vin, b2/stalled

	// Read-based functions
	int GetID(u8 ID, u8 &myID); // ONLY works for ID=0xFE (broadcast) and for single servo attached!! 
	int GetPos(u8 ID, s16 &pos); // get current pos 0~1000 (= 0~240°)
	int GetVin(u8 ID, u16 &vin); // get current Vin 4500mV~12000mV
	int GetTemp(u8 ID, u8 &temp); // get current Temp 50~100°C
	int GetAngleOffset(u8 ID, s8 &angoffset); // get current servo offset -125~+125 (-30°~+30°)
	int GetAngleLimits(u8 ID, u16 &angmin, u16 &angmax); // get current angle limits 0~1000
	int GetTorqueState(u8 ID, u8 &enable); // get current state of Servo: 0 = torque off, 1 = torque on
	int GetMode(u8 ID, u8 &mode, s16 &speed); // 0=Servo (240°)/1=wheel (360°), Speed -1000~1000 (wheel)
	int GetTempLimit(u8 ID, u8 &tempmax); // Tmax (50~100°C), alarm & torque off if T > Tmax
	int GetVinLimits(u8 ID, u16 &vinmin, u16 &vinmax); // min/max of Vin (4500mV~12000mV)
	int GetLEDState(u8 ID, u8 &ledstate); // State of LED, 0 = off, 1 = on
	int GetLEDErrcode(u8 ID, u8 &lederrcode); // Error cond. to flash LED: b0/Tmax, b1/Vin, b2/stalled

public:

protected:
	virtual int writeLSS(u8 *nDat, int nLen) = 0;
	virtual int readLSS(u8 *nDat, int nLen) = 0;
	virtual void flushLSS() = 0;

private:
	u32 flash_delay = 50; // some ms delay after each Flash Command to ensure it is properly written
	u32 bus_turnaround_delay = 100; // some us delay after each Read Command to ensure bus turn-around
	u8 LSSCheckSum(u8 buf[]);
	int LSSIssueReadCMD(u8 ID, u8 CMD_opcode, u8 RX_buf_size, u8 *RX_buf);
	
	// Instruction Op-Codes:
	#define LSS_MOVE_TIME_WRITE      1
	#define LSS_MOVE_TIME_READ       2	// not implemented, any usage scenario?
	#define LSS_MOVE_TIME_WAIT_WRITE 7
	#define LSS_MOVE_TIME_WAIT_READ  8	// not implemented, any usage scenario?
	#define LSS_MOVE_START		 11
	#define LSS_MOVE_STOP            12
	#define LSS_ID_WRITE             13
	#define LSS_ID_READ              14
	#define LSS_ANGLE_OFFSET_ADJUST  17
	#define LSS_ANGLE_OFFSET_WRITE   18
	#define LSS_ANGLE_OFFSET_READ    19
	#define LSS_ANGLE_LIMITS_WRITE   20
	#define LSS_ANGLE_LIMITS_READ    21
	#define LSS_VIN_LIMITS_WRITE     22
	#define LSS_VIN_LIMITS_READ      23
	#define LSS_TEMP_LIMIT_WRITE 	 24
	#define LSS_TEMP_LIMIT_READ      25
	#define LSS_TEMP_READ            26
	#define LSS_VIN_READ             27
	#define LSS_POS_READ             28
	#define LSS_MODE_WRITE  	 29
	#define LSS_MODE_READ   	 30
	#define LSS_TORQUE_WRITE 	 31
	#define LSS_TORQUE_READ  	 32
	#define LSS_LED_CTRL_WRITE       33
	#define LSS_LED_CTRL_READ        34
	#define LSS_LED_ERROR_WRITE      35
	#define LSS_LED_ERROR_READ       36

	// Packet Sizes (sending):
	#define s_LSS_MOVE_TIME_WRITE      7
	#define s_LSS_MOVE_TIME_WAIT_WRITE 7
	#define s_LSS_MOVE_START	   3
	#define s_LSS_MOVE_STOP            3
	#define s_LSS_ID_WRITE             4
	#define s_LSS_ANGLE_OFFSET_ADJUST  4
	#define s_LSS_ANGLE_OFFSET_WRITE   3
	#define s_LSS_ANGLE_LIMITS_WRITE   7
	#define s_LSS_VIN_LIMITS_WRITE     7
	#define s_LSS_TEMP_LIMIT_WRITE 	   4
	#define s_LSS_MODE_WRITE  	   7
	#define s_LSS_TORQUE_WRITE         4
	#define s_LSS_LED_CTRL_WRITE       4
	#define s_LSS_LED_ERROR_WRITE      4

	#define s_LSS_ALL_READ_CMDS	   3

	// Packet Sizes (receiving):
	#define s_LSS_MOVE_TIME_READ       7	// not implemented, any usage scenario?
	#define s_LSS_MOVE_TIME_WAIT_READ  7	// not implemented, any usage scenario?
	#define s_LSS_ID_READ              4
	#define s_LSS_ANGLE_OFFSET_READ    4
	#define s_LSS_ANGLE_LIMITS_READ    7
	#define s_LSS_VIN_LIMITS_READ      7
	#define s_LSS_TEMP_LIMIT_READ  	   4
	#define s_LSS_TEMP_READ            4
	#define s_LSS_VIN_READ             5
	#define s_LSS_POS_READ             5
	#define s_LSS_MODE_READ   	   7
	#define s_LSS_TORQUE_READ  	   4
	#define s_LSS_LED_CTRL_READ        4
	#define s_LSS_LED_ERROR_READ       4
};
#endif
