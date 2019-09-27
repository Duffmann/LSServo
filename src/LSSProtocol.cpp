/*
 * LSSProtocol.cpp
 * v0.2: 2017.11.01
 * Implementation of Command Protocol for
 * LewanSoul LX-16A serial bus servos
 * Author: DUffman71
 *
 * Bugs? 
 * - Why delay of flash_delay required for succsessive Flash commands properly excute???
 * - How much bus-turnaround delay is necessary between Read and Write commands?
 */

#include <stddef.h>
#include "LSSProtocol.h"

LSSProtocol::LSSProtocol()
{
}

/* *************************** internal methods *********************************/

u8 LSSProtocol::LSSCheckSum(u8 buf[])
{
  u8 i;
  u16 chksum = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    chksum += buf[i];
  }
  chksum = ~chksum;
  i = (u8)chksum;
  return i;
}


int LSSProtocol::LSSIssueReadCMD(u8 ID, u8 CMD_opcode, u8 RX_buf_size, u8 *RX_buf)
{
  u8 TX_buf[s_LSS_ALL_READ_CMDS+3];
  bool syntax_check = false;
  int rx_size;  

  TX_buf[0] = TX_buf[1] = LSS_FRAME_HEADER;
  TX_buf[2] = ID;
  TX_buf[3] = s_LSS_ALL_READ_CMDS;
  TX_buf[4] = CMD_opcode;
  TX_buf[5] = LSSCheckSum(TX_buf);

  flushLSS(); 				  	// Empty serial receive buffer
  writeLSS(TX_buf, sizeof(TX_buf));	  	// Send Read Command for 'CMD_opcode'
  rx_size = readLSS(RX_buf, RX_buf_size); 	// Read back 'answer CMD' from servo
  delayMicroseconds(bus_turnaround_delay);	// Ensure serial bus turnaround happend

  syntax_check = (rx_size == RX_buf_size);
  syntax_check = syntax_check && (RX_buf[0] == LSS_FRAME_HEADER);
  syntax_check = syntax_check && (RX_buf[1] == LSS_FRAME_HEADER);
  if (ID != 0xFE)			  	// Broadcast Read CMD returns real ID
  {
	syntax_check = syntax_check && (RX_buf[2] == ID);
  }
  syntax_check = syntax_check && (RX_buf[3] == RX_buf_size-3);
  syntax_check = syntax_check && (RX_buf[4] == CMD_opcode);
  syntax_check = syntax_check && (RX_buf[RX_buf_size-1] == LSSCheckSum(RX_buf));

  return syntax_check;
}



/* *********************** Transient Write commands ***************************/

int LSSProtocol::SetPos(u8 ID, u16 pos, u16 time)
{
  u8 buf[s_LSS_MOVE_TIME_WRITE+3];

  pos = constrain(pos, 0, 1000);
  time = constrain(time, 0, 30000); // time is in ms

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_MOVE_TIME_WRITE;
  buf[4] = LSS_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(pos);
  buf[6] = GET_HIGH_BYTE(pos);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LSSCheckSum(buf);

  return writeLSS(buf, sizeof(buf));
}

int LSSProtocol::SetRegPos(u8 ID, u16 pos, u16 time)
{
  u8 buf[s_LSS_MOVE_TIME_WAIT_WRITE+3];

  pos = constrain(pos, 0, 1000);
  time = constrain(time, 0, 30000); // time is in ms

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_MOVE_TIME_WAIT_WRITE;
  buf[4] = LSS_MOVE_TIME_WAIT_WRITE;
  buf[5] = GET_LOW_BYTE(pos);
  buf[6] = GET_HIGH_BYTE(pos);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LSSCheckSum(buf);

  return writeLSS(buf, sizeof(buf));
}

int LSSProtocol::SetActionStart(u8 ID)
{
  u8 buf[s_LSS_MOVE_START+3];

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_MOVE_START;
  buf[4] = LSS_MOVE_START;
  buf[5] = LSSCheckSum(buf);

  return writeLSS(buf, sizeof(buf));
}

int LSSProtocol::SetActionStop(u8 ID)
{
  u8 buf[s_LSS_MOVE_STOP+3];

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_MOVE_STOP;
  buf[4] = LSS_MOVE_STOP;
  buf[5] = LSSCheckSum(buf);

  return writeLSS(buf, sizeof(buf));
}

int LSSProtocol::SetMode(u8 ID, u8 mode, s16 speed)
{
  u8 buf[LSS_MODE_WRITE+3];

  mode= constrain(mode, 0, 1);
  speed = constrain(speed, -1000, 1000); // negative is CW, pos is CCW

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_MODE_WRITE;
  buf[4] = LSS_MODE_WRITE;
  buf[5] = mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((u16)speed);
  buf[8] = GET_HIGH_BYTE((u16)speed);
  buf[9] = LSSCheckSum(buf);
  return writeLSS(buf, sizeof(buf));
}

int LSSProtocol::SetTorque(u8 ID, u8 enable)
{
  u8 buf[s_LSS_TORQUE_WRITE+3];

  enable= constrain(enable, 0, 1);

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_TORQUE_WRITE;
  buf[4] = LSS_TORQUE_WRITE;
  buf[5] = enable;
  buf[6] = LSSCheckSum(buf);
  return writeLSS(buf, sizeof(buf));
}

int LSSProtocol::SetAngleOffset(u8 ID, s8 offset)
{
  u8 buf[s_LSS_ANGLE_OFFSET_ADJUST+3];

  offset = constrain(offset, -125, 125);

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_ANGLE_OFFSET_ADJUST;
  buf[4] = LSS_ANGLE_OFFSET_ADJUST;
  buf[5] = (u8)offset;
  buf[6] = LSSCheckSum(buf);
  return writeLSS(buf, sizeof(buf));
}

/* ************************* Flash changing commands **************************/

int LSSProtocol::FlashServoID(u8 ID, u8 newID)
{
  u8 buf[s_LSS_ID_WRITE+3];

  newID = constrain(newID, 0, 253);

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_ID_WRITE;
  buf[4] = LSS_ID_WRITE;
  buf[5] = newID;
  buf[6] = LSSCheckSum(buf);

  int bytes_transmitted = writeLSS(buf, sizeof(buf));

  delay(flash_delay); // need to wait until Flash in Servo is written

  return bytes_transmitted;  
}

int LSSProtocol::FlashAngleOffset(u8 ID)
{
  u8 buf[s_LSS_ANGLE_OFFSET_WRITE+3];

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_ANGLE_OFFSET_WRITE;
  buf[4] = LSS_ANGLE_OFFSET_WRITE;
  buf[5] = LSSCheckSum(buf);

  int bytes_transmitted = writeLSS(buf, sizeof(buf));

  delay(flash_delay); // need to wait until Flash in Servo is written

  return bytes_transmitted; 
}

int LSSProtocol::FlashAngleLimits(u8 ID, u16 angmin, u16 angmax)
{
  u8 buf[s_LSS_ANGLE_LIMITS_WRITE+3];

  angmin = constrain(angmin, 0, 1000);
  angmax = constrain(angmax, 0, 1000);
  if (angmin > angmax)
  {
    return 0; // Return error if min angle > max angle
  }

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_ANGLE_LIMITS_WRITE;
  buf[4] = LSS_ANGLE_LIMITS_WRITE;
  buf[5] = GET_LOW_BYTE(angmin);
  buf[6] = GET_HIGH_BYTE(angmin);
  buf[7] = GET_LOW_BYTE(angmax);
  buf[8] = GET_HIGH_BYTE(angmax);
  buf[9] = LSSCheckSum(buf);

  int bytes_transmitted = writeLSS(buf, sizeof(buf));

  delay(flash_delay); // need to wait until Flash in Servo is written

  return bytes_transmitted; 
}

int LSSProtocol::FlashVinLimits(u8 ID, u16 vinmin, u16 vinmax)
{
  u8 buf[s_LSS_VIN_LIMITS_WRITE+3];

  vinmin = constrain(vinmin, 4500, 12000);
  vinmax = constrain(vinmax, 4500, 12000);
  if (vinmin > vinmax)
  {
    return 0; // Return error if min voltage > max voltage
  }

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_VIN_LIMITS_WRITE;
  buf[4] = LSS_VIN_LIMITS_WRITE;
  buf[5] = GET_LOW_BYTE(vinmin);
  buf[6] = GET_HIGH_BYTE(vinmin);
  buf[7] = GET_LOW_BYTE(vinmax);
  buf[8] = GET_HIGH_BYTE(vinmax);
  buf[9] = LSSCheckSum(buf);

  int bytes_transmitted = writeLSS(buf, sizeof(buf));

  delay(flash_delay); // need to wait until Flash in Servo is written

  return bytes_transmitted; 
}

int LSSProtocol::FlashTempLimit(u8 ID, u8 tempmax)
{
  u8 buf[s_LSS_TEMP_LIMIT_WRITE+3];

  tempmax = constrain(tempmax, 50, 100);

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_TEMP_LIMIT_WRITE;
  buf[4] = LSS_TEMP_LIMIT_WRITE;
  buf[5] = tempmax;
  buf[6] = LSSCheckSum(buf);

  int bytes_transmitted = writeLSS(buf, sizeof(buf));

  delay(flash_delay); // need to wait until Flash in Servo is written

  return bytes_transmitted; 
}

int LSSProtocol::FlashLEDState(u8 ID, u8 ledstate)
{
  u8 buf[s_LSS_LED_CTRL_WRITE+3];

  ledstate = constrain(ledstate, 0, 1); // Note: '0' = ON, '1' = OFF

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_LED_CTRL_WRITE;
  buf[4] = LSS_LED_CTRL_WRITE;
  buf[5] = ledstate;
  buf[6] = LSSCheckSum(buf);

  int bytes_transmitted = writeLSS(buf, sizeof(buf));

  delay(flash_delay); // need to wait until Flash in Servo is written

  return bytes_transmitted; 
}

int LSSProtocol::FlashLEDErrCode(u8 ID, u8 lederrcode)
{
  u8 buf[s_LSS_LED_ERROR_WRITE+3];

  lederrcode = constrain(lederrcode, 0, 7); // b0=temp, b1:voltage, b2:stalled  

  buf[0] = buf[1] = LSS_FRAME_HEADER;
  buf[2] = ID;
  buf[3] = s_LSS_LED_ERROR_WRITE;
  buf[4] = LSS_LED_ERROR_WRITE;
  buf[5] = lederrcode;
  buf[6] = LSSCheckSum(buf);

  delay(flash_delay); // need to wait until Flash in Servo is written

  return writeLSS(buf, sizeof(buf));
}

/* ****************************** Read Commands *******************************/

int LSSProtocol::GetID(u8 ID, u8 &myID)
{
  u8 CMD_opcode = LSS_ID_READ;
  u8 RX_buf[s_LSS_ID_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  myID = RX_buf[5];
  return syntax_check;
}

int LSSProtocol::GetPos(u8 ID, s16 &pos)
{
  u8 CMD_opcode = LSS_POS_READ;
  u8 RX_buf[s_LSS_POS_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  pos = BYTE_TO_HW(RX_buf[6], RX_buf[5]);
  return syntax_check;
}

int LSSProtocol::GetVin(u8 ID, u16 &vin)
{
  u8 CMD_opcode = LSS_VIN_READ;
  u8 RX_buf[s_LSS_VIN_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  vin = BYTE_TO_HW(RX_buf[6], RX_buf[5]);
  return syntax_check;
}

int LSSProtocol::GetTemp(u8 ID, u8 &temp)
{
  u8 CMD_opcode = LSS_TEMP_READ;
  u8 RX_buf[s_LSS_TEMP_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  temp = RX_buf[5];
  return syntax_check;
}

int LSSProtocol::GetAngleOffset(u8 ID, s8 &angoffset)
{
  u8 CMD_opcode = LSS_ANGLE_OFFSET_READ;
  u8 RX_buf[s_LSS_ANGLE_OFFSET_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  angoffset = s8(RX_buf[5]);
  return syntax_check;
}

int LSSProtocol::GetAngleLimits(u8 ID, u16 &angmin, u16 &angmax)
{
  u8 CMD_opcode = LSS_ANGLE_LIMITS_READ;
  u8 RX_buf[s_LSS_ANGLE_LIMITS_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  angmin = BYTE_TO_HW(RX_buf[6], RX_buf[5]);
  angmax = BYTE_TO_HW(RX_buf[8], RX_buf[7]);
  return syntax_check;
}

int LSSProtocol::GetTorqueState(u8 ID, u8 &enable)
{
  u8 CMD_opcode = LSS_TORQUE_READ;
  u8 RX_buf[s_LSS_TORQUE_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  enable = RX_buf[5];
  return syntax_check;
}

int LSSProtocol::GetMode(u8 ID, u8 &mode, s16 &speed)
{
  u8 CMD_opcode = LSS_MODE_READ;
  u8 RX_buf[s_LSS_MODE_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  mode = RX_buf[5];
  speed = BYTE_TO_HW(RX_buf[8], RX_buf[7]);
  return syntax_check;
}

int LSSProtocol::GetTempLimit(u8 ID, u8 &tempmax)
{
  u8 CMD_opcode = LSS_TEMP_LIMIT_READ;
  u8 RX_buf[s_LSS_TEMP_LIMIT_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  tempmax = RX_buf[5];
  return syntax_check;
}

int LSSProtocol::GetVinLimits(u8 ID, u16 &vinmin, u16 &vinmax)
{
  u8 CMD_opcode = LSS_VIN_LIMITS_READ;
  u8 RX_buf[s_LSS_VIN_LIMITS_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  vinmin = BYTE_TO_HW(RX_buf[6], RX_buf[5]);
  vinmax = BYTE_TO_HW(RX_buf[8], RX_buf[7]);
  return syntax_check;
}

int LSSProtocol::GetLEDState(u8 ID, u8 &ledstate)
{
  u8 CMD_opcode = LSS_LED_CTRL_READ;
  u8 RX_buf[s_LSS_LED_CTRL_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  ledstate = RX_buf[5];
  return syntax_check;
}

int LSSProtocol::GetLEDErrcode(u8 ID, u8 &lederrcode)
{
  u8 CMD_opcode = LSS_LED_ERROR_READ;
  u8 RX_buf[s_LSS_LED_ERROR_READ+3];
  int syntax_check = LSSIssueReadCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
  lederrcode = RX_buf[5];
  return syntax_check;
}



