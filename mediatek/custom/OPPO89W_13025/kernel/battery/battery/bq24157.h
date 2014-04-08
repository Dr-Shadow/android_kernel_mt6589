/*****************************************************************************
*
* Filename:
* ---------
*   bq24156a.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   bq24156a header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _BQ24157_SW_H_
#define _BQ24157_SW_H_

#define bq24157_read_byte excharger_read_byte
#define bq24157_write_byte excharger_write_byte

#define BQ24157_CON4      0x04

#define CON4_I_CHR_MASK		0x07
#define CON4_I_CHR_SHIFT 	4

extern int excharger_read_byte(kal_uint8 cmd, kal_uint8 *returnData);
extern int excharger_write_byte(kal_uint8 cmd, kal_uint8 writeData);


extern void bq24157_set_ac_charging_current(kal_uint32 val);
#endif // _BQ24157_SW_H_

