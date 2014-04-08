/*************************************************************
 * 
 * Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
 * 
 * Date Created: 2012-11-20 15:56
 * 
 * Author: Mingqiang.Guo@BasicDrv.Sensor
 * 
 * Description : NULL
 * 
 * ------------------ Revision History: ---------------------
 *      <author>        <date>          <desc>
 *
 *************************************************************/ 
#ifndef ALSPS_PRIVATE_H
#define ALSPS_PRIVATE_H

#define ALS_GAIN_2772  10/7 //13/7//5/2
#define ALS_GAIN_2771  1

#define TMD2771_PPCOUNT  0x02
#define TMD2771_CONTROL  0x22	

#define TMD2772_PPCOUNT  0x0a //25 pulse number
#define TMD2772_CONTROL  0x22//0x62 //22-100ma,62-50ma ;A2-25ma ;E2-12.5ma 

#define TMD2772_PSOFFSET  0// 0x40//0x7f

#define AtimeToMS(x) ((256 - (x)) * 272 / 100)
#define A_TIME (0xEE)
#define TSL_GA (90)
#define AGAIN (0x10)
#define CUSTOM_DEVICE_ID 0x39
//#define ALSPS_CALI_DATA_RELOAD
#endif
