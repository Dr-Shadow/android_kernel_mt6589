/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID_NT35590 (0x90)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#if defined(MTK_WFD_SUPPORT)
#define   LCM_DSI_CMD_MODE							1
#else
#define   LCM_DSI_CMD_MODE							0
#endif


static void init_lcm_registers(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00023902;                          
    data_array[1] = 0x00009036;                 
    dsi_set_cmdq(data_array, 2, 1); 

	data_array[0] = 0x00023902;//CMD1                           
    data_array[1] = 0x000000FF;                 
    dsi_set_cmdq(data_array, 2, 1);     
    	
    data_array[0] = 0x00023902;//03 4lane  02 3lanes               
    data_array[1] = 0x000002BA;                 
    dsi_set_cmdq(data_array, 2, 1);    
    	
    data_array[0] = 0x00023902;//03 Video 08 command
    #if (LCM_DSI_CMD_MODE)
		data_array[1] = 0x000008C2; 
    #else
		data_array[1] = 0x000003C2; 
    #endif                
    dsi_set_cmdq(data_array, 2, 1);   
    	
    data_array[0] = 0x00023902;//CMD2,Page0  
    data_array[1] = 0x000001FF;                 
    dsi_set_cmdq(data_array, 2, 1);   
    	
    data_array[0] = 0x00023902;//720*1280 
    data_array[1] = 0x00003A00;                 
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00023902;
    data_array[1] = 0x00003301; //4401                
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00023902;
    data_array[1] = 0x00005302; //5402               
    dsi_set_cmdq(data_array, 2, 1); 
	
	data_array[0] = 0x00023902;//VGL=-6V 
    data_array[1] = 0x00008509; //0309                
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00023902;//VGH=+8.6V 
    data_array[1] = 0x0000250E;                 
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00023902;//turn off VGLO regulator   
    data_array[1] = 0x00000A0F;                 
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00023902;//GVDDP=4V     
    data_array[1] = 0x0000970B;                 
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00023902;
    data_array[1] = 0x0000970C;                 
    dsi_set_cmdq(data_array, 2, 1);  

    data_array[0] = 0x00023902; 
    data_array[1] = 0x00008611; //8611                
    dsi_set_cmdq(data_array, 2, 1); 

	data_array[0] = 0x00023902;//VCOMDC 
    data_array[1] = 0x00000312;                 
    dsi_set_cmdq(data_array, 2, 1); 
    	
    data_array[0] = 0x00023902;  
    data_array[1] = 0x00007B36;                 
    dsi_set_cmdq(data_array, 2, 1);
	
#if 1
	data_array[0] = 0x00023902;  
    data_array[1] = 0x000080B0;                 
    dsi_set_cmdq(data_array, 2, 1); 

	data_array[0] = 0x00023902;  
    data_array[1] = 0x000002B1;                 
    dsi_set_cmdq(data_array, 2, 1); 
#endif 

    data_array[0] = 0x00023902;//GVDDP=4V     
    data_array[1] = 0x00002C71;                 
    dsi_set_cmdq(data_array, 2, 1);  
#if 1
    data_array[0] = 0x00023902;
    data_array[1] = 0x000005FF;         
    dsi_set_cmdq(data_array, 2, 1);   

	data_array[0] = 0x00023902; /////////////LTPS 
    data_array[1] = 0x00000001;                   
    dsi_set_cmdq(data_array, 2, 1);              
    data_array[0] = 0x00023902;                   
    data_array[1] = 0x00008D02;                   
    dsi_set_cmdq(data_array, 2, 1);              
    data_array[0] = 0x00023902;                   
    data_array[1] = 0x00008D03;                   
    dsi_set_cmdq(data_array, 2, 1);              
    data_array[0] = 0x00023902;                   
    data_array[1] = 0x00008D04;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00003005;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;//06         
    data_array[1] = 0x00003306;             
    dsi_set_cmdq(data_array, 2, 1); 
	
    data_array[0] = 0x00023902;             
    data_array[1] = 0x00007707;             
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902;             
    data_array[1] = 0x00000008;        
    dsi_set_cmdq(data_array, 2, 1);   
    data_array[0] = 0x00023902;        
    data_array[1] = 0x00000009;        
    dsi_set_cmdq(data_array, 2, 1);   
    data_array[0] = 0x00023902;        
    data_array[1] = 0x0000000A;        
    dsi_set_cmdq(data_array, 2, 1);   
    data_array[0] = 0x00023902;        
    data_array[1] = 0x0000800B;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;//0C 
    data_array[1] = 0x0000C80C;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902; //0D
    data_array[1] = 0x0000000D;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00001B0E; 
	
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000070F;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00005710;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00000011;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;//12 
    data_array[1] = 0x00000012;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;            
    data_array[1] = 0x00001E13;            
    dsi_set_cmdq(data_array, 2, 1);       
    data_array[0] = 0x00023902;            
    data_array[1] = 0x00000014;            
    dsi_set_cmdq(data_array, 2, 1);       
    data_array[0] = 0x00023902;            
    data_array[1] = 0x00001A15;            
    dsi_set_cmdq(data_array, 2, 1);       
    data_array[0] = 0x00023902;            
    data_array[1] = 0x00000516;            
    dsi_set_cmdq(data_array, 2, 1); 
	
    data_array[0] = 0x00023902;            
    data_array[1] = 0x00000017;             
    dsi_set_cmdq(data_array, 2, 1);     
    data_array[0] = 0x00023902;//12 
    data_array[1] = 0x00001E18;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;            
    data_array[1] = 0x0000FF19;            
    dsi_set_cmdq(data_array, 2, 1);       
    data_array[0] = 0x00023902;            
    data_array[1] = 0x0000001A;            
    dsi_set_cmdq(data_array, 2, 1);       
    data_array[0] = 0x00023902;            
    data_array[1] = 0x0000FC1B;            
    dsi_set_cmdq(data_array, 2, 1);       
    data_array[0] = 0x00023902;            
    data_array[1] = 0x0000801C;            
    dsi_set_cmdq(data_array, 2, 1);       
    data_array[0] = 0x00023902;            
    data_array[1] = 0x0000001D; //101D            
    dsi_set_cmdq(data_array, 2, 1);     
    data_array[0] = 0x00023902;
	data_array[1] = 0x0000101E; //011E            
	dsi_set_cmdq(data_array, 2, 1);     
			                                     
	data_array[0] = 0x00023902;          
    data_array[1] = 0x0000771F;          
    dsi_set_cmdq(data_array, 2, 1);  
	data_array[0] = 0x00023902;                                   
    data_array[1] = 0x00000020;          
    dsi_set_cmdq(data_array, 2, 1);     
    data_array[0] = 0x00023902;          
    data_array[1] = 0x00000221;         
    dsi_set_cmdq(data_array, 2, 1);     
    data_array[0] = 0x00023902;          
    data_array[1] = 0x00000022; //5522          
    dsi_set_cmdq(data_array, 2, 1);      
    data_array[0] = 0x00023902;            
    data_array[1] = 0x00000D23;            
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902;//06 
    data_array[1] = 0x0000A031;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00000032;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000B833;         
    dsi_set_cmdq(data_array, 2, 1);
	
    data_array[0] = 0x00023902;            
    data_array[1] = 0x0000BB34;            
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902;             
    data_array[1] = 0x00001135;             
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902;             
    data_array[1] = 0x00000136;             
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902;//0C         
    data_array[1] = 0x00000B37;             
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902; //0D        
    data_array[1] = 0x00000138;             
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902;             
    data_array[1] = 0x00000B39;             
    dsi_set_cmdq(data_array, 2, 1); 	
    data_array[0] = 0x00023902;             
    data_array[1] = 0x00000844;             
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902;             
    data_array[1] = 0x00008045;             
    dsi_set_cmdq(data_array, 2, 1); 
	
    data_array[0] = 0x00023902;                
    data_array[1] = 0x0000CC46;                
    dsi_set_cmdq(data_array, 2, 1);           
    data_array[0] = 0x00023902;//12            
    data_array[1] = 0x00000447;                
    dsi_set_cmdq(data_array, 2, 1);           
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000048;                          
    dsi_set_cmdq(data_array, 2, 1);                     
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000049;                                 
    dsi_set_cmdq(data_array, 2, 1);                            
    data_array[0] = 0x00023902;                                 
    data_array[1] = 0x0000014A;                                 
    dsi_set_cmdq(data_array, 2, 1);  
	data_array[0] = 0x00023902;                                 
    data_array[1] = 0x0000036C;                                 
    dsi_set_cmdq(data_array, 2, 1);                            
    data_array[0] = 0x00023902;                                 
    data_array[1] = 0x0000036D;                                 
    dsi_set_cmdq(data_array, 2, 1);                            
    data_array[0] = 0x00023902;//18                             
    data_array[1] = 0x00002F6E;                                 
	dsi_set_cmdq(data_array, 2, 1); 		
			
    data_array[0] = 0x00023902; ////
    data_array[1] = 0x00000043;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000234B;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000014C;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;      
    data_array[1] = 0x00002350;      
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902;      
    data_array[1] = 0x00000151;      
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902;//06  
    data_array[1] = 0x00002358;      
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902;      
    data_array[1] = 0x00000159;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000235D;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000015E;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00002362;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00000163;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;//0C 
    data_array[1] = 0x00002367;       
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902; //0D
    data_array[1] = 0x00000168;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00000089;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000018D;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000648E;
    dsi_set_cmdq(data_array, 2, 1);
	
    data_array[0] = 0x00023902;                       
    data_array[1] = 0x0000208F;                       
    dsi_set_cmdq(data_array, 2, 1); 	
	data_array[0] = 0x00023902;//12                   
    data_array[1] = 0x00008E97;                       
    dsi_set_cmdq(data_array, 2, 1);                  
    data_array[0] = 0x00023902;                                 
    data_array[1] = 0x00008C82;                                 
    dsi_set_cmdq(data_array, 2, 1);                            
    data_array[0] = 0x00023902;                                 
    data_array[1] = 0x00000283;                                 
    dsi_set_cmdq(data_array, 2, 1);                            
    data_array[0] = 0x00023902;                                 
    data_array[1] = 0x00000ABB;                                 
    dsi_set_cmdq(data_array, 2, 1);                            
    data_array[0] = 0x00023902;                                 
    data_array[1] = 0x00000ABC; // 02BC                                
    dsi_set_cmdq(data_array, 2, 1);                            
    data_array[0] = 0x00023902;                                 
    data_array[1] = 0x00002524;                                 
    dsi_set_cmdq(data_array, 2, 1);                            
    data_array[0] = 0x00023902;//18                             
    data_array[1] = 0x00005525;                                 
	dsi_set_cmdq(data_array, 2, 1); 	
			
	data_array[0] = 0x00023902;      
    data_array[1] = 0x00000526;      
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902;      
    data_array[1] = 0x00002327;      
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902;      
    data_array[1] = 0x00000128;      
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902;      
    data_array[1] = 0x00003129; // 0029     
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902;      
    data_array[1] = 0x00005D2A;      
    dsi_set_cmdq(data_array, 2, 1); 
    data_array[0] = 0x00023902;//06 
    data_array[1] = 0x0000012B;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x0000002F;     
    dsi_set_cmdq(data_array, 2, 1);
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00001030; 
    dsi_set_cmdq(data_array, 2, 1); 
	
    data_array[0] = 0x00023902;             
    data_array[1] = 0x000012A7;             
    dsi_set_cmdq(data_array, 2, 1);        
    data_array[0] = 0x00023902;             
    data_array[1] = 0x0000032D;             
    dsi_set_cmdq(data_array, 2, 1);
#endif

    data_array[0] = 0x00023902;////CMD1 
    data_array[1] = 0x000000FF;                 
    dsi_set_cmdq(data_array, 2, 1);    
    data_array[0] = 0x00023902;
    data_array[1] = 0x000001FB;                 
    dsi_set_cmdq(data_array, 2, 1);    
    data_array[0] = 0x00023902;//CMD2,Page0 
    data_array[1] = 0x000001FF;                 
    dsi_set_cmdq(data_array, 2, 1);       
    data_array[0] = 0x00023902;
    data_array[1] = 0x000001FB;                 
    dsi_set_cmdq(data_array, 2, 1);   
    data_array[0] = 0x00023902;//CMD2,Page1 
    data_array[1] = 0x000002FF;                 
    dsi_set_cmdq(data_array, 2, 1);       	
    data_array[0] = 0x00023902;
    data_array[1] = 0x000001FB;                 
    dsi_set_cmdq(data_array, 2, 1);       
    	
    data_array[0] = 0x00023902;//CMD2,Page2 
    data_array[1] = 0x000003FF;                 
    dsi_set_cmdq(data_array, 2, 1);       	
    data_array[0] = 0x00023902;
    data_array[1] = 0x000001FB;                 
    dsi_set_cmdq(data_array, 2, 1);     
    data_array[0] = 0x00023902;//CMD2,Page3
    data_array[1] = 0x000004FF;         
    dsi_set_cmdq(data_array, 2, 1);                                        
    data_array[0] = 0x00023902;         
    data_array[1] = 0x000001FB;         
    dsi_set_cmdq(data_array, 2, 1);    
    data_array[0] = 0x00023902;//CMD2,Page4
    data_array[1] = 0x000005FF;         
    dsi_set_cmdq(data_array, 2, 1);    
    data_array[0] = 0x00023902;         
    data_array[1] = 0x000001FB;         
    dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00023902;     ////CMD1     
    data_array[1] = 0x000000FF;         
    dsi_set_cmdq(data_array, 2, 1); 

	/*******debug-----start********/
	data_array[0] = 0x00110500;                
    dsi_set_cmdq(data_array, 1, 1); 
    MDELAY(120); 
    	
    data_array[0] = 0x00023902;//not open CABC    
    data_array[1] = 0x0000FF51;         
    dsi_set_cmdq(data_array, 2, 1);    
    	                                    
    data_array[0] = 0x00023902;         
    data_array[1] = 0x00002C53;         
    dsi_set_cmdq(data_array, 2, 1); 
    	
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00000055;         
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00290500;                
    dsi_set_cmdq(data_array, 1, 1); 
    	
    data_array[0] = 0x00023902;         
    data_array[1] = 0x000000FF;         
    dsi_set_cmdq(data_array, 2, 1); 
    	
    data_array[0] = 0x00023902;     
    data_array[1] = 0x00000035;         
    dsi_set_cmdq(data_array, 2, 1); 
	
	data_array[0] = 0x00033902;
	data_array[1] = (((FRAME_HEIGHT/2)&0xFF) << 16) | (((FRAME_HEIGHT/2)>>8) << 8) | 0x44;
	dsi_set_cmdq(data_array, 2, 1);
	/*******debug-----end********/

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 1;// 3    2
		params->dsi.vertical_backporch					= 1;// 20   1
		params->dsi.vertical_frontporch					= 2; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;// 50  2
		params->dsi.horizontal_backporch				= 12;
		params->dsi.horizontal_frontporch				= 80;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
		params->dsi.pll_select=1;
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
#if (LCM_DSI_CMD_MODE)
		params->dsi.fbk_div =19;
#else
		params->dsi.fbk_div =20;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif

}

static void lcm_init(void)
{
	    unsigned int data_array[16];
		
		SET_RESET_PIN(0);
		MDELAY(20); 
		SET_RESET_PIN(1);
		MDELAY(20); 
	
		data_array[0] = 0x00023902;
		data_array[1] = 0x0000EEFF; 				
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(2); 
		data_array[0] = 0x00023902;
		data_array[1] = 0x00000826; 				
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(2); 
		data_array[0] = 0x00023902;
		data_array[1] = 0x00000026; 				
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(2); 
		data_array[0] = 0x00023902;
		data_array[1] = 0x000000FF; 				
		dsi_set_cmdq(data_array, 2, 1);
		
		SET_RESET_PIN(0);
		MDELAY(20); 
		SET_RESET_PIN(1);
		MDELAY(20); 
	
		data_array[0] = 0x00023902; 						 
		data_array[1] = 0x00009036; 				
		dsi_set_cmdq(data_array, 2, 1); 
	
		data_array[0] = 0x00023902;//CMD1							
		data_array[1] = 0x000000FF; 				
		dsi_set_cmdq(data_array, 2, 1); 	
			
		data_array[0] = 0x00023902;//03 4lane  02 3lanes			   
		data_array[1] = 0x000002BA; 				
		dsi_set_cmdq(data_array, 2, 1);    
			
		data_array[0] = 0x00023902;//03 Video 08 command
    #if (LCM_DSI_CMD_MODE)
			data_array[1] = 0x000008C2; 
    #else
			data_array[1] = 0x000003C2; 
    #endif                
		dsi_set_cmdq(data_array, 2, 1);   
			
		data_array[0] = 0x00023902;//CMD2,Page0  
		data_array[1] = 0x000001FF; 				
		dsi_set_cmdq(data_array, 2, 1);   
			
		data_array[0] = 0x00023902;//720*1280 
		data_array[1] = 0x00003A00; 				
		dsi_set_cmdq(data_array, 2, 1);  
			
		data_array[0] = 0x00023902;
		data_array[1] = 0x00003301; //4401				  
		dsi_set_cmdq(data_array, 2, 1);  
			
		data_array[0] = 0x00023902;
		data_array[1] = 0x00005302; //5402				 
		dsi_set_cmdq(data_array, 2, 1); 
		
		data_array[0] = 0x00023902;//VGL=-6V 
		data_array[1] = 0x00008509; //0309				  
		dsi_set_cmdq(data_array, 2, 1);  
			
		data_array[0] = 0x00023902;//VGH=+8.6V 
		data_array[1] = 0x0000250E; 				
		dsi_set_cmdq(data_array, 2, 1);  
			
		data_array[0] = 0x00023902;//turn off VGLO regulator   
		data_array[1] = 0x00000A0F; 				
		dsi_set_cmdq(data_array, 2, 1);  
			
		data_array[0] = 0x00023902;//GVDDP=4V	  
		data_array[1] = 0x0000970B; 				
		dsi_set_cmdq(data_array, 2, 1);  
			
		data_array[0] = 0x00023902;
		data_array[1] = 0x0000970C; 				
		dsi_set_cmdq(data_array, 2, 1);  
	
		data_array[0] = 0x00023902; 
		data_array[1] = 0x00008611; //8611				  
		dsi_set_cmdq(data_array, 2, 1); 
	
		data_array[0] = 0x00023902;//VCOMDC 
		data_array[1] = 0x00000312; 				
		dsi_set_cmdq(data_array, 2, 1); 
			
		data_array[0] = 0x00023902;  
		data_array[1] = 0x00007B36; 				
		dsi_set_cmdq(data_array, 2, 1);
		
#if 1
		data_array[0] = 0x00023902;  
		data_array[1] = 0x000080B0; 				
		dsi_set_cmdq(data_array, 2, 1); 
	
		data_array[0] = 0x00023902;  
		data_array[1] = 0x000002B1; 				
		dsi_set_cmdq(data_array, 2, 1); 
#endif 
	
		data_array[0] = 0x00023902;//GVDDP=4V	  
		data_array[1] = 0x00002C71; 				
		dsi_set_cmdq(data_array, 2, 1);  
#if 1
		data_array[0] = 0x00023902;
		data_array[1] = 0x000005FF; 		
		dsi_set_cmdq(data_array, 2, 1);   
	
		data_array[0] = 0x00023902; /////////////LTPS 
		data_array[1] = 0x00000001; 				  
		dsi_set_cmdq(data_array, 2, 1); 			 
		data_array[0] = 0x00023902; 				  
		data_array[1] = 0x00008D02; 				  
		dsi_set_cmdq(data_array, 2, 1); 			 
		data_array[0] = 0x00023902; 				  
		data_array[1] = 0x00008D03; 				  
		dsi_set_cmdq(data_array, 2, 1); 			 
		data_array[0] = 0x00023902; 				  
		data_array[1] = 0x00008D04; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00003005; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902;//06 		
		data_array[1] = 0x00003306; 			
		dsi_set_cmdq(data_array, 2, 1); 
		
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x00007707; 			
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x00000008; 	   
		dsi_set_cmdq(data_array, 2, 1);   
		data_array[0] = 0x00023902; 	   
		data_array[1] = 0x00000009; 	   
		dsi_set_cmdq(data_array, 2, 1);   
		data_array[0] = 0x00023902; 	   
		data_array[1] = 0x0000000A; 	   
		dsi_set_cmdq(data_array, 2, 1);   
		data_array[0] = 0x00023902; 	   
		data_array[1] = 0x0000800B; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902;//0C 
		data_array[1] = 0x0000C80C; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; //0D
		data_array[1] = 0x0000000D; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00001B0E; 
		
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000070F; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00005710; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00000011; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902;//12 
		data_array[1] = 0x00000012; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x00001E13; 		   
		dsi_set_cmdq(data_array, 2, 1); 	  
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x00000014; 		   
		dsi_set_cmdq(data_array, 2, 1); 	  
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x00001A15; 		   
		dsi_set_cmdq(data_array, 2, 1); 	  
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x00000516; 		   
		dsi_set_cmdq(data_array, 2, 1); 
		
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x00000017; 			
		dsi_set_cmdq(data_array, 2, 1); 	
		data_array[0] = 0x00023902;//12 
		data_array[1] = 0x00001E18; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x0000FF19; 		   
		dsi_set_cmdq(data_array, 2, 1); 	  
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x0000001A; 		   
		dsi_set_cmdq(data_array, 2, 1); 	  
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x0000FC1B; 		   
		dsi_set_cmdq(data_array, 2, 1); 	  
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x0000801C; 		   
		dsi_set_cmdq(data_array, 2, 1); 	  
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x0000001D; //101D			  
		dsi_set_cmdq(data_array, 2, 1); 	
		data_array[0] = 0x00023902;
		data_array[1] = 0x0000101E; //011E			  
		dsi_set_cmdq(data_array, 2, 1); 	
													 
		data_array[0] = 0x00023902; 		 
		data_array[1] = 0x0000771F; 		 
		dsi_set_cmdq(data_array, 2, 1);  
		data_array[0] = 0x00023902; 								  
		data_array[1] = 0x00000020; 		 
		dsi_set_cmdq(data_array, 2, 1); 	
		data_array[0] = 0x00023902; 		 
		data_array[1] = 0x00000221; 		
		dsi_set_cmdq(data_array, 2, 1); 	
		data_array[0] = 0x00023902; 		 
		data_array[1] = 0x00000022; //5522			
		dsi_set_cmdq(data_array, 2, 1); 	 
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x00000D23; 		   
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902;//06 
		data_array[1] = 0x0000A031; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00000032; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000B833; 		
		dsi_set_cmdq(data_array, 2, 1);
		
		data_array[0] = 0x00023902; 		   
		data_array[1] = 0x0000BB34; 		   
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x00001135; 			
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x00000136; 			
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902;//0C 		
		data_array[1] = 0x00000B37; 			
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902; //0D		
		data_array[1] = 0x00000138; 			
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x00000B39; 			
		dsi_set_cmdq(data_array, 2, 1); 	
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x00000844; 			
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x00008045; 			
		dsi_set_cmdq(data_array, 2, 1); 
		
		data_array[0] = 0x00023902; 			   
		data_array[1] = 0x0000CC46; 			   
		dsi_set_cmdq(data_array, 2, 1); 		  
		data_array[0] = 0x00023902;//12 		   
		data_array[1] = 0x00000447; 			   
		dsi_set_cmdq(data_array, 2, 1); 		  
		data_array[0] = 0x00023902; 						 
		data_array[1] = 0x00000048; 						 
		dsi_set_cmdq(data_array, 2, 1); 					
		data_array[0] = 0x00023902; 						 
		data_array[1] = 0x00000049; 								
		dsi_set_cmdq(data_array, 2, 1); 						   
		data_array[0] = 0x00023902; 								
		data_array[1] = 0x0000014A; 								
		dsi_set_cmdq(data_array, 2, 1);  
		data_array[0] = 0x00023902; 								
		data_array[1] = 0x0000036C; 								
		dsi_set_cmdq(data_array, 2, 1); 						   
		data_array[0] = 0x00023902; 								
		data_array[1] = 0x0000036D; 								
		dsi_set_cmdq(data_array, 2, 1); 						   
		data_array[0] = 0x00023902;//18 							
		data_array[1] = 0x00002F6E; 								
		dsi_set_cmdq(data_array, 2, 1); 		
				
		data_array[0] = 0x00023902; ////
		data_array[1] = 0x00000043; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000234B; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000014C; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	 
		data_array[1] = 0x00002350; 	 
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902; 	 
		data_array[1] = 0x00000151; 	 
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902;//06  
		data_array[1] = 0x00002358; 	 
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902; 	 
		data_array[1] = 0x00000159; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000235D; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000015E; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00002362; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00000163; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902;//0C 
		data_array[1] = 0x00002367; 	  
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902; //0D
		data_array[1] = 0x00000168; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00000089; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000018D; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000648E;
		dsi_set_cmdq(data_array, 2, 1);
		
		data_array[0] = 0x00023902; 					  
		data_array[1] = 0x0000208F; 					  
		dsi_set_cmdq(data_array, 2, 1); 	
		data_array[0] = 0x00023902;//12 				  
		data_array[1] = 0x00008E97; 					  
		dsi_set_cmdq(data_array, 2, 1); 				 
		data_array[0] = 0x00023902; 								
		data_array[1] = 0x00008C82; 								
		dsi_set_cmdq(data_array, 2, 1); 						   
		data_array[0] = 0x00023902; 								
		data_array[1] = 0x00000283; 								
		dsi_set_cmdq(data_array, 2, 1); 						   
		data_array[0] = 0x00023902; 								
		data_array[1] = 0x00000ABB; 								
		dsi_set_cmdq(data_array, 2, 1); 						   
		data_array[0] = 0x00023902; 								
		data_array[1] = 0x00000ABC; // 02BC 							   
		dsi_set_cmdq(data_array, 2, 1); 						   
		data_array[0] = 0x00023902; 								
		data_array[1] = 0x00002524; 								
		dsi_set_cmdq(data_array, 2, 1); 						   
		data_array[0] = 0x00023902;//18 							
		data_array[1] = 0x00005525; 								
		dsi_set_cmdq(data_array, 2, 1); 	
				
		data_array[0] = 0x00023902; 	 
		data_array[1] = 0x00000526; 	 
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902; 	 
		data_array[1] = 0x00002327; 	 
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902; 	 
		data_array[1] = 0x00000128; 	 
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902; 	 
		data_array[1] = 0x00003129; // 0029 	
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902; 	 
		data_array[1] = 0x00005D2A; 	 
		dsi_set_cmdq(data_array, 2, 1); 
		data_array[0] = 0x00023902;//06 
		data_array[1] = 0x0000012B; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x0000002F; 	
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00001030; 
		dsi_set_cmdq(data_array, 2, 1); 
		
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x000012A7; 			
		dsi_set_cmdq(data_array, 2, 1); 	   
		data_array[0] = 0x00023902; 			
		data_array[1] = 0x0000032D; 			
		dsi_set_cmdq(data_array, 2, 1);
#endif
	
		data_array[0] = 0x00023902;////CMD1 
		data_array[1] = 0x000000FF; 				
		dsi_set_cmdq(data_array, 2, 1);    
		data_array[0] = 0x00023902;
		data_array[1] = 0x000001FB; 				
		dsi_set_cmdq(data_array, 2, 1);    
		data_array[0] = 0x00023902;//CMD2,Page0 
		data_array[1] = 0x000001FF; 				
		dsi_set_cmdq(data_array, 2, 1); 	  
		data_array[0] = 0x00023902;
		data_array[1] = 0x000001FB; 				
		dsi_set_cmdq(data_array, 2, 1);   
		data_array[0] = 0x00023902;//CMD2,Page1 
		data_array[1] = 0x000002FF; 				
		dsi_set_cmdq(data_array, 2, 1); 		
		data_array[0] = 0x00023902;
		data_array[1] = 0x000001FB; 				
		dsi_set_cmdq(data_array, 2, 1); 	  
			
		data_array[0] = 0x00023902;//CMD2,Page2 
		data_array[1] = 0x000003FF; 				
		dsi_set_cmdq(data_array, 2, 1); 		
		data_array[0] = 0x00023902;
		data_array[1] = 0x000001FB; 				
		dsi_set_cmdq(data_array, 2, 1); 	
		data_array[0] = 0x00023902;//CMD2,Page3
		data_array[1] = 0x000004FF; 		
		dsi_set_cmdq(data_array, 2, 1); 									   
		data_array[0] = 0x00023902; 		
		data_array[1] = 0x000001FB; 		
		dsi_set_cmdq(data_array, 2, 1);    
		data_array[0] = 0x00023902;//CMD2,Page4
		data_array[1] = 0x000005FF; 		
		dsi_set_cmdq(data_array, 2, 1);    
		data_array[0] = 0x00023902; 		
		data_array[1] = 0x000001FB; 		
		dsi_set_cmdq(data_array, 2, 1);
		data_array[0] = 0x00023902; 	////CMD1	 
		data_array[1] = 0x000000FF; 		
		dsi_set_cmdq(data_array, 2, 1); 
	
		/*******debug-----start********/
		data_array[0] = 0x00110500; 			   
		dsi_set_cmdq(data_array, 1, 1); 
		MDELAY(120); 
	
		data_array[0] = 0x00023902;
		data_array[1] = 0x0000EEFF; 				
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(1); 
		data_array[0] = 0x00023902;
		data_array[1] = 0x00005012; 				
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(1); 
		data_array[0] = 0x00023902;
		data_array[1] = 0x00000213; 				
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(1); 
		data_array[0] = 0x00023902;
		data_array[1] = 0x0000606A; 				
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(1); 
		data_array[0] = 0x00023902;
		data_array[1] = 0x000000FF; 				
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(1); 
		data_array[0] = 0x00023902; 	
		data_array[1] = 0x00000035; 		
		dsi_set_cmdq(data_array, 2, 1); 
		
		data_array[0] = 0x00033902;
		data_array[1] = (((FRAME_HEIGHT/2)&0xFF) << 16) | (((FRAME_HEIGHT/2)>>8) << 8) | 0x44;
		dsi_set_cmdq(data_array, 2, 1);
			
		data_array[0] = 0x00290500; 			   
		dsi_set_cmdq(data_array, 1, 1); 
		MDELAY(50);
}



static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	
	SET_RESET_PIN(1);	
	SET_RESET_PIN(0);
	MDELAY(1); // 1ms
	
	SET_RESET_PIN(1);
	MDELAY(120);      
}


static void lcm_resume(void)
{
	lcm_init();

    #ifdef BUILD_LK
	  printf("[LK]---cmd---nt35590----%s------\n",__func__);
    #else
	  printk("[KERNEL]---cmd---nt35590----%s------\n",__func__);
    #endif	
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20); 

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
    #ifdef BUILD_LK
		printf("%s, LK nt35590 debug: nt35590 id = 0x%08x\n", __func__, id);
    #else
		printk("%s, kernel nt35590 horse debug: nt35590 id = 0x%08x\n", __func__, id);
    #endif

    if(id == LCM_ID_NT35590)
    	return 1;
    else
        return 0;


}


static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x36, buffer, 1);
	if(buffer[0]==0x90)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	lcm_resume();

	return TRUE;
}



LCM_DRIVER nt35590_hd720_dsi_vdo_truly_lcm_drv = 
{
    .name			= "nt35590_hd720_dsi_vdo_truly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
