/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2009
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

/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <linux/pmic6326_sw.h>
#include <linux/delay.h>
#include "yusu_android_speaker.h"

#if defined(MT6575)
#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_clock_manager.h>
#include <mach/mt_pmic_feature_api.h>
#elif defined(MT6577)
#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_clock_manager.h>
#include <mach/mt_pmic_feature_api.h>
#endif

/*****************************************************************************
*                C O M P I L E R      F L A G S
******************************************************************************
*/
//#define CONFIG_DEBUG_MSG
#ifdef CONFIG_DEBUG_MSG
#define PRINTK(format, args...) printk( KERN_EMERG format,##args )
#else
#define PRINTK(format, args...)
#endif
//#define AMP_CLASS_AB
#define AMP_CLASS_D
#define ENABLE_2_IN_1_SPK

#if !defined(AMP_CLASS_AB) && !defined(AMP_CLASS_D)
#error "MT6329 SPK AMP TYPE does not be defined!!!"
#endif
/*****************************************************************************
*                          C O N S T A N T S
******************************************************************************
*/

#define SPK_WARM_UP_TIME        (55) //unit is ms
#define SPK_AMP_GAIN            (4)  //4:15dB
#define RCV_AMP_GAIN            (1)  //1:-3dB
#define SPK_R_ENABLE            (1)
#define SPK_L_ENABLE            (1)
/*****************************************************************************
*                         D A T A      T Y P E S
******************************************************************************
*/
static int Speaker_Volume=0;
static bool gsk_on=false; // speaker is open?
static bool gsk_resume=false;
static bool gsk_forceon=false;
/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/
extern void Yusu_Sound_AMP_Switch(BOOL enable);

bool Speaker_Init(void)
{
   PRINTK("+Speaker_Init Success");
#if defined(AMP_CLASS_AB)
   hwSPKClassABInit();
   hwSPKClassABVolumeControl(SPK_AMP_GAIN,SPK_AMP_GAIN);//4:15dB, 6:21dB
#elif defined(AMP_CLASS_D)
   hwSPKClassDInit();
   hwSPKClassDVolumeControl(SPK_AMP_GAIN,SPK_AMP_GAIN);//4:15dB, 6:21dB
#endif
   hwSPKClassABVoiceBypassAnalogGain(RCV_AMP_GAIN);
   PRINTK("-Speaker_Init Success");
   return true;
}

bool Speaker_Register(void)
{
   return false;
}

int ExternalAmp(void)
{
	return 0;
}

bool Speaker_DeInit(void)
{
	return false;
}

void Sound_SpeakerL_SetVolLevel(int level)
{
   PRINTK(" Sound_SpeakerL_SetVolLevel level=%d\n",level);
}

void Sound_SpeakerR_SetVolLevel(int level)
{
   PRINTK(" Sound_SpeakerR_SetVolLevel level=%d\n",level);
}

void Sound_Speaker_Turnon(int channel)
{
    PRINTK("Sound_Speaker_Turnon channel = %d\n",channel);
    if(gsk_on)
        return;
#if defined(ENABLE_2_IN_1_SPK)
#if defined(AMP_CLASS_D)
    hwSPKClassDInit();
    hwSPKClassDVolumeControl(SPK_AMP_GAIN,SPK_AMP_GAIN);//4:15dB, 6:21dB
#endif
#endif
    hw2in1SpeakerSetting(0,0);//0=>spk mode; 1=>receiver mode
#if defined(AMP_CLASS_AB)
    hwSPKClassABTurnOn(SPK_L_ENABLE,SPK_R_ENABLE);
#elif defined(AMP_CLASS_D)
    hwSPKClassDTurnOn(SPK_L_ENABLE,SPK_R_ENABLE);
#endif
    msleep(SPK_WARM_UP_TIME);
    gsk_on = true;
}

void Sound_Speaker_Turnoff(int channel)
{
    PRINTK("Sound_Speaker_Turnoff channel = %d\n",channel);
	if(!gsk_on)
		return;
#if defined(AMP_CLASS_AB)
    hwSPKClassABTurnOff();
#elif defined(AMP_CLASS_D)
    hwSPKClassDTurnOff();
#endif
	gsk_on = false;
}

void Sound_Speaker_SetVolLevel(int level)
{
    Speaker_Volume =level;
}

void Sound_Headset_Turnon(void)
{
}

void Sound_Headset_Turnoff(void)
{
}

void Sound_Earpiece_Turnon(void)
{
#if defined(ENABLE_2_IN_1_SPK)
#if defined(AMP_CLASS_D)
        hwSPKClassABInit();
#endif
        hw2in1SpeakerSetting(1,1);//0=>spk mode; 1=>receiver mode
        hwSPKClassABVoiceBypassAnalogGain(RCV_AMP_GAIN);
        hwSPKClassABTurnOn(SPK_L_ENABLE,SPK_R_ENABLE);
#endif
}

void Sound_Earpiece_Turnoff(void)
{
#if defined(ENABLE_2_IN_1_SPK)
        hwSPKClassABTurnOff();
        //hw2in1SpeakerSetting(0,0);//0=>spk mode; 1=>receiver mode
#if defined(AMP_CLASS_D)
        hwSPKClassDInit();
#endif
#endif
}

//kernal use
void AudioAMPDevice_Suspend(void)
{
	PRINTK("AudioDevice_Suspend\n");
	if(gsk_on)
	{
		Sound_Speaker_Turnoff(Channel_Stereo);
		gsk_resume = true;
	}

}
void AudioAMPDevice_Resume(void)
{
	PRINTK("AudioDevice_Resume\n");
	if(gsk_resume)
		Sound_Speaker_Turnon(Channel_Stereo);
	gsk_resume = false;
}
void AudioAMPDevice_SpeakerLouderOpen(void)
{
	PRINTK("AudioDevice_SpeakerLouderOpen\n");
	gsk_forceon = false;
	if(gsk_on)
		return;
	Sound_Speaker_Turnon(Channel_Stereo);
	gsk_forceon = true;
	return ;

}
void AudioAMPDevice_SpeakerLouderClose(void)
{
	PRINTK("AudioDevice_SpeakerLouderClose\n");

	if(gsk_forceon)
		Sound_Speaker_Turnoff(Channel_Stereo);
	gsk_forceon = false;

}
void AudioAMPDevice_mute(void)
{
	PRINTK("AudioDevice_mute\n");
	if(gsk_on)
	    Sound_Speaker_Turnoff(Channel_Stereo);
}

int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count)
{
    switch(type){
        case EAMP_EARPIECE_OPEN:
            //printk("Audio_eamp_command EAMP_EARPIECE_OPEN");
            Sound_Earpiece_Turnon();
            break;
        case EAMP_EARPIECE_CLOSE:
            //printk("Audio_eamp_command EAMP_EARPIECE_CLOSE");
            Sound_Earpiece_Turnoff();
            break;
        default :
            break;
     }
     return 0;
}

static char *ExtFunArray[] =
{
    "InfoMATVAudioStart",
    "InfoMATVAudioStop",
    "End",
};

kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size)
{
	int i = 0;
	int funNum = -1;

	//Search the supported function defined in ExtFunArray
	while(strcmp("End",ExtFunArray[i]) != 0 ) {		//while function not equal to "End"

	    if (strcmp(name,ExtFunArray[i]) == 0 ) {		//When function name equal to table, break
	    	funNum = i;
	    	break;
	    }
	    i++;
	}

	switch (funNum) {
	    case 0:			//InfoMATVAudioStart
	        printk("RunExtFunction InfoMATVAudioStart \n");
	        break;

	    case 1:			//InfoMATVAudioStop
	        printk("RunExtFunction InfoMATVAudioStop \n");
	        break;

	    default:
	    	 break;
	}

	return 1;
}


