/********************************************************************************************
 *     LEGAL DISCLAIMER 
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES 
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED 
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS 
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED, 
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR 
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY 
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, 
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK 
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION 
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *     
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH 
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, 
 *     TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE 
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
 *     
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS 
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.  
 ************************************************************************************************/
#ifndef _VDEC_DRV_JPEG_INFO_H_
#define _VDEC_DRV_JPEG_INFO_H_

//#include "x_os.h"
//#include "x_bim.h"
//#include "x_assert.h"
//#include "x_timer.h"

#include "drv_common.h"

#include "vdec_common_if.h"
#include "vdec_usage.h"

//#include "vdec_info_mpeg.h"
#include "vdec_info_common.h"

/******************************************************************************
* Local definition
******************************************************************************/



extern void vJPEGInitProc(UCHAR ucEsId);
extern INT32 i4JPEGVParseProc(UCHAR ucEsId, UINT32 u4VParseType);
extern BOOL fgJPEGVParseChkProc(UCHAR ucEsId);
extern INT32 i4JPEGUpdInfoToFbg(UCHAR ucEsId);
extern void vJPEGStartToDecProc(UCHAR ucEsId);
extern void vJPEGISR(UCHAR ucEsId);
extern BOOL fgIsJPEGDecEnd(UCHAR ucEsId);
extern BOOL fgIsJPEGDecErr(UCHAR ucEsId);
extern BOOL fgJPEGResultChk(UCHAR ucEsId);
extern BOOL fgIsJPEGInsToDispQ(UCHAR ucEsId);
extern BOOL fgIsJPEGGetFrmToDispQ(UCHAR ucEsId);
extern void vJPEGEndProc(UCHAR ucEsId);
extern BOOL fgJPEGFlushDPB(UCHAR ucEsId, BOOL fgWithOutput);    
extern void vJPEGReleaseProc(UCHAR ucEsId, BOOL fgResetHw);
extern INT32 i4JPEGParser(VDEC_ES_INFO_T* prVDecEsInfo, UINT32 u4VParseType);


extern VDEC_DRV_IF* VDec_GetJPEGIf(void);


#endif // _VDEC_DRV_JPEG_INFO_H_