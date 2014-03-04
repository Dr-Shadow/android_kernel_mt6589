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
*  permission of MediaTek Inc. (C) 2007                                                        
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


                                                                                               
#include <mach/mt_typedefs.h>                                                              
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/xlog.h>

                                                                                               
#include "jpeg_drv_6589_reg.h"                                                                 
#include "jpeg_drv_6589_common.h"  
                                                                                               


#define JPEG_ENC_RST_BIT                        0x1

#define JPEG_ENC_CTRL_ENABLE_BIT                0x01
#define JPEG_ENC_CTRL_DIS_GMC_BIT               0x02
#define JPEG_ENC_CTRL_INT_EN_BIT                0x04
#define JPEG_ENC_CTRL_YUV_BIT                   0x18
#define JPEG_ENC_CTRL_FILE_FORMAT_BIT           0x20
#define JPEG_ENC_CTRL_GRAY_EN_BIT               0x80
#define JPEG_ENC_CTRL_ULTRA_HIGH_EN_BIT         0x200
#define JPEG_ENC_CTRL_RESTART_EN_BIT            0x400
#define JPEG_ENC_CTRL_BURST_TYPE_MASK           0x00007000
#define JPEG_ENC_CTRL_BURST_TYPE_SHIFT_COUNT    12

#define JPEG_ENC_EN_DIS_GMC                     (1 << 2)
#define JPEG_ENC_EN_JFIF_EXIF                   (1 << 5)
#define JPEG_ENC_EN_SELF_INIT                   (1 << 16)

#define JPEG_ENC_DEBUG_INFO0_GMC_IDLE_MASK      (1 << 13)

#if 0
#define JPEG_MSG(...)   xlog_printk(ANDROID_LOG_DEBUG, "xlog/jpeg", __VA_ARGS__)
#define JPEG_WRN(...)   xlog_printk(ANDROID_LOG_WARN,  "xlog/jpeg", __VA_ARGS__)
#define JPEG_ERR(...)   xlog_printk(ANDROID_LOG_ERROR, "xlog/jpeg", __VA_ARGS__)
#else
#define JPEG_MSG printk
#define JPEG_WRN printk
#define JPEG_ERR printk
#endif
kal_uint32 _jpeg_enc_int_status = 0; 




int jpeg_isr_enc_lisr(void)
{
   unsigned int tmp, tmp1 ;
    //_jpeg_enc_int_status = REG_JPEG_ENC_INTERRUPT_STATUS;
    tmp1 = REG_JPEG_ENC_INTERRUPT_STATUS ;
    tmp = tmp1 & 0x3;
    if(tmp){
      _jpeg_enc_int_status = tmp ;
      
      /// clear the interrupt status register
      //if(_jpeg_enc_int_status)
      //{
          IMG_REG_WRITE( 0, REG_ADDR_JPEG_ENC_INTERRUPT_STATUS);   //REG_JPEG_ENC_INTERRUPT_STATUS = 0;
          return 0;
      //}
    }else if(_jpeg_enc_int_status){
      IMG_REG_WRITE( 0, REG_ADDR_JPEG_ENC_INTERRUPT_STATUS);   //REG_JPEG_ENC_INTERRUPT_STATUS = 0;
      return 0; 
    }

    return -1;
}





kal_uint32 jpeg_drv_enc_set_src_image(kal_uint32 width, kal_uint32 height, kal_uint32 yuv_format, kal_uint32 totalEncDU)
{

   kal_uint32 ret = 1;   
   
   ret &= jpeg_drv_enc_set_img_size(width, height);

   ret &= jpeg_drv_enc_set_encFormat(yuv_format) ;    
   
   ret &= jpeg_drv_enc_set_blk_num(totalEncDU);
   
        
   return ret;   
   
}



kal_uint32 jpeg_drv_enc_set_src_buf(kal_uint32 yuv_format, kal_uint32 img_stride, kal_uint32 mem_stride, kal_uint32 srcAddr, kal_uint32 srcAddr_C)
{
   kal_uint32 ret = 1;
   if( yuv_format == 0x00 || yuv_format == 0x01 ){
     if( (mem_stride & 0x1f) || (img_stride & 0x1f) ){
       JPEG_MSG("JPEGENC: set image/memory stride not align 0x1f in fmt %x(%x/%x)!!\n", yuv_format, mem_stride, img_stride);
       ret = 0; 
     }
   }
    
   ret &= jpeg_drv_enc_set_image_stride(img_stride);

   ret &= jpeg_drv_enc_set_memory_stride(mem_stride);
    
   ret &= jpeg_drv_enc_set_luma_addr(srcAddr);
    
   ret &= jpeg_drv_enc_set_chroma_addr(srcAddr_C);
     
   return ret;
}


///////////////////////////////////////////////////////////////////////////////





kal_uint32 jpeg_drv_enc_ctrl_cfg( kal_uint32 exif_en, kal_uint32 quality, kal_uint32 restart_interval)
{
   
    jpeg_drv_enc_set_quality(quality);

    jpeg_drv_enc_set_restart_interval(restart_interval);

    jpeg_drv_enc_set_EncodeMode(exif_en);
    
    return 1;

    //if (0 != ctrl_cfg.gmc_disable)
    //    REG_JPEG_ENC_CTRL |= JPEG_ENC_CTRL_DIS_GMC_BIT;
    //
    //REG_JPEG_ENC_CTRL |= JPEG_ENC_EN_SELF_INIT;
}

void jpeg_drv_enc_dump_reg(void)
{
    unsigned int reg_value = 0;
    unsigned int index = 0;

    JPEG_MSG("===== JPEG ENC DUMP =====\n");
    for(index = 0x100 ; index < JPEG_ENC_REG_COUNT ; index += 4){
#ifdef FPGA_VERSION      
        reg_value = *(volatile kal_uint32 *)(JPEG_ENC_BASE + index);
#else        
        reg_value = ioread32(JPEG_ENC_BASE + index);
#endif        
        JPEG_MSG("+0x%x 0x%08x\n", index, reg_value);
    }
}

void jpeg_drv_enc_start(void)
{
   unsigned int u4Value ;
   
   u4Value = REG_JPEG_ENC_CTRL ;
   u4Value |= (JPEG_ENC_CTRL_INT_EN_BIT | JPEG_ENC_CTRL_ENABLE_BIT);
   IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_CTRL);   
   //REG_JPEG_ENC_CTRL |= (JPEG_ENC_CTRL_INT_EN_BIT | JPEG_ENC_CTRL_ENABLE_BIT);
}

void jpeg_drv_enc_reset(void)
{
   IMG_REG_WRITE( (0), REG_ADDR_JPEG_ENC_RSTB);  //REG_JPEG_ENC_RSTB = 0;
   IMG_REG_WRITE( (1), REG_ADDR_JPEG_ENC_RSTB);  //REG_JPEG_ENC_RSTB = 1;

   _jpeg_enc_int_status = 0;
}


kal_uint32 jpeg_drv_enc_warm_reset(void)
{
    kal_uint32 timeout = 0xFFFFF;
    
    REG_JPEG_ENC_CTRL &= ~JPEG_ENC_CTRL_ENABLE_BIT;
    REG_JPEG_ENC_CTRL |= JPEG_ENC_CTRL_ENABLE_BIT;


    while (0 == (REG_JPEG_ENC_DEBUG_INFO0 & JPEG_ENC_DEBUG_INFO0_GMC_IDLE_MASK))
    {
        timeout--;
        if (0 == timeout)
        {
            JPEG_MSG("Wait for GMC IDLE timeout\n");
            return 0;
        }
    }
       
    REG_JPEG_ENC_RSTB &= ~(JPEG_ENC_RST_BIT);
    REG_JPEG_ENC_RSTB |= JPEG_ENC_RST_BIT;

    _jpeg_enc_int_status = 0;

    return 1;
}


kal_uint32 jpeg_drv_enc_set_encFormat(kal_uint32 encFormat)
{
   kal_uint32 val ;
   unsigned int u4Value ;
   
   if(encFormat & (~3)){
       JPEG_ERR("JPEG_DRV_ENC: set encFormat Err %d!!\n", encFormat );
       return 0;
   }
         
   val = (encFormat & 3) << 3;
#if 0
//   REG_JPEG_ENC_CTRL &= ~JPEG_ENC_CTRL_YUV_BIT;   
//   
//   REG_JPEG_ENC_CTRL |= val;
#else

   u4Value = REG_JPEG_ENC_CTRL ;
   
   u4Value &= ~JPEG_ENC_CTRL_YUV_BIT;   
   
   u4Value |= val;   
   
   IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_CTRL);  

#endif   
   
   return 1;
      
}



kal_uint32 jpeg_drv_enc_set_quality(kal_uint32 quality)
{
   unsigned int u4Value ;
    if (quality == 0x8 || quality == 0xC)
    {
        JPEG_MSG("JPEGENC: set quality failed\n");
        return 0;
    }
    u4Value = REG_JPEG_ENC_QUALITY;
    u4Value = (u4Value & 0xFFFF0000) | quality ;
    IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_QUALITY);  //REG_JPEG_ENC_QUALITY = (REG_JPEG_ENC_QUALITY & 0xFFFF0000) | quality;
    return 1;
}

kal_uint32 jpeg_drv_enc_set_img_size(kal_uint32 width, kal_uint32 height)
{
   unsigned int u4Value ;
    if ((width & 0xffff0000) || (height & 0xffff0000))
    {
        JPEG_MSG("JPEGENC: img size exceed 65535, (%x, %x)!!\n", width, height);
        return 0;
    }
    u4Value = (width << 16) | height ;
    
    IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_IMG_SIZE);   //REG_JPEG_ENC_IMG_SIZE = (width << 16) | height;

    return 1;
}

kal_uint32 jpeg_drv_enc_set_blk_num(kal_uint32 blk_num)  //NO_USE
{
    if (blk_num < 4)
    {        
        return 0;
    }

    IMG_REG_WRITE( (blk_num), REG_ADDR_JPEG_ENC_BLK_NUM);   //REG_JPEG_ENC_BLK_NUM = blk_num;
    
    return 1;
}

kal_uint32 jpeg_drv_enc_set_luma_addr(kal_uint32 src_luma_addr)
{
    if (src_luma_addr & 0x0F)
    {
        JPEG_MSG("JPEGENC: set LUMA addr not align (%x)!!\n", src_luma_addr);
        //return 0;
    }

    IMG_REG_WRITE( (src_luma_addr), REG_ADDR_JPEG_ENC_SRC_LUMA_ADDR);  //REG_JPEG_ENC_SRC_LUMA_ADDR = src_luma_addr;

    return 1;
}

kal_uint32 jpeg_drv_enc_set_chroma_addr(kal_uint32 src_chroma_addr)
{
    if (src_chroma_addr & 0x0F)
    {
        JPEG_MSG("JPEGENC: set CHROMA addr not align (%x)!!\n", src_chroma_addr);
        //return 0;
    }

    IMG_REG_WRITE( (src_chroma_addr), REG_ADDR_JPEG_ENC_SRC_CHROMA_ADDR);  //REG_JPEG_ENC_SRC_CHROMA_ADDR = src_chroma_addr;

    return 1;
}

kal_uint32 jpeg_drv_enc_set_memory_stride(kal_uint32 mem_stride)
{
    if ( mem_stride & 0x0F )
    {
        JPEG_MSG("JPEGENC: set memory stride failed, not align to 0x1f (%x)!!\n", mem_stride);
        return 0;
    }
    
    IMG_REG_WRITE( (mem_stride), REG_ADDR_JPEG_ENC_STRIDE);  //REG_JPEG_ENC_STRIDE = (mem_stride);
    
    return 1;
}

kal_uint32 jpeg_drv_enc_set_image_stride(kal_uint32 img_stride)
{

    if (img_stride & 0x0F)
    {
        JPEG_MSG("JPEGENC: set image stride failed, not align to 0x0f (%x)!!\n", img_stride);    
        return 0;
    }
    
    IMG_REG_WRITE( (img_stride), REG_ADDR_JPEG_ENC_IMG_STRIDE); //REG_JPEG_ENC_IMG_STRIDE = (img_stride);
    
    return 1;
}

void jpeg_drv_enc_set_restart_interval(kal_uint32 restart_interval)
{
   unsigned int u4Value ;
   
   u4Value = REG_JPEG_ENC_CTRL ;
   if (0 != restart_interval)
   {
      u4Value |= JPEG_ENC_CTRL_RESTART_EN_BIT ;
      IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_CTRL); //REG_JPEG_ENC_CTRL |= JPEG_ENC_CTRL_RESTART_EN_BIT;
   }
   else
   {
      u4Value &= ~JPEG_ENC_CTRL_RESTART_EN_BIT;  
      IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_CTRL);  //REG_JPEG_ENC_CTRL &= ~JPEG_ENC_CTRL_RESTART_EN_BIT;
   }
   IMG_REG_WRITE( (restart_interval), REG_ADDR_JPEG_ENC_RST_MCU_NUM);  //REG_JPEG_ENC_RST_MCU_NUM = restart_interval;
}


kal_uint32 jpeg_drv_enc_set_offset_addr(kal_uint32 offset)
{
   if (offset & 0x0F)
   {
      JPEG_MSG("JPEGENC:WARN set offset addr %x\n", offset);
      //return 0;
   }

   IMG_REG_WRITE( (offset), REG_ADDR_JPEG_ENC_OFFSET_ADDR);  //REG_JPEG_ENC_OFFSET_ADDR = offset;

   return 1;
}

kal_uint32 jpeg_drv_enc_set_dst_buff(kal_uint32 dst_addr, kal_uint32 stall_size, kal_uint32 init_offset, kal_uint32 offset_mask)
{
    if (stall_size < 624)
    {
        JPEG_MSG("JPEGENC:stall offset less than 624 to write header %d!!\n",stall_size);        
        return 0;
    }

   if (offset_mask & 0x0F)
   {
      JPEG_MSG("JPEGENC: set offset addr %x\n", offset_mask);
      //return 0;
   }

   IMG_REG_WRITE( (init_offset & (~0xF)), REG_ADDR_JPEG_ENC_OFFSET_ADDR);   //REG_JPEG_ENC_OFFSET_ADDR = init_offset & (~0xF);

   IMG_REG_WRITE( (offset_mask & 0xF), REG_ADDR_JPEG_ENC_BYTE_OFFSET_MASK);   //REG_JPEG_ENC_BYTE_OFFSET_MASK = (offset_mask & 0xF);
   
   IMG_REG_WRITE( (dst_addr & (~0xF)), REG_ADDR_JPEG_ENC_DST_ADDR0);   //REG_JPEG_ENC_DST_ADDR0  = dst_addr & (~0xF);
   
   IMG_REG_WRITE( ((dst_addr + stall_size) & (~0xF)), REG_ADDR_JPEG_ENC_STALL_ADDR0);   //REG_JPEG_ENC_STALL_ADDR0 = (dst_addr + stall_size) & (~0xF);
   

   return 1;
}

// 0:JPG mode, 1:JFIF/EXIF mode
void jpeg_drv_enc_set_EncodeMode(kal_uint32 exif_en)
{
   unsigned int u4Value ;
   
   u4Value = REG_JPEG_ENC_CTRL;
   u4Value &= ~(JPEG_ENC_CTRL_FILE_FORMAT_BIT);
   IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_CTRL);   //REG_JPEG_ENC_CTRL &= ~(JPEG_ENC_CTRL_FILE_FORMAT_BIT);

   if (exif_en)
   {
      u4Value = REG_JPEG_ENC_CTRL; 
      u4Value |= JPEG_ENC_EN_JFIF_EXIF;
      IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_CTRL);   //REG_JPEG_ENC_CTRL |= JPEG_ENC_EN_JFIF_EXIF;
   }
}

void jpeg_drv_enc_set_gmc_disable_bit(void)
{
   unsigned int u4Value ;
   u4Value = REG_JPEG_ENC_CTRL ;
   u4Value |= JPEG_ENC_EN_DIS_GMC;
   IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_CTRL);
   
}


void jpeg_drv_enc_set_burst_type(kal_uint32 burst_type)
{
   unsigned int u4Value ;
   
   u4Value = REG_JPEG_ENC_CTRL ;
   
   u4Value &= ~JPEG_ENC_CTRL_BURST_TYPE_MASK;
   u4Value |= (burst_type << JPEG_ENC_CTRL_BURST_TYPE_SHIFT_COUNT);
   
   
   IMG_REG_WRITE( (u4Value), REG_ADDR_JPEG_ENC_CTRL);
}

kal_uint32  jpeg_drv_enc_get_cycle_count()
{
    return REG_JPEG_ENC_TOTAL_CYCLE;
}


kal_uint32 jpeg_drv_enc_get_file_size()
{
    return REG_JPEG_ENC_DMA_ADDR0 - REG_JPEG_ENC_DST_ADDR0;
    //return REG_JPEG_ENC_CURR_DMA_ADDR - REG_JPEG_ENC_DST_ADDR0; 
}

#ifdef FPGA_VERSION

kal_uint32 jpeg_drv_enc_get_result()
{
    kal_uint32 file_size;
    file_size = jpeg_drv_enc_get_file_size();
    return file_size;
}
#else

kal_uint32 jpeg_drv_enc_get_result(kal_uint32 *fileSize)
{
    *fileSize = jpeg_drv_enc_get_file_size();

    if(_jpeg_enc_int_status & JPEG_DRV_ENC_INT_STATUS_DONE)
    {
        return 0;
    }
    else if(_jpeg_enc_int_status & JPEG_DRV_ENC_INT_STATUS_STALL)
    {
        return 1;
    }
    JPEG_MSG("JPEGENC: int_st %x\n", _jpeg_enc_int_status);
    return 2;
}



#endif


                                                          