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

#ifndef __EINT_H__
#define __EINT_H__

/*
 * Define hardware registers.
 */

#define EINT_STA_BASE         ((EINT_BASE + 0x000))
#define EINT_INTACK_BASE      ((EINT_BASE + 0x040))
#define EINT_MASK_BASE        ((EINT_BASE + 0x080))
#define EINT_MASK_SET_BASE    ((EINT_BASE + 0x0c0))
#define EINT_MASK_CLR_BASE    ((EINT_BASE + 0x100))
#define EINT_SENS_BASE        ((EINT_BASE + 0x140))
#define EINT_SENS_SET_BASE    ((EINT_BASE + 0x180))
#define EINT_SENS_CLR_BASE    ((EINT_BASE + 0x1c0))
#define EINT_SOFT_BASE        ((EINT_BASE + 0x200))
#define EINT_SOFT_SET_BASE    ((EINT_BASE + 0x240))
#define EINT_SOFT_CLR_BASE    ((EINT_BASE + 0x280))
#define EINT_POL_BASE         ((EINT_BASE + 0x300))
#define EINT_POL_SET_BASE     ((EINT_BASE + 0x340))
#define EINT_POL_CLR_BASE     ((EINT_BASE + 0x380))
#define EINT_D0_EN_BASE       ((EINT_BASE + 0x400))
#define EINT_D1_EN_BASE       ((EINT_BASE + 0x420))
#define EINT_D2_EN_BASE       ((EINT_BASE + 0x440))
#define EINT_DBNC_BASE        ((EINT_BASE + 0x500))
#define EINT_DBNC_SET_BASE    ((EINT_BASE + 0x600))
#define EINT_DBNC_CLR_BASE    ((EINT_BASE + 0x700))
#define DEINT_CON_BASE        ((EINT_BASE + 0x800))
#define DEINT_SEL_BASE        ((EINT_BASE + 0x840))
#define DEINT_SEL_SET_BASE    ((EINT_BASE + 0x880))
#define DEINT_SEL_CLR_BASE    ((EINT_BASE + 0x8c0))
#define EINT_EEVT_BASE	      ((EINT_BASE + 0x900))
#define EINT_EMUL_BASE        ((EINT_BASE + 0xF00))
#define EINT_DBNC_SET_DBNC_BITS    (4)
#define EINT_DBNC_CLR_DBNC_BITS    (4)
#define EINT_DBNC_SET_EN_BITS      (0)
#define EINT_DBNC_CLR_EN_BITS      (0)
#define EINT_DBNC_SET_RST_BITS     (1)

#define EINT_DBNC_EN_BIT           (0x1)
#define EINT_DBNC_RST_BIT          (0x1)

#define EINT_DBNC_0_MS             (0x7)
#define EINT_DBNC                  (0x7)
#define EINT_DBNC_SET_EN           (0x1)
#define EINT_DBNC_CLR_EN           (0x1)


/* */

#define EINT_STA_DEFAULT	0x00000000
#define EINT_INTACK_DEFAULT	0x00000000
#define EINT_EEVT_DEFAULT	0x00000001
#define EINT_MASK_DEFAULT	0x00000000
#define EINT_MASK_SET_DEFAULT	0x00000000
#define EINT_MASK_CLR_DEFAULT	0x00000000
#define EINT_SENS_DEFAULT	0x0000FFFF
#define EINT_SENS_SET_DEFAULT	0x00000000
#define EINT_SENS_CLR_DEFAULT	0x00000000
#define EINT_D0EN_DEFAULT	0x00000000
#define EINT_D1EN_DEFAULT	0x00000000
#define EINT_D2EN_DEFAULT	0x00000000
#define EINT_DBNC_DEFAULT(n)	0x00000000
#define DEINT_MASK_DEFAULT      0x00000000
#define DEINT_MASK_SET_DEFAULT  0x00000000
#define DEINT_MASK_CLR_DEFAULT  0x00000000

/*Sten: PMIC wrapper start*/
#define PMIC_BASE (0x0000)
#define PMIC_WRP_CKPDN            (PMIC_BASE+0x011A) //0x0056

#define PMIC_EINT_STA_BASE         ((EINT_BASE + 0xA00))
#define PMIC_EINT_INTACK_BASE      ((EINT_BASE + 0xA08))
#define PMIC_EINT_EEVT_BASE	   ((EINT_BASE + 0xA10))
#define PMIC_EINT_MASK_BASE        ((EINT_BASE + 0xA18))
#define PMIC_EINT_MASK_SET_BASE    ((EINT_BASE + 0xA20))
#define PMIC_EINT_MASK_CLR_BASE    ((EINT_BASE + 0xA28))
#define PMIC_EINT_SENS_BASE        ((EINT_BASE + 0xA30))
#define PMIC_EINT_SENS_SET_BASE    ((EINT_BASE + 0xA38))
#define PMIC_EINT_SENS_CLR_BASE    ((EINT_BASE + 0xA40))
#define PMIC_EINT_POL_BASE        ((EINT_BASE + 0xA48))
#define PMIC_EINT_POL_SET_BASE    ((EINT_BASE + 0xA50))
#define PMIC_EINT_POL_CLR_BASE    ((EINT_BASE + 0xA58))
#define PMIC_EINT_CON(n)	((EINT_BASE + 0xA80 + 4 * (n)))
#define PMIC_EINT_D0_EN_BASE       ((EINT_BASE + 0xA60))
#define PMIC_EINT_D1_EN_BASE       ((EINT_BASE + 0xA68))
#define PMIC_EINT_D2_EN_BASE       ((EINT_BASE + 0xA70))
#define PMIC_EINT_EMUL_BASE        ((EINT_BASE + 0xB28))

#define EINT_AT_PMIC                    ((EINT_BASE + 0x0200))
#define PMIC_EINT_INPUT_MUX_BASE             ((EINT_BASE + 0x0C10))
#define PMIC_EINT_INPUT_MUX_SET_BASE         ((EINT_BASE + 0x0C14))
#define PMIC_EINT_INPUT_MUX_CLR_BASE         ((EINT_BASE + 0x0C18))
#define PMIC_EINT_INPUT_SOFT_BASE            ((EINT_BASE + 0x0C20))
#define PMIC_EINT_INPUT_SOFT_SET_BASE        ((EINT_BASE + 0x0C24))
#define PMIC_EINT_INPUT_SOFT_CLR_BASE        ((EINT_BASE + 0x0C28))
#define PMIC_EINT_SOFT_BASE PMIC_EINT_INPUT_SOFT_BASE
#define PMIC_EINT_SOFT_SET_BASE PMIC_EINT_INPUT_SOFT_SET_BASE
#define PMIC_EINT_SOFT_CLR_BASE PMIC_EINT_INPUT_SOFT_CLR_BASE
/*
 * Define constants.
 */
#define EINT_AP_MAXNUMBER 192
//#define EINT_MAX_CHANNEL 192
#define EINT_MAX_CHANNEL 217
#define MT65XX_EINT_POL_NEG (0)
#define MT65XX_EINT_POL_POS (1)
#define MAX_HW_DEBOUNCE_CNT 16
#define MAX_DEINT_CNT 8

/*
 * Define function prototypes.
 */

extern void mt65xx_eint_mask(unsigned int eint_num);
extern void mt65xx_eint_unmask(unsigned int eint_num);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern int mt65xx_eint_init(void);
extern void mt_eint_print_status(void);
#endif  /*!__EINT_H__ */
