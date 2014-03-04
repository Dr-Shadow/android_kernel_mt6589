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

#ifndef __MT_MON_H__
#define __MT_MON_H__

#include "mach/pmu_v7.h"
#include <linux/threads.h>

struct mt_mon_log
{

    __u32 cpu_cyc[NR_CPUS];
    __u32 cpu_cnt0[NR_CPUS];
    __u32 cpu_cnt1[NR_CPUS];
    __u32 cpu_cnt2[NR_CPUS];
    __u32 cpu_cnt3[NR_CPUS];

    __u32 BM_BCNT;
    __u32 BM_TACT;
    __u32 BM_TSCT;
    __u32 BM_WACT;
    __u32 BM_WSCT;
    __u32 BM_BACT;
    __u32 BM_BSCT;
    __u32 BM_TSCT2;
    __u32 BM_WSCT2;
    __u32 BM_TSCT3;
    __u32 BM_WSCT3;
    __u32 BM_WSCT4;
    __u32 BM_TTYPE1;
    __u32 BM_TTYPE3;
    __u32 BM_TTYPE4;
    __u32 BM_TTYPE5;
    __u32 BM_TTYPE6;
    __u32 BM_TTYPE7;
    
    __u32 BM_TTYPE9;
    __u32 BM_TTYPE11;
    __u32 BM_TTYPE12;
    __u32 BM_TTYPE13;
    __u32 BM_TTYPE14;
    __u32 BM_TTYPE15;

    __u32 MCI_CNT0;
    __u32 MCI_CNT1;

    __u32 BM_TPCT1;
    
    __u32 DRAMC_PageHit;
    __u32 DRAMC_PageMiss;
    __u32 DRAMC_Interbank;
    __u32 DRAMC_Idle;   
};
typedef enum 
{
    MODE_MANUAL_USER,
    MODE_MANUAL_KERNEL,
    MODE_SCHED_SWITCH,
    MODE_PERIODIC,
    MODE_MANUAL_TRACER,
    MODE_FREE    
} MonitorMode;

extern void set_mt65xx_mon_period(long time_ns);
extern long get_mt65xx_mon_period(void);
extern void set_mt65xx_mon_manual_start(unsigned int bStart);
extern unsigned int get_mt65xx_mon_manual_start(void);
extern void set_mt65xx_mon_mode(MonitorMode mode);
extern MonitorMode get_mt65xx_mon_mode(void);

extern int mt65xx_mon_init(void);
extern int mt65xx_mon_deinit(void);
extern int mt65xx_mon_enable(void);
extern int mt65xx_mon_disable(void);
extern unsigned int mt65xx_mon_log(void*);

struct l2c_cfg{
    u32 l2c_evt[2];
};


struct mtk_monitor {
    int				(*init)(void);
    int				(*deinit)(void);
    int            	(*enable)(void);
    int            	(*disable)(void);
        unsigned int	(*mon_log)(void *);
    void 			(*set_pmu)(struct pmu_cfg *p_cfg);
    void 			(*get_pmu)(struct pmu_cfg *p_cfg);
    void 			(*set_l2c)(struct l2c_cfg *l_cfg);
    void 			(*get_l2c)(struct l2c_cfg *l_cfg);
    void			(*set_bm_rw)(int type);
    struct mt_mon_log *log_buff;
};

int register_monitor(struct mtk_monitor **mtk_mon, MonitorMode mode);
void unregister_monitor(struct mtk_monitor **mtk_mon);

#endif  /* !__MT65XX_MON_H__ */
