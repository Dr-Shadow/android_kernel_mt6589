/***
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK-DISTRIBUTED SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK-DISTRIBUTED SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK-DISTRIBUTED
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK-DISTRIBUTED SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM.
 */
#ifndef __MTK_MEMCFG_H__
#define __MTK_MEMCFG_H__

#ifdef CONFIG_MTK_MEMCFG

#define MTK_MEMCFG_LOG_AND_PRINTK(fmt, arg...)  \
    do {    \
        printk(fmt, ##arg); \
        mtk_memcfg_write_memory_layout_buf(fmt, ##arg); \
    } while (0)

extern void mtk_memcfg_write_memory_layout_buf(char *, ...); 
extern unsigned long mtk_memcfg_get_force_inode_gfp_lowmem(void);
extern unsigned long mtk_memcfg_set_force_inode_gfp_lowmem(unsigned long);
#ifdef CONFIG_SLUB_DEBUG
extern unsigned long mtk_memcfg_get_bypass_slub_debug_flag(void);
extern unsigned long mtk_memcfg_set_bypass_slub_debug_flag(unsigned long);
#else
#define mtk_memcfg_get_bypass_slub_debug_flag() (0)
#define mtk_memcfg_set_bypass_slub_debug_flag(flag) (0)
#endif /* end CONFIG_SLUB_DEBUG */

#else

#define MTK_MEMCFG_LOG_AND_PRINTK(fmt, arg...)  \
    do {    \
        printk(fmt, ##arg); \
    } while (0)

#define mtk_memcfg_get_force_inode_gfp_lowmem() (0)
#define mtk_memcfg_set_force_inode_gfp_lowmem(flag) (0)
#define mtk_memcfg_get_bypass_slub_debug_flag() (0)
#define mtk_memcfg_set_bypass_slub_debug_flag(flag) (0)
#define mtk_memcfg_write_memory_layout_buf(fmt, arg...) do { } while (0)

#endif /* end CONFIG_MTK_MEMCFG */

#endif /* end __MTK_MEMCFG_H__ */
