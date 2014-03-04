/*****************************************************************************
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *  ft5306_driver.c
 *
 * Project:
 * --------
 *  W900
 *
 * Author:
 * -------
 *  cheehwa,"yuanjianhua@konka.com"
 *
 * Description:
 * ------------
 *  CTP's driver for FocalTech's chip FT5306
 *
 *
 *============================================================================
 * History:
 * Sun Feb 12 2012, creat
 *
 *============================================================================
 ****************************************************************************/
#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include "tpd_custom_ft5306.h"
#include "cust_gpio_usage.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>


/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/
extern struct tpd_device *tpd;
extern int tpd_firmware_version[2];
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

/*****************************************************************************
*           F U N C T I O N      D E L A R A T I O N
******************************************************************************
*/
void tpd_button(unsigned int x, unsigned int y, unsigned int down);
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static void tpd_eint_interrupt_handler(void);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

/*****************************************************************************
*                          C O N S T A N T S
******************************************************************************
*/
static int boot_mode = 0;
static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;

#define TP_DEBUG
//#define TP_LIB_UPGRADE_SWITCH
#define TP_LIB_NAME               "Ft5x16_mz.i"
#define TP_LIB_VERSION            0x14
#define FT5X0X_REG_FIRMID         0xA6
#define FT5X06_MOD_ID             0xA8

#define I2C_CTP_ADDRESS    0x70

#define FTS_PACKET_LENGTH         128
#define TPD_OK 0
#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02
/*
#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12

#define TOUCH4_XH 0x15                  ///               WU     
#define TOUCH4_XL 0x16
#define TOUCH4_YH 0x17
#define TOUCH4_YL 0x18

#define TOUCH5_XH 0x1B
#define TOUCH5_XL 0x1C
#define TOUCH5_YH 0x1D
#define TOUCH5_YL 0x1E       
*/
#define FTS_NULL                    0x0
#define FTS_TRUE                    0x1
#define FTS_FALSE                   0x0

#if defined(TP_DEBUG)
#define pr_tp(format, args...) printk("<0>" format, ##args)
#define pr_ch(format, arg...)                      \
    printk("<0>" "%s <%d>,%s(),cheehwa_print:\n\t"  \
           format,__FILE__,__LINE__,__func__, ##arg)
#else
#define pr_tp(format, args...)  do {} while (0)
#define pr_ch(format, arg...)  do {} while (0)
#undef pr_k(format, arg...)
#define pr_k(format, arg...)  do {} while (0)
#endif

/*****************************************************************************
*                         D A T A      T Y P E S
******************************************************************************
*/
typedef unsigned char         FTS_BYTE;
typedef unsigned short        FTS_WORD;
typedef unsigned int          FTS_DWRD;
typedef unsigned char         FTS_BOOL;

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

typedef struct _REPORT_FINGER_INFO_T
{
    short   ui2_id;               /* ID information, from 0 to  CFG_MAX_POINT_NUM - 1*/
    short    u2_pressure;    /* ***pressure information, valid from 0 -63 **********/
    short    i2_x;                /*********** X coordinate, 0 - 2047 ****************/
    short    i2_y;                /* **********Y coordinate, 0 - 2047 ****************/
} REPORT_FINGER_INFO_T;


struct touch_info
{
    int y[5];
    int x[5];
    int p[5];
    int count;
};
/*
int ft5206_keycodes[4] =
{
	KEY_HOME,
    KEY_MENU,
    KEY_BACK,
    KEY_SEARCH
};
*/

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
} E_UPGRADE_ERR_TYPE;

static const struct i2c_device_id tpd_id[] = {{TPD_DEVICE, 0}, {}};
//static unsigned short force[] = {0, I2C_CTP_ADDRESS, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata ft5306_i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (I2C_CTP_ADDRESS>>1))};

/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/
static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
        .name = TPD_DEVICE,
        .owner = THIS_MODULE,
    },
    .probe = tpd_probe,
    .remove = __devexit_p(tpd_remove),
    .id_table = tpd_id,
    .detect = tpd_detect,
//    .address_data = &addr_data,
};


static unsigned char CTPM_FW[] =
{
	#ifdef TP_LIB_UPGRADE_SWITCH //shaokai
	#include TP_LIB_NAME
	#endif
};


static int fts_i2c_txdata(u8 *txdata, int length)
{
    int ret;
    struct i2c_msg msg;
    msg.addr = i2c_client->addr;
    msg.flags = 0;
    msg.len = length;
    msg.buf = txdata;
    //msg.timing=100;//shaokai
    ret = i2c_transfer(i2c_client->adapter, &msg, 1);

    if(ret < 0)
    {
        pr_err("%s i2c write error: %d\n", __func__, ret);
    }

    return ret;
}

static bool i2c_write_interface(u8 *pbt_buf, int dw_lenth)
{
    int ret;
    ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);

    // ret = i2c_smbus_write_i2c_block_data(i2c_client, *pbt_buf, dw_lenth-1, pbt_buf+1);
    if(ret <= 0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

static bool i2c_read_interface(u8 *pbt_buf, int dw_lenth)
{
    int ret;
    ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);
    //  ret=i2c_smbus_read_i2c_block_data(i2c_client, *pbt_buf, dw_lenth-1, pbt_buf+1);

    if(ret <= 0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

static bool byte_write(u8 *buffer, int length)
{
    return i2c_write_interface(buffer, length);
}

static bool byte_read(u8 *buffer, int length)
{
    return i2c_read_interface(buffer, length);
}

static bool fts_register_write(u8 reg_name, u8 *tx_buf)
{
    u8 write_cmd[2] = {0};
    write_cmd[0] = reg_name;
    write_cmd[1] = *tx_buf;
    /*call the write callback function*/
    return i2c_write_interface(write_cmd, 2);
}

static bool fts_register_read(u8 reg_name, u8 *rx_buf, int rx_length)
{
    u8 read_cmd[2] = {0};
    u8 cmd_len  = 0;
    read_cmd[0] = reg_name;
    cmd_len = 1;

    /*send register addr*/
    if(!i2c_write_interface(&read_cmd[0], cmd_len))
    {
        return FTS_FALSE;
    }

    /*call the read callback function to get the register value*/
    if(!i2c_read_interface(rx_buf, rx_length))
    {
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

static bool fts_register_read_data(u8 reg_name, u8 *rx_buf, int rx_length)
{
    u8 read_cmd[4] = {0};
    u8 cmd_len  = 0;
    read_cmd[0] = reg_name;
    cmd_len = 4;

    /*send register addr*/
    if(!i2c_write_interface(&read_cmd[0], cmd_len))
    {
        return FTS_FALSE;
    }

    /*call the read callback function to get the register value*/
    if(!i2c_read_interface(rx_buf, rx_length))
    {
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/***********************************************************************************************
Name    :

Input   :


Output  :

function    :

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
    int ret;
    struct i2c_msg msg[] =
    {
        {
            .addr   = i2c_client->addr,
            .flags  = 0,
            .len    = length,
            .buf    = txdata,
            //.timing=100//shaokai
        },
    };
    //msleep(1);
    ret = i2c_transfer(i2c_client->adapter, msg, 1);

    if(ret < 0)
    {
        pr_err("%s i2c write error: %d\n", __func__, ret);
    }

    return ret;
}

/***********************************************************************************************
Name    :    ft5x0x_write_reg

Input   :   addr -- address
                     para -- parameter

Output  :

function    :   write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;
    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);

    if(ret < 0)
    {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }

    return 0;
}


/***********************************************************************************************
Name    :   ft5x0x_read_reg

Input   :   addr
                     pdata

Output  :

function    :   read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
    int ret;
    u8 buf[2];
    struct i2c_msg msgs[2];
    //
    buf[0] = addr;    //register address
    msgs[0].addr = i2c_client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = buf;
    //msgs[0].timing=200;//shaokai
    msgs[1].addr = i2c_client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = buf;
    //msgs[1].timing=200;//shaokai
    ret = i2c_transfer(i2c_client->adapter, msgs, 2);

    if(ret < 0)
    {
        pr_err("msg %s i2c read error: %d\n", __func__, ret);
    }

    *pdata = buf[0];
    return ret;
}


static bool cmd_write(u8 btcmd, u8 btPara1, u8 btPara2, u8 btPara3, u8 num)
{
    u8 write_cmd[4] = {0};
    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(write_cmd, num);
}

static E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8 *pbt_buf, int dw_lenth)
{
    u8  cmd, reg_val[2] = {0};
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    u8  auc_i2c_write_buf[10];
    u8  bt_ecc;
    int  j, temp, lenght, i_ret, packet_number, i = 0;
    int  i_is_new_protocol = 0;
    msleep(50);
    /******write 0xaa to register 0xfc******/
    cmd = 0xaa;
    fts_register_write(0xfc, &cmd);
    msleep(50);
    /******write 0x55 to register 0xfc******/
    cmd = 0x55;
    fts_register_write(0xfc, &cmd);
    printk("[TSP] Step_1: Reset CTPM test\n");
    msleep(30);
    /*******Step 2:Enter upgrade mode ****/
    printk("\n[TSP] Step_2:enter new update mode\n");
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;

    do
    {
        i ++;
        i_ret = fts_i2c_txdata(auc_i2c_write_buf, 2);
        msleep(5);
    }
    while(i_ret <= 0 && i < 10);

    if(i > 1)
    {
        i_is_new_protocol = 1;
    }

    /********Step 3:check READ-ID********/
    msleep(20);
    cmd_write(0x90, 0x00, 0x00, 0x00, 4);
    byte_read(reg_val, 2);
    //    fts_register_read_data(0x90, reg_val, 2);
    printk("AF byte_read ID1 = 0x%x,ID2 = 0x%x by cheehwa\n", reg_val[0], reg_val[1]);
    #if 1
    if(reg_val[0] == 0x79 && reg_val[1] == 0x07)
    {
        printk("[TSP] Step_3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x by cheehwa\n", reg_val[0], reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }
    #endif
    /*********Step 4:erase app**********/
    if(i_is_new_protocol)
    {
        cmd_write(0x61, 0x00, 0x00, 0x00, 1);
    }
    else
    {
        cmd_write(0x60, 0x00, 0x00, 0x00, 1);
    }

    mdelay(1500);
    printk("[TSP] Step_4: erase. \n");
    /*Step 5:write firmware(FW) to ctpm flash*/
    bt_ecc = 0;
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    printk("[TSP] Step_5: start upgrade. dw_lenth=%d,packet_number=%d\n", dw_lenth, packet_number);

    for(j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp >> 8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght >> 8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for(i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        byte_write(&packet_buf[0], FTS_PACKET_LENGTH + 6);
        mdelay(FTS_PACKET_LENGTH / 6 + 1);

        if((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
            printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }

        // printk("In the for loop: packet_number=%d,j=%d\n", packet_number, j);
    }

    if((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp >> 8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp >> 8);
        packet_buf[5] = (FTS_BYTE)temp;

        for(i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[ packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        byte_write(&packet_buf[0], temp + 6);
        mdelay(20);
    }

    /***********send the last six byte**********/
    for(i = 0; i < 6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp >> 8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp = 1;
        packet_buf[4] = (FTS_BYTE)(temp >> 8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];
        byte_write(&packet_buf[0], 7);
        mdelay(20);
    }

    /********send the opration head************/
    cmd_write(0xcc, 0x00, 0x00, 0x00, 1);
    byte_read(reg_val, 1);
    //    fts_register_read_data(0xcc,reg_val,1);
    printk("[TSP] Step_6: ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);

    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*******Step 7: reset the new FW**********/
    cmd_write(0x07, 0x00, 0x00, 0x00, 1);
    mdelay(300);
    printk("[TSP] Step_7: reset the new FW, Congratulations upgrade success by cheehwa!! ");
    return ERR_OK;
}

static int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;
    u8 tx_buf = 0x40;
    u8 tx_buf1 = 0x04;
    printk("[FTS] start auto CLB.\n");
    msleep(200);
    fts_register_write(0, &tx_buf);
    //ft5x0x_write_reg(0, 0x40);
    msleep(100);   //make sure already enter factory mode
    fts_register_write(0x4D, &tx_buf1);
    //ft5x0x_write_reg(0x4D, 0x4);  //write command to start calibration
    msleep(300);

    for(i = 0; i < 100; i++)
    {
        //ft5x0x_read_reg(0x4D, &uc_temp);
	fts_register_read(0x4D, &uc_temp,1);
        if(uc_temp == 0x50)    //return to normal mode, calibration finish
        {
            break;
        }

        msleep(200);
        printk("[FTS] waiting_calibration %d\n", i);
    }

    /*******reset the new FW**********/
    ft5x0x_write_reg(0, 0x0); //return to normal mode
    msleep(300);
    printk("[FTS] calibration OK.\n");
    return 0;
}

static int fts_ctpm_fw_upgrade_with_i_file(void)
{
    FTS_BYTE     *pbt_buf = FTS_NULL;
    int i_ret;
    //=========FW upgrade========================*/
    pbt_buf = CTPM_FW;
    /*call the upgrade function*/
    i_ret =  fts_ctpm_fw_upgrade(pbt_buf, sizeof(CTPM_FW));

    if(i_ret != 0)
    {
        printk("[FTS] upgrade_failed i_ret = %d.\n", i_ret);
        //error handling ...
        //TBD
    }
    else
    {
        printk("[FTS] upgrade_successfully.\n");
        //fts_ctpm_auto_clb();  //start auto CLB
    }

    return i_ret;
}

static  void tpd_down(int x, int y, int p)
{
    #if 0//defined(TPD_HAVE_BUTTON)
    if((y==1069)&&((x==0)||(x==1)||(x==2)||(x==3)))
    {
    	y=tpd_keys_dim_local[x][1]; 
    	x=tpd_keys_dim_local[x][0];	
    }
    if((boot_mode == FACTORY_BOOT)||(boot_mode == RECOVERY_BOOT))
    {
    	tpd_button(x, y, 1);
    }
    #endif	    
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    TPD_DOWN_DEBUG_TRACK(x, y);
}
static void tpd_up(int x, int y, int p)
{
    #if 0//defined(TPD_HAVE_BUTTON)
    if((y==1069)&&((x==0)||(x==1)||(x==2)||(x==3)))
    {
    	y=tpd_keys_dim_local[x][1]; 
    	x=tpd_keys_dim_local[x][0];	
    }
    if((boot_mode == FACTORY_BOOT)||(boot_mode == RECOVERY_BOOT))
    {
    	tpd_button(x, y, 0);
    }    
    #endif    	
    input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    //printk("U[%4d %4d %4d] ", x, y, 0);
    input_mt_sync(tpd->dev);
    TPD_UP_DEBUG_TRACK(x, y);
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
    int i = 0;
    char data[32] = {0};
    u16 high_byte, low_byte;
    u8 report_rate =0;
	int mzx = 0;
	int mzy = 0;

    p_point_num = point_num;
    memcpy(pinfo, cinfo, sizeof(struct touch_info));
    memset(cinfo, 0, sizeof(struct touch_info));
    i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
    i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
    i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
    i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));
    i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[32]));
    //i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &report_rate);
	
    TPD_DEBUG("FW version=%x]\n", data[24]);
    TPD_DEBUG("received raw data from touch panel as following:\n");
    TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n", data[0], data[1], data[2], data[3], data[4], data[5]);
    TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n", data[9], data[10], data[11], data[12]);
    TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n", data[15], data[16], data[17], data[18]);




  /*   TPD_DEBUG("[data[1B]=%x,data[1C]= %x ,data[1D]=%x ,data[1E]=%x]\n", data[1B], data[1C], data[1D], data[1E]);               //     WU         
	

    /* Device Mode[2:0] == 0 :Normal operating Mode*/
    if(data[0] & 0x70 != 0)
    {
        return false;
    }

    /*get the number of the touch points*/
    point_num = data[2] & 0x0f;
    TPD_DEBUG("point_num =%d\n", point_num);
    //  if(point_num == 0) return false;
    TPD_DEBUG("Procss raw data...\n");

    for(i = 0; i < point_num; i++)
    {
        cinfo->p[i] = data[3 + 6 * i] >> 6; //event flag
        /*get the X coordinate, 2 bytes*/
        high_byte = data[3 + 6 * i];
        high_byte <<= 8;
        high_byte &= 0x0f00;
        low_byte = data[3 + 6 * i + 1];
	mzx = high_byte | low_byte;
        //cinfo->x[i] = high_byte | low_byte;
        //cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
        /*get the Y coordinate, 2 bytes*/
        high_byte = data[3 + 6 * i + 2];
        high_byte <<= 8;
        high_byte &= 0x0f00;
        low_byte = data[3 + 6 * i + 3];
        //cinfo->y[i] = high_byte | low_byte;
	mzy = high_byte | low_byte;
	
	cinfo->x[i] = 800 - mzy;
	cinfo->y[i] = mzx;
        //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
        cinfo->x[i] = 800 - cinfo->x[i];
	cinfo->y[i] = 1280 - cinfo->y[i];

        cinfo->count++;
    }

    TPD_DEBUG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);
    TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);
    TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);

  /*  TPD_DEBUG(" cinfo->x[3]= %d, cinfo->y[3]= %d, cinfo->p[3] = %d\n", cinfo->x[3], cinfo->y[3], cinfo->p[3]);               //       
    TPD_DEBUG(" cinfo->x[4]= %d, cinfo->y[4]= %d, cinfo->p[4] = %d\n", cinfo->x[4], cinfo->y[4], cinfo->p[4]);           */                
    
    return true;
};

static int touch_event_handler(void *unused)
{
    struct touch_info cinfo, pinfo;
    int i = 0;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

    do
    {
        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        set_current_state(TASK_RUNNING);

        if(tpd_touchinfo(&cinfo, &pinfo))                                          //五点触摸的事件逻辑需要自己去写
        {
            TPD_DEBUG("point_num = %d\n", point_num);

            if(point_num > 0)
            {
                printk("In fun %s,The X=%d,Y=%d\n", __func__, cinfo.x[0], cinfo.y[0]);
		for(i = 0;i<point_num;i++)
		{
			tpd_down(cinfo.x[i], cinfo.y[i], 1);
		}
                //tpd_down(cinfo.x[0], cinfo.y[0], 1);
#if 0
                if(point_num > 1)
                {
                    tpd_down(cinfo.x[1], cinfo.y[1], 1);

                    if(point_num > 2)
                    {
                        tpd_down(cinfo.x[2], cinfo.y[2], 1);
                    }
                }
#endif
                TPD_DEBUG("press --->\n");
            }
            else
            {
                if(p_point_num > 0)
                {
                    for(i = 0; i < p_point_num; i++)
                    {
                        tpd_up(pinfo.x[i], pinfo.y[i], pinfo.p[i]);
                    }
                }
                else
                {
                    input_mt_sync(tpd->dev);
                }

                TPD_DEBUG("release --->\n");
            }

            input_sync(tpd->dev);
        }
    }
    while(!kthread_should_stop());

    return 0;
}

static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

static void tpd_eint_interrupt_handler(void)
{
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}

static unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);

    if(ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        return 0xff;
    }
}
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = TPD_OK, firmwareid_reg;
    char data, i;
    i2c_client = client;
    //i2c_client->timing = 300;//shaokai


    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);

    //power on, need confirm with SA
    hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
    hwPowerOn(MT65XX_POWER_LDO_VGP5, VOL_1800, "TP");
    msleep(300);
/////////////////////////////////////////////////////////////////////////////////
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(300);
///////////////////////////////////////////////////////////////////////////////////
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	   {
		   printk("FT5306^^^^^^I2C transfer error, line: %d\n", __LINE__);
		   return -1; 
	   }

    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    msleep(10);


    tpd_load_status = 1;
    fts_register_read(FT5X06_MOD_ID, &firmwareid_reg, 1);
    printk("The FT5X06_REG_FIRMID=0x%x\n", firmwareid_reg);

    if(/*0x5D == firmwareid_reg*/1)
    {
        pr_tp("The Baoming's TP\n");
#ifdef TP_LIB_UPGRADE_SWITCH
        {
            u8 reg_version = 0, reg_value, lib_version = 0;
            int err = 0, reg_val[2];
            msleep(200);
            fts_register_read(FT5X0X_REG_FIRMID, &reg_version, 1);
            lib_version = fts_ctpm_get_upg_ver();
            printk("read lib file version=0x%x ,read chip register ft5206_version= 0x%x by cheehwa\n", lib_version, reg_version);

            if(lib_version != reg_version)
            {
                printk("upgrading,the verison not match 0x%2x != 0x%2x\n", lib_version, reg_version);
                msleep(200);
                err = fts_ctpm_fw_upgrade_with_i_file();
                printk("Return fts_ctpm_fw_upgrade_with_i_file_ err=%d by cheehwa\n", err);

                if(err == 0)
                {
                    #if 0
                    printk("[TSP] ugrade successfuly.\n");
                    msleep(300);
                    fts_register_read(FT5X0X_REG_FIRMID, &reg_value, 1);
                    printk("FTS_DBG from old_version 0x%2x to new_version = 0x%2x\n", reg_version, reg_value);
                    msleep(50);
                    /********Step 3:check READ-ID********/
                    cmd_write(0x90, 0x00, 0x00, 0x00, 4);
                    byte_read(reg_val, 2);

                    if(reg_val[0] == 0x79 && reg_val[1] == 0x07)
                    {
                        printk("[TSP] Step 3: CTPM_ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
                        err =  fts_ctpm_fw_upgrade_with_i_file();
                    }
		    #endif
                }
                else
                {
                    printk("[TSP]  ugrade fail err=%d, line = %d.\n",
                           err, __LINE__);
                }

                msleep(4000);
            }
        }
#endif
    }
    else
    {
        pr_tp("The liwei's TP\n");
    }
    
        fts_register_read(FT5X0X_REG_FIRMID, &tpd_firmware_version[0], 1);
        thread = kthread_run(touch_event_handler, 0, TPD_DEVICE); // fts5206_read_data

        if(IS_ERR(thread))
        {
            retval = PTR_ERR(thread);
            TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
        }

        TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
    	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
        return 0;
    }

    static int __devexit tpd_remove(struct i2c_client * client)
    {
        TPD_DEBUG("TPD removed\n");
        return 0;
    }
    static int tpd_local_init(void)
    {
        TPD_DMESG("Focaltech FT5206 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
        boot_mode = get_boot_mode();

        // Software reset mode will be treated as normal boot
        if(boot_mode == 3)
        {
            boot_mode = NORMAL_BOOT;
        }

        if(i2c_add_driver(&tpd_i2c_driver) != 0)
        {
            TPD_DMESG("unable to add i2c driver.\n");
            return -1;
        }

    if(tpd_load_status == 0)    
    {    	
		TPD_DMESG("ft5206@@@@@@@add error touch panel driver.\n");    	
		i2c_del_driver(&tpd_i2c_driver);    	
      	return -1;   
    }

#ifdef TPD_HAVE_BUTTON
        tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
        TPD_DO_WARP = 1;
        memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
        memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
        memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
        memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif
        TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);
        tpd_type_cap = 1;
        return 0;
    }
    static int tpd_resume(struct i2c_client * client)
    {
        int retval = TPD_OK;
   TPD_DEBUG("ft5306 TPD wake up\n");
#ifdef TPD_CLOSE_POWER_IN_SLEEP
        hwPowerOn(TPD_POWER_SOURCE, VOL_3300, "TP");
#else
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    	msleep(10);  
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(20); 
#endif
        mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        return retval;
    }
    static int tpd_suspend(struct i2c_client * client, pm_message_t message)
    {
        int retval = TPD_OK;
        static char data = 0x3;
	 TPD_DEBUG("ft5306 TPD enter sleep\n");
        mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef TPD_CLOSE_POWER_IN_SLEEP
        hwPowerDown(TPD_POWER_SOURCE, "TP");
#else
        i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
#ifdef MT6573
        mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
#endif
#endif
        return retval;
    }
    static struct tpd_driver_t tpd_device_driver =
    {
        .tpd_device_name = "FT5206",
        .tpd_local_init = tpd_local_init,
        .suspend = tpd_suspend,
        .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
        .tpd_have_button = 1,
#else
        .tpd_have_button = 0,
#endif
    };
    static int __init tpd_driver_init(void)
    {
        printk("MediaTek FT5206 touch panel driver init\n");

	i2c_register_board_info(0, &ft5306_i2c_tpd, 1);
        if(tpd_driver_add(&tpd_device_driver) < 0)
        {
            TPD_DMESG("add FT5206 driver failed\n");
        }

        return 0;
    }
    static void __exit tpd_driver_exit(void)
    {
        printk("MediaTek FT5206 touch panel driver exit\n");
        tpd_driver_remove(&tpd_device_driver);
    }
    module_init(tpd_driver_init);
    module_exit(tpd_driver_exit);
    MODULE_DESCRIPTION("CTP driver for FocalTech's ft5306 on MT6577 platform");
    MODULE_AUTHOR("Jianhua   Yuan <yuanjianhua@konka.com>");
    MODULE_LICENSE("GPL");
