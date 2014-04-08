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

#ifndef _BQ24156A_SW_H_
#define _BQ24156A_SW_H_

#define bq24156a_read_byte excharger_read_byte
#define bq24156a_write_byte excharger_write_byte

extern int excharger_read_byte(kal_uint8 cmd, kal_uint8 *returnData);
extern int excharger_write_byte(kal_uint8 cmd, kal_uint8 writeData);

#ifdef VENDOR_EDIT
#define GPIO_BQ24156A_STAT_NUM           GPIOEXT26
#define GPIO_BQ24156A_STAT_MODE          GPIO_MODE_GPIO
#define GPIO_BQ24156A_STAT_DIR           GPIO_DIR_OUT
#define GPIO_BQ24156A_STAT_OFF_OUT       GPIO_OUT_ONE
#define GPIO_BQ24156A_STAT_ON_OUT        GPIO_OUT_ZERO

#define GPIO_BQ24156A_DISABLE_NUM        GPIOEXT20
#define GPIO_BQ24156A_DISABLE_MODE       GPIO_MODE_GPIO
#define GPIO_BQ24156A_DISABLE_DIR        GPIO_DIR_OUT
#define GPIO_BQ24156A_DISABLE_OFF_OUT    GPIO_OUT_ONE
#define GPIO_BQ24156A_DISABLE_ON_OUT     GPIO_OUT_ZERO

#define GPIO_BQ24156A_RST_NUM            GPIO38
#define GPIO_BQ24156A_RST_MODE           GPIO_MODE_GPIO
#define GPIO_BQ24156A_RST_DIR            GPIO_DIR_OUT
#define GPIO_BQ24156A_RST_LOW_OUT        GPIO_OUT_ZERO
#define GPIO_BQ24156A_RST_HIGH_OUT       GPIO_OUT_ONE
#endif
//#define HIGH_BATTERY_VOLTAGE_SUPPORT

#define BQ24156A_CON0      0x00
#define BQ24156A_CON1      0x01
#define BQ24156A_CON2      0x02
#define BQ24156A_CON3      0x03
#define BQ24156A_CON4      0x04
#define BQ24156A_CON5      0x05
#define BQ24156A_CON6      0x06

/**********************************************************
  *
  *   [MASK/SHIFT] 
  *
  *********************************************************/
//CON0
#define CON0_TMR_RST_MASK 	0x01
#define CON0_TMR_RST_SHIFT 	7

#define CON0_SLRST_MASK 	0x01
#define CON0_SLRST_SHIFT 	7

#define CON0_EN_STAT_MASK 	0x01
#define CON0_EN_STAT_SHIFT 	6

#define CON0_STAT_MASK 		0x03
#define CON0_STAT_SHIFT 	4

#define CON0_FAULT_MASK 	0x07
#define CON0_FAULT_SHIFT 	0

//CON1
#define CON1_LIN_LIMIT_MASK 	0x03
#define CON1_LIN_LIMIT_SHIFT 	6

#define CON1_LOW_V_2_MASK 	0x01
#define CON1_LOW_V_2_SHIFT 	5

#define CON1_LOW_V_1_MASK 	0x01
#define CON1_LOW_V_1_SHIFT 	4

#define CON1_TE_MASK 	0x01
#define CON1_TE_SHIFT 	3

#define CON1_CE_MASK 	0x01
#define CON1_CE_SHIFT 	2

#define CON1_HZ_MODE_MASK 	0x01
#define CON1_HZ_MODE_SHIFT 	1

#define CON1_OPA_MODE_MASK 	0x01
#define CON1_OPA_MODE_SHIFT 0

//CON2
#define CON2_CV_VTH_MASK 	0x3F
#define CON2_CV_VTH_SHIFT 	2

//CON3
#define CON3_VENDER_CODE_MASK 	0x07
#define CON3_VENDER_CODE_SHIFT 	5

#define CON3_PIN_MASK 	0x03
#define CON3_PIN_SHIFT 	3

#define CON3_REVISION_MASK 		0x07
#define CON3_REVISION_SHIFT 	0

//CON4
#define CON4_RESET_MASK		0x01
#define CON4_RESET_SHIFT 	7

#define CON4_I_CHR_MASK		0x0F
#define CON4_I_CHR_SHIFT 	3

#define CON4_I_TERM_MASK	0x07
#define CON4_I_TERM_SHIFT 	0

//CON5
#define CON5_LOW_CHG_MASK	0x01
#define CON5_LOW_CHG_SHIFT 	5

#define CON5_DPM_STATUS_MASK	0x01
#define CON5_DPM_STATUS_SHIFT 	4

#define CON5_CD_STATUS_MASK		0x01
#define CON5_CD_STATUS_SHIFT 	3

#define CON5_VSREG_MASK		0x07
#define CON5_VSREG_SHIFT 	0

//CON6
#define CON6_MCHRG_MASK		0x0F
#define CON6_MCHRG_SHIFT 	4

#define CON6_MREG_MASK		0x0F
#define CON6_MREG_SHIFT 	0

/**********************************************************
  *
  *   [Extern Function] 
  *
  *********************************************************/
//CON0
extern void bq24156a_set_tmr_rst(kal_uint32 val);
extern kal_uint32 bq24156a_get_slrst_status(void);
extern void bq24156a_set_en_stat(kal_uint32 val);
extern kal_uint32 bq24156a_get_chip_status(void);
extern kal_uint32 bq24156a_get_fault_reason(void);

//CON1
extern void bq24156a_set_lin_limit(kal_uint32 val);
extern void bq24156a_set_lowv_2(kal_uint32 val);
extern void bq24156a_set_lowv_1(kal_uint32 val);
extern void bq24156a_set_te(kal_uint32 val);
extern void bq24156a_set_ce(kal_uint32 val);
extern void bq24156a_set_hz_mode(kal_uint32 val);
extern void bq24156a_set_opa_mode(kal_uint32 val);

//CON2
extern void bq24156a_set_cv_vth(kal_uint32 val);

//CON3
extern kal_uint32 bq24156a_get_vender_code(void);
extern kal_uint32 bq24156a_get_pin(void);
extern kal_uint32 bq24156a_get_revision(void);

//CON4
extern void bq24156a_set_reset(kal_uint32 val);
extern void bq24156a_set_ac_charging_current(kal_uint32 val);
extern void bq24156a_set_termination_current(kal_uint32 val);

//CON5
extern void bq24156a_set_low_chg(kal_uint32 val);
extern kal_uint32 bq24156a_get_dpm_status(void);
extern kal_uint32 bq24156a_get_cd_status(void);
extern void bq24156a_set_vsreg(kal_uint32 val);

//CON6
extern void bq24156a_set_mchrg(kal_uint32 val);
extern void bq24156a_set_mreg(kal_uint32 val);
//---------------------------------------------------------
extern void bq24156a_dump_register(void);

#endif // _BQ24156A_SW_H_

