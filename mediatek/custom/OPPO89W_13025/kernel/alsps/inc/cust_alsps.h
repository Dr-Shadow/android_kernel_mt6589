#ifndef __CUST_ALSPS_H__
#define __CUST_ALSPS_H__

#define C_CUST_ALS_LEVEL    16
#define C_CUST_I2C_ADDR_NUM 4

#ifdef VENDOR_EDIT
//Lycan.Wang@BasicDrv.Sensor, 2012-11-07 Add for ps adjust algo
struct ps_adjust_para {
    int ps_up;
    int ps_thd_low_notrend;
    int ps_thd_high_notrend; //TODO when ps_up != ps_thd_high
    int ps_thd_low_trend;
    int ps_thd_high_trend;
    int ps_thd_low_highlight;
    int ps_thd_high_highlight;
    int ps_adjust_min;
    int ps_adjust_max;
    int highlight_limit;
    int sampling_time; //Unit:ms
    int sampling_count;
    int dirty_adjust_limit;
    int dirty_adjust_low_thd;
    int dirty_adjust_high_thd;
};

#define PS_ADJUST_TREND_STATE       0
#define PS_ADJUST_NOTREND_STATE     1
#define PS_ADJUST_HIGHLIGHT_STATE   2
#define PS_ADJUST_AVOID_DIRTY_STATE 3

struct set_ps_thd_para {
    int low_threshold;
    int high_threshold;
    int ps_average;
    int algo_state; //PS_ADJUST_XXX_STATE
};
#endif /* VENDOR_EDIT */

struct alsps_hw {
    int i2c_num;                                    /*!< the i2c bus used by ALS/PS */
    int power_id;                                   /*!< the power id of the chip */
    int power_vol;                                  /*!< the power voltage of the chip */
	int polling_mode;                               /*!< 1: polling mode ; 0:interrupt mode*/
	int polling_mode_ps;                               /*!< 1: polling mode ; 0:interrupt mode*/
	int polling_mode_als;                               /*!< 1: polling mode ; 0:interrupt mode*/
    unsigned char   i2c_addr[C_CUST_I2C_ADDR_NUM];  /*!< i2c address list, some chip will have multiple address */
    unsigned int    als_level[C_CUST_ALS_LEVEL-1];  /*!< (C_CUST_ALS_LEVEL-1) levels divides all range into C_CUST_ALS_LEVEL levels*/
    unsigned int    als_value[C_CUST_ALS_LEVEL];    /*!< the value reported in each level */
    unsigned int    ps_threshold;                   /*!< the threshold of proximity sensor */
	unsigned int    als_window_loss;                /*!< the window loss  */
	unsigned int    ps_threshold_high;
	unsigned int    ps_threshold_low;
	unsigned int    als_threshold_high;
	unsigned int    als_threshold_low;
#ifdef VENDOR_EDIT
    //Lycan.Wang@BasicDrv.Sensor, 2012-11-15 Add for ps adjust algo
    struct ps_adjust_para *p_ps_adjust_para;
    //Shaoyu.Huang@BasicDrv.Sensor, add 2012/5/24 to config power for sensor of difference prj
    int (*power)(struct alsps_hw *hw, unsigned int on, char *devname);
#endif/*VENDOR_EDIT*/

};

extern struct alsps_hw* get_cust_alsps_hw_tmd2772(void);
extern struct alsps_hw* get_cust_alsps_hw_tmd2771(void);
extern struct alsps_hw* get_cust_alsps_hw_ltr558(void);
extern struct alsps_hw* get_cust_alsps_hw_stk31xx(void);
extern struct alsps_hw* get_cust_alsps_hw_stk3x1x(void);
extern struct alsps_hw *get_cust_alsps_hw_apds9930(void);
extern struct alsps_hw *get_cust_alsps_hw_apds9930a(void);

#endif
