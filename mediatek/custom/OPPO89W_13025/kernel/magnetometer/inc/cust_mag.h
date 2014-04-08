#ifndef __CUST_MAG_H__
#define __CUST_MAG_H__

struct mag_hw {
    int i2c_num;
    int direction;
    int power_id;
    int power_vol;
    #ifdef VENDOR_EDIT//xiaohua.tian@wxkf.wxBasicDrv.Sensor, add 2013/03/01 to config power for sensor of difference prj
    int (*power)(struct mag_hw *hw, unsigned int on, char *devname);
    #endif/*VENDOR_EDIT*/
};

extern struct mag_hw* get_cust_mag_hw_akm8975(void);
extern struct mag_hw* get_cust_mag_hw_mmc328xma(void);
extern struct mag_hw* get_cust_mag_hw_mmc3461(void);
extern struct mag_hw* get_cust_mag_hw_yas533(void); 

#endif 
