#ifdef MOZ_AW2013_SUPPORT
typedef struct
{
	unsigned char type;
	unsigned char enable;
	unsigned char breath;
	unsigned char ratio;
} LED_CFG_AW;



#define LED_IOW(num, dtype)     _IOW('H', num, dtype)
#define LED_IOR(num, dtype)     _IOR('H', num, dtype)
#define LED_IOWR(num, dtype)    _IOWR('H', num, dtype)
#define LED_IO(num)             _IO('H', num)


#define MZ_LED_RED_DEV           LED_IOWR(1, LED_CFG_AW)
#define MZ_LED_GREEN_DEV           LED_IOWR(2, LED_CFG_AW)
#define MZ_LED_BLUE_DEV           LED_IOWR(3, LED_CFG_AW)
#define MZ_LED_WHITE_DEV           LED_IOWR(4, LED_CFG_AW)
#define MZ_LED_STANDBY_DEV       LED_IOWR(5,LED_CFG_AW)



/* REG MAPS */
#define AW2013_RESET_REG                 (0x00)
#define AW2013_GLOBALCTRL_REG            (0x01)
#define AW2013_STATUS_REG                (0x02)

#define AW2013_LED_CHANNEL_ENABLE_REG    (0x30)
#define AW2013_LED0_CFG_REG              (0x31)
#define AW2013_LED1_CFG_REG              (0x32)
#define AW2013_LED2_CFG_REG              (0x33)
#define AW2013_PWM0_REG                  (0x34)
#define AW2013_PWM1_REG                  (0x35)
#define AW2013_PWM2_REG                  (0x36)

#define AW2013_LED0_TIME_REG0            (0x37)
#define AW2013_LED0_TIME_REG1            (0x38)
#define AW2013_LED0_TIME_REG2            (0x39)
#define AW2013_LED1_TIME_REG0            (0x3a)
#define AW2013_LED1_TIME_REG1            (0x3b)
#define AW2013_LED1_TIME_REG2            (0x3c)
#define AW2013_LED2_TIME_REG0            (0x3d)
#define AW2013_LED2_TIME_REG1            (0x3e)
#define AW2013_LED2_TIME_REG2            (0x3f)

#define AW2013_LED_TIME_0_13S    (0)
#define AW2013_LED_TIME_0_26S    (1)
#define AW2013_LED_TIME_0_52S    (2)
#define AW2013_LED_TIME_1_04S    (3)
#define AW2013_LED_TIME_2_08S    (4)
#define AW2013_LED_TIME_4_16S    (5)
#define AW2013_LED_TIME_8_32S    (6)  /* hold time not use */
#define AW2013_LED_TIME_16_64S  (7)  /* hold time not use */

#define AW2013_LED_DELAY_TIME_0S   (0)  /* delay time add 1 on time other */
#define AW2013_LED_DELAY_TIME_0_13S    (1)
#define AW2013_LED_DELAY_TIME_0_26S    (2)
#define AW2013_LED_DELAY_TIME_0_52S    (3)
#define AW2013_LED_DELAY_TIME_1_04S    (4)
#define AW2013_LED_DELAY_TIME_2_08S    (5)
#define AW2013_LED_DELAY_TIME_4_16S    (6)
#define AW2013_LED_DELAY_TIME_8_32S    (7)
#define AW2013_LED_DELAY_TIME_16_64S  (8) 


#define AW2013_LED_REPEAT_ONCE      (1)
#define AW2013_LED_REPEAT_ALWAYS  (0)


#endif

