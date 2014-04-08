
#include <linux/init.h>        /* For init/exit macros */
#include <linux/module.h>      /* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/xlog.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include "bq2202a.h"
#include <oppo_chg_def.h>


#ifdef OPPO_HIGH_BATTERY_VOLTAGE
extern kal_uint8 oppo_high_battery_status;
extern kal_uint8 oppo_check_ID_status;
#endif

#define DEBUG_BQ2202A           
#define READ_PAGE_BQ2202A

#define BQ2202A_GPIO GPIO80

#define READ_ID_CMD                 0x33            // read ROM
#define SKIP_ROM_CMD               0xCC           // skip ROM
#define WRITE_EPROM_CMD        0x0F           // write EPROM 
#define READ_PAGE_ID_CMD        0xF0          // read EPROM  PAGE
#define READ_FIELD_ID_CMD       0xC3          // read EPROM  FIELD

#ifdef READ_PAGE_BQ2202A
#define AddressLow                     0x20        // EPROM start address LOW 
#define AddressHigh                     0x00        // EPROM start address  HIGH
#else
#define AddressLow                     0x00        // EPROM start address LOW 
#define AddressHigh                     0x00        // EPROM start address  HIGH
#endif

static  unsigned char ReadIDDataByte[8];     //8*8=64bit            ID ROM 
#ifdef READ_PAGE_BQ2202A
static  unsigned char CheckIDDataByte[32];    // 32*8=256bit   EPROM  PAGE1
#else
static  unsigned char CheckIDDataByte[128];    // 128*8=1024bit   EPROM  PAGE1
#endif


/**********************************************************************/
/* 		void wait_us(int usec)										  */
/*																      */
/*	Description :   Creates a delay of approximately (usec + 5us) 	  */
/*				  		when usec is greater.						  */
/* 	Arguments : 		None										  */
/*	Global Variables:	None   										  */
/*  Returns: 			None								          */
/**********************************************************************/
#if 0
void wait_us(int usec)
{
    while (usec--);
}
#else
#define wait_us(n) udelay(n)
#define wait_ms(n) mdelay(n)
#endif
/**********************************************************************/
/* 	static void SendReset(void)										  */
/*																      */
/*	Description : 		Creates the Reset signal to initiate SDQ 	  */
/*				  		communication.								  */
/* 	Arguments : 		None										  */
/*	Global Variables:	None   										  */
/*  Returns: 			None								          */
/**********************************************************************/
static void SendReset(void)
{
    mt_set_gpio_mode(BQ2202A_GPIO, GPIO_MODE_00);	//Set GPIO P9.3 as Output
    mt_set_gpio_dir(BQ2202A_GPIO, GPIO_DIR_OUT);
    mt_set_gpio_out(BQ2202A_GPIO, 1);	            //Set High
    wait_us(20);								//Allow PWR cap to charge and power IC	~ 25us
    mt_set_gpio_out(BQ2202A_GPIO, 0);            	//Set Low
    wait_us(500);								//Reset time greater then 480us
    mt_set_gpio_out(BQ2202A_GPIO, 1);            	//Set High
    mt_set_gpio_dir(BQ2202A_GPIO, GPIO_DIR_IN);		//Set GPIO P9.3 as Input
}

/**********************************************************************/
/* 	static unsigned char TestPresence(void)							  */
/*																      */
/*	Description : 		Detects if a device responds to Reset signal  */
/* 	Arguments : 		PresenceTimer - Sets timeout if no device	  */
/*							present									  */
/*						InputData - Actual state of GPIO			  */
/*						GotPulse - States if a pulse was detected	  */
/*	Global Variables:	None   										  */
/*  Returns: 			GotPulse         							  */
/**********************************************************************/
static unsigned char TestPresence(void)
{
    unsigned int PresenceTimer;
    static volatile unsigned char InputData;
    static volatile unsigned char GotPulse;

    mt_set_gpio_dir(BQ2202A_GPIO, GPIO_DIR_IN);	        //Set GPIO P9.3 as Input
    PresenceTimer = 300;                                                //Set timeout, enough time to allow presence pulse
    GotPulse = 0;                                                           //Initialize as no pulse detected

    while ((PresenceTimer > 0) && (GotPulse == 0)) 
    {
        InputData = mt_get_gpio_in(BQ2202A_GPIO);       //Monitor logic state of GPIO 
        if (InputData == 0)                                             //If GPIO is Low,
        {                           
            GotPulse = 1;                                               //it means that device responded
        }
        else                                                                //If GPIO is high      
        {                                          
            GotPulse = 0;			                            //it means that device has not responded
            --PresenceTimer;		                            //Decrease timeout until enough time has been allowed for response
        }
    }
    wait_us(200);					                    //Wait some time to continue SDQ communication
    #ifdef DEBUG_BQ2202A
    printk("PresenceTimer=%d\n",PresenceTimer);
    #endif
    return GotPulse;				                    //Return if device detected or not
}

/**********************************************************************/
/* 	static void WriteOneBit(unsigned char OneZero)					  */
/*																      */
/*	Description : 		This procedure outputs a bit in SDQ to the 	  */
/*				  		slave.								  		  */
/* 	Arguments : 		OneZero - value of bit to be written		  */
/*	Global Variables:	None   										  */
/*  Returns: 			None								          */
/**********************************************************************/
static void WriteOneBit(unsigned char OneZero)
{
    mt_set_gpio_dir(BQ2202A_GPIO, GPIO_DIR_OUT);			//Set GPIO P9.3 as Output
    mt_set_gpio_out(BQ2202A_GPIO, 1);		            //Set High
    mt_set_gpio_out(BQ2202A_GPIO, 0);	                //Set Low
    if (OneZero != 0x00)
    {
        wait_us(7);									//approximately 7us	for a Bit 1
        mt_set_gpio_out(BQ2202A_GPIO, 1);	            //Set High
        wait_us(65);								//approximately 65us
    }
    else
    { 
        wait_us(65);								//approximately 65us for a Bit 0
        mt_set_gpio_out(BQ2202A_GPIO, 1);	            //Set High
        wait_us(7);					   				//approximately 7us
    }
    wait_us(5);	  									//approximately 5us
}

/**********************************************************************/
/* 	static void WriteOneByte(unsigned char Data2Send)				  */
/*																      */
/*	Description : 		This procedure calls the WriteOneBit() 		  */
/*				  		function 8 times to send a byte in SDQ.		  */
/* 	Arguments : 		Data2Send - Value of byte to be sent in SDQ	  */
/*	Global Variables:	None   										  */
/*  Returns: 			None								          */
/**********************************************************************/
static void WriteOneByte(unsigned char Data2Send)
{
    unsigned char i;
    unsigned char MaskByte;
    unsigned char Bit2Send;

    MaskByte = 0x01;
	
    for (i = 0; i < 8; i++) 
    {
        Bit2Send = Data2Send & MaskByte;		//Selects the bit to be sent
        WriteOneBit(Bit2Send);					//Writes the selected bit
        MaskByte <<= 1;							//Moves the bit mask to the next most significant position
    }
}

/**********************************************************************/
/* 	static unsigned char ReadOneBit(void)							  */
/*																      */
/*	Description : 		This procedure receives the bit value returned*/
/*				  		by the SDQ slave.							  */
/* 	Arguments : 		InBit - Bit value returned by slave			  */
/*	Global Variables:	None   										  */
/*  Returns: 			InBit								          */
/**********************************************************************/
static unsigned char ReadOneBit(void)
{
    static unsigned char InBit;
	
    mt_set_gpio_dir(BQ2202A_GPIO, GPIO_DIR_OUT);			//Set GPIO P9.3 as Output
    mt_set_gpio_out(BQ2202A_GPIO, 1);		            //Set High
    mt_set_gpio_out(BQ2202A_GPIO, 0);	                //Set Low
    mt_set_gpio_dir(BQ2202A_GPIO, GPIO_DIR_IN);			//Set GPIO P9.3 as Input
    wait_us(12);		   								//Strobe window	~ 12us
    InBit = mt_get_gpio_in(BQ2202A_GPIO);		        //This function takes about 3us
													//Between the wait_us and GPIO_ReadBit functions
													//approximately 15us should occur to monitor the 
													//GPIO line and determine if bit read is one or zero
    wait_us(65);									//End of Bit
    mt_set_gpio_dir(BQ2202A_GPIO, GPIO_DIR_OUT);			//Set GPIO P9.3 as Output
    mt_set_gpio_out(BQ2202A_GPIO, 1);		            //Set High
    return InBit;									//Return bit value
}

/**********************************************************************/
/* 	static unsigned char ReadOneByte(void)							  */
/*																      */
/*	Description : 		This procedure reads 8 bits on the SDQ line   */
/*				  		and returns the byte value.					  */
/* 	Arguments : 		Databyte - Byte value returned by SDQ slave	  */
/*						MaskByte - Used to seperate each bit	      */
/*						i - Used for 8 time loop					  */
/*	Global Variables:	None   										  */
/*  Returns: 			DataByte							          */
/**********************************************************************/
static unsigned char ReadOneByte(void)
{
    unsigned char i;
    unsigned char DataByte;
    unsigned char MaskByte;

    DataByte = 0x00;			 				//Initialize return value
	
    for (i = 0; i < 8; i++)                                      //Select one bit at a time
    {                  
        MaskByte = ReadOneBit();				    //Read One Bit
        MaskByte <<= i;							//Determine Bit position within the byte
        DataByte = DataByte | MaskByte;			//Keep adding bits to form the byte
    }
    return DataByte;							//Return byte value read
}

/**********************************************************************/
/* 	void ReadBq2202aID(void)               							  */
/*																      */
/*	Description : 		This procedure reads BQ2202A'S ID on the SDQ  */
/*				  		line.                   					  */
/* 	Arguments : 		None                    					  */
/*	Global Variables:	None   										  */
/*  Returns: 			None       							          */
/**********************************************************************/
void ReadBq2202aID(void)
{
    unsigned char i;

    printk("dengnanwei_read_ID \n");
     
    SendReset();
    wait_us(2);
    i=TestPresence();
    #ifdef DEBUG_BQ2202A
    printk("TestPresence=%d\n",i);
    #endif
    
    WriteOneByte(READ_ID_CMD);                      // read rom commond
    for(i=0;i<8;i++)
    {
        //ReadIDDataByte[i] = ReadOneByte();      // read rom Partition 64bits = 8Bits
        ReadIDDataByte[7-i] = ReadOneByte();      // read rom Partition 64bits = 8Bits
    }

    #ifndef DEBUG_BQ2202A
    for(i=0;i<8;i++)
    {
        printk("ReadBq2202aID[%d]=%d\n",i,ReadIDDataByte[i]);
    }
    #endif
}
/**********************************************************************/
/* 	void CheckBq2202aID(void)               							  */
/*																      */
/*	Description : 		This procedure reads BQ2202A'S ID on the SDQ  */
/*				  		line.                   					  */
/* 	Arguments : 		None                    					  */
/*	Global Variables:	None   										  */
/*  Returns: 			None       							          */
/**********************************************************************/
void CheckBq2202aID(void)
{
    unsigned char i;
    
    printk("dengnanwei_check_ID \n");
     
    SendReset();
    wait_us(2);
    i=TestPresence();
#ifdef DEBUG_BQ2202A
    printk("TestPresence=%d\n",i);
#endif

    WriteOneByte(SKIP_ROM_CMD);              // skip rom commond
    wait_us(60);

#ifdef READ_PAGE_BQ2202A
    WriteOneByte(READ_PAGE_ID_CMD);     // read eprom Partition for page mode
#else
    WriteOneByte(READ_FIELD_ID_CMD);     // read eprom Partition for field mode
#endif
    wait_us(60);
    WriteOneByte(AddressLow);               // read eprom Partition Starting address low
    wait_us(60);
    WriteOneByte(AddressHigh);               // read eprom Partition Starting address high

#ifdef READ_PAGE_BQ2202A
    for(i=0;i<32;i++)
    {
        CheckIDDataByte[i] = ReadOneByte();   // read eprom Partition page1  256bits = 32Bits
    }

    #ifndef DEBUG_BQ2202A
    for(i=0;i<32;i++)
    {
        printk("CheckBq2202aID[%d]=%d\n",i,CheckIDDataByte[i]);
    }
    #endif
#else 
    for(i=0;i<128;i++)
    {
        CheckIDDataByte[i] = ReadOneByte();   // read eprom Partition field  1024bits = 128Bits
    }

    #ifndef DEBUG_BQ2202A
    for(i=0;i<128;i++)
    {
        printk("CheckBq2202aID[%d]=%d\n",i,CheckIDDataByte[i]);
    }
    #endif
#endif
}
/**********************************************************************/
/* 	void CheckIDCompare(void)               							  */
/*																      */
/*	Description : 		This procedure reads BQ2202A'S ID on the SDQ  */
/*				  		line.                   					  */
/* 	Arguments : 		None                    					  */
/*	Global Variables:	None   										  */
/*  Returns: 			None       							          */
/**********************************************************************/
#if 1
void CheckIDCompare(void) 
{
    unsigned char i,j;  
    int IDReadSign=1;
    if(IDReadSign==1)
    {
        printk( "<dengnanwei_dayin_ID> %s: LINE= %d : \n", __FUNCTION__,__LINE__);
        for(i=0;i<1;i++)
        {
            ReadBq2202aID();
            CheckBq2202aID();
            oppo_check_ID_status=0;
            if(ReadIDDataByte[7] == 0x09)
            {
                for(j=1;j<7;j++)
                {
                    if(ReadIDDataByte[j] ==CheckIDDataByte[j+16])
                    {
                        oppo_check_ID_status++;
                    }
                }
                if(oppo_check_ID_status==6)
                {
                    IDReadSign=0;
                    return;
                }
            }
            else
            {
                continue;
            }
        } 
        IDReadSign=0;
    }
}
#else
void CheckIDCompare(void) 
{
    unsigned char i,j;
    unsigned char IDCheckSign;
    static int IDReadSign=1;

    if(IDReadSign==1)
    {
        for(i=0;i<3;i++)
        {
            printk( "<dengnanwei_dayin_ID> %s: LINE= %d : \n", __FUNCTION__,__LINE__);
            ReadBq2202aID();
            CheckBq2202aID();

            IDCheckSign=0;
            if(ReadIDDataByte[7] == 0x09)
            {
                for(j=1;j<7;j++)
                {
                    if(ReadIDDataByte[j] ==CheckIDDataByte[j+16])
                    {
                        IDCheckSign++;
                    }
                }
                if(IDCheckSign==6)
                {
                    oppo_high_battery_status =1;// RIGHT
                    printk( "<dengnanwei_dayin_ID = %d: oppo_high_battery_status =%d :%s: LINE= %d : \n", IDCheckSign,oppo_high_battery_status,__FUNCTION__,__LINE__);
                    IDReadSign=0;
                    IDCheckSign=0;
                    return;
                }
            }
            else
            {
                continue;
            }
        } 

        if(IDCheckSign!=6)
        {
            oppo_high_battery_status=0;// wrong
            printk( "<dengnanwei_dayin_ID = %d: oppo_high_battery_status =%d :%s: LINE= %d : \n", IDCheckSign,oppo_high_battery_status,__FUNCTION__,__LINE__);
            IDCheckSign=0;
        }
        IDReadSign=0;
    }
}
#endif


///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int bq2202a_eprom_probe(struct platform_device *dev)	
{

	return 0;
}

static int bq2202a_eprom_remove(struct platform_device *dev)	
{
    xlog_printk(ANDROID_LOG_INFO, "Serial/Eprom", "******** BQ2202A eprom driver remove!! ********\n" );

    return 0;
}

static void bq2202a_eprom_shutdown(struct platform_device *dev)	
{
    xlog_printk(ANDROID_LOG_INFO, "Serial/Eprom", "******** BQ2202A eprom driver shutdown!! ********\n" );

}

static int bq2202a_eprom_suspend(struct platform_device *dev, pm_message_t state)	
{
    xlog_printk(ANDROID_LOG_INFO, "Serial/Eprom", "******** BQ2202A eprom driver suspend!! ********\n" );

    return 0;
}

static int bq2202a_eprom_resume(struct platform_device *dev)
{
    xlog_printk(ANDROID_LOG_INFO, "Serial/Eprom", "******** BQ2202A eprom driver resume!! ********\n" );

	//g_battery_flag_resume=1;

    return 0;
}

struct platform_device BQ2202A_eprom_device = {
    .name   = "bq2202a-eprom",
    .id	    = -1,
};

static struct platform_driver bq2202a_eprom_driver = {
    .probe		= bq2202a_eprom_probe,
    .remove		= bq2202a_eprom_remove,
    .shutdown	= bq2202a_eprom_shutdown,
    //#ifdef CONFIG_PM
    .suspend	= bq2202a_eprom_suspend,
    .resume		= bq2202a_eprom_resume,
    //#endif
    .driver     = {
        .name = "bq2202a-eprom",
    },
};

static int __init bq2202a_eprom_init(void)
{
    int ret;

    ret = platform_device_register(&BQ2202A_eprom_device);
    if (ret) {
	xlog_printk(ANDROID_LOG_INFO, "Serial/Eprom", "****[BQ2202A_eprom_device] Unable to device register(%d)\n", ret);
	return ret;
    }
    ret = platform_driver_register(&bq2202a_eprom_driver);
    if (ret) {
	xlog_printk(ANDROID_LOG_INFO, "Serial/Eprom", "****[bq2202a_eprom_driver] Unable to register driver (%d)\n", ret);
	return ret;
    }
    
    xlog_printk(ANDROID_LOG_INFO, "Serial/Eprom", "****[bq2202a_eprom_driver] Initialization : DONE !!\n");

    return 0;
}

static void __exit bq2202a_eprom_exit (void)
{
}

module_init(bq2202a_eprom_init);
module_exit(bq2202a_eprom_exit);

MODULE_AUTHOR("Terry Lee");
MODULE_DESCRIPTION("BQ2202A Eprom Device Driver");
MODULE_LICENSE("GPL");
