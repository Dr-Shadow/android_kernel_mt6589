#ifndef _OPPO_CHG_DEF_H_
#define _OPPO_CHG_DEF_H_

#define OPPO_NEW_CHARGING
#ifdef OPPO_NEW_CHARGING
#define OPPO_USE_EXCHARGER			// Jiada.Li@ProDrv.chager, modify 2013/05/18 for charger DC CHIP Compatible
#define OPPO_USE_II_BQ24157
#define OPPO_USE_II_BQ24156A
#define OPPO_USE_FAIRCHILD_FAN5405

#define OPPO_HIGH_BATTERY_VOLTAGE	// Jiada.Li@ProDrv.chager, modify 2013/05/18 for charger High-voltage Core
//#define OPPO_NO_USE_FGADC			// Jiada.Li@ProDrv.chager, modify 2013/05/18 for charger no use Fuel Gauge
//#define OPPO_USE_CHARGER_RESET		// Nanwei.Deng@ProDrv.chager, modify 2013/05/18 for charger use Hardware Reset

#ifdef OPPO_USE_II_BQ24157			// // Jiada.Li@ProDrv.chager, modify 2013/05/18 for charger  Resistance compatible
//#define OPPO_BQ24157_R100
#define OPPO_BQ24157_R68
#endif

#endif

#endif /* _OPPO_CHG_DEF_H_ */ 
