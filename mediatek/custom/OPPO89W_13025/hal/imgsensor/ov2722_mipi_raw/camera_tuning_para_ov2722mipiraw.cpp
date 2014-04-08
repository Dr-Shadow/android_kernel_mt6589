#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

#include "camera_custom_nvram.h"
#include "camera_custom_sensor.h"
#include "image_sensor.h"
#include "kd_imgsensor_define.h"
#include "camera_AE_PLineTable_ov2722mipiraw.h"
#include "camera_info_ov2722mipiraw.h"
#include "camera_custom_AEPlinetable.h"
const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =
{{
    //Version
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,
    //SensorId
    SensorId: SENSOR_ID,
    ISPComm:{
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    },
    ISPPca:{
        #include INCLUDE_FILENAME_ISP_PCA_PARAM
    },
    ISPRegs:{
        #include INCLUDE_FILENAME_ISP_REGS_PARAM
        },
    ISPMfbMixer:{{
        {//00: MFB mixer for ISO 100
            0x00000000, 0x00000000
        },
        {//01: MFB mixer for ISO 200
            0x00000000, 0x00000000
        },
        {//02: MFB mixer for ISO 400
            0x00000000, 0x00000000
        },
        {//03: MFB mixer for ISO 800
            0x00000000, 0x00000000
        },
        {//04: MFB mixer for ISO 1600
            0x00000000, 0x00000000
        },
        {//05: MFB mixer for ISO 2400
            0x00000000, 0x00000000
        },
        {//06: MFB mixer for ISO 3200
            0x00000000, 0x00000000
        }
    }},
    ISPCcmPoly22:{
        75900,    // i4R_AVG
        9060,    // i4R_STD
        79300,    // i4B_AVG
        20471,    // i4B_STD
        {  // i4P00[9]
            3620000, -966667, -93333, -380000, 2673333, 266667, 243333, -1286667, 3603333
        },
        {  // i4P10[9]
            -33982, 59872, -25891, 38836, 35600, -74436, 11327, 69581, -80909
        },
        {  // i4P01[9]
            92357, -162725, 70367, -105551, -96755, 202306, -30786, -189112, 219898
        },
        {  // i4P20[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P11[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        },
        {  // i4P02[9]
            0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    }
}};
const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =
{
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version
    SENSOR_ID, // SensorId

    // AE NVRAM
    {
        // rDevicesInfo
        {
            1160,    // u4MinGain, 1024 base = 1x
            16320,    // u4MaxGain, 16x
            59,    // u4MiniISOGain, ISOxx  
            64,    // u4GainStepUnit, 1x/8 
            29723,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            29723,    // u4VideoExpUnit  
            30,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            29723,    // u4CapExpUnit 
            30,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            20,    // u4LensFno, Fno = 2.0
            350    // u4FocusLength_100x
        },
        // rHistConfig
        {
            2,    // u4HistHighThres
            40,    // u4HistLowThres
            2,    // u4MostBrightRatio
            1,    // u4MostDarkRatio
            160,    // u4CentralHighBound
            20,    // u4CentralLowBound
            {240, 230, 220, 210, 200},    // u4OverExpThres[AE_CCT_STRENGTH_NUM] 
            {86, 108, 128, 148, 170},    // u4HistStretchThres[AE_CCT_STRENGTH_NUM] 
            {18, 22, 26, 30, 34}    // u4BlackLightThres[AE_CCT_STRENGTH_NUM] 
        },
        // rCCTConfig
        {
            TRUE,    // bEnableBlackLight
            TRUE,    // bEnableHistStretch
            FALSE,    // bEnableAntiOverExposure
            TRUE,    // bEnableTimeLPF
            TRUE,    // bEnableCaptureThres
            TRUE,    // bEnableVideoThres
            TRUE,    // bEnableStrobeThres
            60,    // u4AETarget
            47,    // u4StrobeAETarget
            50,    // u4InitIndex
            8,    // u4BackLightWeight
            32,    // u4HistStretchWeight
            4,    // u4AntiOverExpWeight
            4,    // u4BlackLightStrengthIndex
            2,    // u4HistStretchStrengthIndex
            2,    // u4AntiOverExpStrengthIndex
            2,    // u4TimeLPFStrengthIndex
            {1, 3, 5, 7, 8},    // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM] 
            90,    // u4InDoorEV = 9.0, 10 base 
            10,    // i4BVOffset delta BV = value/10 
            64,    // u4PreviewFlareOffset
            64,    // u4CaptureFlareOffset
            5,    // u4CaptureFlareThres
            64,    // u4VideoFlareOffset
            5,    // u4VideoFlareThres
            32,    // u4StrobeFlareOffset
            2,    // u4StrobeFlareThres
            80,    // u4PrvMaxFlareThres
            0,    // u4PrvMinFlareThres
            80,    // u4VideoMaxFlareThres
            0,    // u4VideoMinFlareThres
            18,    // u4FlatnessThres    // 10 base for flatness condition.
            100    // u4FlatnessStrength
        }
    },
    // AWB NVRAM
    {
        // AWB calibration data
        {
            // rUnitGain (unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rGoldenGain (golden sample gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)
            {
                0,    // i4R
                0,    // i4G
                0    // i4B
            },
            // rD65Gain (D65 WB gain: 1.0 = 512)
            {
                813,    // i4R
                512,    // i4G
                571    // i4B
            }
        },
        // Original XY coordinate of AWB light source
        {
           // Strobe
            {
                131,    // i4X
                -211    // i4Y
            },
            // Horizon
            {
                -392,    // i4X
                -284    // i4Y
            },
            // A
            {
                -270,    // i4X
                -284    // i4Y
            },
            // TL84
            {
                -111,    // i4X
                -292    // i4Y
            },
            // CWF
            {
                -80,    // i4X
                -356    // i4Y
            },
            // DNP
            {
                23,    // i4X
                -245    // i4Y
            },
            // D65
            {
                131,    // i4X
                -211    // i4Y
            },
            // DF
            {
                0,    // i4X
                0    // i4Y
            }
        },
        // Rotated XY coordinate of AWB light source
        {
            // Strobe
            {
                93,    // i4X
                -230    // i4Y
            },
            // Horizon
            {
                -435,    // i4X
                -212    // i4Y
            },
            // A
            {
                -315,    // i4X
                -233    // i4Y
            },
            // TL84
            {
                -159,    // i4X
                -268    // i4Y
            },
            // CWF
            {
                -140,    // i4X
                -337    // i4Y
            },
            // DNP
            {
                -19,    // i4X
                -245    // i4Y
            },
            // D65
            {
                93,    // i4X
                -230    // i4Y
            },
            // DF
            {
                0,    // i4X
                0    // i4Y
            }
        },
        // AWB gain of AWB light source
        {
            // Strobe 
            {
                813,    // i4R
                512,    // i4G
                571    // i4B
            },
            // Horizon 
            {
                512,    // i4R
                593,    // i4G
                1479    // i4B
            },
            // A 
            {
                522,    // i4R
                512,    // i4G
                1084    // i4B
            },
            // TL84 
            {
                654,    // i4R
                512,    // i4G
                883    // i4B
            },
            // CWF 
            {
                743,    // i4R
                512,    // i4G
                924    // i4B
            },
            // DNP 
            {
                736,    // i4R
                512,    // i4G
                692    // i4B
            },
            // D65 
            {
                813,    // i4R
                512,    // i4G
                571    // i4B
            },
            // DF 
            {
                512,    // i4R
                512,    // i4G
                512    // i4B
            }
        },
        // Rotation matrix parameter
        {
            10,    // i4RotationAngle
            252,    // i4Cos
            44    // i4Sin
        },
        // Daylight locus parameter
        {
            -181,    // i4SlopeNumerator
            128    // i4SlopeDenominator
        },
        // AWB light area
        {
            // Strobe:FIXME
            {
            0,    // i4RightBound
            0,    // i4LeftBound
            0,    // i4UpperBound
            0    // i4LowerBound
            },
            // Tungsten
            {
            -229,    // i4RightBound
            -889,    // i4LeftBound
            -172,    // i4UpperBound
            -272    // i4LowerBound
            },
            // Warm fluorescent
            {
            -229,    // i4RightBound
            -889,    // i4LeftBound
            -272,    // i4UpperBound
            -392    // i4LowerBound
            },
            // Fluorescent
            {
            -69,    // i4RightBound
            -229,    // i4LeftBound
            -161,    // i4UpperBound
            -302    // i4LowerBound
            },
            // CWF
            {
            -69,    // i4RightBound
            -229,    // i4LeftBound
            -302,    // i4UpperBound
            -387    // i4LowerBound
            },
            // Daylight
            {
            118,    // i4RightBound
            -69,    // i4LeftBound
            -140,    // i4UpperBound
            -310    // i4LowerBound
            },
            // Shade
            {
            478,    // i4RightBound
            118,    // i4LeftBound
            -150,    // i4UpperBound
            -310    // i4LowerBound
            },
            // Daylight Fluorescent
            {
            138,    // i4RightBound
            -69,    // i4LeftBound
            -310,    // i4UpperBound
            -400    // i4LowerBound
            }
        },
        // PWB light area
        {
            // Reference area
            {
            478,    // i4RightBound
            -889,    // i4LeftBound
            0,    // i4UpperBound
            -400    // i4LowerBound
            },
            // Daylight
            {
            143,    // i4RightBound
            -69,    // i4LeftBound
            -140,    // i4UpperBound
            -310    // i4LowerBound
            },
            // Cloudy daylight
            {
            243,    // i4RightBound
            68,    // i4LeftBound
            -140,    // i4UpperBound
            -310    // i4LowerBound
            },
            // Shade
            {
            343,    // i4RightBound
            68,    // i4LeftBound
            -140,    // i4UpperBound
            -310    // i4LowerBound
            },
            // Twilight
            {
            -69,    // i4RightBound
            -229,    // i4LeftBound
            -140,    // i4UpperBound
            -310    // i4LowerBound
            },
            // Fluorescent
            {
            143,    // i4RightBound
            -259,    // i4LeftBound
            -180,    // i4UpperBound
            -387    // i4LowerBound
            },
            // Warm fluorescent
            {
            -215,    // i4RightBound
            -415,    // i4LeftBound
            -180,    // i4UpperBound
            -387    // i4LowerBound
            },
            // Incandescent
            {
            -215,    // i4RightBound
            -415,    // i4LeftBound
            -150,    // i4UpperBound
            -310    // i4LowerBound
            },
            // Gray World
            {
            5000,    // i4RightBound
            -5000,    // i4LeftBound
            5000,    // i4UpperBound
            -5000    // i4LowerBound
            }
        },
        // PWB default gain	
        {
            // Daylight
            {
            759,    // i4R
            512,    // i4G
            619    // i4B
            },
            // Cloudy daylight
            {
            865,    // i4R
            512,    // i4G
            514    // i4B
            },
            // Shade
            {
            914,    // i4R
            512,    // i4G
            475    // i4B
            },
            // Twilight
            {
            618,    // i4R
            512,    // i4G
            829    // i4B
            },
            // Fluorescent
            {
            749,    // i4R
            512,    // i4G
            766    // i4B
            },
            // Warm fluorescent
            {
            564,    // i4R
            512,    // i4G
            1146    // i4B
            },
            // Incandescent
            {
            515,    // i4R
            512,    // i4G
            1075    // i4B
            },
            // Gray World
            {
            512,    // i4R
            512,    // i4G
            512    // i4B
            }
        },
        // AWB preference color	
        {
            // Tungsten
            {
            5,    // i4SliderValue    // need to modefy
            6239    // i4OffsetThr
            },
            // Warm fluorescent	
            {
            0,    // i4SliderValue
            5455    // i4OffsetThr
            },
            // Shade
            {
            0,    // i4SliderValue    // need to modefy
            1346    // i4OffsetThr
            },
            // Daylight WB gain
            {
            709,    // i4R
            512,    // i4G
            680    // i4B
            },
            // Preference gain: strobe
            {
            502,    // i4R
            512,    // i4G
            512    // i4B
            },
            // Preference gain: tungsten
            {
            510,    // i4R
            512,    // i4G
            450    // i4B
            },
            // Preference gain: warm fluorescent
            {
            502,    // i4R
            512,    // i4G
            485    // i4B
            },
            // Preference gain: fluorescent
            {
            510,    // i4R
            512,    // i4G
            505    // i4B
            },
            // Preference gain: CWF
            {
            510,    // i4R
            512,    // i4G
            505    // i4B
            },
            // Preference gain: daylight
            {
            510,    // i4R
            512,    // i4G
            500    // i4B
            },
            // Preference gain: shade
            {
            510,    // i4R
            512,    // i4G
            500    // i4B
            },
            // Preference gain: daylight fluorescent
            {
            510,    // i4R
            512,    // i4G
            500    // i4B
            }
        },
        {// CCT estimation
            {// CCT
                2300,    // i4CCT[0]
                2850,    // i4CCT[1]
                4100,    // i4CCT[2]
                5100,    // i4CCT[3]
                6500    // i4CCT[4]
            },
            {// Rotated X coordinate
                -528,    // i4RotatedXCoordinate[0]
                -408,    // i4RotatedXCoordinate[1]
                -252,    // i4RotatedXCoordinate[2]
                -112,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
            }
        }
    },
    {0}
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM
//};  //  namespace


typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),
                                             sizeof(NVRAM_LENS_PARA_STRUCT),
                                             sizeof(AE_PLINETABLE_T)};

    if (CameraDataType > CAMERA_DATA_AE_PLINETABLE || NULL == pDataBuf || (size < dataSize[CameraDataType]))
    {
        return 1;
    }

    switch(CameraDataType)
    {
        case CAMERA_NVRAM_DATA_ISP:
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_3A:
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));
            break;
        case CAMERA_NVRAM_DATA_SHADING:
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));
            break;
        case CAMERA_DATA_AE_PLINETABLE:
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));
            break;
        default:
            break;
    }
    return 0;
}}; // NSFeature


