#ifndef DOBOTTYPE_H
#define DOBOTTYPE_H

#ifdef _MSC_VER
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned  long long uint64_t;
typedef signed long long int64_t;
#else
#include <stdint.h>
#endif

/*********************************************************************************************************
** Data structures
*********************************************************************************************************/

/*********************************************************************************************************
** Common parts
*********************************************************************************************************/
#pragma pack(push)
#pragma pack(1)

/*
 * Real-time pose
 */
typedef struct tagPose {
    float x;
    float y;
    float z;
    float r;
    float jointAngle[4];
}Pose;

/*
 * Kinematics parameters
 */
typedef struct tagKinematics {
    float velocity;
    float acceleration;
}Kinematics;

/*
 * HOME related
 */
typedef struct tagHOMEParams {
    float x;
    float y;
    float z;
    float r;
}HOMEParams;

typedef struct tagAutoLevelingCmd {
    uint8_t controlFlag;
    float precision;
}AutoLevelingCmd;

/*
 * Hand hold teach
 */
typedef enum tagHHTTrigMode {
    TriggeredOnKeyReleased,
    TriggeredOnPeriodicInterval
}HHTTrigMode;

/*
 * End effector
 */
typedef struct tagEndEffectorParams {
    float xBias;
    float yBias;
    float zBias;
}EndEffectorParams;

/*
 * Arm orientation
 */
typedef enum tagArmOrientation {
    LeftyArmOrientation,
    RightyArmOrientation,
}ArmOrientation;

/*
 * JOG related
 */
typedef struct tagJOGJointParams {
    float velocity[4];
    float acceleration[4];
}JOGJointParams;

typedef struct tagJOGCoordinateParams {
    float velocity[4];
    float acceleration[4];
}JOGCoordinateParams;

typedef struct tagJOGLParams {
    float velocity;
    float acceleration;
}JOGLParams;

typedef struct tagJOGCommonParams {
    float velocityRatio;
    float accelerationRatio;
}JOGCommonParams;

enum {
    JogIdle,
    JogAPPressed,
    JogANPressed,
    JogBPPressed,
    JogBNPressed,
    JogCPPressed,
    JogCNPressed,
    JogDPPressed,
    JogDNPressed,
    JogEPPressed,
    JogENPressed
};

typedef struct tagJOGCmd {
    uint8_t isJoint;
    uint8_t cmd;
}JOGCmd;

/*
 * PTP related
 */
typedef struct tagPTPJointParams {
    float velocity[4];
    float acceleration[4];
}PTPJointParams;

typedef struct tagPTPCoordinateParams {
    float xyzVelocity;
    float rVelocity;
    float xyzAcceleration;
    float rAcceleration;
}PTPCoordinateParams;

typedef struct tagPTPLParams {
    float velocity;
    float acceleration;
}PTPLParams;

typedef struct tagPTPJumpParams {
    float jumpHeight;
    float zLimit;
    uint32_t isUsingZLimt;
}PTPJumpParams;

typedef struct tagPTPJump2Params {
    float startJumpHeight;
    float endJumpHeight;
    float zLimit;
}PTPJump2Params;

typedef struct tagPTPCommonParams {
    float velocityRatio;
    float accelerationRatio;
}PTPCommonParams;

enum PTPMode {
    PTPJUMPXYZMode,
    PTPMOVJXYZMode,
    PTPMOVLXYZMode,

    PTPJUMPANGLEMode,
    PTPMOVJANGLEMode,
    PTPMOVLANGLEMode,

    PTPMOVJANGLEINCMode,
    PTPMOVLXYZINCMode,
    PTPMOVJXYZINCMode,

    PTPJUMPMOVLXYZMode,
};

typedef struct tagPTPCmd {
    uint8_t ptpMode;
    float x;
    float y;
    float z;
    float r;
}PTPCmd;

typedef struct tagPTPWithLCmd {
    uint8_t ptpMode;
    float x;
    float y;
    float z;
    float r;
    float l;
}PTPWithLCmd;

typedef struct tagParallelOutputCmd {
    uint8_t ratio;
    uint16_t address;
    uint8_t level;
}ParallelOutputCmd;

/*
 * CP related
 */
typedef struct tagCPParams
{
    float planAcc;
    float juncitionVel;
    union {
        float acc;
        float period;
    };
    uint8_t realTimeTrack;
}CPParams;

enum CPMode {
    CPRelativeMode,
    CPAbsoluteMode
};

typedef struct tagCPCmd {
    uint8_t cpMode;
    float x;
    float y;
    float z;
    union {
        float velocity;
        float power;
    };
}CPCmd;

/*
 * ARC related
 */
typedef struct tagARCParams {
    float xyzVelocity;
    float rVelocity;
    float xyzAcceleration;
    float rAcceleration;
}ARCParams;

typedef struct tagARCCmd {
    struct {
        float x;
        float y;
        float z;
        float r;
    }cirPoint;
    struct {
        float x;
        float y;
        float z;
        float r;
    }toPoint;
}ARCCmd;

typedef struct tagCircleCmd {
    struct {
        float x;
        float y;
        float z;
        float r;
    }cirPoint;
    struct {
        float x;
        float y;
        float z;
        float r;
    }toPoint;
    uint32_t count;
}CircleCmd;

typedef struct tagWAITCmd {
    uint32_t timeout;
}WAITCmd;

typedef enum tagTRIGMode {
    TRIGInputIOMode,
    TRIGADCMode
}TRIGMode;

typedef enum tagTRIGInputIOCondition {
    TRIGInputIOEqual,
    TRIGInputIONotEqual
}TRIGInputIOCondition;

typedef enum tagTRIGADCCondition {
    TRIGADCLT,  // Lower than
    TRIGADCLE,  // Lower than or Equal
    TRIGADCGE,  // Greater than or Equal
    TRIGADCGT   // Greater Than
}TRIGADCCondition;

typedef struct tagTRIGCmd {
    uint8_t address;
    uint8_t mode;
    uint8_t condition;
    uint16_t threshold;
}TRIGCmd;

typedef enum tagIOFunction {
    IOFunctionDummy,
    IOFunctionDO,
    IOFunctionPWM,
    IOFunctionDI,
    IOFunctionADC
}IOFunction;

typedef struct tagIOMultiplexing {
    uint8_t address;
    uint8_t multiplex;
}IOMultiplexing;

typedef struct tagIODO {
    uint8_t address;
    uint8_t level;
}IODO;

typedef struct tagIODAC {
    uint8_t address;
    uint16_t value;
}IODAC;

typedef struct tagIODI {
    uint8_t address;
    uint8_t level;
}IODI;

typedef struct tagIOADC {
    uint8_t address;
    uint16_t value;
}IOADC;

typedef struct tagEMotor {
    uint8_t index;
    uint8_t isEnabled;
    float speed;
}EMotor;

typedef struct tagEMotorS {
    uint8_t index;
    uint8_t isEnabled;
    int deltaPulse;
}EMotorS;

/*
 * WIFI related
 */
typedef struct tagWIFIConfig{
    uint8_t dhcp;
    uint8_t enable; //is available
    uint8_t status; //enum WifiStatus
    uint8_t bssid[6];
    uint8_t ssid[32];
    uint8_t password[32];
    uint8_t addr[16];
    uint8_t mask[16];
    uint8_t gateway[16];
    uint8_t dns[16];
}WIFIConfig;

enum WifiStatus{
    DisConnected,
    Connected,
    Scanning,
    Connecting,
    Disabled,
    Error
};

typedef struct tagWIFIListItem{
    uint8_t ssid[32];
    uint8_t encryp;
    uint8_t bssid[6];
}WIFIListItem;

typedef struct tagWIFIList{
    uint8_t count;
    uint8_t pageIndex;
    WIFIListItem item[4];
    //format:one item is len|encryp|ssid //len = encryp + ssid
}WIFIList;

typedef struct tagWIFIStatus{
    uint8_t enable; //is available
    uint8_t status; //enum WifiStatus
}WIFIStatus;

/*
 * Test
 */
typedef struct tagUserParams{
    float params[8];
}UserParams;

/*
 * Firmware related
 */

enum FirmwareSwitchMode{
    NO_SWITCH,
    DOBOT_SWITCH,
    PRINTING_SWITCH,
    DRIVER1_SWITCH,
    DRIVER2_SWITCH,
    DRIVER3_SWITCH,
    DRIVER4_SWITCH,
    DRIVER5_SWITCH,
    FPGA_SWITCH,
    SWITCH_FM_MAX
};

enum FirmwareMode{
    INVALID_MODE = 0,
    DOBOT_MODE,
    PRINTING_MODE,
    OFFLINE_MODE,
    MTEST_MODE,
    BROKEN_MODE
};

enum FirmwareSPStatus{
    ERROR = -1, // 错误
    DOING = 0, // 正在运行
    FINISHED = 1, //运行完成
    IDLE = 2 //空闲,准备烧写
};

typedef struct tagFirmwareProgress {
    uint8_t  status; //enum FirmwareSPStatus
    uint8_t  progress; //0-100
}FirmwareProgress;

typedef struct tagFirmwareModes {
    uint8_t  mode;
    uint8_t  ctl; //0 or 1
}FirmwareModes;

typedef struct tagLanConfig {
    uint8_t status; //enum WifiStatus
    uint8_t dhcp;
    uint8_t addr[16];
    uint8_t mask[16];
    uint8_t gateway[16];
    uint8_t dns[16];
}LanConfig;

struct SelfCheckErrorFlag{
  unsigned int fpga: 1;
  unsigned int spiflash: 1;
  unsigned int dac: 1;
  unsigned int motor_coder: 4;  //0 1 2 3
  unsigned int motor_eeprom: 4; //0 1 2 3
  unsigned int motor_temp1: 4;  //0 1 2 3
  unsigned int motor_temp2: 4;  //0 1 2 3
  unsigned int home_switch: 4;  //0 1 2 3
  unsigned int end_ioboard: 1;
  unsigned int eth0:1;
  unsigned int usb:1;
  unsigned int uart:1;
  unsigned int :0;
};

typedef enum tagServoControlLoop{
    ServoPositionLoop,
    ServoVelocityLoop,
    ServoCurrentLoop
}ServoControlLoop;

typedef struct tagPIDParams{
    float p;
    float i;
    float d;
    float v;
    float a;
}PIDParams;

typedef struct tagHardwareInfo{
    char machineNum[11];
    char mainBoard[11];
    char driverRearArm[11];
    char driverFrontArm[11];
    char driverZArm[11];
    char driverRArm[11];
    char encoderRearArm[11];
    char encoderFrontArm[11];
    char encoderZArm[11];
    char encoderRArm[11];
    char brakeBoard[11];
    char endIOBoard[11];
}HardwareInfo;

typedef struct tagPID{
    uint8_t index;
    uint8_t controlLoop;
    PIDParams params;
}PID;

//test

typedef struct
{
    float x;
    float y;
    float z;
    float r;
}stScaraPose, CoordParams, *pstScaraPose;


/*********************************************************************************************************
** API result
*********************************************************************************************************/
enum {
    DobotConnect_NoError,
    DobotConnect_NotFound,
    DobotConnect_Occupied
};

enum {
    DobotCommunicate_NoError,
    DobotCommunicate_BufferFull,
    DobotCommunicate_Timeout,
    DobotCommunicate_InvalidParams
};

#pragma pack(pop)
#endif
