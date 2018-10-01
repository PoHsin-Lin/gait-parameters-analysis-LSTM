/**************************************************************************************************
Filename:       simpleBLEPeripheral.c
Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
Revision:       $Revision: 23333 $

Description:    This file contains the Simple BLE Peripheral sample application
for use with the CC2540 Bluetooth Low Energy Protocol Stack.

Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Texas Instruments Incorporated (the "License").  You may not use this
Software unless you agree to abide by the terms of the License. The License
limits your use, and you acknowledge, that the Software may not be modified,
copied or distributed unless embedded on a Texas Instruments microcontroller
or used solely and exclusively in conjunction with a Texas Instruments radio
frequency transceiver, which is integrated into your product.  Other than for
the foregoing purpose, you may not use, reproduce, copy, prepare derivative
works of, modify, distribute, perform, display or sell this Software and/or
its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
* INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#if defined( CC2540_MINIDK )
#include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
#include "oad.h"
#include "oad_target.h"
#endif

#include "epl_hal_uart.h"
#include "epl_MPU9250.h"
#include "epl_IMU.h"
#include "IMUprofile.h"
#include "reminderProfile.h"
#include "Calibration_value.h"

#include <math.h>

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#define PI                                    3.14159

#define VEL_BUFFER_LEN                        5
#define ACC_BUFFER_LEN                        3

#define ABS(x) (((x)>0) ? (x) : -1*(x))


// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#define TIMER_16BIT_VAL                       62500

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

enum {_BROADCAST=4, _VIBRATION, _TRAINNUMBER_H, _TRAINNUMBER_L, _DEPART, _DESTINATION,
_DOXACH,_DOXACL,_DOYACH,_DOYACL,_DOZACH,_DOZACL};

uint8 start_detect = 0;

int ax, ay, az, gx, gy, gz, mx, my, mz;   
float acc[3], gyro[3], acc_divider, gyro_divider, tc_acc[3], li_acc[3], li_vel[3]={0}, li_Pos[3]={0};

float mag[3]={0};
float gyroBias[3], accBias[3];
float angles[3];
float rotMat[3][3];
int16 data[10]; 
float magBias[3] = {0, 0, 0}, magScale[3] = {0, 0, 0};
int connectFlag = 0;
uint8 one = 0x01;

uint16 get_tick=0;
//uint16  tStart = 0, tEnd = 0;
//uint16   tStart2 = 0, tEnd2 = 0;
float latency = 0;
float tick;
//int inNum = 0;

//add by Michael
float ea_list[3][10];
float avg_ea[3];
long int counter=0;
bool first = false;
int i,j,tmp_sum,diff,timeCount= 0;
uint8 alarm = 1;
float magCalibration[3]={0,0,0};
//linear acc calibration
uint8 liacc_counter = 0;
float liaccBias[3] = {0}, liacc_sum[3] = {0};
float K_state[3][5];
float acc_buffer[3][ACC_BUFFER_LEN] = {0};
float acc_mean=0;
uint8 acc_buffer_index = 0, acc_pre, acc_pre_pre;
uint8 cali_liacc_flag = 1;
float diff1, diff2, diff_1_2;

//linear velocity filter
float vel_buffer[3][VEL_BUFFER_LEN] = {0};
uint8 vel_buffer_index = 0;

//for UART debug
char str[100];

//Sample period
int firstPeriod = 1;
unsigned int t_count=0, t_pre=0;
unsigned int t_count_x=0, t_pre_count_x=0;
float x=0;
float samplePeriod=0;
float seconds = 0;

union {
  float f[3];
  uint8 b[12];
}ea;

int ccount = 1;
bool interruptFront = false;
bool interruptBack = false;
//bool stopNotification = false;

#define ULTRA_INIT 0
#define ULTRA_HEEL 1
#define ULTRA_BOTH 2

unsigned int ult_start,ult_end = 0;
float ultraperiod;
uint8 ultraState = ULTRA_INIT;

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

void triggleUltrasonic(/*bool* powerSwitch*/);
/*********************************************************************
* LOCAL VARIABLES
*/
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x08,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'I',
  'M',
  'U',
  'U',
  'U',
  'U',
  'U',
  
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  
  0x0F,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  0x01,
  0x02,
  0x00, //broadcast
  0x00, //vibration
  0x00, //train number higher byte
  0x00, //train number lower byte
  0x00, //depart
  0x00, //Destination
  
  0x00, //high data of x accelerormeter
  0x00, //low data of x accelerormeter
  0x00, //high data of y accelerormeter
  0x00, //low data of y accelerormeter
  0x00, //high data of z accelerormeter
  0x00, //low data of z accelerormeter
  
  
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  HI_UINT16( REMINDERPROFILE_SERV_UUID ),     
  LO_UINT16( REMINDERPROFILE_SERV_UUID ),
  HI_UINT16(CALIBRATION_VALUE_SERV_UUID ),
  LO_UINT16( CALIBRATION_VALUE_SERV_UUID )
    
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "ReminderTest";

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );


//add our call back function
static void reminderProfileChangeCB( uint8 paramID );
//static void IMUProfileChangeCB( uint8 paramID );

#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif




/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
//static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
//{
//  simpleProfileChangeCB    // Charactersitic value change callback
//};

//static motiontrackProfileCBs_t motiontrack_ProfileCBs =
//{
//  motiontrackProfileChangeCB
//};
//
//static IMUProfileCBs_t IMU_ProfileCBs =
//{
//  IMUProfileChangeCB
//};

static reminderProfileCBs_t simpleBLEPeripheral_reminderProfileCBs =
{
  reminderProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      SimpleBLEPeripheral_Init
*
* @brief   Initialization function for the Simple BLE Peripheral App Task.
*          This is called during initialization and should contain
*          any application specific initialization (ie. hardware
*          initialization/setup, table initialization, power up
*          notificaiton ... ).
*
* @param   task_id - the ID assigned by OSAL.  This ID should be
*                    used to send messages and set timers.
*
* @return  none
*/
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
#if defined( CC2540_MINIDK )
    // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
    uint8 initial_advertising_enable = FALSE;
#else
    // For other hardware platforms, device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;
#endif
    
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
    
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
  
  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
    
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }
  
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  //  IMUProfile_BLAddService( GATT_ALL_SERVICES );
  reminderProfile_AddService( GATT_ALL_SERVICES );
  Calibration_value_AddService( GATT_ALL_SERVICES );
  
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif
  // Setup reminder Charteristic parameters
  {
    uint8 trainNumber[2] = {0x00,0x00};
    uint8 depart = 0x00;
    uint8 destination = 0x00;
    uint8 vibration = 0x00;
    uint8 broacast = 0x00;
    uint8 startdetect = 0x00;
    reminderProfile_SetParameter(TRAINNUMBER,2,trainNumber);
    reminderProfile_SetParameter(DEPART,1,&depart);
    reminderProfile_SetParameter(DESTINATION,1,&destination);
    reminderProfile_SetParameter(VIBRATION,1,&vibration);
    reminderProfile_SetParameter(BROADCAST,1,&broacast);
    reminderProfile_SetParameter(STARTDETECT,1,&startdetect);
    
    uint8 dataclean[12] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    Calibration_value_SetParameter( MAG_CALI,12,dataclean );  
    Calibration_value_SetParameter( MAG_BIAS,12,dataclean);
    Calibration_value_SetParameter( MAG_SCALE,12,dataclean);     
    Calibration_value_SetParameter( ACC_BIAS,12,dataclean);
    Calibration_value_SetParameter( GYO_BIAS,12,dataclean);
    
    //Calibration_value_SetParameter(OTHER4_UUID,1,&depart);
  }
  
#if defined( CC2540_MINIDK )
  
  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.
  
  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO
  
  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
  // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output
  
  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low
  
#endif // #if defined( CC2540_MINIDK )
  
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
  
#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
#else
  HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
#endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD
  
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
  
  VOID reminderProfile_RegisterAppCBs( &simpleBLEPeripheral_reminderProfileCBs );
  
  //  VOID IMUProfile_RegisterAppCBs( &IMU_ProfileCBs );
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
  
#if defined ( DC_DC_P0_7 )
  
  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );
  
#endif // defined ( DC_DC_P0_7 )
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
  
  
//  mpu9250_init(BITS_DLPF_CFG_184HZ, BITS_ADLPF_184HZ);
//  set_acc_scale(BITS_FS_8G);
//  set_gyro_scale(BITS_FS_500DPS);
//  calibrateMPU9250(gyroBias, accBias);
//  Calibration_value_SetParameter(ACC_BIAS,12,accBias);
//  Calibration_value_SetParameter(GYO_BIAS,12,gyroBias);
//  
//  //magcalMPU9250(magBias, magScale);
//  //Calibration_value_SetParameter( MAG_BIAS,12,magBias);
//  //Calibration_value_SetParameter( MAG_SCALE,12,magScale);
//  get_Magnetometer_ASA(magCalibration);
//  Calibration_value_SetParameter( MAG_CALI,12,magCalibration );
  
//  P2SEL = 0; // Configure Port 2 as GPIO
//  P2DIR = 0x00;
  
  P2DIR &= ~0x06; 
  P2IEN |= 0x06; // P2_1, P2_2
  IEN2 |= 0x02;
  
//  uartInit(HAL_UART_BR_115200);
//  uartWriteString("hello\r\n");
  P0DIR |= 0x04;
  P0DIR &= ~0x08;
  P0IEN |= 0x08;
  EA = 1;
 
  start_detect = 1;
  osal_start_timerEx( simpleBLEPeripheral_TaskID, SMP_IMU_INIT_EVT, 50 );
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_ProcessEvent
*
* @brief   Simple BLE Peripheral Application Task event processor.  This function
*          is called to process all events for the task.  Events
*          include timers, messages and any other user defined events.
*
* @param   task_id  - The OSAL assigned task ID.
* @param   events - events to process.  This is a bit map and can
*                   contain more than one event.
*
* @return  events not processed
*/
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
//    sprintf(str,"SYS_EVENT_MSG\r\n");
//    uartWriteString(str);
    uint8 *pMsg;
    
    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if ( events & SBP_START_DEVICE_EVT )
  {
//    sprintf(str,"SBP_START_DEVICE_EVT\r\n");
//    uartWriteString(str);
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );
    
    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );
    
    return ( events ^ SBP_START_DEVICE_EVT );
  }
  
  if ( events & SMP_IMU_INIT_EVT )
  {
    /*Initial and calibration when connection*/
    if(gapProfileState == GAPROLE_CONNECTED){
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_9DOF_EVT, 100 );
      // T1CTL = 0x09;
      EA = 1;
    }
    else  
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SMP_IMU_INIT_EVT, 50);
    } 
    
    return (events ^ SMP_IMU_INIT_EVT);
  }
  
  if ( events & SBP_9DOF_EVT )
  {
    if(gapProfileState != GAPROLE_CONNECTED){
      connectFlag = 0;
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_9DOF_EVT, 50);
    } else if(gapProfileState == GAPROLE_CONNECTED && connectFlag == 0){
        connectFlag = 1;
        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_REQ, sizeof( uint8 ), &one );
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_9DOF_EVT, 50);
    } else{ // gapProfileState == GAPROLE_CONNECTED && connectFlag != 0
//        MPU9250_getData(data);
//        t_count = (unsigned int)T1CNTL;
//        t_count = (unsigned int)t_count | T1CNTH<<8;
//        if(t_pre > t_count) {
//          get_tick = t_count+0xFFFF-t_pre;
//        } else{ 
//          get_tick = t_count-t_pre;
//        }
//        t_pre = t_count;
      
        get_tick = 0x00;
        if(interruptFront == true){
          get_tick |= 0x8000;
          interruptFront = false;
        }
        if(interruptBack == true){
          get_tick |= 0x4000;
          interruptBack = false;
        }
//        data[9]=get_tick;
        
        reminderProfile_SetParameter(EULER_ANGLE,2, &get_tick);
//        if(stopNotification){
//          osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_9DOF_EVT, 1000);
//        } else{
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_9DOF_EVT, 5);
//        }
    }
    
    // check whether to trigger ultrasonic
//    if(ultraState == ULTRA_HEEL){
//      ultraState = ULTRA_INIT;
//      osal_start_timerEx( simpleBLEPeripheral_TaskID, TRIGGLESONIC_EVT, 500);
////      T1CTL = 0x05; 
////      IEN1 |= 0x20;
////     
////      triggleUltrasonic();
////      
////      IEN1 &= ~0x20;   
////        
////      if(ult_start > ult_end)
////        ultraperiod = (double)(ult_end + 0xFFFF - ult_start)/4000.0f;//ms
////      else        
////        ultraperiod = (double)(ult_end - ult_start)/4000.0f;//ms     
////      float meter = 33700*0.5*ultraperiod*0.001;
////      T1CTL = 0x09;
////      reminderProfile_SetParameter(BROADCAST, sizeof(meter) ,&meter); 
//    }
    
    return (events ^ SBP_9DOF_EVT);
  }
  
  if ( events & TRIGGLESONIC_EVT )
  {
    T1CTL = 0x09; 
    IEN1 |= 0x20;
   
    triggleUltrasonic();
    
//    IEN1 &= ~0x20;   
//      
//    if(ult_start > ult_end)
//      ultraperiod = (float)(ult_end + 0xFFFF - ult_start)/1000.0f;//ms
//    else        
//      ultraperiod = (float)(ult_end - ult_start)/1000.0f;//ms     
//    
//    float meter = 33700*0.5*ultraperiod*0.001;
//    
//    T1CTL = 0x09;
//    reminderProfile_SetParameter(BROADCAST, sizeof(meter) ,&meter); 
    osal_start_timerEx( simpleBLEPeripheral_TaskID, RECEIVESONIC_EVT, 25);
    
    return (events ^ TRIGGLESONIC_EVT);
  }
  
  if (events & RECEIVESONIC_EVT)
  {
    IEN1 &= ~0x20;   
      
    if(ult_start > ult_end)
      ultraperiod = (float)(ult_end + 0xFFFF - ult_start)/1000.0f;//ms
    else        
      ultraperiod = (float)(ult_end - ult_start)/1000.0f;//ms
    
//    stopNotification = false;
    float meter = 33700*0.5*ultraperiod*0.001;
    
    T1CTL = 0x09;
    reminderProfile_SetParameter(BROADCAST, sizeof(meter) ,&meter); 
    
    return (events ^ RECEIVESONIC_EVT);
  }
  
  // Discard unknown events
  return 0;
}

void triggleUltrasonic(/*bool* powerSwitch*/)
{
  //P0 |= 0x04;
  PICTL &= ~0x01;
  
  WAIT_US(10);
  P0_2 = 1;
  WAIT_US(40);
  P0_2 = 0;
  // WAIT_MS(25);
   
  //P0 &= ~0x04;
}

/*********************************************************************
* @fn      simpleBLEPeripheral_ProcessOSALMsg
*
* @brief   Process an incoming task message.
*
* @param   pMsg - message to process
*
* @return  none
*/
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
#if defined( CC2540_MINIDK )
  case KEY_CHANGE:
    simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
    break;
#endif // #if defined( CC2540_MINIDK )
    
  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
* @fn      simpleBLEPeripheral_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;
  
  VOID shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }
  
  if ( keys & HAL_KEY_SW_2 )
  {
    
    SK_Keys |= SK_KEY_RIGHT;
    
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    // Note:  If PLUS_BROADCASTER is define this condition is ignored and
    //        Device may advertise during connections as well. 
#ifndef PLUS_BROADCASTER  
    if( gapProfileState != GAPROLE_CONNECTED )
    {
#endif // PLUS_BROADCASTER
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;
      
      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
      
      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }
      
      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
#ifndef PLUS_BROADCASTER
    }
#endif // PLUS_BROADCASTER
  }
  
  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
* @fn      peripheralStateNotificationCB
*
* @brief   Notification from the profile of a state change.
*
* @param   newState - new state
*
* @return  none
*/
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
  case GAPROLE_STARTED:
    {
      uint8 ownAddress[B_ADDR_LEN];
      uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
      
      GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
      
      // use 6 bytes of device address for 8 bytes of system ID value
      systemId[0] = ownAddress[0];
      systemId[1] = ownAddress[1];
      systemId[2] = ownAddress[2];
      
      // set middle bytes to zero
      systemId[4] = 0x00;
      systemId[3] = 0x00;
      
      // shift three bytes up
      systemId[7] = ownAddress[5];
      systemId[6] = ownAddress[4];
      systemId[5] = ownAddress[3];
      
      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      
    }
    break;
    
  case GAPROLE_ADVERTISING:
    {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
    }
    break;
    
  case GAPROLE_CONNECTED:
    {        
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      
#ifdef PLUS_BROADCASTER
      // Only turn advertising on for this state when we first connect
      // otherwise, when we go from connected_advertising back to this state
      // we will be turning advertising back on.
      if ( first_conn_flag == 0 ) 
      {
        uint8 adv_enabled_status = 1;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
        first_conn_flag = 1;
      }
#endif // PLUS_BROADCASTER
    }
    break;
    
  case GAPROLE_CONNECTED_ADV:
    {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
    }
    break;      
  case GAPROLE_WAITING:
    {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
    }
    break;
    
  case GAPROLE_WAITING_AFTER_TIMEOUT:
    {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      
#ifdef PLUS_BROADCASTER
      // Reset flag for next connection.
      first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
    }
    break;
    
  case GAPROLE_ERROR:
    {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
    }
    break;
    
  default:
    {
#if (defined HAL_LCD) && (HAL_LCD == TRUE)
      HalLcdWriteString( "",  HAL_LCD_LINE_3 );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
    }
    break;
    
  }
  
  gapProfileState = newState;
  
#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
  // "CC2540 Slave" configurations
#endif
  
  
}


static void reminderProfileChangeCB(uint8 paramID)
{
  uint8 newValue, newValue2[2];
  
  switch( paramID )
  {
  case STARTDETECT:
//    stopNotification = true;
    osal_start_timerEx( simpleBLEPeripheral_TaskID, TRIGGLESONIC_EVT, 5);
  default:
    break;
  }
}


#pragma vector = P2INT_VECTOR   
__interrupt void P2INT_Isr(void)
{
  if( (P2IFG & 0x04) == 0x04){
    if(ultraState == ULTRA_INIT){
      interruptFront = true;
    }
  } else if( (P2IFG & 0x02) == 0x02 ){
    interruptBack = true;
//    if(ultraState == ULTRA_INIT){
//      ultraState = ULTRA_HEEL;
//    }
  }  else{
    
  }
  
  P2IFG = 0;
  IRCON2  = 0x00;
  P2IF = 0;
}

#pragma vector = P0INT_VECTOR 
__interrupt void P0INT(void)
{
  if((P0IFG & 0x08)== 8)
  {
    if((PICTL & 0x01) == 1) // falling
    {
            
      ult_end = (unsigned int) T1CNTL;
      ult_end = (unsigned int) T1CNTH << 8 | ult_end;
      
      PICTL &= ~0x01;//P2 rising
    }
    else //rising
    {
      ult_start = (unsigned int) T1CNTL;
      ult_start = (unsigned int) T1CNTH << 8 | ult_start;
      
      PICTL |= 0x01;
    }
    
  }
  
  P0IFG = 0;
  IRCON = 0x00;
  P0IF = 0;

}
