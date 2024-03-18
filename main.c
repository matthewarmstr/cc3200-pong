//*****************************************************************************
//
// Application Name     - TV Remote Decoder (TV Code: Zonda 1355)
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <stdint.h>
#include <math.h>

// Simplelink includes
#include "simplelink.h"

// Driverlib includes
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "spi.h"
#include "hw_nvic.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "prcm.h"
#include "utils.h"
#include "systick.h"
#include "rom_map.h"
#include "timer.h"

// Common interface includes
#include "timer_if.h"
#include "uart.h"
#include "uart_if.h"
#include "gpio.h"
#include "gpio_if.h"
#include "i2c_if.h"
#include "common.h"

// Pin configurations
#include "pin_mux_config.h"

// Source file imports
#include "i2cSource.h"

// AWS connectivity resources
#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2018"
#define SERVER_NAME             "af42jxgcmhc34-ats.iot.us-west-1.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "/cert/rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                15    /* Current Date */
#define MONTH               3     /* Month 1-12 */
#define YEAR                2024  /* Current year */
#define HOUR                3    /* Time - hours */
#define MINUTE              10    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /things/pong_game_majg/shadow HTTP/1.1\r\n"
#define HOSTHEADER "Host: af42jxgcmhc34-ats.iot.us-west-1.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

//#define DATA1 "{\"state\": {\n\"desired\" : {\n\"var\" : \"Hello phone, message from CC3200 via AWS IoT!\"\n}}}\n\n"
#define DATA1 "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \"Hello phone, message from CC3200 via AWS IoT!\"\r\n}}}\r\n\r\n"

#define EMAIL_MESSAGE "{\"state\": {\n\"desired\" : {\n\"message\" : \"Test, message from CC3200 via AWS IoT!\"\n}}}\n\n"
#define IR_WINNER_MESSAGE "{\"state\": {\n\"desired\" : {\n\"message\" : \"Player with IR Remote won!\"\n}}}\n\n"
#define ACCEL_WINNER_MESSAGE "{\"state\": {\n\"desired\" : {\n\"message\" : \"Player with Accelerometer won!\"\n}}}\n\n"

#define GETHEADER "GET /things/pong_game_majg/shadow HTTP/1.1\r\n"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;

#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

// OLED parameters (1 = left OLED, 0 = right OLED)
#define OLED_LEFT           1
#define OLED_MAX_PIXELS     128

// On-screen object characteristics
#define BAR_LENGTH              32
#define BAR_WIDTH               3
#define BAR_BOUNCE_THRESHOLD            4
#define BAR_SPECIAL_BOUNCE_THRESHOLD    8
#define BAR_OFFSET_FROM_SIDE            20
#define LEFT_BAR_MOVEMENT_SIZE  4
#define RIGHT_BAR_MOVEMENT_SIZE 2
#define BALL_RADIUS             2

// Object boundary characteristics
#define MIN_X_POSITION \
    BAR_WIDTH + BALL_RADIUS
#define MAX_X_POSITION \
    OLED_MAX_PIXELS - BAR_WIDTH - BALL_RADIUS - 1
#define MIN_Y_POSITION \
    BAR_WIDTH + BALL_RADIUS + BAR_OFFSET_FROM_SIDE + 1

// IR remote button hex values
#define CH_UP       0x210706F9
#define CH_DOWN     0x21078A75
#define VOL_UP      0x21070AF5
#define VOL_DOWN    0x21078679

#define START_BYTE 'S'
#define STOP_BYTE 'X'

#define GAME_START_BYTE 'G'
#define GAME_END_BYTE 'E'
#define BALL_SEND 'B'
#define SCORE_SENT 'P'

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;

// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

#define I2C_BASE              0x40020000

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

extern void (* const g_pfnVectors[])(void);

volatile unsigned char button_press_received;
volatile unsigned int position;
volatile unsigned int starting;
volatile unsigned int data_processing;
volatile uint32_t ir_hex_val;
volatile int8_t ir_hex_val_bit_count;
volatile unsigned int bar_position;

volatile unsigned int game_Start_Check = 0;
volatile unsigned int game_Over_Check = 0;
//volatile unsigned int ball_Not_Here = 0;

volatile unsigned int balls_Not_Here[] = {1,0};

volatile int self_Score = 0;

extern signed char tilt_value;

//int x_position = 64;
//int x_velocity = 2;
//
//int y_position = 64;
//int y_velocity = -1;

int x_positions[] = {64, 64};
int y_positions[] = {64, 120};

int x_velocities[] = {2, -1};
int y_velocities[] = {-1, -1};

int opponent_Score = 0;
int left_OLED_won = 0;

unsigned char queued_letters[256];
unsigned char game_over = 0;

static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

SlDateTime g_time;


//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;

static volatile uint8_t timer_started = 0;


//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int send_message(int);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

//            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
//                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
//                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
//                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
//                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
//                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
//                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
//                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
//                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
//                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
//                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
//                           g_ucConnectionBSSID[5]);
            }
            else {
//                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
//                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
//                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
//                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
//                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
//                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
//            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
//                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

//            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
//                       "Gateway=%d.%d.%d.%d\n\r",
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
//            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
//            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
//                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
//    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
//               pDevEvent->EventData.deviceEvent.status,
//               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE:
//                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
//                                "failed to transmit all queued packets\n\n",
//                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
//                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
//                                "(%d) \n\n",
//                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
//            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End breadcrumb: s18_df
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

//    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
//    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
//    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
//    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
//    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
//    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
//    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);



    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

//    UART_PRINT("Attempting connection to access point: ");
//    UART_PRINT(SSID_NAME);
//    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

//    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}




long printErrConvenience(char * msg, long retVal) {
//    UART_PRINT(msg);
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}


//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP, uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
//    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256 // does not work (-340, handshake fails)
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }



/////////////////////////////////
// START: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
// END: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
/////////////////////////////////


    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal >= 0) {
//        UART_PRINT("Device has connected to the website:");
//        UART_PRINT(SERVER_NAME);
//        UART_PRINT("\n\r");
    }
    else if(lRetVal == SL_ESECSNOVERIFY) {
//        UART_PRINT("Device has connected to the website (UNVERIFIED):");
//        UART_PRINT(SERVER_NAME);
//        UART_PRINT("\n\r");
    }
    else if(lRetVal < 0) {
//        UART_PRINT("Device couldn't connect to server:");
//        UART_PRINT(SERVER_NAME);
//        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



int connectToAccessPoint() {
    long lRetVal = -1;
    GPIO_IF_LedConfigure(LED1|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
//          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

//    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
//    UART_PRINT("Opening sl_start\n\r");
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
//        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

//    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
//        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

//    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    systick_cnt = 0;
}

unsigned int DeltaToLogicalValueInt(uint64_t delta) {
    // Between 300 - 700 us = logical 0
    if (delta > 24000 && delta < 56000) {
        return 0;
    }

    // Between 1300 - 1900 us = logical 1
    if (delta > 104000 && delta < 152000) {
        return 1;
    }

    // No data: 2000 - 2500 us
    if (delta > 160000 && delta < 200000) {
        return 2;
    }

    // Start of data: 4000 - 4500 us
    if (delta > 340000 && delta < 380000) {
        return 3;
    }

    // Begin transmission: 8.75 - 9.25 ms
    if (delta > 700000 && delta < 740000) {
        return 4;
    }

    return 5;
}

static int send_message(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

//    unsigned char initialJSON[] = "{\"state\": {\n\"desired\" : {\n\"message\" : \"";
//    unsigned char finalJSON[] =   "\"\n}}}\n\n";
//    strcat(message, initialJSON);
//    strcat(message, email_message_chars);
//    strcat(message, finalJSON);

    unsigned char message[128] = "";
    if (left_OLED_won) {
        strcat(message, IR_WINNER_MESSAGE);
    } else {
        strcat(message, ACCEL_WINNER_MESSAGE);
    }

//    printf("Sending message:\n%s\n", message);

    int dataLength = strlen(message);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, message);
    pcBufHeaders += strlen(message);

    int testDataLength = strlen(pcBufHeaders);

//    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
//        UART_PRINT("POST for email failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
//        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
//        UART_PRINT(acRecvbuff);
//        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

static void GPIOIntHandler(void) {
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (GPIOA3_BASE, true);
    MAP_GPIOIntClear(GPIOA3_BASE, ulStatus);

    uint64_t delta = SYSTICK_RELOAD_VAL - SysTickValueGet();
    uint32_t logical_int = DeltaToLogicalValueInt(delta);

    if (position == 1 && starting == 1 && data_processing == 0) {
        // Entering trough after seeing starting pulse width
        SysTickReset();
        position = 0;

        if (logical_int == 2) {
            // Button is held down - no new data to read
            starting = 0;
            button_press_received = 1;
            return;
        }
        if (logical_int == 3) {
            // Ack length indicates data is coming
            starting = 0;
            data_processing = 1;
            ir_hex_val_bit_count = 32;
            return;
        }
        return;

    } else if (position == 1 && starting == 0 && data_processing == 0){
        // Entering trough and not processing data
        position = 0;

    } else if (position == 0 && starting == 0 && data_processing == 0) {
        // Exiting trough when not processing data - check if pulse width matches start behavior
        if (logical_int == 4) {
            starting = 1;
        }

        position = 1;

    } else if (position == 1 && starting == 0 && data_processing == 1) {
        position = 0;

        // Shift in new bit from transmission
        if (ir_hex_val_bit_count > 0) {
            ir_hex_val_bit_count--;
            ir_hex_val = (ir_hex_val << 1) | logical_int;
        }

        if (ir_hex_val_bit_count == 0){
            button_press_received = 1;
        }

    } else if (position == 0 && starting == 0 && data_processing == 1) {
        position = 1;
        if (ir_hex_val_bit_count == 0) {
            data_processing = 0;
        }
    }

    SysTickReset();
}

void uartSendInteger(int intToSend) {
    int i;
    for (i = 0; i < 4; i ++) {
        UARTCharPut(UARTA1_BASE, (intToSend>>i*8) & 0xFF);
    }
}

int getSentInteger() {
    unsigned char bytes[4];

    int i;
    for (i = 0; i < 4; i ++) {
        bytes[i] = UARTCharGet(UARTA1_BASE);
    }

    return *(int *)bytes;
}

// START GAME
void gameReadyCheck(void) {
    UARTCharPut(UARTA1_BASE, START_BYTE);
    UARTCharPut(UARTA1_BASE, GAME_START_BYTE);
    UARTCharPut(UARTA1_BASE, STOP_BYTE);
//    printf("Done sending\n");
}

void awsConnectAndSend() {
    if (OLED_LEFT) {
        // Setup AWS connection to left OLED board for game score tracking
        long lRetVal = -1;
        //Connect the CC3200 to the local access point
        lRetVal = connectToAccessPoint();
        //Set time so that encryption can be used
        lRetVal = set_time();
        if(lRetVal < 0) {
//            UART_PRINT("Unable to set time in the device");
            LOOP_FOREVER();
         }
         //Connect to the website with TLS encryption
         lRetVal = tls_connect();
         if(lRetVal < 0) {
             ERR_PRINT(lRetVal);
         }
         send_message(lRetVal);
    }
}

// END GAME
void gameEndSignal(void) {
    // This player lost
    UARTCharPut(UARTA1_BASE, START_BYTE);
    UARTCharPut(UARTA1_BASE, GAME_END_BYTE);
    UARTCharPut(UARTA1_BASE, STOP_BYTE);
    awsConnectAndSend();
}

// SCORE INCREASE
void sendScoreNotif(void) {
    UARTCharPut(UARTA1_BASE, START_BYTE);
    UARTCharPut(UARTA1_BASE, SCORE_SENT);
    UARTCharPut(UARTA1_BASE, STOP_BYTE);
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
}


/**
 * Initializes SysTick Module
 */
static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}

int SideBounce(int ballIndex) {
    int next_x_position = x_positions[ballIndex] + x_velocities[ballIndex];
    if (next_x_position > MAX_X_POSITION || next_x_position < MIN_X_POSITION) {
        x_velocities[ballIndex] *= -1;
        return 1;
    }

    int next_y_position = y_positions[ballIndex] + y_velocities[ballIndex];
    if (next_y_position > OLED_MAX_PIXELS) {
        // Send ball info to other board over UARies[ballIndex]
        return -1;
    }

    return 0;
}

void CheckPlatformBounce(int ballIndex) {
    int next_x_position = x_positions[ballIndex] + x_velocities[ballIndex];
    int next_y_position = y_positions[ballIndex] + y_velocities[ballIndex];
    if (next_y_position < MIN_Y_POSITION) {
        int bar_bounce_lower_bound = bar_position - BAR_BOUNCE_THRESHOLD;
        int bar_bounce_upper_bound = bar_position + BAR_LENGTH + BAR_BOUNCE_THRESHOLD;
        unsigned char regular_bounce = (next_x_position > bar_bounce_lower_bound) && (next_x_position < bar_bounce_upper_bound);

        int bar_special_bounce_lower_bound = bar_position - BAR_SPECIAL_BOUNCE_THRESHOLD;
        int bar_special_bounce_upper_bound = bar_position + BAR_LENGTH + BAR_SPECIAL_BOUNCE_THRESHOLD;
        unsigned char special_bounce = (!regular_bounce) && (next_x_position > bar_special_bounce_lower_bound) && (next_x_position < bar_special_bounce_upper_bound);

        if (regular_bounce) {
            y_velocities[ballIndex] *= -1;
        } else if (special_bounce) {
            x_velocities[ballIndex] *= -1;
            y_velocities[ballIndex] *= -1;
        } else {
            if (opponent_Score == 6) {
                game_Over_Check = 1;
                sendScoreNotif();
                gameEndSignal();
                return;
            }
            opponent_Score += 1;
            sendScoreNotif();
            x_positions[ballIndex] = 90;
            y_positions[ballIndex] = 120;
            x_velocities[ballIndex] = 2;
            y_velocities[ballIndex] = -1;
        }
    }
}

void ballCollisions(void) {
    if (abs((int)(x_positions[0]-x_positions[1])) < 5 && abs((int)(y_positions[0]-y_positions[1])) < 5) {
        if (balls_Not_Here[0] == 1 || balls_Not_Here[1] == 1){
            return;
        }
        y_velocities[0] *= -1;
        y_velocities[1] *= -1;
    }
}

void UpdateBallPosition(void) {
//    int old_x_positions[] = &x_positions;
//    int old_y_positions[] = &y_positions;
 
    // int old_x_position = x_position;
    // int old_y_position = y_position;
    int i;
    for (i = 0; i < 2; i ++) {
        if (balls_Not_Here[i] == 1) {
            continue;
        }
        fillCircle(x_positions[i], y_positions[i], BALL_RADIUS, BLACK);
        int sideRetVal = SideBounce(i);
        if (sideRetVal == -1) {
    //        fillCircle(old_x_position, old_y_position, BALL_RADIUS, BLACK);

            UARTCharPut(UARTA1_BASE, START_BYTE);
            UARTCharPut(UARTA1_BASE, BALL_SEND);
            uartSendInteger(i);
            uartSendInteger(x_positions[i]);
            uartSendInteger(x_velocities[i]);
            uartSendInteger(y_velocities[i]);
            UARTCharPut(UARTA1_BASE, STOP_BYTE);

            balls_Not_Here[i] = 1;

            return;
        }

        if (sideRetVal == 0) {
            CheckPlatformBounce(i);
        }

        x_positions[i] += x_velocities[i];
        y_positions[i] += y_velocities[i];

        //    fillCircle(old_x_position, old_y_position, BALL_RADIUS, BLACK);
        fillCircle(x_positions[i], y_positions[i], BALL_RADIUS, WHITE);
    }

    ballCollisions();
}

void UpdateLeftPlatformPosition(void) {
    // Get values for pushed button value and platform position
    uint32_t remote_input = ir_hex_val;
    int new_bar_position = bar_position;
    int bar_top = new_bar_position + BAR_LENGTH;
    int bar_bottom = new_bar_position;

    // Display updated position of platform
    if ((remote_input == VOL_UP || remote_input == CH_UP) && bar_top < OLED_MAX_PIXELS) {
        // Move IR-controlled platform up
        fillRect(new_bar_position, BAR_OFFSET_FROM_SIDE, LEFT_BAR_MOVEMENT_SIZE, BAR_WIDTH, BLACK);
        new_bar_position += LEFT_BAR_MOVEMENT_SIZE;
        fillRect(bar_top, BAR_OFFSET_FROM_SIDE, LEFT_BAR_MOVEMENT_SIZE, BAR_WIDTH, WHITE);

    } else if ((remote_input == VOL_DOWN || remote_input == CH_DOWN) && bar_bottom > 0) {
        // Move IR-controlled platform down
        fillRect(new_bar_position + BAR_LENGTH - LEFT_BAR_MOVEMENT_SIZE, BAR_OFFSET_FROM_SIDE, LEFT_BAR_MOVEMENT_SIZE, BAR_WIDTH, BLACK);
        new_bar_position -= LEFT_BAR_MOVEMENT_SIZE;
        fillRect(new_bar_position, BAR_OFFSET_FROM_SIDE, LEFT_BAR_MOVEMENT_SIZE, BAR_WIDTH, WHITE);
    }
    bar_position = new_bar_position;
}

void UpdateRightPlatformPosition(void) {
    // Get values for accelerometer tilt and platform position
    char acCmdStore[512] = "readreg 0x18 0x2 6\n";
    ParseNProcessCmd(acCmdStore);
    int new_bar_position = bar_position;
    int bar_top = new_bar_position + BAR_LENGTH;
    int bar_bottom = new_bar_position;

    // Display updated position of platform
    if (tilt_value < 0 && bar_top < OLED_MAX_PIXELS) {
        // Move accelerometer-controlled platform up
        fillRect(new_bar_position, BAR_OFFSET_FROM_SIDE, RIGHT_BAR_MOVEMENT_SIZE, BAR_WIDTH, BLACK);
        new_bar_position += RIGHT_BAR_MOVEMENT_SIZE;
        fillRect(bar_top, BAR_OFFSET_FROM_SIDE, RIGHT_BAR_MOVEMENT_SIZE, BAR_WIDTH, WHITE);

    } else if (tilt_value > 0 && bar_bottom > 0) {
        // Move accelerometer-controlled platform down
        fillRect(new_bar_position + BAR_LENGTH - RIGHT_BAR_MOVEMENT_SIZE, BAR_OFFSET_FROM_SIDE, RIGHT_BAR_MOVEMENT_SIZE, BAR_WIDTH, BLACK);
        new_bar_position -= RIGHT_BAR_MOVEMENT_SIZE;
        fillRect(new_bar_position, BAR_OFFSET_FROM_SIDE, RIGHT_BAR_MOVEMENT_SIZE, BAR_WIDTH, WHITE);
    }
    bar_position = new_bar_position;
}

void updateOLEDScore(void) {
    drawChar(OLED_MAX_PIXELS - 10, 2, (unsigned char) (self_Score + 48), WHITE, BLACK, 0x2);
}

// UART Interrupt Handler
static void UARTIntHandler(void) {
    unsigned int intSignal = UARTIntStatus(UARTA1_BASE, 1);
    MAP_UARTIntClear(UARTA1_BASE, intSignal);
    unsigned char validStartByte = UARTCharGetNonBlocking(UARTA1_BASE);
    if (validStartByte != START_BYTE) {
        MAP_UARTIntClear(UARTA1_BASE, UART_INT_TX | UART_INT_RX);
        return;
    }
    int communicationType = UARTCharGet(UARTA1_BASE);

    if (communicationType == GAME_START_BYTE) {
        game_Start_Check = 1;
        game_Over_Check = 0;
        if (!OLED_LEFT) {
            gameReadyCheck();
        }

    } else if (communicationType == GAME_END_BYTE) {
        // This player won
        game_Over_Check = 1;
        unsigned char check_Stop_Byte = UARTCharGet(UARTA1_BASE);
        awsConnectAndSend();
        return;
    } else if (communicationType == BALL_SEND) {
        int ballIndex = getSentInteger();
        x_positions[ballIndex] = OLED_MAX_PIXELS - getSentInteger();
        x_velocities[ballIndex] = -1*getSentInteger();
        y_velocities[ballIndex] = -1*getSentInteger();
        y_positions[ballIndex] = 120;

        balls_Not_Here[ballIndex] = 0;
    } else if (communicationType == SCORE_SENT) {
        self_Score += 1;
        printf("Sending my updated score: %d\n", self_Score);
        updateOLEDScore();
    } else {
        printf("Invalid command\n");
    }

    unsigned char check_Stop_Byte = UARTCharGet(UARTA1_BASE);
    if (check_Stop_Byte != STOP_BYTE) {
        printf("Error in communicating, no stop byte\n");
    }
}

//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//!
//! \return None.
//
//****************************************************************************
int main() {

    BoardInit();

    PinMuxConfig();

    // Enable SysTick
    SysTickInit();

    // Initialize UART right_movement
    InitTerm();

    // Clear UART Terminal
    ClearTerm();

    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                      UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(GPIOA3_BASE, GPIOIntHandler);

    //
    // Configure rising edge interrupts on SW3
    //
    MAP_GPIOIntTypeSet(GPIOA3_BASE, 0x10, GPIO_BOTH_EDGES);

    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus(GPIOA3_BASE, false);
    MAP_GPIOIntClear(GPIOA3_BASE, ulStatus);

    // Setup UART interrupts
    UARTFIFOEnable(UARTA1_BASE);
    UARTTxIntModeSet(UARTA1_BASE, UART_TXINT_MODE_FIFO);
    UARTFIFOLevelSet(UARTA1_BASE, 0,0);
    UARTEnable(UARTA1_BASE);
    UARTIntRegister(UARTA1_BASE, UARTIntHandler);
    UARTIntEnable(UARTA1_BASE, UART_INT_RX | UART_INT_TX);

    MAP_IntPrioritySet(INT_UARTA1, INT_PRIORITY_LVL_2);
    MAP_IntPrioritySet(INT_GPIOA3, INT_PRIORITY_LVL_1);
    MAP_IntPrioritySet(INT_I2CA0, INT_PRIORITY_LVL_0);

    // Setup I2C thingy
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    Adafruit_Init();

    MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
            SPI_CS_ENABLE|SPI_CS_DISABLE);

    // clear global variables
    starting = 0;
    position = 1;
    data_processing = 0;
    button_press_received=0;
    ir_hex_val_bit_count = 0;
    bar_position = 32;

    MAP_GPIOIntEnable(GPIOA3_BASE, 0x10);

    printf("GPIO & UART INTERUPTS READY\n");

    // Show objects on OLED
    setTextSize(0x1);
    fillScreen(BLACK);

    fillRect(0, BAR_WIDTH + BAR_OFFSET_FROM_SIDE, BAR_WIDTH, OLED_MAX_PIXELS - BAR_WIDTH, WHITE);
    fillRect(OLED_MAX_PIXELS - BAR_WIDTH, BAR_WIDTH + BAR_OFFSET_FROM_SIDE, BAR_WIDTH, OLED_MAX_PIXELS - BAR_WIDTH, WHITE);

    fillRect(bar_position, BAR_OFFSET_FROM_SIDE, BAR_LENGTH, BAR_WIDTH, WHITE);
    printf("LEFT OLED READY\n");

    // Signal that game can start
    if (OLED_LEFT) {
        balls_Not_Here[1] = 1;
        balls_Not_Here[0] = 0;
        gameReadyCheck();
    }

    updateOLEDScore();

    while (1) {
        while (game_Start_Check == 0) {
            continue;
        }

        if (game_Over_Check == 1) {
            printf("GAME OVER!\n");
            while (game_Over_Check == 1) {
                continue;
            }
        }

        if (OLED_LEFT && button_press_received) {
            UpdateLeftPlatformPosition();
            button_press_received = 0;
        } else if (!OLED_LEFT) {
            UpdateRightPlatformPosition();
        }
        

        // if (ball_Not_Here == 1) {
        //     continue;
        // }
        UpdateBallPosition();
    }

    printf("Error Encountered, while broke\n");
    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
