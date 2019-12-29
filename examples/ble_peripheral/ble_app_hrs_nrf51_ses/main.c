/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bandSensors.h"


#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "fstorage.h"
#include "fds.h"
#include "peer_manager.h"
#include "nrf_delay.h"

#include "bsp.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "nrf_gpio.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"

//SENSOR INCLUDES
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_gpiote.h"
//
#include <stdio.h>
//#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

//END SENSOR INCLUDES



//SENSOR STUFF
 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127
 #define PIN_IN 5
/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0); //TWI instance 0
static const nrf_drv_twi_t m_twi1 = NRF_DRV_TWI_INSTANCE(1); //TWI instance 1
uint8_t readi2cOneByte1(uint8_t deviceAddr, uint8_t read_reg_addr);

// UART code copied over
//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
//END SENSOR STUFF



#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "our_service.h"
//#include "SEGGER_RTT.h"
//#include "msgs.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Nordic_Template"                           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

//#define APP_TIMER_KEEPS_RTC_ACTIVE      1 //sdk_config.h line 1705

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define TIMER_INTERVAL                  APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define FDS_INIT_INTERVAL               APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)

static ble_os_t m_cus;
static ble_os_t m_cus2;
static ble_os_t m_cus3;
static ble_os_t m_cus4;
static ble_os_t m_cus5;
static ble_os_t m_cus6;
static ble_os_t m_cus7;
static ble_os_t m_cus8;


APP_TIMER_DEF(m_fds_init_timer_id);
APP_TIMER_DEF(m_app_timer_id);

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static uint8_t fds_init_wait_flag = 0;

static uint8_t m_custom_value = 0;
static uint8_t notif_bool = 0;

//ble_os_t m_our_service;

/* YOUR_JOB: Declare all services structure your application is using
   static ble_xx_service_t                     m_xxs;
   static ble_yy_service_t                     m_yys;
 */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

static void advertising_start(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_DEBUG("Connected to previously bonded device\r\n");
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_BONDED_PEER_CONNECTED

        case PM_EVT_CONN_SEC_START:
            break; // PM_EVT_CONN_SEC_START

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_DEBUG("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                                 ble_conn_state_role(p_evt->conn_handle),
                                 p_evt->conn_handle,
                                 p_evt->params.conn_sec_succeeded.procedure);
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_CONN_SEC_SUCCEEDED

        case PM_EVT_CONN_SEC_FAILED:
        {
            /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
             *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
             *  be restarted until the link is disconnected and reconnected. Sometimes it is
             *  impossible, to secure the link, or the peer device does not support it. How to
             *  handle this error is highly application dependent. */
            switch (p_evt->params.conn_sec_failed.error)
            {
                case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
                    // Rebond if one party has lost its keys.
                    err_code = pm_conn_secure(p_evt->conn_handle, true);
                    if (err_code != NRF_ERROR_INVALID_STATE)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                    break; // PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING

                default:
                    break;
            }
        } break; // PM_EVT_CONN_SEC_FAILED

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break; // PM_EVT_CONN_SEC_CONFIG_REQ

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break; // PM_EVT_STORAGE_FULL

        case PM_EVT_ERROR_UNEXPECTED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
            break; // PM_EVT_ERROR_UNEXPECTED

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            break; // PM_EVT_PEER_DATA_UPDATE_SUCCEEDED

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            // Assert.
            APP_ERROR_CHECK_BOOL(false);
            break; // PM_EVT_PEER_DATA_UPDATE_FAILED

        case PM_EVT_PEER_DELETE_SUCCEEDED:
            break; // PM_EVT_PEER_DELETE_SUCCEEDED

        case PM_EVT_PEER_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
            break; // PM_EVT_PEER_DELETE_FAILED

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start();
            break; // PM_EVT_PEERS_DELETE_SUCCEEDED

        case PM_EVT_PEERS_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
            break; // PM_EVT_PEERS_DELETE_FAILED

        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break; // PM_EVT_LOCAL_DB_CACHE_APPLIED

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
            break; // PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED

        case PM_EVT_SERVICE_CHANGED_IND_SENT:
            break; // PM_EVT_SERVICE_CHANGED_IND_SENT

        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
            break; // PM_EVT_SERVICE_CHANGED_IND_SENT

        default:
            // No implementation needed.
            break;
    }
}

static void fds_init_timeout_handler(void * p_context)
{
    fds_init_wait_flag = 1;
}

static void timer_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    // Increment the value of m_custom_value before nortifing it.
    m_custom_value++;
  
     /*
    if(m_custom_value % 2 == 1){
        //sensor_code();
    }
    */
    
    if(notif_bool == 1){
       //err_code = ble_cus_custom_value_update(&m_cus, m_custom_value);
      err_code = ble_cus_custom_value_update(&m_cus, m_custom_value, &m_cus.custom_value_handles);
    }
    
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       uint32_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
       uint32_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); 
  
       app_timer_create(&m_fds_init_timer_id, APP_TIMER_MODE_SINGLE_SHOT, fds_init_timeout_handler);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
   static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
   {
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
   }*/


static void on_cus_evt(ble_os_t     * p_cus_service,
                       ble_cus_evt_t * p_evt)
{
    ret_code_t err_code;
    
    switch(p_evt->evt_type)
    {
        case BLE_CUS_EVT_NOTIFICATION_ENABLED:
            
             //err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
             //APP_ERROR_CHECK(err_code);
             notif_bool = 1;
             break;

        case BLE_CUS_EVT_NOTIFICATION_DISABLED:

            //err_code = app_timer_stop(m_notification_timer_id);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_CUS_EVT_CONNECTED:
            break;

        case BLE_CUS_EVT_DISCONNECTED:
              break;

        default:
              // No implementation needed.
              break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    /* YOUR_JOB: Add code to initialize the services used by the application.
       uint32_t                           err_code;
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
      ret_code_t          err_code;
      ble_cus_init_t      cus_init = {0};
      
      // Initialize CUS Service init structure to zero.
      cus_init.evt_handler                = on_cus_evt;
      
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.cccd_write_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm);
     
      //our_service_init (&m_our_service, & cus_init);
      our_service_init(&m_cus, &cus_init);
      
      ble_cus_char2_init(&m_cus, &cus_init); //Second custom characteristic init
      
      ble_cus_char3_init(&m_cus, &cus_init); //Third custom characteristic init
      
      //Second custom service init
      
      ble_cus_init_t      cus_init2 = {0};
      
      // Initialize CUS Service init structure to zero.
      cus_init2.evt_handler                = on_cus_evt;
      
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init2.custom_value_char_attr_md.cccd_write_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init2.custom_value_char_attr_md.read_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init2.custom_value_char_attr_md.write_perm);
     
      //our_service_init (&m_our_service, & cus_init);
      our_service_init2(&m_cus2, &cus_init2);
  
  //Third custom service init
      
      ble_cus_init_t      cus_init3 = {0};
      
      // Initialize CUS Service init structure to zero.
      cus_init3.evt_handler                = on_cus_evt;
      
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init3.custom_value_char_attr_md.cccd_write_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init3.custom_value_char_attr_md.read_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init3.custom_value_char_attr_md.write_perm);
     
      //our_service_init (&m_our_service, & cus_init);
      our_service_init3(&m_cus3, &cus_init3);
  
  //Fourth custom service init
      
      ble_cus_init_t      cus_init4 = {0};
      
      // Initialize CUS Service init structure to zero.
      cus_init4.evt_handler                = on_cus_evt;
      
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init4.custom_value_char_attr_md.cccd_write_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init4.custom_value_char_attr_md.read_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init4.custom_value_char_attr_md.write_perm);
     
      //our_service_init (&m_our_service, & cus_init);
      our_service_init4(&m_cus4, &cus_init4);
  
  //Fifth custom service init
      
      ble_cus_init_t      cus_init5 = {0};
      
      // Initialize CUS Service init structure to zero.
      cus_init5.evt_handler                = on_cus_evt;
      
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init5.custom_value_char_attr_md.cccd_write_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init5.custom_value_char_attr_md.read_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init5.custom_value_char_attr_md.write_perm);
     
      //our_service_init (&m_our_service, & cus_init);
      our_service_init5(&m_cus5, &cus_init5);
  
  //Sixth custom service init
      
      ble_cus_init_t      cus_init6 = {0};
      
      // Initialize CUS Service init structure to zero.
      cus_init6.evt_handler                = on_cus_evt;
      
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init6.custom_value_char_attr_md.cccd_write_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init6.custom_value_char_attr_md.read_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init6.custom_value_char_attr_md.write_perm);
     
      //our_service_init (&m_our_service, & cus_init);
      our_service_init6(&m_cus6, &cus_init6);
  
  //Seventh custom service init
      
      ble_cus_init_t      cus_init7 = {0};
      
      // Initialize CUS Service init structure to zero.
      cus_init7.evt_handler                = on_cus_evt;
      
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init7.custom_value_char_attr_md.cccd_write_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init7.custom_value_char_attr_md.read_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init7.custom_value_char_attr_md.write_perm);
     
      //our_service_init (&m_our_service, & cus_init);
      our_service_init7(&m_cus7, &cus_init7);
  
  //Eighth custom service init
      
      ble_cus_init_t      cus_init8 = {0};
      
      // Initialize CUS Service init structure to zero.
      cus_init8.evt_handler                = on_cus_evt;
      
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init8.custom_value_char_attr_md.cccd_write_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init8.custom_value_char_attr_md.read_perm);
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init8.custom_value_char_attr_md.write_perm);
     
      //our_service_init (&m_our_service, & cus_init);
      our_service_init8(&m_cus8, &cus_init8);
    

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       uint32_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code);
        
       app_timer_start(m_fds_init_timer_id, FDS_INIT_INTERVAL, NULL);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
  
    ble_cus_on_ble_evt(p_ble_evt, &m_cus);
    ble_cus_on_ble_evt(p_ble_evt, &m_cus2);

  
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    /*YOUR_JOB add calls to _on_ble_evt functions from each service your application is using
       ble_xxs_on_ble_evt(&m_xxs, p_ble_evt);
       ble_yys_on_ble_evt(&m_yys, p_ble_evt);
     */
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


static ble_uuid_t m_adv_uuids[] = 
{
    {
        BLE_UUID_OUR_SERVICE,
        BLE_UUID_TYPE_VENDOR_BEGIN
    },
    {
      BLE_UUID_OUR_SERVICE2,
      BLE_UUID_TYPE_VENDOR_BEGIN
    },
  {
      BLE_UUID_OUR_SERVICE3,
      BLE_UUID_TYPE_VENDOR_BEGIN
    },
  {
      BLE_UUID_OUR_SERVICE4,
      BLE_UUID_TYPE_VENDOR_BEGIN
    },
  {
      BLE_UUID_OUR_SERVICE5,
      BLE_UUID_TYPE_VENDOR_BEGIN
    },
  {
      BLE_UUID_OUR_SERVICE6,
      BLE_UUID_TYPE_VENDOR_BEGIN
    },
  {
      BLE_UUID_OUR_SERVICE7,
      BLE_UUID_TYPE_VENDOR_BEGIN
    },
  {
      BLE_UUID_OUR_SERVICE8,
      BLE_UUID_TYPE_VENDOR_BEGIN
    }
};

/**@brief Function for initializing the Advertising functionality.
 */
/*
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
} */

static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = 1;//sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);

    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
}
//===================================SENSOR STUFF=====================================
void uart_error_handle(app_uart_evt_t * p_event){
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void MUX_init(){
    nrf_gpio_cfg_output(30); //S0
    nrf_gpio_cfg_output(0); //S1
    nrf_gpio_cfg_output(1); //S2
    nrf_gpio_cfg_output(2); //S3
}
void vibro_init(){
    nrf_gpio_cfg_output(14);
}
void button_init_simple(){
    nrf_gpio_cfg_input(3,NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(5,NRF_GPIO_PIN_PULLUP);
}
void twi_init (void){
        //make sure TWO0 and TWI1 are both enabled in config text file somewhere else in project
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 16, //16
       .sda                = 15, //15
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void twi1_init (void){
        //make sure TWO0 and TWI1 are both enabled in config text file somewhere else in project
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi1_config = {
       .scl                = 18, //18
       .sda                = 17, //17
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi1, &twi1_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi1);
}

//write to twi0
void writei2c(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write_high, uint8_t data_to_write_low){
    //power on spectrum sensor
    uint8_t dataToWrite[2] = {data_to_write_high, data_to_write_low};
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}
void writei2cSingle(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write_high){
    //power on spectrum sensor
    uint8_t dataToWrite[2] = {reg_addr, data_to_write_high};
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}

//write to twi1
void writei2c1(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write_high, uint8_t data_to_write_low){
    //power on spectrum sensor
    uint8_t dataToWrite[2] = {data_to_write_high, data_to_write_low};
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi1, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}
void writei2cOneByte(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write){
    //power on spectrum sensor
    uint8_t dataToWrite[1] = {data_to_write};
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}

//use twi1
void writei2cOneByte1(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write){
    //power on spectrum sensor
    uint8_t dataToWrite[1] = {data_to_write};
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi1, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}

uint16_t readi2cHighLow(uint8_t deviceAddr, uint8_t read_reg_addr){
    //ret_code_t err_code; //to hold return code, currently do nothing with it
    
    //READ 1st byte
    uint16_t data_both;
    //first half of read
    uint8_t dataToSend4[1] = {read_reg_addr};
    nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToSend4[0], sizeof(dataToSend4), true);


    uint8_t read_data2[2];
    nrf_drv_twi_rx(&m_twi, deviceAddr, &read_data2[0], 2);
    //printf("addr: %x, data: 0x%x 0x%x \r\n",read_reg_addr ,read_data2[1], read_data2[0]);
    
    data_both = read_data2[0] | (read_data2[1] << 8);
    //printf("both: 0x%x \r\n",data_both);
    
    return data_both;
}

uint8_t readi2cOneByte(uint8_t deviceAddr, uint8_t read_reg_addr){
    //ret_code_t err_code; //to hold return code, currently do nothing with it
    
    //READ 1st byte
    //first half of read
    uint8_t dataToSend4[1] = {read_reg_addr};
    nrf_drv_twi_tx(&m_twi, (deviceAddr), &dataToSend4[0], sizeof(dataToSend4), true);
    
    uint8_t read_data2[1];
    nrf_drv_twi_rx(&m_twi, (deviceAddr), &read_data2[0], 1);
    return read_data2[0];
}


//use twi1
uint8_t readi2cOneByte1(uint8_t deviceAddr, uint8_t read_reg_addr){
    //ret_code_t err_code; //to hold return code, currently do nothing with it
    
    //READ 1st byte
    //first half of read
    uint8_t dataToSend4[1] = {read_reg_addr};
    nrf_drv_twi_tx(&m_twi1, (deviceAddr<<1)|0x0, &dataToSend4[0], sizeof(dataToSend4), true);
    
    uint8_t read_data2[1];
    nrf_drv_twi_rx(&m_twi1, (deviceAddr<<1)|0x1, &read_data2[0], 1);
    
    return read_data2[0];
}

bool GPIOEXP0_init(){
    //make sure twi_init() was run before this function.
    //AND make sure MUX is set to 1111
                //adr,R/!W      config hi low
    writei2c((0x22<<1), 0x0c, 0x0,0x0); //config port0 to output
    writei2c((0x22<<1), 0x0d, 0x0,0x0); //config port1 to output
    writei2c((0x22<<1), 0x0e, 0x0,0x0); //config port2 to output
    
    return 0;
}

bool GPIOEXP1_init(){
    //make sure twi_init() was run before this function.
                //adr,R/!W      config hi low
    writei2c1((0x23<<1), 0x0c, 0x0,0x0); //config port0 to output
    writei2c1((0x23<<1), 0x0d, 0x0,0x0); //config port1 to output
    writei2c1((0x23<<1), 0x0e, 0x0,0x0); //config port2 to output
    
    return 0;
}

uint8_t GPIOEXP1_readPort0(){
    return readi2cOneByte((0x23<<1)|0x1, 0x4);
}

uint8_t GPIOEXP0_readPort0(){
//  return readi2cOneByte((0x22<<1)|0x1, 0x4);
    return readi2cOneByte((0x22<<1)|0x1, 0x4);
}

bool LED_BT_blue(){
    //make sure GPIOEXP1_init was run before this function.
    uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
    res = (res & ~0x38) | (0x30 & 0x38); //0banything & 0b000[fix]111
    writei2cSingle(0x23, 0x04, res); //set Blue light closer to USB on
    return 0;
}
bool LED_BT_green(){
    //make sure GPIOEXP1_init was run before this function.
    uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
    res = (res & ~0x38) | (0x28 & 0x38); //0banything & 0b000[fix]111
    writei2cSingle(0x23, 0x04, res); //set Blue light closer to USB on
    return 0;
}
bool LED_BT_off(){
    //make sure GPIOEXP1_init was run before this function.
    uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
    res = (res & ~0x38) | (0x38 & 0x38); //0banything & 0b000[fix]111
    writei2cSingle(0x23, 0x04, res); //set Blue light closer to USB on
    return 0;
}
bool LED_PWR_red(){
    //make sure GPIOEXP1_init was run before this function.
    uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
    res = (res & ~0x07) | (0x3 & 0x07); //0banything & 0b000000[011]
    writei2cSingle(0x23, 0x04, res); //set red light away from USB ON
    return 0;
}
bool LED_PWR_green(){
    //make sure GPIOEXP1_init was run before this function.
    //see explanation in LED_BT_blue and LED_PWR_red
    uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
    res = (res & ~0x07) | (0x05 & 0x07);; //0banything & 0b000000[011]
    writei2cSingle(0x23, 0x04, res); //set green light away from USB on
    return 0;
}
bool LED_PWR_bluegreen(){
    //make sure GPIOEXP1_init was run before this function.
    //see explanation in LED_BT_blue and LED_PWR_red
    uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
    res = (res & ~0x07) | (0x04 & 0x07);; //0banything & 0b000000[011]
    writei2cSingle(0x23, 0x04, res); //set green light away from USB on
    return 0;
}
bool LED_PWR_yellow(){
    //make sure GPIOEXP1_init was run before this function.
    //see explanation in LED_BT_blue and LED_PWR_red
    uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
    res = (res & ~0x07) | (0x01 & 0x07);; //0banything & 0b000000[011]
    writei2cSingle(0x23, 0x04, res); //set green light away from USB on
    return 0;
}
bool LED_PWR_off(){
    //make sure GPIOEXP1_init was run before this function.
    //see explanation in LED_BT_blue and LED_PWR_red
    uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
    res = (res & ~0x07) | (0x07 & 0x07);; //0banything & 0b000000[011]
    writei2cSingle(0x23, 0x04, res); //set green light away from USB on
    return 0;
}

void LED_indicator_init(){
        //configuration registers set all outputs to outputs instead of default input
        writei2cSingle(0x23, 0x0C, 0x0); 
        writei2cSingle(0x23, 0x0D, 0x0); 
        writei2cSingle(0x23, 0x0E, 0x0); 
}
void LED_vertBoard_init(){
        //configuration registers set all outputs to outputs instead of default input
        writei2cSingle(0x22, 0x0C, 0x0); 
        writei2cSingle(0x22, 0x0D, 0x0); 
        writei2cSingle(0x22, 0x0E, 0x0); 
}

bool LED_WHITE_on(){
    //make sure GPIOEXP1_init was run before this function.
    //read current GPIOEXP1 settings:
    
    //LEDS WORK, BUT THIS DRIVER FUNCTION HASNT BEEN IMPLEMENTED CORRECTLY YET  
    writei2cSingle(0x22, 0x04, 0x0);
    writei2cSingle(0x22, 0x05, 0x0); 
    writei2cSingle(0x22, 0x06, 0x0); 
    
    return 0;
}

void MUX_set(bool s3, bool s2, bool s1, bool s0){
    s3?nrf_gpio_pin_set(2):nrf_gpio_pin_clear(2);
    s2?nrf_gpio_pin_set(1):nrf_gpio_pin_clear(1);
    s1?nrf_gpio_pin_set(0):nrf_gpio_pin_clear(0);
    s0?nrf_gpio_pin_set(30):nrf_gpio_pin_clear(30);
}

void vibro_start(){
    nrf_gpio_pin_set(14);
}
void vibro_stop(){
    nrf_gpio_pin_clear(14);
}

bool RGBW_on(){
        //turns RGBW sensor on, returns 0 if success, 1 if falied
                    //addr,  reg,  hi, low
        writei2c(0x10,0x00,0x00,0x00); //enable RGBW sensors with 80ms integration time

        //read back settings
        uint16_t i2cResult = readi2cHighLow(0x10, 0x00);
    
        return i2cResult != 0x00;
}
bool RGBW_off(){
        //turns RGBW sensor on, returns 0 if success, 1 if falied
                    //addr,  reg,  hi, low
        writei2c(0x10,0x00,0x00,0x01); //enable RGBW sensors with 80ms integration time

        //read back settings
        uint16_t i2cResult = readi2cHighLow(0x10, 0x00);
    
        return i2cResult != 0x01;
}
double* RGBW_get(){
    //returns an array of doubles, max value is 255 for each color, skips matrix multiplications
    bool printResult = true;
    static double RGBreturn[3] = {0,0,0};
    double r,g,b, w;
    
    //read colors
    r = readi2cHighLow(0x10, 0x08);// *100 / 96.0;  
    g = readi2cHighLow(0x10, 0x09);// *100 / 74.0;
    b = readi2cHighLow(0x10, 0x0A);// *100 / 56.0;
    w = readi2cHighLow(0x10, 0x0B);
    
    //do some math
    double normalizedR = r;
    double normalizedG = g;
    double normalizedB = b;
    normalizedR = normalizedR/255;
    normalizedG = normalizedG/255;
    normalizedB = normalizedB/255;
    int hexR, hexB, hexG;
    hexR = normalizedR;
    hexB = normalizedB;
    hexG = normalizedG;
    
    if(printResult){
        printf("  raw red: 0x%x\r\n",r );
        printf("raw green: 0x%x\r\n",g );
        printf(" raw blue:  0x%x\r\n",b );
        printf("raw white:  0x%x\r\n",w );
        printf("RGB: %x %x %x\r\n\r\n",hexR,hexG,hexB);
    }
    RGBreturn[0]= hexR;
    RGBreturn[1]= hexB;
    RGBreturn[2]= hexG;
    
    return RGBreturn;
    
}
 
void band_uart_init(){
uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          //RX_PIN_NUMBER,
          //TX_PIN_NUMBER,
                    9,
                    10,
                    12,
                    11,
          //RTS_PIN_NUMBER,
          //CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
    printf("\r\nUART works\r\n");
}

void scan_twi(){ //works 12/27/2019
    //check for TWI dvices
    uint8_t sample_data;
    uint8_t address;
    uint32_t err_code;
    for (address = 1; address <= 127; address++){
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            printf("TWI device detected at address 0x%x.\r\n", address);
        }
    }
}

void scan_twi1(){

    
    //check for TWI dvices
    uint8_t sample_data;
    uint8_t address;
    uint32_t err_code;
    for (address = 1; address <= 127; address++){
        err_code = nrf_drv_twi_rx(&m_twi1, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            printf("TWI1 device detected at address 0x%x.\r\n", address);
        }
    }
}

void LED_demo(){
    LED_indicator_init();
    LED_BT_blue();
    LED_PWR_yellow();
        
    int delayTime = 300;
    
    hi:
    
    for(int i = 0; i< 4; i++){
        LED_BT_blue();
        LED_PWR_red();
        nrf_delay_ms(delayTime);
        LED_BT_off();
        LED_PWR_off();
        nrf_delay_ms(delayTime);
    }
    for(int i = 0; i< 4; i++){
        LED_BT_blue();
        LED_PWR_yellow();
        nrf_delay_ms(delayTime);
        LED_BT_off();
        LED_PWR_off();
        nrf_delay_ms(delayTime);
    }
    for(int i = 0; i< 4; i++){
        LED_BT_blue();
        LED_PWR_green();
        nrf_delay_ms(delayTime);
        LED_BT_off();
        LED_PWR_off();
        nrf_delay_ms(delayTime);
    }
    LED_PWR_green();
    LED_BT_green();
    nrf_delay_ms(4*delayTime);
    LED_PWR_off();
    LED_BT_off();
    
    while(true){
        if(nrf_gpio_pin_read(3)==0){
            goto hi;
        }
        if(nrf_gpio_pin_read(5) == 0){
            goto hi;
        }
    }
}

void VIBRO_test(){
    //vibro tests
    vibro_init();
    while(true){
        //vibration motor test
        vibro_start();
        nrf_delay_ms(1000);
        vibro_stop();
        nrf_delay_ms(1000);
    }
    //vibro results V2: The voltage at the motor is 0.2V too low for the motor to run
    //to fix, choose another MOSFET or connect Vibro to VUSB
}

void bt_btn_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
  //place code here to run when the bluetooth button is pressed
    LED_BT_blue();
    nrf_delay_ms(200);
    LED_BT_off();
    printf("bt button pressed!\r\n");
}
void pwr_btn_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
    //place code here to run when the power button is pressed
    LED_PWR_red();
    nrf_delay_ms(200);
    LED_PWR_off();
    printf("pwr button pressed!\r\n");
}
void button_init_interrupt(){
    int pwr_pin = 5;
    int bt_pin = 3;
    ret_code_t err_code;

        //set up GPIOTE drivers
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    //set the pin we want to be a sense pin
        nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

        //attach GPIO tasker to power button
    err_code = nrf_drv_gpiote_in_init(pwr_pin, &in_config, pwr_btn_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(pwr_pin, true);
    
        //attach GPIO event to bluetooth button
        err_code = nrf_drv_gpiote_in_init(bt_pin, &in_config, bt_btn_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(bt_pin, true);
}

//===============================END SENSOR STUFF=====================================

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool     erase_bonds;

    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    
  
    timers_init();
    application_timers_start();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init(erase_bonds);
    if (erase_bonds == true)
    {
        NRF_LOG_INFO("Bonds erased!\r\n");
    }
    /*
    while(fds_init_wait_flag != 1)
    {
      power_manage();
    }

    if(fds_init_wait_flag == 1){
        fds_init_wait_flag = 0; 
    }*/
    
    //sensor code
        //These are direct NRF GPIO
    MUX_init();                     //setup nrf GPIO to output
    MUX_set(1,1,1,1);       //1111 for SDA -> LEDs  (s3 s2 s1 s0)
    band_uart_init();       // setup UART
        
    //then do twi init, since it tends to get stuck here
    twi_init();
    //twi1_init();
    
    nrf_delay_ms(1000);
    scan_twi(); // do a twi scan
    //scan_twi1();
    
    //buttons
    //button_init_simple();
    button_init_interrupt(); //turns on BT light when button pressed
    
    
    //indicator LED stuff on 0x23 expander
    LED_indicator_init();
    //LED_demo();
    LED_PWR_yellow();
    
    

    
    //indicator LED stuff on 0x22 expander
    LED_vertBoard_init();
    //LED_WHITE_on();
    
    LED_demo();
    
    //RGBW setup
    MUX_set(0,0,1,1);
    RGBW_on();

    MUX_set(1,1,1,1);   
    //end sensor code   
    while(m_custom_value <= 1)
    {
      power_manage();
    } 
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("Template started\r\n");
    //application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
