#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"


#define BLE_UUID_OUR_BASE_UUID              {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_OUR_SERVICE                0xABCD // Just a random, but recognizable value
#define BLE_UUID_OUR_SERVICE2								0xCDEF
#define BLE_UUID_OUR_SERVICE3								0x3000
#define BLE_UUID_OUR_SERVICE4								0x4000
#define BLE_UUID_OUR_SERVICE5								0x5000
#define BLE_UUID_OUR_SERVICE6								0x6000
#define BLE_UUID_OUR_SERVICE7								0x7000
#define BLE_UUID_OUR_SERVICE8								0x8000


#define BLE_UUID_OUR_CHARACTERISTC_UUID          0x1400
#define BLE_UUID_OUR_CHARACTERISTC_UUID2 					0x1500

/**@brief Custom Service event type. */
typedef enum
{
    BLE_CUS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_CUS_EVT_NOTIFICATION_DISABLED,                             /**< Custom value notification disabled event. */
    BLE_CUS_EVT_DISCONNECTED,
    BLE_CUS_EVT_CONNECTED
} ble_cus_evt_type_t;

/**@brief Custom Service event. */
typedef struct
{
    ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
} ble_cus_evt_t;

// Forward declaration of the ble_os_t type.
typedef struct ble_os_s ble_os_t;


/**@brief Custom Service event handler type. */
typedef void (*ble_cus_evt_handler_t) (ble_os_t * p_bas, ble_cus_evt_t * p_evt);

typedef struct 
{
    ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;

/**
 * @brief This structure contains various status information for our service. 
 * It only holds one entry now, but will be populated with more items as we go.
 * The name is based on the naming convention used in Nordic's SDKs. 
 * 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
 * ‘os’ is short for Our Service). 
 */
typedef struct ble_os_s
{
		ble_cus_evt_handler_t         evt_handler;   
		uint16_t    service_handle;     /**< Handle of Our Service (as provided by the BLE stack). */	
	  uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
		ble_gatts_char_handles_t      custom_value_handles;           /**< Handles related to the Custom Value characteristic. */
		ble_gatts_char_handles_t      custom_value_handles2;           /**< Handles related to the Custom Value characteristic. */
		ble_gatts_char_handles_t      custom_value_handles3;
		uint8_t                     uuid_type;
		
}ble_os_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
//void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
void ble_cus_on_ble_evt(ble_evt_t * p_ble_evt, ble_os_t * p_cus);
//void ble_cus_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
//void our_service_init(ble_os_t * p_our_service);
uint32_t our_service_init(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init);
uint32_t our_service_init2(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init);
uint32_t our_service_init3(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init);
uint32_t our_service_init4(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init);
uint32_t our_service_init5(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init);
uint32_t our_service_init6(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init);
uint32_t our_service_init7(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init);
uint32_t our_service_init8(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init);


uint32_t ble_cus_char2_init(ble_os_t * p_cus, const ble_cus_init_t * p_cus_init);
uint32_t ble_cus_char3_init(ble_os_t * p_cus, const ble_cus_init_t * p_cus_init);


/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_bas          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code. */
//void our_characteristic_update(ble_os_t *p_our_service, uint8_t *value);

//uint32_t ble_cus_custom_value_update(ble_os_t * p_cus, uint8_t custom_value);
uint32_t ble_cus_custom_value_update(ble_os_t * p_cus, uint8_t custom_value, ble_gatts_char_handles_t * p_cus_char);

#endif  /* _ OUR_SERVICE_H__ */
