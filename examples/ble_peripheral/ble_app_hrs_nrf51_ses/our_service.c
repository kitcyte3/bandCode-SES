#include <stdint.h>
#include <string.h>
#include "our_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"
#include "fstorage.h"
#include "fds.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "softdevice_handler.h"


#define FILE_ID     0x1111
#define REC_KEY     0x2222

#define FDS_WRITE_INTERVAL               APP_TIMER_TICKS(5, 0)

APP_TIMER_DEF(m_fds_write_timer_id);

static uint8_t fds_write_wait_flag = 0;
static void fds_write_timeout_handler(void * p_context)
{
    fds_write_wait_flag = 1;
}


static uint8_t five = 5;
static uint8_t six = 6;
__ALIGN(4) static uint8_t nine = 9;
__ALIGN(4) static uint8_t eleven = 11;

static fds_record_chunk_t const m_dummy_record_chunk =
{
    .p_data = &eleven,
    .length_words = (sizeof(eleven) + 3) / sizeof(uint32_t)
};

static fds_record_chunk_t const m_dummy_record_chunk2 =
{
    .p_data = &nine,
    .length_words = (sizeof(nine) + 3) / sizeof(uint32_t)
};
static fds_record_t const m_dummy_record =
{
    .file_id = FILE_ID,
    .key = REC_KEY,
    .data.p_chunks = &m_dummy_record_chunk,
    .data.num_chunks  = 1
};

static fds_record_t const m_dummy_record2 =
{
    .file_id = FILE_ID,
    .key = REC_KEY,
    .data.p_chunks = &m_dummy_record_chunk2,
    .data.num_chunks  = 1
};



void flash_callback(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if(p_evt->result == FDS_SUCCESS)
            {               
            }
            break;
            
        case FDS_EVT_WRITE:
            if (p_evt->result == FDS_SUCCESS)
            {
            }
        case FDS_EVT_UPDATE:
        case FDS_EVT_DEL_RECORD:
        case FDS_EVT_DEL_FILE:
        case FDS_EVT_GC:
          break;
    }
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_os_t * p_cus, ble_evt_t const * p_ble_evt)
{
    p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_CONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_os_t * p_cus, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_cus_evt_t evt;

    evt.evt_type = BLE_CUS_EVT_DISCONNECTED;

    p_cus->evt_handler(p_cus, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_os_t * p_cus, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Custom Value Characteristic Written to.
    if (p_evt_write->handle == p_cus->custom_value_handles.value_handle)
    {
        //nrf_gpio_pin_toggle(LED_4);
        /*
        if(*p_evt_write->data == 0x01)
        {
            nrf_gpio_pin_clear(20); 
        }
        else if(*p_evt_write->data == 0x02)
        {
            nrf_gpio_pin_set(20); 
        }
        else
        {
          //Do nothing
        }
        */
    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_cus->custom_value_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {
        // CCCD written, call application event handler
        if (p_cus->evt_handler != NULL)
        {
            ble_cus_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_cus->evt_handler(p_cus, &evt);
        }
    }

}


void ble_cus_on_ble_evt(ble_evt_t * p_ble_evt, ble_os_t * p_cus)
//void ble_cus_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    //ble_os_t * p_cus = (ble_os_t *) p_context;
    
    //NRF_LOG_INFO("BLE event received. Event type = %d\r\n", p_ble_evt->header.evt_id); 
    if (p_cus == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_cus, p_ble_evt);
            break;
/* Handling this event is not necessary
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            NRF_LOG_INFO("EXCHANGE_MTU_REQUEST event received.\r\n");
            break;
*/
        default:
            // No implementation needed.
            break;
    }
}

static uint32_t custom_value_char_add(ble_os_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = BLE_UUID_OUR_CHARACTERISTC_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
   

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

static uint32_t custom_value_char_add2(ble_os_t * p_cus, const ble_cus_init_t * p_cus_init)
{
		/*
		fds_record_desc_t desc0 = {0};
		ret_code_t ret = fds_record_write(&desc0, &m_dummy_record);
		if (ret != FDS_SUCCESS)
		{
		// Handle error.
		}
		*/
	
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = BLE_UUID_OUR_CHARACTERISTC_UUID2;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
		
		// add 1-2 second delay for fds_write to complete
		
		
		app_timer_create(&m_fds_write_timer_id, APP_TIMER_MODE_SINGLE_SHOT, fds_write_timeout_handler);
    app_timer_start(m_fds_write_timer_id, FDS_WRITE_INTERVAL, NULL);

    while(fds_write_wait_flag != 1)
    {
      power_manage();
    }

    if(fds_write_wait_flag == 1){
        fds_write_wait_flag = 0; 
    }
		
		//nrf_delay_ms(1000);
		fds_flash_record_t  flash_record;
		fds_record_desc_t read_desc = {0};
    fds_find_token_t  tok  = {0};
		
		ret_code_t rc_read;
    rc_read = fds_record_find(FILE_ID, REC_KEY, &read_desc, &tok);
		
		if (rc_read == FDS_SUCCESS)
    {
       ret_code_t rc_open;
			 /* Open the record and read its contents. */
       rc_open = fds_record_open(&read_desc, &flash_record);
			 if (rc_open == FDS_SUCCESS)
        {
					memcpy(&attr_char_value.p_value, &flash_record.p_data, sizeof(flash_record.p_data));
        }
			
				if (rc_open != FDS_SUCCESS)
        {
					attr_char_value.p_value =  &five;
        }
			
				if (fds_record_close(&read_desc) != FDS_SUCCESS)
				{
					// Handle error.
				}
		}
		if (rc_read != FDS_SUCCESS)
    {	
			attr_char_value.p_value =  &six;
		}
			
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
   

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles2);
		ret_code_t rc_update;
    rc_update = fds_record_update(&read_desc, &m_dummy_record2);
		
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

static uint32_t custom_value_char_add3(ble_os_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = BLE_UUID_OUR_CHARACTERISTC_UUID2;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
		
		// add 1-2 second delay for fds_write to complete
		app_timer_create(&m_fds_write_timer_id, APP_TIMER_MODE_SINGLE_SHOT, fds_write_timeout_handler);
    app_timer_start(m_fds_write_timer_id, FDS_WRITE_INTERVAL, NULL);

    while(fds_write_wait_flag != 1)
    {
      power_manage();
    }

    if(fds_write_wait_flag == 1){
        fds_write_wait_flag = 0; 
    }
		
		fds_flash_record_t  flash_record;
		fds_record_desc_t read_desc = {0};
    fds_find_token_t  tok  = {0};
		
		ret_code_t rc_read;
    rc_read = fds_record_find(FILE_ID, REC_KEY, &read_desc, &tok);
		
		if (rc_read == FDS_SUCCESS)
    {
       ret_code_t rc_open;
			 /* Open the record and read its contents. */
       rc_open = fds_record_open(&read_desc, &flash_record);
			 if (rc_open == FDS_SUCCESS)
        {
					memcpy(&attr_char_value.p_value, &flash_record.p_data, sizeof(flash_record.p_data));
        }
			
				if (rc_open != FDS_SUCCESS)
        {
					attr_char_value.p_value =  &five;
        }
			
				if (fds_record_close(&read_desc) != FDS_SUCCESS)
				{
					// Handle error.
				}
		}
		if (rc_read != FDS_SUCCESS)
    {	
			attr_char_value.p_value =  &six;
		}
			
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
   

    err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_cus->custom_value_handles3);
		
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}



/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
//void our_service_init(ble_os_t * p_our_service)
uint32_t  our_service_init(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init)
{
	
		if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }
		
		ret_code_t ret = fds_register(flash_callback);
		if (ret != FDS_SUCCESS)
		{
			// Registering of the FDS event handler has failed.
		}

    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
   
		  // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
		//err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
		
		/*
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE;
		*/
    APP_ERROR_CHECK(err_code);    
		
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE;
    
    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);
/*
    SEGGER_RTT_WriteString(0, "Executing our_service_init().\n");
    SEGGER_RTT_printf(0, "Service UUID: 0x%#04x\n", service_uuid.uuid);
    SEGGER_RTT_printf(0, "Service UUID type: 0x%#02x\n", service_uuid.type);
    SEGGER_RTT_printf(0, "Service handle: 0x%#04x\n", p_our_service->service_handle
	*/
	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    err_code = custom_value_char_add(p_cus, p_cus_init);
		APP_ERROR_CHECK(err_code);
		
		return NRF_SUCCESS;
}

uint32_t ble_cus_char2_init(ble_os_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Add Custom Value characteristic
    return custom_value_char_add2(p_cus, p_cus_init);
}

uint32_t ble_cus_char3_init(ble_os_t * p_cus, const ble_cus_init_t * p_cus_init)
{
    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Add Custom Value characteristic
    return custom_value_char_add3(p_cus, p_cus_init);
}

uint32_t  our_service_init2(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init)
{
	
		if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
		  // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
		//err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);

    APP_ERROR_CHECK(err_code);    
		
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE2;
    
    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);
	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    err_code = custom_value_char_add(p_cus, p_cus_init);
		APP_ERROR_CHECK(err_code);
		
		return NRF_SUCCESS;
}

uint32_t  our_service_init3(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init)
{
	
		if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
		  // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);

    APP_ERROR_CHECK(err_code);    
		
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE3;
    
    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);
	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    err_code = custom_value_char_add(p_cus, p_cus_init);
		APP_ERROR_CHECK(err_code);
		
		return NRF_SUCCESS;
}

uint32_t  our_service_init4(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init)
{
	
		if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
		  // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);

    APP_ERROR_CHECK(err_code);    
		
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE4;
    
    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);
	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    err_code = custom_value_char_add(p_cus, p_cus_init);
		APP_ERROR_CHECK(err_code);
		
		return NRF_SUCCESS;
}

uint32_t  our_service_init5(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init)
{
	
		if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
		  // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);

    APP_ERROR_CHECK(err_code);    
		
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE5;
    
    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);
	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    err_code = custom_value_char_add(p_cus, p_cus_init);
		APP_ERROR_CHECK(err_code);
		
		return NRF_SUCCESS;
}

uint32_t  our_service_init6(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init)
{
	
		if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
		  // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);

    APP_ERROR_CHECK(err_code);    
		
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE6;
    
    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);
	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    err_code = custom_value_char_add(p_cus, p_cus_init);
		APP_ERROR_CHECK(err_code);
		
		return NRF_SUCCESS;
}

uint32_t  our_service_init7(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init)
{
	
		if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
		  // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);

    APP_ERROR_CHECK(err_code);    
		
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE7;
    
    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);
	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    err_code = custom_value_char_add(p_cus, p_cus_init);
		APP_ERROR_CHECK(err_code);
		
		return NRF_SUCCESS;
}

uint32_t  our_service_init8(ble_os_t * p_cus,  const ble_cus_init_t * p_cus_init)
{
	
		if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions
    
		  // Initialize service structure
    p_cus->evt_handler               = p_cus_init->evt_handler;
    p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // OUR_JOB: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    
		err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);

    APP_ERROR_CHECK(err_code);    
		
		service_uuid.type = p_cus->uuid_type;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE8;
    
    // OUR_JOB: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cus->service_handle);
    APP_ERROR_CHECK(err_code);
	
	// OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    err_code = custom_value_char_add(p_cus, p_cus_init);
		APP_ERROR_CHECK(err_code);
		
		return NRF_SUCCESS;
}

//void our_characteristic_update(ble_os_t *p_cus, uint8_t *value)

//uint32_t ble_cus_custom_value_update(ble_os_t * p_cus, uint8_t custom_value)
uint32_t ble_cus_custom_value_update(ble_os_t * p_cus, uint8_t custom_value, ble_gatts_char_handles_t * p_cus_char)
{
    // OUR_JOB: Step 3.E, Update characteristic value
	if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &custom_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      //p_cus->custom_value_handles.value_handle,
																			p_cus_char->value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        //hvx_params.handle = p_cus->custom_value_handles.value_handle;
				hvx_params.handle = p_cus_char->value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
        //NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        //NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }


    return err_code;
}
