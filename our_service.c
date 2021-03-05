
#include "our_service.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include <stdint.h>
#include <string.h>

// BLE_WRITE:
/**@brief Function for handling the Write event.
 *
 * @param[in] p_our_service     Our Service structure.
 * @param[in] p_ble_evt         Event received from the BLE stack.
 */
void on_write(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt)
{
    NRF_LOG_INFO("on_write: called");
    
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_our_service->char_handles_1.value_handle)
    {
        NRF_LOG_INFO("characteristic1_value: Write Happened!");
        
       
        // Make sure that the data is 4 bytes
        // It has to match the exact byte size of the characteristic to avoid problems
        int8_t len = p_evt_write->len;
        if (len != 4)
        {
            NRF_LOG_INFO("ERROR: incomplete package");
            NRF_LOG_INFO("len: %d", len);
            return;
        }
        
        // Data must be sent from in Little Endian Format and 4 bytes
        // Convert the little endian 4 bytes of data into 32 bit unsigned int
        uint32_t *characteristic1_value_adr;
        uint32_t characteristic1_value_val;
        characteristic1_value_adr = (uint32_t*) p_evt_write->data;
        characteristic1_value_val = *characteristic1_value_adr;

        NRF_LOG_INFO("characteristic1_value: %d", characteristic1_value_val);

        // Call the write handler function. Implementation is in the main.
        p_our_service->characteristic1_value_write_handler(characteristic1_value_val);
       
    }

}

// Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_our_service_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
  // Implement switch case handling BLE events related to our service.
  ble_os_t *p_our_service = (ble_os_t *)p_context;
  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    break;
  case BLE_GAP_EVT_DISCONNECTED:
    p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
    break;
  // BLE_WRITE:
  // Write: Data was received to the module
  case BLE_GATTS_EVT_WRITE:
    on_write(p_our_service, p_ble_evt);
    break;  
  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated previously
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t our_char_add(ble_os_t *p_our_service) {
  // Add a custom characteristic UUID
  uint32_t err_code;
  ble_uuid_t char_uuid;
  ble_uuid128_t base_uuid = BLE_UUID_OUR_BASE_UUID;
  char_uuid.uuid = BLE_UUID_OUR_CHARACTERISTIC_UUID;
  err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
  APP_ERROR_CHECK(err_code);

  // Add read/write properties to our characteristic
  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 1;

  // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
  ble_gatts_attr_md_t cccd_md;
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  char_md.char_props.notify = 1;

  // Configure the attribute metadata
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;

  // Set read/write security levels to our characteristic
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  // Configure the characteristic value attribute
  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;

  // Set characteristic length in number of bytes
  attr_char_value.max_len = 4;
  attr_char_value.init_len = 4;
  uint8_t value[4] = {0x12, 0x34, 0x56, 0x78};
  attr_char_value.p_value = value;

  // Add our new characteristic to the service
  err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
        &char_md,
        &attr_char_value,
        &p_our_service->char_handles);
  APP_ERROR_CHECK(err_code);
  return NRF_SUCCESS;
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated previously
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t our_char_add_1(ble_os_t *p_our_service) {
  // Add a custom characteristic UUID
  uint32_t err_code;
  ble_uuid_t char_uuid;
  ble_uuid128_t base_uuid = BLE_UUID_OUR_BASE_UUID;
  char_uuid.uuid = BLE_UUID_OUR_CHARACTERISTIC_UUID_1;
  err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
  APP_ERROR_CHECK(err_code);

  // Add read/write properties to our characteristic
  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 1;

  // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
  ble_gatts_attr_md_t cccd_md;
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  char_md.char_props.notify = 1;

  // Configure the attribute metadata
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;

  // Set read/write security levels to our characteristic
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  // Configure the characteristic value attribute
  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;

  // Set characteristic length in number of bytes
  attr_char_value.max_len = 4;
  attr_char_value.init_len = 4;
  uint8_t value[4] = {0x12, 0x34, 0x56, 0x78};
  attr_char_value.p_value = value;

  // Add our new characteristic to the service
  err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
        &char_md,
        &attr_char_value,
        &p_our_service->char_handles_1);
  APP_ERROR_CHECK(err_code);
  return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void our_service_init(ble_os_t *p_our_service, ble_os_init_t * init) {
  uint32_t err_code; // Variable to hold return codes from library and softdevice functions

  // Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
  ble_uuid_t service_uuid;
  ble_uuid128_t base_uuid = BLE_UUID_OUR_BASE_UUID;
  service_uuid.uuid = BLE_UUID_OUR_SERVICE_UUID;
  err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
  APP_ERROR_CHECK(err_code);

  // Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
  p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;

  // BLE_WRITE: transfer the pointers from the init instance to the module instance
  p_our_service->characteristic1_value_write_handler = init->characteristic1_value_write_handler;

  // Add our service
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
        &service_uuid,
        &p_our_service->service_handle);

  APP_ERROR_CHECK(err_code);

  // Call the function our_char_add() to add our new characteristic to the service.
  our_char_add(p_our_service);
  our_char_add_1(p_our_service);
}

// Function to be called when updating characteristic value
void our_potentiostat_characteristic_update(ble_os_t *p_our_service, int32_t *potential_value) {
  // Update characteristic value
  if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint16_t len = 4;
    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_our_service->char_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = (uint8_t *)potential_value;

    sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
  }
}

// Function to be called when updating characteristic value
void our_potentiostat_settings_characteristic_update(ble_os_t *p_our_service, uint8_t *register_val) {
  // Update characteristic value
  if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint16_t len = 4;
    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_our_service->char_handles_1.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = (uint8_t *)register_val;

    sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
  }
}