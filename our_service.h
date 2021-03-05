
#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

// Defining 16-bit service and 128-bit base UUIDs
#define BLE_UUID_OUR_BASE_UUID              {{0x08, 0x55, 0xDA, 0xEC, 0x3A, 0xAF, 0x96, 0xB3, 0x3B, 0x43, 0x10, 0xAF, 0x55, 0x14, 0xE9, 0x0E}} // 128-bit base UUID
#define BLE_UUID_OUR_SERVICE_UUID                0x4B6A // Just a random, but recognizable value

// Defining 16-bit characteristic UUID
#define BLE_UUID_OUR_CHARACTERISTIC_UUID          0xBFFE // Just a random, but recognizable value
#define BLE_UUID_OUR_CHARACTERISTIC_UUID_1        0xBFAB

// BLE_WRITE:
/** @brief Our Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
 // This is used to pass the write handlers for different characteristics from main.c
 // This is essentially like public constructor. All of the content will be copied to instance.
 // Note that "uint32_t characteristic1_value" part had to match from Step 1
typedef void (*ble_os_characteristic1_value_write_handler_t) (uint32_t register_value);

// Add other handlers here...
typedef struct
{
    /**< Event handler to be called when the Characteristic1 is written */
    ble_os_characteristic1_value_write_handler_t characteristic1_value_write_handler; 
    // Add other handlers here...

} ble_os_init_t;

// This structure contains various status information for our service. 
// The name is based on the naming convention used in Nordics SDKs. 
// 'ble indicates that it is a Bluetooth Low Energy relevant structure and 
// os is short for Our Service). 
typedef struct
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */

    // BLE_WRITE: Write handlers. Upon BLE write, these handler will be called. Their implementation is in the main.c
    ble_os_characteristic1_value_write_handler_t characteristic1_value_write_handler;  /**< Event handler to be called when the Characteristic1 is written. */

    // Add handles for the characteristic attributes to our struct
    ble_gatts_char_handles_t char_handles;
    ble_gatts_char_handles_t char_handles_1;
}ble_os_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void our_service_init(ble_os_t *p_our_service, ble_os_init_t * init);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void our_potentiostat_characteristic_update(ble_os_t *p_our_service, int32_t *potential_value);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void our_potentiostat_settings_characteristic_update(ble_os_t *p_our_service, uint8_t *register_val);

/**@brief Function for handling the Write event.
 *
 * @param[in] p_our_service     Our Service structure.
 * @param[in] p_ble_evt         Event received from the BLE stack.
 */
void on_write(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt);

#endif  /* _ OUR_SERVICE_H__ */
