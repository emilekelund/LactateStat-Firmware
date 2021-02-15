/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "sensorsim.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "our_service.h"
#include "LMP91000.h"

#define DEVICE_NAME "LactateStat"             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "Chemical Sensing Group KTH" /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL 300                           /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION 36000  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 0                               /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SAMPLES_IN_BUFFER 1
#define ALPHA 0.1 // Alpha value (between 0-1) for the Exponential Weighted Moving Average (EWMA) filter, lower value = more smoothing. A value of 1 effectively disables the filter
#define SAADC_CALIBRATION_INTERVAL 50000 // Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.

#define MENB_PIN 10

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Variable init */
static nrf_saadc_value_t m_buffer[SAMPLES_IN_BUFFER];
static int32_t adc_val = 0;
static int32_t adc_val_old = 0;
static float voltage = 0;
static uint32_t m_adc_evt_counter = 0;
static bool m_saadc_calibrate = false;
static bool m_saadc_first_start_calibrate = true;
static uint8_t TIACN_reg = 0;
static uint8_t REFCN_reg = 0;
static uint8_t MODECN_reg = 0;

NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

// Declare a service structure for our application
ble_os_t m_our_service;

// Declare two app_timer id variables and define our timer intervals
APP_TIMER_DEF(m_our_char_timer_id);
#define OUR_CHAR_TIMER_INTERVAL APP_TIMER_TICKS(1000) //Send data to BLE device every 1s
APP_TIMER_DEF(m_our_adc_sample_timer);
#define OUR_ADC_SAMPLE_TIMER_INTERVAL APP_TIMER_TICKS(10) // Sample every 10ms
APP_TIMER_DEF(m_our_lmp91000_settings_timer_id);
#define OUR_LMP91000_SETTINGS_TIMER_INTERVAL APP_TIMER_TICKS(500) // Send the lmp91000 register settings 0.5s after ble connection initiated

// Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifiers. */
    {
        {BLE_UUID_OUR_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}};

static void advertising_start(bool erase_bonds);

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
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void read_lmp91000_registers() {
  TIACN_reg = lmp91000_read_register(&m_twi, LMP91000_TIACN_REG);
  REFCN_reg = lmp91000_read_register(&m_twi, LMP91000_REFCN_REG);
  MODECN_reg = lmp91000_read_register(&m_twi, LMP91000_MODECN_REG);
  NRF_LOG_INFO("TIACN: %d", TIACN_reg);
  NRF_LOG_INFO("REFCN: %d", REFCN_reg);
  NRF_LOG_INFO("MODECN: %d", MODECN_reg);
}

// Applies suitable start settings, can be changed by the user via software
static void init_lmp91000_settings() {
  nrf_delay_ms(0.5);
  lmp91000_sleep(&m_twi);
  nrf_delay_ms(0.5);
  lmp91000_set_gain(&m_twi, 7);
  lmp91000_set_r_load(&m_twi, 3);
  lmp91000_set_int_ref_source(&m_twi);
  lmp91000_set_int_z(&m_twi, 0);
  lmp91000_set_neg_bias(&m_twi);
  lmp91000_set_bias(&m_twi, 0, 0);
  nrf_delay_ms(0.5);
  read_lmp91000_registers();
}

// BLE_WRITE:
/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] characteristic1_value     value that was received from the phone
 */
// called from our_services.c from on_write();
static void characteristic1_value_write_handler(uint32_t characteristic1_value) {
  NRF_LOG_INFO("We have received the value:  %d", characteristic1_value);
  uint8_t registerData[2] =
    {
      ((uint16_t) characteristic1_value >> 8) & 0xFF,
      ((uint16_t) characteristic1_value >> 0) & 0xFF,
    };
    NRF_LOG_INFO("TIACN REG: %d", registerData[0]);
    NRF_LOG_INFO("REFCN REG: %d", registerData[1]);
    lmp91000_write_register(&m_twi, LMP91000_TIACN_REG, registerData[0], false);
    lmp91000_write_register(&m_twi, LMP91000_REFCN_REG, registerData[1], false);
}

/**@brief Callback function for SAADC
 */
void saadc_callback(nrf_drv_saadc_evt_t const *p_event) {
  ret_code_t err_code;
  if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {

    if ((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0 || m_saadc_first_start_calibrate == true) //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
    {
      nrf_drv_saadc_abort();    // Abort all ongoing conversions. Calibration cannot be run if SAADC is busy
      m_saadc_first_start_calibrate = false; // Set to false so that we only calibrate on start
      m_saadc_calibrate = true; // Set flag to trigger calibration in main context when SAADC is stopped
    }

    if (m_saadc_calibrate == false) {
      err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
      APP_ERROR_CHECK(err_code);
      adc_val = adc_val_old + ALPHA * ((p_event->data.done.p_buffer[0]) - adc_val_old); // Set the raw value from the saadc buffer and apply EWMA filter
      adc_val_old = adc_val;
    }
    m_adc_evt_counter++;
  } else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE) {

    err_code = nrf_drv_saadc_buffer_convert(m_buffer, SAMPLES_IN_BUFFER); //Set buffer so the SAADC can write to it again.
    APP_ERROR_CHECK(err_code);
  }
}

// ADC sample timer timeout handler
static void adc_timer_timeout_handler(void *p_context) {
  nrf_drv_saadc_sample();
}

// Potentiostat settings broadcast timer 
// Send the current settings 0.5s after connecting to the smartphone
// This is a single shot timer
static void lmp91000_settings_broadcast_handler(void *p_context) {
  our_potentiostat_settings_characteristic_update(&m_our_service, &TIACN_reg);
}

// Timer event handler
static void ble_timer_timeout_handler(void *p_context) {
  voltage = adc_val / 1241.2121212121f;
  NRF_LOG_INFO("Adc value: %d\r\n", adc_val);
  NRF_LOG_INFO("Voltage: " NRF_LOG_FLOAT_MARKER "V\r\n", NRF_LOG_FLOAT(voltage));
  our_potentiostat_characteristic_update(&m_our_service, &adc_val); // Broadcast the raw ADC value over BLE
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
  ret_code_t err_code;

  switch (p_evt->evt_id) {
  case PM_EVT_BONDED_PEER_CONNECTED: {
    NRF_LOG_INFO("Connected to a previously bonded device.");
  } break;

  case PM_EVT_CONN_SEC_SUCCEEDED: {
    NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
        ble_conn_state_role(p_evt->conn_handle),
        p_evt->conn_handle,
        p_evt->params.conn_sec_succeeded.procedure);
  } break;

  case PM_EVT_CONN_SEC_FAILED: {
    /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
  } break;

  case PM_EVT_CONN_SEC_CONFIG_REQ: {
    // Reject pairing request from an already bonded peer.
    pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
    pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
  } break;

  case PM_EVT_STORAGE_FULL: {
    // Run garbage collection on the flash.
    err_code = fds_gc();
    if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES) {
      // Retry.
    } else {
      APP_ERROR_CHECK(err_code);
    }
  } break;

  case PM_EVT_PEERS_DELETE_SUCCEEDED: {
    advertising_start(false);
  } break;

  case PM_EVT_PEER_DATA_UPDATE_FAILED: {
    // Assert.
    APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
  } break;

  case PM_EVT_PEER_DELETE_FAILED: {
    // Assert.
    APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
  } break;

  case PM_EVT_PEERS_DELETE_FAILED: {
    // Assert.
    APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
  } break;

  case PM_EVT_ERROR_UNEXPECTED: {
    // Assert.
    APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
  } break;

  case PM_EVT_CONN_SEC_START:
  case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
  case PM_EVT_PEER_DELETE_SUCCEEDED:
  case PM_EVT_LOCAL_DB_CACHE_APPLIED:
  case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
    // This can happen when the local DB has changed.
  case PM_EVT_SERVICE_CHANGED_IND_SENT:
  case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
  default:
    break;
  }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
  // Initialize timer module.
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  // Initiate our timers
  app_timer_create(&m_our_char_timer_id, APP_TIMER_MODE_REPEATED, ble_timer_timeout_handler);
  app_timer_create(&m_our_adc_sample_timer, APP_TIMER_MODE_REPEATED, adc_timer_timeout_handler);
  app_timer_create(&m_our_lmp91000_settings_timer_id, APP_TIMER_MODE_SINGLE_SHOT, lmp91000_settings_broadcast_handler);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void) {

  uint32_t err_code;
  nrf_ble_qwr_init_t qwr_init = {0};

  // BLE_WRITE: Initialize Our Service module.
  ble_os_init_t init = {0}; // Init Our Service module
  init.characteristic1_value_write_handler = characteristic1_value_write_handler;

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  // Initialize the services used by the application.
  our_service_init(&m_our_service, &init);
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
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  ret_code_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void) {
  ret_code_t err_code;

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
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
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  ret_code_t err_code;

  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    NRF_LOG_INFO("Fast advertising.");
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

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  ret_code_t err_code = NRF_SUCCESS;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected.");
    
    // Stop timers on BLE disconnect
    app_timer_stop(m_our_char_timer_id);
    app_timer_stop(m_our_adc_sample_timer);
    lmp91000_sleep(&m_twi); // Set lmp91000 to sleep when disconnected.

    break;

  case BLE_GAP_EVT_CONNECTED:
    NRF_LOG_INFO("Connected.");
    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);

    lmp91000_set_three_lead(&m_twi); // Set the lmp91000 to amperometric mode when connected. Sleep when disconnected.

    read_lmp91000_registers();

    // When connected; start our timer to start regular LMP91000 measurements
    app_timer_start(m_our_char_timer_id, OUR_CHAR_TIMER_INTERVAL, NULL);
    app_timer_start(m_our_adc_sample_timer, OUR_ADC_SAMPLE_TIMER_INTERVAL, NULL);
    app_timer_start(m_our_lmp91000_settings_timer_id, OUR_LMP91000_SETTINGS_TIMER_INTERVAL, NULL);
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    // No implementation needed.
    break;
  }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

  // Set up a BLE event observer to call ble_our_service_on_ble_evt() to do housekeeping of ble connections related to our service and characteristics.
  NRF_SDH_BLE_OBSERVER(m_our_service_observer, APP_BLE_OBSERVER_PRIO, ble_our_service_on_ble_evt, (void *)&m_our_service);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void) {
  ble_gap_sec_params_t sec_param;
  ret_code_t err_code;

  err_code = pm_init();
  APP_ERROR_CHECK(err_code);

  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_param.bond = SEC_PARAM_BOND;
  sec_param.mitm = SEC_PARAM_MITM;
  sec_param.lesc = SEC_PARAM_LESC;
  sec_param.keypress = SEC_PARAM_KEYPRESS;
  sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob = SEC_PARAM_OOB;
  sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc = 1;
  sec_param.kdist_own.id = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id = 1;

  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void) {
  ret_code_t err_code;

  NRF_LOG_INFO("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
  ret_code_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = true;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief SAADC init
 */
void saadc_init(void) {

  ret_code_t err_code;
  nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

  channel_config.burst = NRF_SAADC_BURST_ENABLED;
  channel_config.reference = NRF_SAADC_REFERENCE_VDD4;
  channel_config.gain = NRF_SAADC_GAIN1_4;

  err_code = nrf_drv_saadc_init(NULL, saadc_callback);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_buffer_convert(m_buffer, SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) {
    delete_bonds();
    // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
  } else {
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
  }
}

/**
 * @brief TWI initialization.
 */
static void twi_init(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_LMP91000_config = {
      .scl = 9,
      .sda = 8,
      .frequency = NRF_DRV_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi, &twi_LMP91000_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}

/**@brief Function for application main entry.
 */
int main(void) {
  bool erase_bonds = true;

  // Initialize.
  log_init();
  timers_init();
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();

  services_init();
  advertising_init();

  conn_params_init();
  peer_manager_init();
  nrf_gpio_cfg_output(MENB_PIN);
  nrf_gpio_pin_clear(MENB_PIN);

  twi_init();
  saadc_init();
  init_lmp91000_settings();


  advertising_start(erase_bonds);

  // Enter main loop.
  for (;;) {
      if (m_saadc_calibrate == true) {
        while (nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS)
        ; //Trigger calibration task
      m_saadc_calibrate = false;
    }

    idle_state_handle();
    nrf_pwr_mgmt_run();
    NRF_LOG_FLUSH();

  }
}