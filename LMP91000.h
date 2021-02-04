/* 
 FILENAME: LMP91000.h
 AUTHOR: Emil Ekelund
 EMAIL: eekelund@kth.se

Please refer to the .c source file for an extended description, instructions and version updates.

*/

#include "nrf_drv_twi.h"
#include <stdint.h>
#include <stdio.h>

#define LMP91000_ADDRESS                (0x48) /* The LMP91000 has a fixed address, 1001000 */

// LMP91000 registers
#define LMP91000_STATUS_REG             (0x00) /* Read only status register */
#define LMP91000_LOCK_REG               (0x01) /* Lock protection register */
#define LMP91000_TIACN_REG              (0x10) /* TIA control register */
#define LMP91000_REFCN_REG              (0x11) /* Reference control register */
#define LMP91000_MODECN_REG             (0x12) /* Mode control register */


// Register variables
#define LMP91000_READY                  (0x01)
#define LMP91000_NOT_READY              (0x00) // Default

#define LMP91000_WRITE_LOCK             (0x01) // Default
#define LMP91000_WRITE_UNLOCK           (0x00)

#define LMP91000_TIA_GAIN_EXT           (0x00) // Default
#define LMP91000_TIA_GAIN_2P75K         (0x04)
#define LMP91000_TIA_GAIN_3P5K          (0x08)
#define LMP91000_TIA_GAIN_7K            (0x0C)
#define LMP91000_TIA_GAIN_14K           (0x10)
#define LMP91000_TIA_GAIN_35K           (0x14)
#define LMP91000_TIA_GAIN_120K          (0x18)
#define LMP91000_TIA_GAIN_350K          (0x1C)

#define LMP91000_RLOAD_10OHM            (0x00)
#define LMP91000_RLOAD_33OHM            (0x01)
#define LMP91000_RLOAD_50OHM            (0x02)
#define LMP91000_RLOAD_100OHM           (0x03) // Default

#define LMP91000_REF_SOURCE_INT         (0x00) // Default
#define LMP91000_REF_SOURCE_EXT         (0x80)

#define LMP91000_INT_Z_20PCT            (0x00)
#define LMP91000_INT_Z_50PCT            (0x20) // Default
#define LMP91000_INT_Z_67PCT            (0x40)
#define LMP91000_INT_Z_BYPASS           (0x60)

#define LMP91000_BIAS_SIGN_NEG          (0x00) // Default
#define LMP91000_BIAS_SIGN_POS          (0x10)

#define LMP91000_BIAS_0PCT              (0x00) // Default
#define LMP91000_BIAS_1PCT              (0x01)
#define LMP91000_BIAS_2PCT              (0x02)
#define LMP91000_BIAS_4PCT              (0x03)
#define LMP91000_BIAS_6PCT              (0x04)
#define LMP91000_BIAS_8PCT              (0x05)
#define LMP91000_BIAS_10PCT             (0x06)
#define LMP91000_BIAS_12PCT             (0x07)
#define LMP91000_BIAS_14PCT             (0x08)
#define LMP91000_BIAS_16PCT             (0x09)
#define LMP91000_BIAS_18PCT             (0x0A)
#define LMP91000_BIAS_20PCT             (0x0B)
#define LMP91000_BIAS_22PCT             (0x0C)
#define LMP91000_BIAS_24PCT             (0x0D)

#define LMP91000_FET_SHORT_DISABLED     (0x00) // Default
#define LMP91000_FET_SHORT_ENABLED      (0x80)

#define LMP91000_OP_MODE_DEEP_SLEEP     (0x00) // Default
#define LMP91000_OP_MODE_GALVANIC       (0x01)
#define LMP91000_OP_MODE_STANDBY        (0x02)
#define LMP91000_OP_MODE_AMPEROMETRIC   (0x03)
#define LMP91000_OP_MODE_TIA_OFF        (0x06)
#define LMP91000_OP_MODE_TIA_ON         (0x07)

#define LMP91000_NOT_PRESENT            (0xA8) // Arbitrary library status code


// Helper arrays for Gain, Bias and internal zero.
extern const uint32_t TIA_GAIN[7];
extern const double TIA_BIAS[14];
extern const double INT_ZERO[3];
extern const uint8_t R_LOAD[4];

// I2C Functions

void lmp91000_write_register(nrf_drv_twi_t const *const p_instance, uint8_t reg, uint8_t data, bool no_stop);

uint8_t lmp91000_read_register(nrf_drv_twi_t const *const p_instance, uint8_t reg);

// Methods to controll the registers of the lmp91000

bool lmp91000_is_ready(nrf_drv_twi_t const *const p_instance);

bool lmp91000_is_locked(nrf_drv_twi_t const *const p_instance);

void lmp91000_lock(nrf_drv_twi_t const *const p_instance);

void lmp91000_unlock(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_gain(nrf_drv_twi_t const *const p_instance, uint8_t user_gain);

uint32_t lmp91000_get_gain();

void lmp91000_set_r_load(nrf_drv_twi_t const *const p_instance, uint8_t user_load);

uint8_t lmp91000_get_r_load();

void lmp91000_set_int_ref_source(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_ext_ref_source(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_ref_source(nrf_drv_twi_t const *const p_instance, uint8_t source);

void lmp91000_set_int_z(nrf_drv_twi_t const *const p_instance, uint8_t user_int_z);

double lmp91000_get_int_z();

void lmp91000_set_pos_bias(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_neg_bias(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_bias_sign(nrf_drv_twi_t const *const p_instance, uint8_t user_sign);

void lmp91000_set_bias(nrf_drv_twi_t const *const p_instance, uint8_t user_bias, uint8_t user_sign);

double lmp91000_get_bias();

uint8_t lmp91000_get_bias_sign();

void lmp91000_enable_fet(nrf_drv_twi_t const *const p_instance);

void lmp91000_disable_fet(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_fet(nrf_drv_twi_t const *const p_instance, uint8_t selection);

void lmp91000_sleep(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_two_lead(nrf_drv_twi_t const *const p_instance);

void lmp91000_standby(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_three_lead(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_temp_no_tia(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_temp_with_tia(nrf_drv_twi_t const *const p_instance);

void lmp91000_set_mode(nrf_drv_twi_t const *const p_instance, uint8_t user_mode);