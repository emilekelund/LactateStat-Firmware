/*
 FILENAME: LMP91000.c
 AUTHOR: Emil Ekelund
 EMAIL: eekelund@kth.se

 A FEW INSTRUCTIONS
 * All methods are defined and coded according to the instructions given in the
 * LMP91000 datsheet, December 2014 Revision, from Texas Instruments. All
 * references to the "datasheet" refer to this specific revision. The datasheet
 * is referenced in the code so that the user can have further consult if he/she
 * needs more information. A copy of the datasheet is included in the software
 * download.
 *
 * All references to "the device" refer to the LMP91000 Sensor AFE System:
 * Configurable AFE Potentiostat for Low-Power Chemical-Sensing Applications
 * Impedance Analyzer from Texas Instruments.
 *
 * TIA - Transimpedance Amplifier
 * TIACN - Transimpedance Amplifier Control Register (0x10)
 * REFCN - Reference Control Register (0x11) 
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "LMP91000.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_error.h"

static uint8_t lmp91000_gain = 0;
static uint8_t lmp91000_r_load = 3;
static uint8_t lmp91000_int_zero = 1;
static uint8_t lmp91000_bias = 0;
static uint8_t lmp91000_bias_sign = 0;
const static int lmp91000_op_voltage = 3300; // The device is powered by 3300mV
const static uint8_t NUM_TIA_BIAS = 14;

const uint32_t TIA_GAIN[] = {2750, 3500, 7000, 14000, 35000, 120000, 350000};
const double TIA_BIAS[] = {0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 0.22, 0.24};
const double INT_ZERO[] = {0.2, 0.5, 0.67};
const uint8_t R_LOAD[] = {10, 33, 50, 100};

/*
 @param p_instance: The TWI instance you are using
 @param reg: The register you want to write
 @param data: The data to be written
 @param no_stop: Decides wheter a stop condition should be sent after the write.
        True equals a no-stop condition
 @brief I2C write function for the LMP91000
*/
void lmp91000_write_register(nrf_drv_twi_t const *const p_instance, uint8_t reg, uint8_t data, bool no_stop) {
    uint8_t tx_data[2];
    uint32_t err_code;

    tx_data[0] = (uint8_t) reg;
    tx_data[1] = (uint8_t) (data & 0xFF);

    err_code = nrf_drv_twi_tx(p_instance, LMP91000_ADDRESS, tx_data, sizeof(tx_data), no_stop);
    APP_ERROR_CHECK(err_code);
}

/*
 @param p_instance: The TWI instance you are using
 @param reg: The register you want to read from
 @return The wanted register value
 @brief i2c read function to read a register value from the LMP91000
*/
uint8_t lmp91000_read_register(nrf_drv_twi_t const *const p_instance, uint8_t reg) {
    uint32_t err_code;
    uint8_t reg_value;

    // First we send a write command to indicate what register we want to read
    err_code = nrf_drv_twi_tx(p_instance, LMP91000_ADDRESS, &reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);

    // Now we can read the value from the selected register
    err_code = nrf_drv_twi_rx(p_instance, LMP91000_ADDRESS, &reg_value, 1);
    APP_ERROR_CHECK(err_code);

    return reg_value;
}

/*
 @return wheter the device is ready or not
 @param p_instance: The TWI instance you are using
 @brief Reads the status register (0x00) of the LMP91000
        to determine wheter or note the device is ready to
        accept commands over i2c. Default state is not ready.
*/
bool lmp91000_is_ready(nrf_drv_twi_t const *const p_instance) {
    return lmp91000_read_register(p_instance, LMP91000_STATUS_REG) == LMP91000_READY;
}

/*
 @return wheter or not the TIACN and REFCN is locked for writing
 @param p_instance: The TWI instance you are using
 @brief Reads the lock register (0x01) of the LMP91000 to see
        wheter or not the TIACN and REFCN are write read-only.
        Default state is read-only
*/
bool lmp91000_is_locked(nrf_drv_twi_t const *const p_instance) {
    return lmp91000_read_register(p_instance, LMP91000_LOCK_REG) == LMP91000_WRITE_LOCK;
}

/*
 @param p_instance: The TWI instance you are using
 @brief Locks the "lock" register (0x01) so that it is
        in read-only mode. TIACN and REFCN cannot be
        written to when in read-only.
*/
void lmp91000_lock(nrf_drv_twi_t const *const p_instance) {
    lmp91000_write_register(p_instance, LMP91000_LOCK_REG, LMP91000_WRITE_LOCK, false);
}

/*
 @param p_instance: The TWI instance you are using
 @brief Unlocks the "lock" register (0x01) so that the
        TIACN and REFCN registers can be written to.
*/
void lmp91000_unlock(nrf_drv_twi_t const *const p_instance) {
    lmp91000_write_register(p_instance, LMP91000_LOCK_REG, LMP91000_WRITE_UNLOCK, false);
}

/*
 @param p_instance: The TWI instance you are using
 @param user_gain: What the gain should be set to

        Param - Value - Gain resistance
        0     - 000   - External resistor (Default)
        1     - 001   - 2.75kOhm
        2     - 010   - 3.5kOhm
        3     - 011   - 7kOhm
        4     - 100   - 14kOhm
        5     - 101   - 35kOhm
        6     - 110   - 120kOhm
        7     - 111   - 350kOhm

 @brief This function sets the Transimpedance amplifier
        (TIA) gain. First, the register is read to ensure
        that other parameters are not affected. The 3 LSBs
        of the gain parameter are written to the bits 2-4
        of the TIACN register (0x10).
*/
void lmp91000_set_gain(nrf_drv_twi_t const *const p_instance, uint8_t user_gain) {
    lmp91000_gain = user_gain;
    lmp91000_unlock(p_instance);

    uint8_t data = lmp91000_read_register(p_instance, LMP91000_TIACN_REG);
    data &= ~(7 << 2); // Clear bits 2-4
    data |= (user_gain << 2); // Writes to bit 2-4

    lmp91000_write_register(p_instance, LMP91000_TIACN_REG, data, false);
}

/*
 @brief Simple getter function to retrieve the 
        current gain.
*/
uint32_t lmp91000_get_gain(void) {
    if (lmp91000_gain == 0) { // Default value
        return lmp91000_gain;
    } else {
        return TIA_GAIN[lmp91000_gain];
    }
}

/*
 @param p_instance: The TWI instance you are using
 @param user_load: The internal load resistor to select

        Param - Value - Load resistor
        0     - 00    - 10 Ohm
        1     - 01    - 33 Ohm
        2     - 10    - 50 Ohm
        3     - 11    - 100 Ohm (Default)

 @brief Sets the internal load resistor. The register is
        first read in order to not affect any other bits.
        The 2 LSBs of the load parameter is written to the
        0th and 1st bit of the TIACN register (0x10). 
*/
void lmp91000_set_r_load(nrf_drv_twi_t const *const p_instance, uint8_t user_load) {
    lmp91000_r_load = user_load;
    lmp91000_unlock(p_instance);

    uint8_t data = lmp91000_read_register(p_instance, LMP91000_TIACN_REG);
    data &= ~3; // Clear 0th and 1st bits
    data |= user_load;
    lmp91000_write_register(p_instance, LMP91000_TIACN_REG, data, false);
}

/*
 @brief Simple getter function to retrieve the
        current internal load resistor value.
*/
uint8_t lmp91000_get_r_load(void) {
    return R_LOAD[lmp91000_r_load];
}

/*
 @param p_instance: The TWI instance you are using
 @brief Set the lmp91000 to use the internal 
        reference (Vdd). First unlocks the REFCN
        register (0x11) then read the current values
        to not affect them. Writes a "0" to the 7th
        bit of the register.
*/
void lmp91000_set_int_ref_source(nrf_drv_twi_t const *const p_instance) {
    lmp91000_unlock(p_instance);
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_REFCN_REG);
    data &= ~(1 << 7); // Clears the 7th bit
    lmp91000_write_register(p_instance, LMP91000_REFCN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @brief Sets the lmp91000 to use the external reference
        at the Vref pin. First unlocks and reads
        the RECN register (0x11) to not affect the
        current values. Writes a "1" to the 7th
        bit of the register.
*/
void lmp91000_set_ext_ref_source(nrf_drv_twi_t const *const p_instance) {
    lmp91000_unlock(p_instance);
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_REFCN_REG);
    data |= (1 << 7); // Writes a "1" to the 7th bit
}

/*
 @param source: Choose internal or external source
 @param p_instance: The TWI instance you are using

 param - result
 0     - Internal reference (Default)
 1     - External reference

 @brief Sets the voltage reference of the lmp91000 to
        either internal or external. 
*/
void lmp91000_set_ref_source(nrf_drv_twi_t const *const p_instance, uint8_t source) {
    if (source == 0) {
        lmp91000_set_int_ref_source(p_instance);
    } else {
        lmp91000_set_ext_ref_source(p_instance);
    }
}

/*
 @param p_instance: The TWI instance you are using
 @param user_int_z: The internal zero selection
 
        Param - Value - Internal zero
        0     - 00    - 20%
        1     - 01    - 50% (Default)
        2     - 10    - 67%
        3     - 11    - Bypassed
 
 @brief Sets the internal zero of the device, 
        particularly the TIA. First unlocks and reads
        the REFCN register (0x11) to make sure other 
        bits are not affected. Then writes to the 5th 
        and 6th bit of the REFCN register. 
*/
void lmp91000_set_int_z(nrf_drv_twi_t const *const p_instance, uint8_t user_int_z) {
    lmp91000_int_zero = user_int_z;
    lmp91000_unlock(p_instance);

    uint8_t data = lmp91000_read_register(p_instance, LMP91000_REFCN_REG);
    data &= ~(3 << 5);
    data |= (user_int_z << 5);
    lmp91000_write_register(p_instance, LMP91000_REFCN_REG, data, false);
}

/*
 @brief Simple getter function to retreive the
        Internal zero of the lmp91000.
*/
double lmp91000_get_int_z(void) {
    return INT_ZERO[lmp91000_int_zero];
}

/*
 @param p_instance: The TWI instance you are using
 @brief Sets the bias sign of the lmp91000 to 
        positive. Unlocks and reads the REFCN
        register (0x11) to not affect any of the
        other bits. Writes a "1" to the 4th bit of the
        REFCN register. 
*/
void lmp91000_set_pos_bias(nrf_drv_twi_t const *const p_instance) {
    lmp91000_unlock(p_instance);
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_REFCN_REG);
    data |= (1 << 4); // Set 4th bit to 1
    lmp91000_write_register(p_instance, LMP91000_REFCN_REG, data, false);
    lmp91000_bias_sign = 1;
}

/*
 @param p_instance: The TWI instance you are using
 @brief Sets the bias sign of the lmp91000 to 
        negative. Unlocks and reads the REFCN
        register (0x11) to not affect any of the
        other bits. Writes a "0" to the 4th bit of the
        REFCN register. This is the default setting.
*/
void lmp91000_set_neg_bias(nrf_drv_twi_t const *const p_instance) {
    lmp91000_unlock(p_instance);
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_REFCN_REG);
    data &= ~(1 << 4); // Clear 4th bit
    lmp91000_write_register(p_instance, LMP91000_REFCN_REG, data, false);
    lmp91000_bias_sign = 0;
}

/*
 @param p_instance: The TWI instance you are using
 @param user_sign: Sets the bias sign. 0 = negative, 1 = positive
 @brief Sets the bias sign of the lmp91000 using the 
        REFCN register (0x11). Writes a "0" for
        negative bias (default) or a "1" for
        positive bias.
*/
void lmp91000_set_bias_sign(nrf_drv_twi_t const *const p_instance, uint8_t user_sign) {
    if (user_sign == 1) {
        lmp91000_set_pos_bias(p_instance);
    } else {
        lmp91000_set_neg_bias(p_instance); // Default
    }
}

/*
 @param p_instance: The TWI instance you are using
 @param user_bias: Set the bias voltage of the lmp91000

        Param - Value - Bias setting
        0     - 0000  - 0% (default)
        1     - 0001  - 1%
        2     - 0010  - 2%
        3     - 0011  - 4%
        4     - 0100  - 6%
        5     - 0101  - 8%
        6     - 0110  - 10%
        7     - 0111  - 12%
        8     - 1000  - 14%
        9     - 1001  - 16%
        10    - 1010  - 18%
        11    - 1011  - 20%
        12    - 1100  - 22%
        13    - 1101  - 24%

 @param user_sign: Sets the bias sign. 0 = negative, 1 = positive
 @brief A function to set both the bias voltage value 
        and the bias voltage sign. Unlocks and reads
        the REFCN register (0x11) to not affect the
        other bits in the register. Clear the first
        five bits and then writes the wanted values.
        The bias is a percentage of either the 
        internal reference or the external reference.
*/
void lmp91000_set_bias(nrf_drv_twi_t const *const p_instance, uint8_t user_bias, uint8_t user_sign) {
    if (user_sign == 1) {
        user_sign = 1;
        lmp91000_bias_sign = user_sign;
    } else {
        user_sign = 0; // Default
        lmp91000_bias_sign = user_sign;
    }

    if (user_bias > 13) {
        user_bias = 0; // Default
    }

    lmp91000_bias = user_bias;

    lmp91000_unlock(p_instance);
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_REFCN_REG);
    data &= ~(0x1F); // Clear the first five bits so that a bitwise or can be used
    data |= user_bias;
    data |= ((user_sign << 4) | user_bias);
    lmp91000_write_register(p_instance, LMP91000_REFCN_REG, data, false);
}

/*
 @brief Simple getter function to get the current bias value
 @return The currently used bias percentage value
*/
double lmp91000_get_bias(void) {
    return TIA_BIAS[lmp91000_bias];
}

/*
 @brief Simple getter function to get the bias sign
 @return 0 = negative bias, 1 = positive bias
*/
uint8_t lmp91000_get_bias_sign(void) {
    return lmp91000_bias_sign;
}

/*
 @param p_instance: The TWI instance you are using
 @brief Enables the FET functional mode of the
        lmp91000. This shorts the path between
        WE and RE. Mainly used in deep sleep to 
        minimize current consumption. Reads the
        bits in the MODECN register (0x12) to not
        affect any of the other bits. Writes a "1"
        to the 7th bit.
*/
void lmp91000_enable_fet(nrf_drv_twi_t const *const p_instance) {
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_MODECN_REG);
    data |= (1 << 7); // Write a "1" to the 7th bit
    lmp91000_write_register(p_instance, LMP91000_MODECN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @brief Disables the FET functional mode of the
        lmp91000. This removes the short between
        WE and RE. Reads the bits in the MODECN 
        register (0x12) to not affect any of the 
        other bits. Writes a "0" to the 7th bit.
        This is the default value.
*/
void lmp91000_disable_fet(nrf_drv_twi_t const *const p_instance) {
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_MODECN_REG);
    data &= ~(1 << 7); // Clear the 7th bit
    lmp91000_write_register(p_instance, LMP91000_MODECN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @param selection: Enables or disables the FET mode
        0 = disable or 1 = enable
 @brief Controls the FET mode of the lmp91000 in
        the MODECN register (0x12). Writes either
        a "0" or a "1" to the 7th bit. 0 is default.
*/
void lmp91000_set_fet(nrf_drv_twi_t const *const p_instance, uint8_t selection) {
    if (selection == 1) {
        lmp91000_enable_fet(p_instance);
    } else {
        lmp91000_disable_fet(p_instance); // Default
    }
}

/*
 @param p_instance: The TWI instance you are using
 @brief Places the lmp91000 in deep sleep mode. In
        this mode the power consumption is really 
        low and draws ~0.6uA. Reads the MODECN 
        register (0x12) to not affect any of the 
        other bits. Then writes "000" to the 3 LSBs 
        of the MODECN register.
*/
void lmp91000_sleep(nrf_drv_twi_t const *const p_instance) {
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_MODECN_REG);
    data &= ~(0x07); // Clear the 3 LSBs
    lmp91000_write_register(p_instance, LMP91000_MODECN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @brief Puts the lmp91000 into two lead mode. Used 
        for 2 lead galvanic cell configurations. 
        For instance, an Oxygen gas sensor. The 
        function reads the MODECN register (0x12)
        to not affect any of the other bits. Then
        the 3 LSBs of the register are set to "001".
*/
void lmp91000_set_two_lead(nrf_drv_twi_t const *const p_instance) {
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_MODECN_REG);
    data &= ~(0x07); // Clear the 3 LSBs so bit wise or can be done
    data |= (0x01); // Writes "001" to the three LSBs
    lmp91000_write_register(p_instance, LMP91000_MODECN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @brief Puts the lmp91000 into standmy mode which
        is useful for quick warm up times between
        tests. Reads the MODECN register (0x12) to
        not affect any of the other bits. Then writes
        "010" to the 3 LSBs of the MODECN register.
*/
void lmp91000_standby(nrf_drv_twi_t const *const p_instance) {
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_MODECN_REG);
    data &= ~(0x07); // Clear the 3 LSBs so bit wise or can be done
    data |= (0x02); // Writes "010" to the three LSBs
    lmp91000_write_register(p_instance, LMP91000_MODECN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @brief Sets the lmp91000 into three lead mode for
        potentiometric measurements. Reads the
        MODECN register (0x12) to not affect any
        of the other bits and then writes "011" to
        the three LSBs of the MODECN register.
*/
void lmp91000_set_three_lead(nrf_drv_twi_t const *const p_instance) {
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_MODECN_REG);
    data &= ~(0x07); // Clear the 3 LSBs so bit wise or can be done
    data |= (0x03); // Writes "011" to the 3 LSBs
    lmp91000_write_register(p_instance, LMP91000_MODECN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @brief Sets the lmp91000 to temperature mode with
        the TIA turned OFF. In this mode, the 
        lmp91000 is in standby mode with the
        temperature sensor turned on. At the VOUT
        pin, the temperature sensors output can be
        read. The MODECN register (0x12) is read 
        to not affect any other bits. "110" is then
        written to the three LSBs of the register.
*/
void lmp91000_set_temp_no_tia(nrf_drv_twi_t const *const p_instance) {
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_MODECN_REG);
    data &= ~(0x07); // Clear the 3 LSBs so bit wise or can be done
    data |= (0x06); // Writes "110" to the 3 LSBs
    lmp91000_write_register(p_instance, LMP91000_MODECN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @brief Sets the lmp91000 to temperature mode with
        the TIA turned ON to be used in three lead
        potentiometric mode. The temperature sensor
        output is available at the VOUT pin while
        the potentiostat output is available at the
        C2 pin. Reads the MODECN register (0x12) to 
        not affect any of the other bits. "111" is 
        then written to the 3 LSBs of the register.
*/
void lmp91000_set_temp_with_tia(nrf_drv_twi_t const *const p_instance) {
    uint8_t data = lmp91000_read_register(p_instance, LMP91000_MODECN_REG);
    data &= ~(0x07); // Clear the 3 LSBs so bit wise or can be done
    data |= (0x07); // Writes "111" to the 3 LSBs
    lmp91000_write_register(p_instance, LMP91000_MODECN_REG, data, false);
}

/*
 @param p_instance: The TWI instance you are using
 @param user_mode: Sets the user mode

        Param - Value - Mode
        0     - 000   - Deep sleep (Default)
        1     - 001   - 2 Lead galvanic cell mode
        2     - 010   - Standby
        3     - 011   - 3 lead potentiometric mode
        4     - 110   - Temp. measurement (TIA OFF)
        5     - 111   - Temp. measurement (TIA ON)

 @brief Sets the mode of the lmp91000. Writes
        to the MODECN register (0x12) so that 
        the wanted mode is activated.
*/
void lmp91000_set_mode(nrf_drv_twi_t const *const p_instance, uint8_t user_mode) {
    switch (user_mode) {
        case 0:
            lmp91000_sleep(p_instance);
            break;

        case 1:
            lmp91000_set_two_lead(p_instance);
            break;

        case 2:
            lmp91000_standby(p_instance);
            break;

        case 3:
            lmp91000_set_three_lead(p_instance);
            break;

        case 4:
            lmp91000_set_temp_no_tia(p_instance);
            break;

        case 5:
            lmp91000_set_temp_with_tia(p_instance);
            break;
            
        default:
            lmp91000_sleep(p_instance);
            break;
    }
}

/*
  @param voltage: The voltage that you want to
         determine the bias for.
  @return The bias closest to input value
  @brief Function to determine the correct
         bias value depending to the voltage
         you input.
*/
signed char determine_lmp91000_bias(int16_t voltage) {
  signed char polarity = 0;

  if (voltage < 0) {
    polarity = -1;
  } else {
    polarity = 0;
  }

  int16_t v1 = 0;
  int16_t v2 = 0;

  voltage = abs(voltage);

  if (voltage == 0) {
    return 0;
  }

  for (int i = 0; i < NUM_TIA_BIAS-1; i++) {

    v1 = lmp91000_op_voltage*TIA_BIAS[i];
    v2 = lmp91000_op_voltage*TIA_BIAS[i+1];

    if (voltage == v1) {

      return polarity * i;

    } else if (voltage > v1 && voltage < v2) {
      
        if (abs(voltage - v1) < abs(voltage - v2)) {

          return polarity * i;

        } else {

          return polarity * (i+1);

        }
    }

  }

  return  0;

}