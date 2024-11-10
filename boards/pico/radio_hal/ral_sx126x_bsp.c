/*!
 * \file      ral_sx126x_bsp.c
 *
 * \brief     Implements the HAL functions for SX126X
 *
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "ral_sx126x_bsp.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void ral_sx126x_bsp_get_reg_mode( const void* context, sx126x_reg_mod_t* reg_mode )
{
    *reg_mode = SX126X_REG_MODE_DCDC;
}

void ral_sx126x_bsp_get_rf_switch_cfg( const void* context, bool* dio2_is_set_as_rf_switch )
{
    *dio2_is_set_as_rf_switch = true;
}

/*
* See 13.1.14 of the datasheet
*/
void ral_sx126x_bsp_get_tx_cfg( const void* context, const ral_sx126x_bsp_tx_cfg_input_params_t* input_params,
                                ral_sx126x_bsp_tx_cfg_output_params_t* output_params )

{


    //workaround 15.2
    uint8_t reg_val = 0;
    sx126x_read_register(context, 0x08D8, &reg_val, 1);
    reg_val |= (0x0F<<1);
    sx126x_write_register(context, 0x08D8, &reg_val, 1);

    int8_t power = input_params->system_output_pwr_in_dbm ;
    if(input_params->system_output_pwr_in_dbm > 22) {
        power = 22;
    } else if(input_params->system_output_pwr_in_dbm < -9) {
        power = -9;
    }

    output_params->pa_ramp_time  = SX126X_RAMP_40_US;
    output_params->pa_cfg.pa_lut = 0x01;  // reserved value, same for sx1261 sx1262 and sx1268

    output_params->pa_cfg.device_sel                 = 0x00;  // select SX1262/SX1268 device
    output_params->pa_cfg.hp_max                     = 0x07;  // to achieve 22dBm
    output_params->pa_cfg.pa_duty_cycle              = 0x04;
    output_params->chip_output_pwr_in_dbm_configured = power;
    output_params->chip_output_pwr_in_dbm_expected   = power;
}

void ral_sx126x_bsp_get_xosc_cfg( const void* context, ral_xosc_cfg_t* xosc_cfg,
                                  sx126x_tcxo_ctrl_voltages_t* supply_voltage, uint32_t* startup_time_in_tick )
{
    // TCXO present on sharewave pico board, controlled via dio3
    *xosc_cfg = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
    *supply_voltage = SX126X_TCXO_CTRL_1_7V;
    *startup_time_in_tick = 5 << 6;
}

void ral_sx126x_bsp_get_trim_cap( const void* context, uint8_t* trimming_cap_xta, uint8_t* trimming_cap_xtb )
{
    // Do nothing, let the driver choose the default values
}

void ral_sx126x_bsp_get_rx_boost_cfg( const void* context, bool* rx_boost_is_activated )
{
    *rx_boost_is_activated = false;
}

void ral_sx126x_bsp_get_ocp_value( const void* context, uint8_t* ocp_in_step_of_2_5_ma )
{
    // Do nothing, let the driver choose the default values
}

void ral_sx126x_bsp_get_lora_cad_det_peak( ral_lora_sf_t sf, ral_lora_bw_t bw, ral_lora_cad_symbs_t nb_symbol,
                                           uint8_t* in_out_cad_det_peak )
{
    // Function used to fine tune the cad detection peak, update if needed
}

/* --- EOF ------------------------------------------------------------------ */
