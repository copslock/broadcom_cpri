/***********************************************************************************
 *                                                                                 *
 * This license is set out in https://raw.githubusercontent.com/Broadcom-Network-Switching-Software/OpenBCM/master/Legal/LICENSE file.
 * 
 * Copyright 2007-2020 Broadcom Inc. All rights reserved.
 */

/**********************************************************************************
 **********************************************************************************
 *  File Name     :  osprey7_pll_config.c                                         *
 *  Created On    :  24 Apr 2018                                                  *
 *  Created By    :  Kiran Divakar                                                *
 *  Description   :  Osprey7 PLL Configuration APIs                               *
 */


/** @file osprey7_pll_config.c
 * Osprey7 PLL Configuration APIs
 */


#include "osprey7_v2l8p1_config.h"
#include "osprey7_v2l8p1_functions.h"
#include "osprey7_v2l8p1_internal.h"
#include "osprey7_v2l8p1_internal_error.h"
#include "osprey7_v2l8p1_select_defns.h"

#define _ndiv_frac_l(x) ((x)&0xFFFF)
#define _ndiv_frac_h(x) ((x)>>16)

#define _ndiv_frac_decode(l_, h_) (((l_) & 0xFFFF) | (((h_) & 0x3) << 16))

static err_code_t _osprey7_v2l8p1_pll_powerdown(srds_access_t *sa__, uint8_t pwrdn_en);
static err_code_t _osprey7_v2l8p1_restore_pll_defaults(srds_access_t *sa__);

/* The pll_fracn_ndiv_int and pll_fracn_frac bitfields have this many bits. */
static const uint32_t pll_fracn_ndiv_int_bits = 10;
static const uint32_t pll_fracn_frac_bits     = 18;

static err_code_t _osprey7_v2l8p1_pll_powerdown(srds_access_t *sa__, uint8_t pwrdn_en) {
    INIT_SRDS_ERR_CODE
    EFUN(wrc_ams_pll_pwrdn_refbuf(pwrdn_en));
    EFUN(wrc_ams_pll_pwrdn_b_sdm(!pwrdn_en));
    EFUN(wrc_ams_pll_pwrdn_cp(pwrdn_en));
    EFUN(wrc_ams_pll_pwrdn_buf(pwrdn_en));
    EFUN(wrc_ams_pll_pwrdn_hbvco(pwrdn_en));
    EFUN(wrc_ams_pll_pwrdn_lbvco(0x1));
    EFUN(wrc_ams_pll_pwrdn_mmd(pwrdn_en));
    EFUN(wrc_ams_pll_pwrdn_rtl(pwrdn_en));
    EFUN(wrc_ams_pll_pwrdn(pwrdn_en));
    return (ERR_CODE_NONE);
}

static err_code_t _osprey7_v2l8p1_restore_pll_defaults(srds_access_t *sa__) {
    INIT_SRDS_ERR_CODE
    EFUN(wrc_ams_pll_fracn_ndiv_int(0x3B));
    EFUN(wrc_ams_pll_fracn_ndiv_frac_l(0x0));
    EFUN(wrc_ams_pll_fracn_ndiv_frac_h(0x0));
    EFUN(wrc_ams_pll_refclk_freq2x_en(0x0));
    EFUN(wrc_ams_pll_refdiv2(0x0));
    EFUN(wrc_ams_pll_refdiv4(0x0));
    EFUN(wrc_ams_pll_div4_2_sel(0x0));
    EFUN(wrc_ams_pll_pll_frac_mode(0x1));
    EFUN(wrc_ams_pll_resetb_mmd(0x1));
    EFUN(wrc_ams_pll_rtl_div_en(0x0));
    EFUN(wrc_ams_pll_en_lb_res(0x0));
    EFUN(wrc_ams_pll_en_lb_ind(0x0));
    EFUN(wrc_ams_pll_lb_sel_pll0(0x0));
    EFUN(wrc_ams_pll_rz_sel(0x4));
    EFUN(wrc_ams_pll_cp_calib_adj(0x7));
    EFUN(wrc_ams_pll_cap_l_pll0(0x0));
    EFUN(wrc_ams_pll_cap_r_pll0(0x0));
    return (ERR_CODE_NONE);
}

err_code_t osprey7_v2l8p1_INTERNAL_configure_pll(srds_access_t *sa__,
                                         enum osprey7_v2l8p1_pll_refclk_enum refclk,
                                         enum osprey7_v2l8p1_pll_div_enum srds_div,
                                         uint32_t vco_freq_khz,
                                         enum osprey7_v2l8p1_pll_option_enum pll_option) {
    INIT_SRDS_ERR_CODE
    uint32_t refclk_freq_hz=0;
    enum osprey7_v2l8p1_pll_option_enum pll_opt;

    pll_opt = (enum osprey7_v2l8p1_pll_option_enum)(pll_option & OSPREY7_V2L8P1_PLL_OPTION_REFCLK_MASK);
    if (pll_opt == OSPREY7_V2L8P1_PLL_OPTION_POWERDOWN) {
        UNUSED(refclk_freq_hz);
        EFUN(_osprey7_v2l8p1_pll_powerdown(sa__, 0x1));
        return (ERR_CODE_NONE);
    }

#ifdef SMALL_FOOTPRINT
    EFUN(osprey7_v2l8p1_INTERNAL_get_refclk_in_hz(sa__, refclk, &refclk_freq_hz));
#else
    EFUN(osprey7_v2l8p1_INTERNAL_resolve_pll_parameters(sa__, refclk, &refclk_freq_hz, &srds_div, &vco_freq_khz, pll_option));
#endif

    /* Use this to restore defaults if reprogramming the PLL under dp-reset (typically Auto-Neg FW) */
    EFUN(_osprey7_v2l8p1_restore_pll_defaults(sa__));

    {
        uint8_t reset_state;
        /* Use core_s_rstb to re-initialize all registers to default before calling this function. */
        ESTM(reset_state = rdc_core_dp_reset_state());

        if(reset_state < 7) {
            EFUN_PRINTF(("ERROR: osprey7_v2l8p1_configure_pll(..) called without core_dp_s_rstb=0\n"));
            return (osprey7_v2l8p1_error(sa__, ERR_CODE_CORE_DP_NOT_RESET));
        }
    }

    /* Clear PLL powerdown */
    EFUN(_osprey7_v2l8p1_pll_powerdown(sa__, 0x0));

    /* Use HBVCO for 50.0 GHz and above */
    EFUN(wrc_ams_pll_pwrdn_hbvco(0x1));
    EFUN(wrc_ams_pll_pwrdn_lbvco(0x1));
    if (vco_freq_khz >= 50000000) {
        EFUN(wrc_ams_pll_en_lb_res(0x0));
        EFUN(wrc_ams_pll_en_lb_ind(0x0));
        EFUN(wrc_ams_pll_vcobuf_bandcntrl(0x4));
        EFUN(wrc_ams_pll_pwrdn_hbvco(0x0));
    }
    else {
        EFUN(wrc_ams_pll_en_lb_res(0x1));
        EFUN(wrc_ams_pll_en_lb_ind(0x1));
        EFUN(wrc_ams_pll_pwrdn_lbvco(0x0));
        EFUN(wrc_ams_pll_lb_sel_pll0(0x1));
    }
    EFUN(wrc_ams_pll_rz_sel(1));
    EFUN(wrc_ams_pll_cp_calib_adj(0x6));
    {
        uint8_t cap_pll = 0;
        if (vco_freq_khz > 55800000) {
            cap_pll = 0;
        }
        else if (vco_freq_khz > 54800000) {
            cap_pll = 1;
        }
        else if (vco_freq_khz > 53600000) {
            cap_pll = 2;
        }
        else if (vco_freq_khz > 52600000) {
            cap_pll = 3;
        }
        else if (vco_freq_khz > 51600000) {
            cap_pll = 4;
        }
        else if (vco_freq_khz > 50600000) {
            cap_pll = 5;
        }
        else if (vco_freq_khz > 49900000) {
            cap_pll = 6;
        }
        else if (vco_freq_khz > 49200000) {
            cap_pll = 0;
        }
        else if (vco_freq_khz > 48200000) {
            cap_pll = 1;
        }
        else if (vco_freq_khz > 47400000) {
            cap_pll = 2;
        }
        else if (vco_freq_khz > 46600000) {
            cap_pll = 3;
        }
        else if (vco_freq_khz > 45800000) {
            cap_pll = 4;
        }
        else if (vco_freq_khz > 45000000) {
            cap_pll = 5;
        }
        else if (vco_freq_khz > 44400000) {
            cap_pll = 6;
        }
        else if (vco_freq_khz > 43600000) {
            cap_pll = 7;
        }
        else if (vco_freq_khz > 43000000) {
            cap_pll = 8;
        }
        else if (vco_freq_khz > 42400000) {
            cap_pll = 9;
        }
        else if (vco_freq_khz > 41800000) {
            cap_pll = 10;
        }
        else if (vco_freq_khz > 41200000) {
            cap_pll = 11;
        }
        else if (vco_freq_khz > 40800000) {
            cap_pll = 12;
        }
        EFUN(wrc_ams_pll_cap_l_pll0(cap_pll));
        EFUN(wrc_ams_pll_cap_r_pll0(cap_pll));
    }
    if (refclk_freq_hz == 312500000 || refclk_freq_hz== 311040000) {
        EFUN(wrc_ams_pll_rtl_div_en(1));
    }
    {
        /* Get information needed for fractional mode configuration.
         * Start with the div value composed of an integer and a wide fractional value.
         *
         * The value programmed into the pll_fracn_* bitfields which must account for the
         * initial div2 stage after the VCO.  For instance, a divide by 147.2 must be
         * programmed with an integer of 73 and a fraction of 0.6.
         *
         * Start with the div value, divided by 2, composed of an integer and a wide fractional value.
         */
        const uint8_t  div_fraction_width = 28; /* Must be less than 32 due to overflow detection below. */
        const uint16_t div_integer        = (uint16_t)(SRDS_INTERNAL_GET_PLL_DIV_INTEGER(srds_div) >> 1);
        const uint32_t div_fraction       = (((SRDS_INTERNAL_GET_PLL_DIV_INTEGER(srds_div) & 1) << (div_fraction_width-1))
                                             | SRDS_INTERNAL_GET_PLL_DIV_FRACTION_NUM(srds_div, div_fraction_width-1));

        /* The div_fraction may have more precision than our pll_fracn_frac bitfield.
         * So round it.  Start by adding 1/2 LSB of the fraction div_fraction.
         */
        const uint32_t div_fraction_0p5 = 1 << (div_fraction_width - pll_fracn_frac_bits - 1);
        const uint32_t div_fraction_plus_0p5 = div_fraction + div_fraction_0p5;

        /* Did div_fraction_plus_p5 have a carry bit? */
        const uint32_t div_fraction_plus_p5_carry = div_fraction_plus_0p5 >> div_fraction_width;

        /* The final rounded div_fraction, including carry up to div_integer.
         * This removes the carry and implements the fixed point truncation.
         */
        const uint16_t pll_fracn_ndiv_int  = (uint16_t)(div_integer + div_fraction_plus_p5_carry);
        const uint32_t pll_fracn_div = ((div_fraction_plus_0p5 & ((1U << div_fraction_width)-1))
                                        >> (div_fraction_width - pll_fracn_frac_bits));

        if (pll_fracn_ndiv_int != (pll_fracn_ndiv_int & ((1 << pll_fracn_ndiv_int_bits)-1))) {
            EFUN_PRINTF(("ERROR:  PLL divide is too large for div value 0x%08X\n", srds_div));
            return (osprey7_v2l8p1_error(sa__, ERR_CODE_PLL_DIV_INVALID));
        }

        /* fracn_ndiv_int restrcited to 12 to 511) */
        if ((pll_fracn_ndiv_int < 12) || (pll_fracn_ndiv_int > 511)) {
            return (osprey7_v2l8p1_error(sa__, ERR_CODE_INVALID_PLL_CFG));
        }

        /* To ensure glitch-free operation - toggle ndiv_valid low; load ndiv values; toggle ndiv_valid high. */
        EFUN(wrc_ams_pll_resetb_mmd(0x0));                                /* reset PLL mmd */
        EFUN(wrc_ams_pll_resetb_mmd(0x1));                                /* release reset */
        EFUN(wrc_ams_pll_i_ndiv_valid(0x0));                              /* assert low before programming fracn PLL div value */
        EFUN(wrc_ams_pll_pll_frac_mode(0x1));                             /* MMD 8/9 mode */
        EFUN(wrc_ams_pll_fracn_ndiv_int   (pll_fracn_ndiv_int));          /* interger part of fracn PLL div */
        EFUN(wrc_ams_pll_fracn_ndiv_frac_l(_ndiv_frac_l(pll_fracn_div))); /* set lower 16 bits of fractional part of fracn PLL div */
        EFUN(wrc_ams_pll_fracn_ndiv_frac_h((uint8_t)(_ndiv_frac_h(pll_fracn_div)))); /* set upper  2 bits of fractional part of fracn PLL div */
        EFUN(wrc_ams_pll_i_ndiv_valid(0x1));                              /* to load fracn_ndiv and ndiv_int */
        EFUN(USR_DELAY_US(5));                                            /* delay of atleast 8 Refclk cycles */
        EFUN(wrc_ams_pll_i_ndiv_valid(0x0));                              /* to latch fracn_ndiv and ndiv_int */
    }

    /* Handle refclk PLL options */
    if (pll_opt == OSPREY7_V2L8P1_PLL_OPTION_REFCLK_DOUBLER_EN) {
        EFUN(wrc_ams_pll_refclk_freq2x_en(0x1));
    } else if (pll_opt == OSPREY7_V2L8P1_PLL_OPTION_REFCLK_DIV2_EN) {
        EFUN(wrc_ams_pll_refdiv2(1));
        EFUN(wrc_ams_pll_div4_2_sel(1));
    } else if (pll_opt == OSPREY7_V2L8P1_PLL_OPTION_REFCLK_DIV4_EN) {
        EFUN(wrc_ams_pll_refdiv4(1));
        EFUN(wrc_ams_pll_div4_2_sel(1));
    }

    /* NOTE: Might have to add some optimized PLL control settings post-DVT*/

    /* Update core variables with the VCO rate. */
    {
        struct osprey7_v2l8p1_uc_core_config_st core_config = UC_CORE_CONFIG_INIT;
        EFUN(osprey7_v2l8p1_get_uc_core_config(sa__, &core_config));
        core_config.vco_rate_in_Mhz = (int32_t)((vco_freq_khz + 500) / 1000);
        core_config.field.vco_rate = MHZ_TO_VCO_RATE(core_config.vco_rate_in_Mhz);
        EFUN(osprey7_v2l8p1_INTERNAL_set_uc_core_config(sa__, core_config));
    }

    return (ERR_CODE_NONE);
} /* osprey7_v2l8p1_configure_pll */

#ifndef SMALL_FOOTPRINT
err_code_t osprey7_v2l8p1_INTERNAL_read_pll_div(srds_access_t *sa__, uint32_t *srds_div) {
    INIT_SRDS_ERR_CODE
    uint16_t pll_fracn_ndiv_int;
    uint32_t pll_fracn_div;
    ESTM(pll_fracn_ndiv_int = rdc_ams_pll_fracn_ndiv_int());
    ESTM(pll_fracn_div = (uint32_t)(_ndiv_frac_decode(rdc_ams_pll_fracn_ndiv_frac_l(), rdc_ams_pll_fracn_ndiv_frac_h())));

    /* The value programmed into the pll_fracn_* bitfields which must
     * account for the initial div2 stage after the VCO.  For instance, a
     * divide by 147.2 must be programmed with an integer of 73 and a
     * fraction of 0.6.
     *
     * So multiply the bitfield reads by 2.
     */

      pll_fracn_ndiv_int   = (uint16_t)(pll_fracn_ndiv_int << 1);
      pll_fracn_div      <<= 1;

      {
          /* If the post-multiplied fractional part overflows, then apply the carry to
           * the integer part.
           */
          const uint32_t pll_fracn_div_masked = pll_fracn_div & ((1U << pll_fracn_frac_bits)-1);
          if (pll_fracn_div_masked != pll_fracn_div) {
              ++pll_fracn_ndiv_int;
              pll_fracn_div = pll_fracn_div_masked;
          }
      }

    *srds_div = SRDS_INTERNAL_COMPOSE_PLL_DIV(pll_fracn_ndiv_int, pll_fracn_div, pll_fracn_frac_bits);
    return (ERR_CODE_NONE);
}
#endif /* SMALL_FOOTPRINT */
