/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include "global.h"

#ifndef TDC_1000_7200_H
#define TDC_1000_7200_H

#define USE_OSC

#define TDC7200_WAKEUP_PERIOD			400

#define TOF_MEASUREMENT					0
#define TMP_MEASUREMENT					1

#define TDC_CONTINUOUS_TRIGGER_STATE	0x0001
#define TDC_TOF_GRAPTH_STATE			0x0002
#define TDC_SINGLE_SHOT_MEASURE_STATE	0x0004
#define TDC_POWER_CYCLE_FLAG			0x0008
#define TDC_DOUBLE_RESOLUTION_FLAG		0x0010
#define TDC_UP_STREAM_BUFFER_READY		0x0020
#define TDC_DOWN_STREAM_BUFFER_READY	0x0040
#define TDC_RTD_BUFFER_READY			0x0080
#define TDC_INTERLEAVED_TEMP_MEASURE	0x0100
#define TDC_TDC7200_ENABLED			0x0200

#define TDC_DEFAULT_FLAG				0x0000

#define TDC7200_ALL_DATA_SIZE	39
#define NUM_SAMPLES				1						// number of samples per block
#define SAMPLE_SIZE				TDC7200_ALL_DATA_SIZE
#define NUM_BLOCKS				1

#define BUF_LENGTH				(NUM_SAMPLES * SAMPLE_SIZE * NUM_BLOCKS) + 1

#define MULTI_CYCLE 2

#define TDC1000_ERR_NONE        0
#define TDC1000_ERR_SIG_HIGH    1
#define TDC1000_ERR_NO_SIG      2
#define TDC1000_ERR_SIG_WEAK    3

// Following are TDC1000 operation modes
#define REGULAR_TOF        0
#define REGULAR_TMP        1
#define AUTO_FLOW          2
#define MANUAL_FLOW        3

#define ENABLED            1
#define DISABLED           0

extern uint8_t TDC7200_reg_local_copy[10];
extern uint8_t TDC1000_reg_local_copy[10];

extern uint8_t upStreamBuf[BUF_LENGTH];
extern uint8_t downStreamBuf[BUF_LENGTH];
extern uint8_t RTDBuf[BUF_LENGTH];

extern volatile uint16_t tdc_state_flag;
extern volatile uint16_t count_per_temp;
extern volatile uint16_t count_meassure_temp;

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define __TARGET_LITTLE_ENDIAN
#elif __BYTE_ORDER == __BIG_ENDIAN
#define __TARGET_BIG_ENDIAN
#else
#error "unknown byte order"
#endif

#ifdef HAVE_CACHE
#define likely(x) __builtin_exp ect(!!(x), 1)
#define unlikely(x) __builtin_exp ect(!!(x), 0)
#else
#define likely(x) (x)
#define unlikely(x) (x)
#endif

/**
  * @brief  tdc1000 register map.
  */
typedef enum {
    REG_CONFIG_0_ADDR = 0,
    REG_CONFIG_1_ADDR,
    REG_CONFIG_2_ADDR,
    REG_CONFIG_3_ADDR,
    REG_CONFIG_4_ADDR,
    REG_TOF_1_ADDR,
    REG_TOF_0_ADDR,
    REG_ERROR_FLAGS_ADDR,
    REG_TIMEOUT_ADDR,
    REG_CLOCK_RATE_ADDR,
    REG_NUM,
}TDC100_REG_ADDR_MAP;

/**
  * @brief  Frequency divider for TX clock and T1.
  */
typedef enum {
    TX_FREQ_DIV_2 = 0,
    TX_FREQ_DIV_4,
    TX_FREQ_DIV_8,
    TX_FREQ_DIV_16,
    TX_FREQ_DIV_32,
    TX_FREQ_DIV_64,
    TX_FREQ_DIV_128,
    TX_FREQ_DIV_256,
}TX_FREQ_DIV;

/**
  * @brief  Number of TX pulses in a burst, ranging from 0 to 31.
  */
typedef enum {
    TX_PULSES_MIN = 0,
    TX_PULSES_MAX = 31,
}NUM_TX_PULSES;

/**
  * @brief  Number of measurement cycles to average in stopwatch/MCU
  */
typedef enum {
    MEASUREMENT_CYCLE_1 = 0,
    MEASUREMENT_CYCLE_2,
    MEASUREMENT_CYCLE_4,
    MEASUREMENT_CYCLE_8,
    MEASUREMENT_CYCLE_16,
    MEASUREMENT_CYCLE_32,
    MEASUREMENT_CYCLE_64,
    MEASUREMENT_CYCLE_128,
}NUM_MEASUREMENT_CYCLE;

/**
  * @brief  Number of expected receive events.
  */
typedef enum {
    RX_EVENTS_MIN = 0,
    RX_EVENTS_MAX = 7,
}NUM_RX_EVENTS;

typedef enum {
    VCOM_SEL_INTERNAL = 0,
    VCOM_SEL_EXTERNAL = 1,
}COMMON_MODE_VREF;

typedef enum {
    TOF_MODE = 0,
    TEMP_MODE,
}MEASUREMENT_MODE;

typedef enum {
    CHANNEL_1 = 0,
    CHANNEL_2,
}CHANNEL_PAIR;

typedef enum {
    TOF_MEASUREMENT_MODE_0 = 0,
    TOF_MEASUREMENT_MODE_1,
    TOF_MEASUREMENT_MODE_2,
}TOF_MEASUREMENT_MODE;

typedef enum {
    TEMP_MODE_REF_RTD1_RTD2,
    TEMP_MODE_REF_RTD1,
}TEMP_MODE;

typedef enum {
    PT1000 = 0,
    PT500,
}RTD_TYPE;

typedef enum {
    TEMP_CLK_DIV_8 = 0,
    TEMP_CLK_USE_TX_FREQ_DIV,
}TEMP_CLK_DIV;

typedef enum {
    THLD_35mV = 0,
    THLD_50mV,
    THLD_75mV,
    THLD_125mV,
    THLD_220mV,
    THLD_410mV,
    THLD_775mV,
    THLD_1500mV,
}ECHO_QUAL_THLD;

typedef enum {
    SINGLE_ECHO = 0,
    MULTI_ECHO,
}RECEIVE_MODE;

typedef enum {
    TRIG_EDGE_RISING = 0,
    TRIG_EDGE_FALLING,
}TRIG_EDGE_POLARITY;

typedef enum {
    TX_PH_SHIFT_POS_MAX = 31,
}TX_PH_SHIFT_POS;

typedef enum {
    PGA_GAIN_0DB = 0,
    PGA_GAIN_3DB,
    PGA_GAIN_6DB,
    PGA_GAIN_9DB,
    PGA_GAIN_12DB,
    PGA_GAIN_15DB,
    PGA_GAIN_18DB,
    PGA_GAIN_21DB,
}PGA_GAIN;

typedef enum {
    LNA_FB_CAP = 0,
    LNA_FB_RES,
}LNA_FB;

typedef enum {
    TOF_BLANK_PERIOD_8T0 = 0,
    TOF_BLANK_PERIOD_16T0,
    TOF_BLANK_PERIOD_32T0,
    TOF_BLANK_PERIOD_64T0,
    TOF_BLANK_PERIOD_128T0,
    TOF_BLANK_PERIOD_256T0,
    TOF_BLANK_PERIOD_512T0,
    TOF_BLANK_PERIOD_1024T0,
}SHORT_TOF_BLANK_PERIOD;

typedef enum {
    TOF_TIMEOUT_128T0 = 0,
    TOF_TIMEOUT_256T0,
    TOF_TIMEOUT_512T0,
    TOF_TIMEOUT_1024T0,
}TOF_TIMEOUT_CTRL;

typedef enum {
    CLKIN_DIV_1 = 0,
    CLKIN_DIV_2,
}CLOCKIN_DIV;

typedef enum {
    AUTOZERO_PERIOD_64T0 = 0,
    AUTOZERO_PERIOD_128T0,
    AUTOZERO_PERIOD_256T0,
    AUTOZERO_PERIOD_512T0,
}AUTOZERO_PERIOD;

/**
  * @brief  CONFIG_0 register Structure definition
            reset = 45h
  */
struct config_0_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t tx_freq_div : 3;    /* Frequency divider for TX clock and T1. */
    uint8_t num_tx : 5;         /* Number of TX pulses in a burst, ranging from 0 to 31. */
#else
    uint8_t num_tx : 5;
    uint8_t tx_freq_div : 3;
#endif
};

/**
  * @brief  CONFIG_1 register Structure definition
            reset = 40h
  */
struct config_1_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t reserved : 2;      /* Reserved, default = 1h. */
    uint8_t num_avg : 3;       /* Number of measurement cycles to average in stopwatch/MCU. */
    uint8_t num_rx : 3;        /* Number of expected receive events */
#else
    uint8_t num_rx : 3;
    uint8_t num_avg : 3;
    uint8_t reserved : 2;
#endif
};

/**
  * @brief  CONFIG_2 register Structure definition
            reset = 0h
  */
struct config_2_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t vcom_sel : 1;          /* Common-mode voltage reference control */
    uint8_t meas_mode : 1;         /* AFE measurement type */
    uint8_t damping : 1;           /* TX burst damping */
    uint8_t ch_swp : 1;            /* Automatic channel swap in Mode 2 of operation.
                                      The setting is ignored if EXT_CHSEL = 1. */
    uint8_t ext_chsel : 1;         /* External channel select by CHSEL pin. */
    uint8_t ch_sel : 1;            /* Active TX/RX channel pair. */
    uint8_t tof_meas_mode : 2;     /* Time-of-flight measurement mode. */
#else
    uint8_t tof_meas_mode : 2;
    uint8_t ch_sel : 1;
    uint8_t ext_chsel : 1;
    uint8_t ch_swp : 1;
    uint8_t damping : 1;
    uint8_t meas_mode : 1;
    uint8_t vcom_sel : 1;
#endif
};

/**
  * @brief  CONFIG_3 register Structure definition
  *         reset = 3h
  */
struct config_3_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t reserved : 1;         /* Reserved, default = 0h */
    uint8_t temp_mode : 1;        /* Temperature measurement channels */
    uint8_t temp_rtd_sel : 1;     /* RTD type, PT1000 or PT500 */
    uint8_t temp_clk_div : 1;     /* Clock divider for temperature mode */
    uint8_t blanking : 1;         /* Power blanking in standard TOF measurements.
                                     The blanking length is controlled with the TIMING_REG field */
    uint8_t echo_qual_thld : 3;   /* Echo qualification DAC threshold level with respect to VCOM */
#else
    uint8_t echo_qual_thld : 3;
    uint8_t blanking : 1;
    uint8_t temp_clk_div : 1;
    uint8_t temp_rtd_sel : 1;
    uint8_t temp_mode : 1;
    uint8_t reserved : 1;
#endif
};

/**
  * @brief  CONFIG_4 register Structure definition
  *         reset = 1Fh
  */
struct config_4_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t reserved : 1;            /* Reserved, default = 0h */
    uint8_t receive_mode : 1;        /* Receive echo mode */
    uint8_t trig_edge_polarity : 1;  /* Trigger edge polarity. */
    uint8_t tx_ph_shift_pos : 5;     /* TX 180бу pulse shift position, ranging from 0 to 31. */
#else
    uint8_t tx_ph_shift_pos : 5;
    uint8_t trig_edge_polarity : 1;
    uint8_t receive_mode : 1;
    uint8_t reserved : 1;
#endif
};

/**
  * @brief  TOF_1 register Structure definition
  *         reset = 0h
  */
struct tof_1_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t pga_gain : 3;            /* PGA gain */
    uint8_t pga_ctrl : 1;            /* PGA control, 0:Active, 1:Bypassed and powered off */
    uint8_t lna_ctrl : 1;            /* Low noise amplifier control, 0:Active, 1:Bypassed and powered off */
    uint8_t lna_fb : 1;              /* Low noise amplifier feedback mode */
    uint8_t timing_reg_9_8 : 2;      /* TIMING_REG field's 2 most-significant bits */
#else
    uint8_t timing_reg_9_8 : 2;
    uint8_t lna_fb : 1;
    uint8_t lna_ctrl : 1;
    uint8_t pga_ctrl : 1;
    uint8_t pga_gain : 3;
#endif
};

/**
  * @brief  TOF_0 register Structure definition
  */
struct tof_0_register {
    uint8_t timing_reg_7_0 : 8;      /* TIMING_REG field's 8 least-significant bits */
};

/**
  * @brief  ERROR_FLAGS register Structure definition
  *         reset = 0h
  */
struct error_flags_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t reserved : 5;            /* Reserved (default = 0h) */
    uint8_t err_sig_weak : 1;        /* The number of received and qualified zero-crossings was less than
                                        the expected number set in NUM_RX field and a timeout occurred */
    uint8_t err_no_sig : 1;          /* No signals were received and timeout occurred. */
    uint8_t err_sig_high : 1;        /* The received echo amplitude exceeds the largest echo qualification
                                        threshold at the input of the comparators. The error is only reported
                                        when ECHO_QUAL_THLD = 0x07. */
#else
    uint8_t err_sig_high : 1;
    uint8_t err_no_sig : 1;
    uint8_t err_sig_weak : 1;
    uint8_t reserved : 5;
#endif
};

/**
  * @brief  TIMEOUT register Structure definition
  *         reset = 19h
  */
struct timeout_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t reserved : 1;                /* Reserved, default = 0h */
    uint8_t force_short_tof : 1;         /* Short time-of-flight control */
    uint8_t short_tof_blank_period : 3;  /* Short time-of-flight blanking period */
    uint8_t echo_timeout : 1;            /* Echo receive timeout control */
    uint8_t tof_timeout_ctrl : 2;        /* Echo listening window timeout */
#else
    uint8_t tof_timeout_ctrl : 2;
    uint8_t echo_timeout : 1;
    uint8_t short_tof_blank_period : 3;
    uint8_t force_short_tof : 1;
    uint8_t reserved : 1;
#endif
};

/**
  * @brief  CLOCK_RATE register Structure definition
  *         reset = 9h
  */
struct clock_rate_register {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t reserved : 5;               /* Reserved, reset = 0h */
    uint8_t clockin_div : 1;            /* CLKIN divider to generate T0 */
    uint8_t autozero_period : 2;        /* Receiver auto-zero period */
#else
    uint8_t autozero_period : 2;
    uint8_t clockin_div : 1;
    uint8_t reserved : 5;
#endif
};

/**
  * @brief  TDC1000 register Structure definition
  */
struct ti_tdc1000_register {
#ifdef __TARGET_BIG_ENDIAN
    struct config_0_register cfg_0;
    struct config_1_register cfg_1;
    struct config_2_register cfg_2;
    struct config_3_register cfg_3;
    struct config_4_register cfg_4;
    struct tof_1_register tof_1;
    struct tof_0_register tof_0;
    struct error_flags_register error;
    struct timeout_register timeout;
    struct clock_rate_register clk_rate;
#else
    struct clock_rate_register clk_rate;
    struct timeout_register timeout;
    struct error_flags_register error;
    struct tof_0_register tof_0;
    struct tof_1_register tof_1;
    struct config_4_register cfg_4;
    struct config_3_register cfg_3;
    struct config_2_register cfg_2;
    struct config_1_register cfg_1;
    struct config_0_register cfg_0;
#endif
};

void Init_TDC1000_TDC7200(void);
void tdc1000_reset(void);
void tdc7200_reset(void);
void tdc_enable_clock(void);
void tdc_disable_clock(void);
void tdc_trigger_measure(void);
void TDC1000_reg_init(void);
void TDC7200_reg_init(void);
void tdc_power_cycle_on(void);
void tdc_power_cycle_off(void);

#endif
