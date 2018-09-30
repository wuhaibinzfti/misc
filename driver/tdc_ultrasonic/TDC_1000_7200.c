
/**
 * @brief  TDC1000 pin map definition.
 */
/* HIGH:normal LOW:reset */
#define TDC1000_RESET_PIN_SET_HIGH
#define TDC1000_RESET_PIN_SET_LOW

#define TDC1000_EN_PIN_SET_HIGH
#define TDC1000_EN_PIN_SET_LOW

#define TDC1000_CS_PIN_SET_HIGH
#define TDC1000_CS_PIN_SET_LOW

/* TRIGGER is controlled by TDC7200 */
#define TDC1000_TRIGGER_PIN_SET_HIGH
#define TDC1000_TRIGGER_PIN_SET_LOW

#define TDC1000_CHSEL_PIN_SET_HIGH
#define TDC1000_CHSEL_PIN_SET_LOW

#define TDC1000_ERRB_PIN_GET

#define TDC1000_SPI_SDO_PIN_SET_HIGH
#define TDC1000_SPI_SDO_PIN_SET_LOW

#define TDC1000_SPI_SDI_PIN_GET

#define TDC1000_SPI_CSB_PIN_SET_HIGH
#define TDC1000_SPI_CSB_PIN_SET_LOW

#define TDC1000_SPI_SCLK_PIN_SET_HIGH
#define TDC1000_SPI_SCLK_PIN_SET_LOW

/* TDC7200 PIN map */
#define TDC7200_EN_PIN_ENABLE
#define TDC7200_EN_PIN_DISABLE

#define TDC7200_CS_PIN_ENABLE     /* cs low: active */
#define TDC7200_CS_PIN_DISABLE    /* cs high */

static uint8_t ti_tdc100_register_config[10] = {0x49, 0x47, 0x12, 0x02, 0x1F, 0x88, 0x19, 0x01, 0x23, 0x01};

uint8_t TDC7200_reg_local_copy[10] = {0x02, 0x44, 0x07, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x0, 0x0 };
uint8_t TDC1000_reg_local_copy[10] = {0x49, 0x47, 0x12, 0x02, 0x1F, 0x88, 0x19, 0x01, 0x23, 0x01};

uint8_t upStreamBuf[BUF_LENGTH];
uint8_t downStreamBuf[BUF_LENGTH];
uint8_t RTDBuf[BUF_LENGTH];

uint8_t TDC1000_Err_Flag, TDC7200_Err_Flag;

volatile uint8_t MEASURE_MODE;
volatile uint16_t tdc_state_flag = TDC_DEFAULT_FLAG;
volatile uint16_t count_per_temp = 0;
volatile uint16_t count_meassure_temp = 0;

struct ti_tdc1000_register tdc1000_register {
    .cfg_0 = {
        .tx_freq_div = TX_FREQ_DIV_8,
        .num_tx = 5,
    },
    .cfg_1 = {
        .reserved = 1,
        .num_avg = MEASUREMENT_CYCLE_1,
        .num_rx = 0,
    },
    .cfg_2 = {
        .vcom_sel = VCOM_SEL_EXTERNAL,
        .meas_mode = TOF_MODE,
        .damping = true,
        .ch_swp = false,
        .ext_chsel = false,
        .ch_sel = CHANNEL_1,
        .tof_meas_mode = TOF_MEASUREMENT_MODE_0,
    },
    .cfg_3 = {
        .reserved = 0,
        .temp_mode = TEMP_MODE_REF_RTD1_RTD2,
        .temp_rtd_sel = PT1000,
        .temp_clk_div = TEMP_CLK_DIV_8,
        .blanking = true,
        .echo_qual_thld = THLD_125mV,
    },
    .cfg_4 = {
        .reserved = 0,
        .receive_mode = MULTI_ECHO,
        .trig_edge_polarity = TRIG_EDGE_RISING,
        .tx_ph_shift_pos = TX_PH_SHIFT_POS_MAX,
    },
    .tof_1 = {
        .pga_gain = PGA_GAIN_0DB,
        .pga_ctrl = true,
        .lna_ctrl = true,
        .lna_fb = LNA_FB_CAP,
        .timing_reg_9_8 = 0,
    },
    .tof_0 = {
        .timing_reg_7_0 = 0,
    },
    .error = {
        .reserved = 0,
        .err_sig_weak = 0,
        .err_no_sig = 0,
        .err_sig_high = 0,
    },
    .timeout = {
        .reserved = 0,
        .force_short_tof = false,
        .short_tof_blank_period = TOF_BLANK_PERIOD_8T0,
        .echo_timeout = false,
        .tof_timeout_ctrl = TOF_TIMEOUT_128T0,
    },
    .clk_rate = {
        .reserved = 0,
        .clockin_div = CLKIN_DIV_1,
        .autozero_period = AUTOZERO_PERIOD_64T0,
    },
};

/**
 * Platform interface
 */
static inline void ti_tdc1000_delay_us(uint32_t usec)
{
    return;
}

static inline int ti_tdc1000_spi_read(uint8_t address, uint8_t *buffer, uint8_t len)
{
    return 0;
}

static inline int ti_tdc1000_spi_write(uint8_t address, uint8_t *buffer, uint8_t len)
{
    return 0;
}

/**
 * Divide the clock source which is connected to the CLKIN pin
 * down to the resonant frequency of the transducer used.
 * f(pulse) = fCLKIN / (2 ^ (TX_FREQ_DIV + 1))
 */
int ti_tdc1000_set_tx_frequency_divider(enum TX_FREQ_DIV freq_div)
{
    struct config_0_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_0_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (freq_div == old.tx_freq_div) {
        return 0;
    } else {
        old.tx_freq_div = freq_div;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_0_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_tx_pulse_number(uint8_t num_tx)
{
    struct config_0_register old;

    if (num_tx > TX_PULSES_MAX) {
        return -1;
    }

    if (ti_tdc1000_spi_read(REG_CONFIG_0_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (num_tx == old.num_tx) {
        return 0;
    } else {
        old.num_tx = num_tx;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_0_ADDR, (uint8_t *)&old, 1);
}

/**
 * Perform the average of multiple TOF measurement cycles.
 */
int ti_tdc1000_set_measurement_cycle_number(NUM_MEASUREMENT_CYCLE num_avg)
{
    struct config_1_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (num_avg == old.num_avg) {
        return 0;
    } else {
        old.num_avg = num_avg;
    }

    return return ti_tdc1000_spi_write(REG_CONFIG_1_ADDR, (uint8_t *)&old, 1);
}

/**
 * The event manager controls the maximum number of STOP pulses
 * to generate on the STOP pin. The number of STOP pulses can be
 * use this function to configure.
 */
int ti_tdc1000_set_rx_event_number(uint8_t num_rx)
{
    struct config_1_register old;

    if (num_rx > RX_EVENTS_MAX) {
        return -1;
    }

    if (ti_tdc1000_spi_read(REG_CONFIG_1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (num_rx == old.num_rx) {
        return 0;
    } else {
        old.num_rx = num_rx;
    }

    return return ti_tdc1000_spi_write(REG_CONFIG_1_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_common_mode_vref(enum COMMON_MODE_VREF vcom_sel)
{
    struct config_2_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (vcom_sel == old.vcom_sel) {
        return 0;
    } else {
        old.vcom_sel = vcom_sel;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_measurement_mode(MEASUREMENT_MODE mode)
{
    struct config_2_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (mode == old.meas_mode) {
        return 0;
    } else {
        old.meas_mode = mode;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_tx_burst_damping(bool enable)
{
    struct config_2_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.damping) {
        return 0;
    } else {
        old.damping = enable;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_channel_swap(bool enable)
{
    struct config_2_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.ch_swp) {
        return 0;
    } else {
        old.ch_swp = enable;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_external_channel(bool enable)
{
    struct config_2_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.ext_chsel) {
        return 0;
    } else {
        old.ext_chsel = enable;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_tx_rx_channel_pair(enum CHANNEL_PAIR channel)
{
    struct config_2_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (channel == old.ch_sel) {
        return 0;
    } else {
        old.ch_sel = channel;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_tof_measurement_mode(TOF_MEASUREMENT_MODE mode)
{
    struct config_2_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (mode == old.tof_meas_mode) {
        return 0;
    } else {
        old.tof_meas_mode = mode;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_temperature_mode(TEMP_MODE mode)
{
    struct config_3_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (mode == old.temp_mode) {
        return 0;
    } else {
        old.temp_mode = mode;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_rtd_type(RTD_TYPE rtd)
{
    struct config_3_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (rtd == old.temp_rtd_sel) {
        return 0;
    } else {
        old.temp_rtd_sel = rtd;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_temp_clock_divider(TEMP_CLK_DIV div)
{
    struct config_3_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (div == old.temp_clk_div) {
        return 0;
    } else {
        old.temp_clk_div = div;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_power_blanking(bool enable)
{
    struct config_3_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.blanking) {
        return 0;
    } else {
        old.blanking = enable;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_echo_qual_dac_threshold(ECHO_QUAL_THLD thld)
{
    struct config_3_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (thld == old.echo_qual_thld) {
        return 0;
    } else {
        old.echo_qual_thld = thld;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_3_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_receive_echo_mode(RECEIVE_MODE mode)
{
    struct config_4_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (mode == old.receive_mode) {
        return 0;
    } else {
        old.receive_mode = mode;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_trigger_edge_polarity(TRIG_EDGE_POLARITY edge)
{
    struct config_4_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (edge == old.trig_edge_polarity) {
        return 0;
    } else {
        old.trig_edge_polarity = edge;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1);
}

/**
 * Add a 180 shift at a position in the TX signal.
 * NUM_TX = 0x07, TX_PH_SHIFT_POS = 0x03.
 * An example for pulse shift:
 *      _   _   _   _   _   _   _
 *  ___| |_| |_| |_| |_| |_| |_| |__     -->normal pulse
 *      _   _   _     _   _   _
 *  ___| |_| |_| |___| |_| |_| |____     -->add a 180 shift pulse
 *
 * pos | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
 */
int ti_tdc1000_set_tx_pulse_shift_position(uint8_t position)
{
    struct config_4_register old;

    if (position > TX_PH_SHIFT_POS_MAX) {
        return -1;
    }

    if (ti_tdc1000_spi_read(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (position == old.tx_ph_shift_pos) {
        return 0;
    } else {
        old.tx_ph_shift_pos = position;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_pga_gain(PGA_GAIN gain)
{
    struct tof_1_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (gain == old.pga_gain) {
        return 0;
    } else {
        old.pga_gain = gain;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_pga_control(bool enable)
{
    struct tof_1_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.pga_ctrl) {
        return 0;
    } else {
        old.pga_gain = enable;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_lna_control(bool enable)
{
    struct tof_1_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.lna_ctrl) {
        return 0;
    } else {
        old.lna_ctrl = enable;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_lna_feedback_mode(LNA_FB mode)
{
    struct tof_1_register old;

    if (ti_tdc1000_spi_read(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (mode == old.lna_fb) {
        return 0;
    } else {
        old.lna_fb = mode;
    }

    return ti_tdc1000_spi_write(REG_CONFIG_4_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_timing(uint16_t timing)
{
    struct tof_0_register tof0;
    struct tof_1_register tof1;
    uint8_t timing_reg_9_8, timing_reg_7_0;

    if (timing > 0x3ff) {
        return -1;
    }

    timing_reg_9_8 = (uint8_t)(timing >> 8 & 0x03);
    timing_reg_7_0 = (uint8_t)(timing & 0xff);

    if (ti_tdc1000_spi_read(REG_TOF_1_ADDR, (uint8_t *)&tof1, 1) < 0) {
        return -1;
    }
    if (ti_tdc1000_spi_read(REG_TOF_0_ADDR, (uint8_t *)&tof0, 1) < 0) {
        return -1;
    }
    if (timing_reg_9_8 != tof1.timing_reg_9_8) {
        tof1.timing_reg_9_8 = timing_reg_9_8;
        if (ti_tdc1000_spi_write(REG_TOF_1_ADDR, (uint8_t *)&tof1, 1)) {
            return -1;
        }
    }
    if (timing_reg_7_0 != tof0.timing_reg_7_0) {
        tof1.timing_reg_7_0 = timing_reg_7_0;
        if (ti_tdc1000_spi_write(REG_TOF_0_ADDR, (uint8_t *)&tof0, 1)) {
            return -1;
        }
    }

    return 0;
}

int ti_tdc1000_get_error_flags()
{
    struct error_flags_register old;

    if (ti_tdc1000_spi_read(REG_ERROR_FLAGS_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }

    if (old.err_sig_weak) {
        return TDC1000_ERR_SIG_HIGH;
    } else if (old.err_no_sig) {
        return TDC1000_ERR_NO_SIG;
    } else if (old.err_sig_high) {
        return TDC1000_ERR_SIG_HIGH;
    }

    return TDC1000_ERR_NONE;
}

int ti_tdc1000_clear_all_error(void)
{
    struct error_flags_register reg;

    reg.reserved = 0;
    reg.err_sig_weak = 0;
    reg.err_no_sig = 0;
    reg.err_sig_high = 0;
    return ti_tdc1000_spi_write(REG_ERROR_FLAGS_ADDR, (uint8_t *)&reg, 1);
}

int ti_tdc1000_set_short_tof_control(bool enable)
{
    struct timeout_register old;

    if (ti_tdc1000_spi_read(REG_TIMEOUT_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.force_short_tof) {
        return 0;
    } else {
        old.force_short_tof = enable;
    }

    return ti_tdc1000_spi_write(REG_TIMEOUT_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_short_tof_blanking_period(SHORT_TOF_BLANK_PERIOD period)
{
    struct timeout_register old;

    if (ti_tdc1000_spi_read(REG_TIMEOUT_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (period == old.short_tof_blank_period) {
        return 0;
    } else {
        old.force_short_tof = period;
    }

    return ti_tdc1000_spi_write(REG_TIMEOUT_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_echo_receive_timeout(bool enable)
{
    struct timeout_register old;

    if (ti_tdc1000_spi_read(REG_TIMEOUT_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.echo_timeout) {
        return 0;
    } else {
        old.echo_timeout = enable;
    }

    return ti_tdc1000_spi_write(REG_TIMEOUT_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_echo_listening_window_timeout(TOF_TIMEOUT_CTRL timeout)
{
    struct timeout_register old;

    if (ti_tdc1000_spi_read(REG_TIMEOUT_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (timeout == old.tof_timeout_ctrl) {
        return 0;
    } else {
        old.tof_timeout_ctrl = timeout;
    }

    return ti_tdc1000_spi_write(REG_TIMEOUT_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_clkin_divider(CLOCKIN_DIV div)
{
    struct clock_rate_register old;

    if (ti_tdc1000_spi_read(REG_CLOCK_RATE_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (div == old.clockin_div) {
        return 0;
    } else {
        old.clockin_div = div;
    }

    return ti_tdc1000_spi_write(REG_CLOCK_RATE_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_set_receiver_auto_zero_period(AUTOZERO_PERIOD period)
{
    struct clock_rate_register old;

    if (ti_tdc1000_spi_read(REG_CLOCK_RATE_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (period == old.autozero_period) {
        return 0;
    } else {
        old.autozero_period = period;
    }

    return ti_tdc1000_spi_write(REG_CLOCK_RATE_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc1000_trigger_measure()
{
	uint8_t temp1, temp2;

	if (tdc_state_flag & (TDC_CONTINUOUS_TRIGGER_STATE + TDC_TOF_GRAPTH_STATE + TDC_SINGLE_SHOT_MEASURE_STATE)) {
        /* Perform power cycle if enabled */
        if (tdc_state_flag & TDC_POWER_CYCLE_FLAG) {
            tdc_power_cycle_on();
        }

        // Clear error flags and reset state machine
        TDC1000_SPIByteWriteReg(TDC1000_ERROR_FLAGS_REG, 0x03);
        TDC7200_SPIByteWriteReg(TDC7200_INTRPT_STATUS_REG, 0x1F);

        // Measure for up stream
        tdc_trigger_common(upStreamBuf);

        if (tdc_state_flag & (TDC_TOF_GRAPTH_STATE + TDC_SINGLE_SHOT_MEASURE_STATE)) {
            tdc_state_flag |= TDC_UP_STREAM_BUFFER_READY;
        }

        // TDC1000 Enable Pin toggling effect: during AUTO_FLOW state m/c is reset
        // need to make downstream measurement also before shutting down
        if (MEASURE_MODE == AUTO_FLOW) {
            // Give delay for transducer to settle down
            delay_ACLK(17);
            tdc_trigger_common(downStreamBuf);
            if (tdc_state_flag & TDC_TOF_GRAPTH_STATE) {
                tdc_state_flag |= TDC_DOWN_STREAM_BUFFER_READY;
            }
        }

        // Temperature measurement
        if (tdc_state_flag & TDC_INTERLEAVED_TEMP_MEASURE) {
            if (!(--count_meassure_temp)) {
                // Enable Temperature measurement bit
                temp1 = TDC1000_SPIByteReadReg(TDC1000_CONFIG2_REG);
                TDC1000_SPIByteWriteReg(TDC1000_CONFIG2_REG, temp1 | 0x40);

                // config TDC7200 for 5 stops
                // clear last 6 bits: both num stops and avgng cycles
                temp2 = TDC7200_SPIByteReadReg(TDC7200_CONFIG2_REG);
                TDC7200_SPIByteWriteReg(TDC7200_CONFIG2_REG, (temp2 & 0xC0) | 0x04);

                tdc_trigger_common(RTDBuf);
                RTDBuf[39] = 0xA5;

                // Restore register
                TDC1000_SPIByteWriteReg(TDC1000_CONFIG2_REG, temp1);
                TDC7200_SPIByteWriteReg(TDC7200_CONFIG2_REG, temp2);

                count_meassure_temp = count_per_temp;
                tdc_state_flag |= TDC_RTD_BUFFER_READY;
            }
        }

        if (tdc_state_flag & TDC_POWER_CYCLE_FLAG) {
            tdc_power_cycle_off();
        }
	}
}

void ti_tdc1000_hw_reset(void)
{
    TDC1000_RESET_PIN_SET_LOW;
    /* wait for 10usec */
    ti_tdc1000_delay_us(10);
    TDC1000_RESET_PIN_SET_HIGH;
    ti_tdc1000_delay_us(10);

    return;
}

int ti_tdc1000_register_init(void)
{
    return ti_tdc1000_spi_write(REG_CONFIG_0_ADDR, (uint8_t *)&tdc1000_register, 10);
}

/**
 * Board config for TDC1000.
 */
int ti_tdc1000_board_init(void)
{
    /* TDC1000 pin initial state configure */
    TDC1000_RESET_PIN_SET_HIGH;
    TDC1000_TRIGGER_PIN_SET_LOW;
    TDC1000_EN_PIN_SET_HIGH;
    TDC1000_CHSEL_PIN_SET_HIGH;

    ti_tdc1000_hw_reset();

    return;
}

/**********************************************************************************************************/









void ti_tdc7200_hw_reset(void)
{
    TDC7200_ENABLE_PxOUT &= ~TDC7200_ENABLE_PIN;
    tdc_disable_clock();
    // wait for 1sec for the cap to discharge
    delay_ACLK(24576);
    TDC7200_ENABLE_PxOUT |= TDC7200_ENABLE_PIN;
    /* give at least 500us */
    nrf_delay_us(500);
    tdc_enable_clock();

    // Unset "Start New Measurement" bit to avoid measurement being triggered
    TDC7200_reg_local_copy[TDC7200_CONFIG1_REG] &= ~0x01;
    TDC7200_SPIAutoIncWriteReg(TDC7200_CONFIG1_REG, TDC7200_reg_local_copy, TDC7200_TOTAL_NUM_CONFIG_REG);
}

void ti_tdc1000_register_init(void)
{
    unsigned int i;

    for (i = 0; i < 10; i++) {
        TDC1000_SPIByteWriteReg(i, TDC1000_reg_local_copy[i]);    /* write in TDC1000 Registers */
    }
    MEASURE_MODE = TDC1000_reg_local_copy[2];
    MEASURE_MODE &= 0x03;

    return;
}

void ti_tdc7200_register_init(void)
{
    TDC7200_SPIAutoIncWriteReg(TDC7200_CONFIG1_REG, TDC7200_reg_local_copy, TDC7200_TOTAL_NUM_CONFIG_REG);
}

void ti_tdc1000_tdc7200_init(void)
{
    //UCB1CTLW0 &= ~UCSWRST;                             /* Switch on SPI module */
    ti_tdc1000_hw_reset();                               /* Reset TDC_1000 */

    TDC1000_EN_PIN_ENABLE;                               /* Enable TDC1000, into ready mode */
    TDC7200_EN_PIN_ENABLE;                               /* Enable TDC7200 */
    nrf_delay_us(TDC7200_WAKEUP_PERIOD);                 /* Reset TDC7200 */

    TDC1000_CS_PIN_DISABLE;                              /* TDC1000 chip disable for SPI */
    TDC7200_CS_PIN_DISABLE;		                     /* TDC7200 chip disable for SPI */

    ti_tdc1000_register_init();
    ti_tdc7200_register_init();

    //UCB1CTLW0 |= UCSWRST;                             /* Switch oFF SPI module */
    TDC1000_CS_PIN_ENABLE;
    TDC7200_CS_PIN_ENABLE;

    TDC1000_EN_PIN_DISABLE;                             /* Disable TDC1000 */
    TDC7200_EN_PIN_DISABLE;                             /* Disable TDC7200 */

    return;
}

uint8_t ti_tdc1000_get_mode(void)
{
    uint8_t mch, chswp, meas_mode, bdata, mx_sel, device_mode;

    bdata = TDC1000_SPIByteReadReg(TDC1000_CONFIG2_REG);
    meas_mode = bdata & 0x03;
    mch = (bdata & 0x08)>>3;
    chswp = (bdata & 0x10)>>4;
    mx_sel = (bdata & 0x40)>>6;

    if (mx_sel == TOF_MEASUREMENT) {
        if ((chswp == ENABLED) && (mch == DISABLED) && (meas_mode == MULTI_CYCLE)) {
            device_mode = AUTO_FLOW;
        } else if (mch == ENABLED) {
            device_mode = MANUAL_FLOW;
        } else {
            device_mode = REGULAR_TOF;
        }
    } else {
        device_mode = REGULAR_TMP;
    }

    return device_mode;
}

void tdc_enable_clock(void)
{
#ifdef USE_OSC
	TDC1000_OSCENABLE_PxOUT |= TDC1000_OSCENABLE_PIN;		// Set pin high: enable afe osc
#else
	// Output SMCLK
	TDC1000_XCLK_PxSEL0	|= TDC1000_XCLK_PIN;
	TDC1000_XCLK_PxSEL1	|= TDC1000_XCLK_PIN;
	TDC1000_XCLK_PxDIR	|= TDC1000_XCLK_PIN;
#endif
}

void tdc_disable_clock(void)
{
#ifdef USE_OSC
	TDC1000_OSCENABLE_PxOUT &= ~TDC1000_OSCENABLE_PIN;		// Set pin high: enable afe osc
#else
	// Output SMCLK
	TDC1000_XCLK_PxOUT	&= ~TDC1000_XCLK_PIN;
	TDC1000_XCLK_PxSEL0	&= ~TDC1000_XCLK_PIN;
	TDC1000_XCLK_PxSEL1	&= ~TDC1000_XCLK_PIN;
	TDC1000_XCLK_PxDIR	|= TDC1000_XCLK_PIN;
#endif

}

void tdc_power_cycle_on(void)
{
	// enable external osc before starting measurement
	tdc_enable_clock();

	delay_ACLK(98);												// use default delay if input is 0

	TDC1000_ENABLE_PxOUT |= TDC1000_ENABLE_PIN;					// enable afe
	TDC7200_ENABLE_PxOUT |= TDC7200_ENABLE_PIN;					// Enable device
	delay_uS(TDC7200_WAKEUP_PERIOD);							// wait for TDC7200 wakeup delay

	// Unset "Start New Measurement" bit to avoid measurement being triggered
	TDC7200_reg_local_copy[TDC7200_CONFIG1_REG] &= ~0x01;

	// init TDC7200 with local saved data
	TDC7200_SPIAutoIncWriteReg(TDC7200_CONFIG1_REG, TDC7200_reg_local_copy, TDC7200_TOTAL_NUM_CONFIG_REG);
}

void tdc_power_cycle_off(void)
{
	// Backup register values before disable
	TDC7200_SPIAutoIncReadReg(TDC7200_CONFIG1_REG, TDC7200_reg_local_copy, TDC7200_TOTAL_NUM_CONFIG_REG);

	// Disable device
	TDC7200_ENABLE_PxOUT &= ~TDC7200_ENABLE_PIN;
	TDC1000_ENABLE_PxOUT &= ~TDC1000_ENABLE_PIN;				// disable afe

	// disable osc after completing measurement
	tdc_disable_clock();
}

void tdc_trigger_common(uint8_t *buf)
{
    uint8_t byte_data;

    byte_data = TDC7200_reg_local_copy[0];
    // set start measurement bit & use default mode or set by user
    byte_data |= 0x01;
    TDC7200_SPIByteWriteReg(TDC7200_CONFIG1_REG, byte_data);
    start_ms_timeout(500);
    //wait for INTB pin to go low
    TDC7200_INTB_PxIFG &= ~TDC7200_INTB_PIN;
    TDC7200_INTB_PxIE |= TDC7200_INTB_PIN;
    LPM3;
    TDC7200_INTB_PxIE &= ~TDC7200_INTB_PIN;
    stop_ms_timeout();
    if (timeout) {
        __no_operation();
    }

    // Read all the result registers
    TDC7200_SPIAutoIncReadReg(TDC7200_TIME1_REG, buf, TDC7200_TOTAL_NUM_RESULT_REG);
    buf[39] = 0x00;

}

//******************************************************************************
void tdc_trigger_measure(void)
{
	uint8_t temp1, temp2;

	if (tdc_state_flag & (TDC_CONTINUOUS_TRIGGER_STATE + TDC_TOF_GRAPTH_STATE + TDC_SINGLE_SHOT_MEASURE_STATE)) {
        // Perform power cycle if enabled
        if (tdc_state_flag & TDC_POWER_CYCLE_FLAG) {
            tdc_power_cycle_on();
        }

        // Clear error flags and reset state machine
        TDC1000_SPIByteWriteReg(TDC1000_ERROR_FLAGS_REG, 0x03);
        TDC7200_SPIByteWriteReg(TDC7200_INTRPT_STATUS_REG, 0x1F);

        // Measure for up stream
        tdc_trigger_common(upStreamBuf);

        if (tdc_state_flag & (TDC_TOF_GRAPTH_STATE + TDC_SINGLE_SHOT_MEASURE_STATE)) {
            tdc_state_flag |= TDC_UP_STREAM_BUFFER_READY;
        }

        // TDC1000 Enable Pin toggling effect: during AUTO_FLOW state m/c is reset
        // need to make downstream measurement also before shutting down
        if (MEASURE_MODE == AUTO_FLOW) {
            // Give delay for transducer to settle down
            delay_ACLK(17);
            tdc_trigger_common(downStreamBuf);
            if (tdc_state_flag & TDC_TOF_GRAPTH_STATE) {
                tdc_state_flag |= TDC_DOWN_STREAM_BUFFER_READY;
            }
        }

        // Temperature measurement
        if (tdc_state_flag & TDC_INTERLEAVED_TEMP_MEASURE) {
            if (!(--count_meassure_temp)) {
                // Enable Temperature measurement bit
                temp1 = TDC1000_SPIByteReadReg(TDC1000_CONFIG2_REG);
                TDC1000_SPIByteWriteReg(TDC1000_CONFIG2_REG, temp1 | 0x40);

                // config TDC7200 for 5 stops
                // clear last 6 bits: both num stops and avgng cycles
                temp2 = TDC7200_SPIByteReadReg(TDC7200_CONFIG2_REG);
                TDC7200_SPIByteWriteReg(TDC7200_CONFIG2_REG, (temp2 & 0xC0) | 0x04);

                tdc_trigger_common(RTDBuf);
                RTDBuf[39] = 0xA5;

                // Restore register
                TDC1000_SPIByteWriteReg(TDC1000_CONFIG2_REG, temp1);
                TDC7200_SPIByteWriteReg(TDC7200_CONFIG2_REG, temp2);

                count_meassure_temp = count_per_temp;
                tdc_state_flag |= TDC_RTD_BUFFER_READY;
            }
        }

        if (tdc_state_flag & TDC_POWER_CYCLE_FLAG) {
            tdc_power_cycle_off();
        }
	}
}

// PORT1 Interrupt Vector (P1IV) handler
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
	switch( P2IV )
	{
	case 0: break;
	case 2: break;
	case 4:	break;
	case 6:	break;
	case 8: break;
	case 10: break;
	case 12: break;
	case 14:
		LPM3_EXIT;
		break;
	case 16: break;
	}
}

