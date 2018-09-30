
/**
 * Platform interface
 */
static inline void ti_tdc7200_delay_us(uint32_t usec)
{
    return;
}

static inline int ti_tdc7200_spi_read(uint8_t address, uint8_t *buffer, uint8_t len)
{
    return 0;
}

static inline int ti_tdc7200_spi_write(uint8_t address, uint8_t *buffer, uint8_t len)
{
    return 0;
}

void ti_tdc7200_hw_reset()
{
    return;
}

int ti_tdc7200_start_measurement(void)
{
    struct tdc7200_config1 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    old.start_meas = 1;

    return ti_tdc7200_spi_write(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_measurement_mode(TDC7200_MEAS_MODE mode)
{
    struct tdc7200_config1 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (mode == old.meas_mode) {
        return 0;
    }
    old.meas_mode = mode;

    return ti_tdc7200_spi_write(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_start_edge_polarity(TDC7200_START_EDGE edge)
{
    struct tdc7200_config1 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (edge == old.start_edge) {
        return 0;
    }
    old.start_edge = edge;

    return ti_tdc7200_spi_write(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_stop_edge_polarity(TDC7200_STOP_EDGE edge)
{
    struct tdc7200_config1 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (edge == old.stop_edge) {
        return 0;
    }
    old.stop_edge = edge;

    return ti_tdc7200_spi_write(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_trigger_edge_polarity(TDC7200_TRIGG_EDGE edge)
{
    struct tdc7200_config1 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (edge == old.stop_edge) {
        return 0;
    }
    old.stop_edge = edge;

    return ti_tdc7200_spi_write(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_parity(bool enable)
{
    struct tdc7200_config1 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.parity_en) {
        return 0;
    }
    old.parity_en = enable;

    return ti_tdc7200_spi_write(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_force_calibration(bool enable)
{
    struct tdc7200_config1 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (enable == old.force_cal) {
        return 0;
    }
    old.force_cal = enable;

    return ti_tdc7200_spi_write(TDC7200_CONFIG1_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_number_of_stops(uint8_t num)
{
    struct tdc7200_config2 old;

    if (num > MAX_STOPS) {
        return -1;
    }
    if (ti_tdc7200_spi_read(TDC7200_CONFIG2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (num == old.num_stop) {
        return 0;
    }
    old.num_stop = num;

    return ti_tdc7200_spi_write(TDC7200_CONFIG2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_average_cycles(TDC7200_AVG_CYCLES cycles)
{
    struct tdc7200_config2 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (cycles == old.avg_cycles) {
        return 0;
    }
    old.avg_cycles = cycles;

    return ti_tdc7200_spi_write(TDC7200_CONFIG2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_set_calibration2_periods(TDC7200_CALIBRATION2_PERIODS period)
{
    struct tdc7200_config2 old;

    if (ti_tdc7200_spi_read(TDC7200_CONFIG2_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }
    if (period == old.calibration_periods) {
        return 0;
    }
    old.avg_cycles = period;

    return ti_tdc7200_spi_write(TDC7200_CONFIG2_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_clear_new_measurement_interrupt(void)
{
    struct tdc7200_interrupt_status old;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }

    /* Requires writing a '1' to clear interrupt status. */
    old.new_meas_int = 1;
    return ti_tdc7200_spi_write(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_get_new_measurement_interrupt(void)
{
    struct tdc7200_interrupt_status stat;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&stat, 1) < 0) {
        return -1;
    }

    return stat.new_meas_int;
}

int ti_tdc7200_clear_coarse_counter_overflow_interrupt(void)
{
    struct tdc7200_interrupt_status old;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }

    /* Requires writing a '1' to clear interrupt status. */
    old.coarse_cntr_ovf_int = 1;
    return ti_tdc7200_spi_write(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_get_coarse_counter_overflow_interrupt(void)
{
    struct tdc7200_interrupt_status stat;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&stat, 1) < 0) {
        return -1;
    }

    return stat.coarse_cntr_ovf_int;
}

int ti_tdc7200_clear_clock_counter_overflow_interrupt(void)
{
    struct tdc7200_interrupt_status old;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }

    /* Requires writing a '1' to clear interrupt status. */
    old.clock_cntr_ovf_int = 1;
    return ti_tdc7200_spi_write(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_clear_measurement_start_flag(void)
{
    struct tdc7200_interrupt_status old;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }

    /* Requires writing a '1' to clear interrupt status. */
    old.meas_started_flag = 1;
    return ti_tdc7200_spi_write(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_get_measurement_start_flag(void)
{
    struct tdc7200_interrupt_status stat;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&stat, 1) < 0) {
        return -1;
    }

    return stat.meas_started_flag;
}

int ti_tdc7200_clear_measurement_complete_flag(void)
{
    struct tdc7200_interrupt_status old;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1) < 0) {
        return -1;
    }

    /* Requires writing a '1' to clear interrupt status. */
    old.meas_complete_flag = 1;
    return ti_tdc7200_spi_write(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&old, 1);
}

int ti_tdc7200_get_measurement_complete_flag(void)
{
    struct tdc7200_interrupt_status stat;

    if (ti_tdc7200_spi_read(TDC7200_INT_STATUS_REG_ADDR, (uint8_t *)&stat, 1) < 0) {
        return -1;
    }

    return stat.meas_complete_flag;
}

int ti_tdc7200_set_new_measurement_interrupt(bool enable)
{
    struct tdc7200_interrupt_mask mask;

    if (ti_tdc7200_spi_read(TDC7200_INT_MASK_REG_ADDR, (uint8_t *)&mask, 1) < 0) {
        return -1;
    }

    if (enable == mask.new_meas_mask) {
        return 0;
    }

    mask.new_meas_mask = enable;
    return ti_tdc7200_spi_write(TDC7200_INT_MASK_REG_ADDR, (uint8_t *)&mask, 1);
}

int ti_tdc7200_set_coarse_counter_overflow_interrupt(bool enable)
{
    struct tdc7200_interrupt_mask mask;

    if (ti_tdc7200_spi_read(TDC7200_INT_MASK_REG_ADDR, (uint8_t *)&mask, 1) < 0) {
        return -1;
    }

    if (enable == mask.coarse_cntr_ovf_mask) {
        return 0;
    }

    mask.coarse_cntr_ovf_mask = enable;
    return ti_tdc7200_spi_write(TDC7200_INT_MASK_REG_ADDR, (uint8_t *)&mask, 1);
}

int ti_tdc7200_set_clock_counter_overflow_interrupt(bool enable)
{
    struct tdc7200_interrupt_mask mask;

    if (ti_tdc7200_spi_read(TDC7200_INT_MASK_REG_ADDR, (uint8_t *)&mask, 1) < 0) {
        return -1;
    }

    if (enable == mask.clock_cntr_ovf_mask) {
        return 0;
    }

    mask.clock_cntr_ovf_mask = enable;
    return ti_tdc7200_spi_write(TDC7200_INT_MASK_REG_ADDR, (uint8_t *)&mask, 1);
}


int ti_tdc7200_set_coarse_counter_overflow()
{

}

int ti_tdc7200_get_coarse_counter_overflow()
{

}

int ti_tdc7200_set_clock_counter_overflow()
{

}

int ti_tdc7200_get_clock_counter_overflow()
{

}


