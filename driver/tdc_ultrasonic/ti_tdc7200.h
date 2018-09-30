
/**
  * @brief  tdc1000 register map.
  */
typedef enum {
    TDC7200_CONFIG1_ADDR = 0,
    TDC7200_CONFIG2_ADDR,
    TDC7200_INT_STATUS_REG_ADDR,
    TDC7200_INT_MASK_REG_ADDR,
    TDC7200_COARSE_CNTR_OVF_H_REG_ADDR,
    TDC7200_COARSE_CNTR_OVF_L_REG_ADDR,
    TDC7200_CLOCK_CNTR_OVF_H_REG_ADDR,
    TDC7200_CLOCK_CNTR_OVF_L_REG_ADDR,
    TDC7200_CLOCK_CNTR_STOP_MASK_H_REG_ADDR,
    TDC7200_CLOCK_CNTR_STOP_MASK_L_REG_ADDR,
    TDC7200_TIME1_REG_ADDR,
    TDC7200_CLOCK_COUNTER1_REG_ADDR,
    TDC7200_TIME2_REG_ADDR,
    TDC7200_CLOCK_COUNTER2_REG_ADDR,
    TDC7200_TIME3_REG_ADDR,
    TDC7200_CLOCK_COUNTER3_REG_ADDR,
    TDC7200_TIME4_REG_ADDR,
    TDC7200_CLOCK_COUNTER4_REG_ADDR,
    TDC7200_TIME5_REG_ADDR,
    TDC7200_CLOCK_COUNTER5_REG_ADDR,
    TDC7200_TIME6_REG_ADDR,
    TDC7200_CALIBRATION1_REG_ADDR,
    TDC7200_CALIBRATION2_REG_ADDR,
    REG_NUM,
}TDC7200_REG_ADDR_MAP;

/**
  * @brief  tdc7200 measurement mode.
  */
typedef enum {
    MEAS_MODE_1 = 0,                  /* Measurement Mode 1 (for expected time-of-flight < 500 ns). */
    MEAS_MODE_2,                      /* Measurement Mode 2 (recommended) */
}TDC7200_MEAS_MODE;

/**
  * @brief  tdc7200 start signal edge.
  */
typedef enum {
    START_EDGE_RISING = 0,            /* Measurement is started on Rising edge of START signal. */
    START_EDGE_FALING,                /* Measurement is started on Falling edge of START signal. */
}TDC7200_START_EDGE;

/**
  * @brief  tdc7200 stop signal edge.
  */
typedef enum {
    START_EDGE_RISING = 0,            /* Measurement is stopped on Rising edge of STOP signal. */
    START_EDGE_FALING,                /* Measurement is stopped on Falling edge of STOP signal. */
}TDC7200_STOP_EDGE;

/**
  * @brief  tdc7200 trigger signal edge.
  */
typedef enum {
    TRIGG_EDGE_RISING = 0,            /* TRIGG is output as a Rising edge signal. */
    TRIGG_EDGE_FALING,                /* TRIGG is output as a Falling edge signal. */
}TDC7200_TRIGG_EDGE;

/**
  * @brief  tdc7200 stop signal number.
  */
typedef enum {
    MAX_STOPS = 5,
}TDC7200_NUM_STOP;

/**
  * @brief  tdc7200 average cycles.
  */
typedef enum {
    AVG_CYCLES_1,
    AVG_CYCLES_2,
    AVG_CYCLES_4,
    AVG_CYCLES_8,
    AVG_CYCLES_16,
    AVG_CYCLES_32,
    AVG_CYCLES_64,
    AVG_CYCLES_128,
}TDC7200_AVG_CYCLES;

/**
  * @brief  tdc7200 calibration2 periods.
  */
typedef enum {
    PERIODS_2_CLK,
    PERIODS_10_CLK,
    PERIODS_20_CLK,
    PERIODS_40_CLK,
}TDC7200_CALIBRATION2_PERIODS;

/**
  * @brief  CONFIG_1 register Structure definition
            reset = 00h
  */
struct tdc7200_config1 {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t force_cal : 1;
    uint8_t parity_en : 1;
    uint8_t trigg_edge : 1;
    uint8_t stop_edge : 1;
    uint8_t start_edge : 1;
    uint8_t meas_mode : 2;
    uint8_t start_meas : 1;
#else
    uint8_t start_meas : 1;
    uint8_t meas_mode : 2;            /* 00: Measurement Mode 1 (for expected time-of-flight < 500 ns). */
    uint8_t start_edge : 1;
    uint8_t stop_edge : 1;
    uint8_t trigg_edge : 1;
    uint8_t parity_en : 1;
    uint8_t force_cal : 1;
#endif
};

/**
  * @brief  CONFIG_2 register Structure definition
            reset = 40h
  */
struct tdc7200_config2 {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t calibration_periods : 2;
    uint8_t avg_cycles : 3;
    uint8_t num_stop : 3;
#else
    uint8_t num_stop : 3;
    uint8_t avg_cycles : 3;
    uint8_t calibration_periods : 2;
#endif
};

/**
  * @brief  INT_STATUS register Structure definition
            reset = 40h
  */
struct tdc7200_interrupt_status {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t reserved : 3;
    uint8_t meas_complete_flag : 1;
    uint8_t meas_started_flag : 1;
    uint8_t clock_cntr_ovf_int : 1;
    uint8_t coarse_cntr_ovf_int : 1;
    uint8_t new_meas_int : 1;
#else
    uint8_t new_meas_int : 1;
    uint8_t coarse_cntr_ovf_int : 1;
    uint8_t clock_cntr_ovf_int : 1;
    uint8_t meas_started_flag : 1;
    uint8_t meas_complete_flag : 1;
    uint8_t reserved : 3;
#endif
};

/**
  * @brief  INT_MASK register Structure definition
            reset = 07h
  */
struct tdc7200_interrupt_mask {
#ifdef __TARGET_BIG_ENDIAN
    uint8_t reserved : 5;
    uint8_t clock_cntr_ovf_mask : 1;
    uint8_t coarse_cntr_ovf_mask : 1;
    uint8_t new_meas_mask : 1;
#else
    uint8_t new_meas_mask : 1;
    uint8_t coarse_cntr_ovf_mask : 1;
    uint8_t clock_cntr_ovf_mask : 1;
    uint8_t reserved : 5;
#endif
};

/**
  * @brief  COARSE_CNTR_OVF_H register Structure definition
            reset = 04h
  */
struct coarse_cntr_ovf_register {
    uint8_t h;
    uint8_t l;
};

/**
  * @brief  COARSE_CNTR_OVF_H register Structure definition
            reset = 04h
  */
struct clock_cntr_ovf_register {
    uint8_t h;
    uint8_t l;
};

/**
  * @brief  COARSE_CNTR_OVF_H register Structure definition
            reset = 04h
  */
struct clock_cntr_stop_mask_register {
    uint8_t h;
    uint8_t l;
};

/**
  * @brief  COARSE_CNTR_OVF_H register Structure definition
            reset = 04h
  */
struct time_register {
#ifdef __TARGET_BIG_ENDIAN
    uint32_t reserved : 8;
    uint32_t parity : 1;
    uint32_t result : 23;
#else
    uint32_t result : 23;
    uint32_t parity : 1;
    uint32_t reserved : 8;
#endif
};

/**
  * @brief  COARSE_CNTR_OVF_H register Structure definition
            reset = 04h
  */
struct clock_count_register {
#ifdef __TARGET_BIG_ENDIAN
    uint32_t reserved : 8;
    uint32_t parity : 1;
    uint32_t not_used : 7;
    uint32_t result : 16;
#else
    uint32_t result : 16;
    uint32_t not_used : 7;
    uint32_t parity : 1;
    uint32_t reserved : 8;
#endif
};

/**
  * @brief  COARSE_CNTR_OVF_H register Structure definition
            reset = 04h
  */
struct calibration_register {
#ifdef __TARGET_BIG_ENDIAN
    uint32_t reserved : 8;
    uint32_t parity : 1;
    uint32_t calibration : 23;
#else
    uint32_t calibration : 23;
    uint32_t parity : 1;
    uint32_t reserved : 8;
#endif
}
