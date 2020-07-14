
#include "drv8301.hpp"
#include "utils.hpp"

#include "cmsis_os.h"
#include <math.h>
#include <array>
#include <algorithm>

Drv8301::Drv8301(Stm32SpiArbiter* spi_arbiter, Stm32Gpio ncs_gpio,
                 Stm32Gpio enable_gpio, Stm32Gpio nfault_gpio)
        : spi_arbiter_(spi_arbiter), ncs_gpio_(ncs_gpio),
          enable_gpio_(enable_gpio), nfault_gpio_(nfault_gpio)
{
    spi_task_.config = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_16BIT,
        .CLKPolarity = SPI_POLARITY_LOW,
        .CLKPhase = SPI_PHASE_2EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial = 10,
    };

    spi_task_.ncs_gpio = ncs_gpio_;
    spi_task_.tx_buf = (uint8_t*)&tx_buf_async_;
    spi_task_.rx_buf = (uint8_t*)&rx_buf_async_;
    spi_task_.length = 1;
    spi_task_.on_complete = [](void* ctx, bool success) { ((Drv8301*)ctx)->on_spi_complete(success); };
    spi_task_.cb_ctx = this;
    spi_task_.next = nullptr;
}

bool Drv8301::config(float requested_gain, float* actual_gain) {
    // Calculate gain setting: Snap down to have equal or larger range as
    // requested or largest possible range otherwise

    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    uint16_t gain_setting = 3;
    float gain_choices[] = {10.0f, 20.0f, 40.0f, 80.0f};
    while (gain_setting && (gain_choices[gain_setting] > requested_gain)) {
        gain_setting--;
    }

    if (actual_gain) {
        *actual_gain = gain_choices[gain_setting];
    }

    RegisterFile new_config;

    new_config.control_register_1 =
          (21 << 6) // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
        | (0b01 << 4) // OCP_MODE: latch shut down
        | (0b0 << 3) // 6x PWM mode
        | (0b0 << 2) // don't reset latched faults
        | (0b00 << 0); // gate-drive peak current: 1.7A

    new_config.control_register_2 =
          (0b0 << 6) // OC_TOFF: cycle by cycle
        | (0b00 << 4) // calibration off (normal operation)
        | (gain_setting << 2) // select gain
        | (0b00 << 0); // report both over temperature and over current on nOCTW pin

    bool regs_equal = (regs_.control_register_1 == new_config.control_register_1)
                   && (regs_.control_register_2 == new_config.control_register_2);

    if (!regs_equal) {
        regs_ = new_config;
        is_ready_ = false;
    }

    return true;
}

bool Drv8301::init() {
    uint16_t val;

    if (is_ready_) {
        return true;
    }

    // Reset DRV chip. The enable pin also controls the SPI interface, not only
    // the driver stages.
    enable_gpio_.write(false);
    delay_us(40); // mimumum pull-down time for full reset: 20us
    enable_gpio_.write(true);
    osDelay(20); // t_spi_ready, max = 10ms

    // Write current configuration
    bool did_write_regs = write_reg(kRegNameControl1, regs_.control_register_1)
                       && write_reg(kRegNameControl1, regs_.control_register_1)
                       && write_reg(kRegNameControl1, regs_.control_register_1)
                       && write_reg(kRegNameControl1, regs_.control_register_1)
                       && write_reg(kRegNameControl1, regs_.control_register_1)
                       && write_reg(kRegNameControl2, regs_.control_register_2)
                       && read_reg(kRegNameControl1, &val) && (val == regs_.control_register_1)
                       && read_reg(kRegNameControl2, &val) && (val == regs_.control_register_2);
    
    if (!did_write_regs) {
        return false;
    }

    if (get_error() != FaultType_NoFault) {
        return false;
    }

    read_reg(kRegNameControl1, &val);

    last_liveness_timestamp_ = next_liveness_timestamp_;
    is_ready_ = true;

    return true;
}

bool Drv8301::is_ready(uint32_t timestamp) {
    const uint32_t max_delay_us = 5000; // maximum age of the last successful liveness check

    if (!nfault_gpio_.read()) {
        is_ready_ = false;
    }

    if ((int32_t)(timestamp - last_liveness_timestamp_) < 0) {
        // liveness timestamp in the future (this can happen in some preemtion scenarios)
    } else if ((timestamp - last_liveness_timestamp_) > (uint32_t)((uint64_t)max_delay_us * (uint64_t)TIM_1_8_CLOCK_HZ / 1000000ULL)) {
        is_ready_ = false;
    }

    return is_ready_;
}

void Drv8301::update(uint32_t timestamp) {
    next_liveness_timestamp_ = timestamp;
    if (is_ready_) {
        if (!__atomic_exchange_n(&spi_task_active_, true, __ATOMIC_SEQ_CST)) {
            tx_buf_async_ = build_ctrl_word(DRV8301_CtrlMode_Read, kRegNameControl1, 0);
            rx_buf_async_ = 0;
            spi_arbiter_->transfer_async(&spi_task_);
        }
    }
}

Drv8301::FaultType_e Drv8301::get_error() {
    uint16_t fault1, fault2;

    if (!read_reg(kRegNameStatus1, &fault1) ||
        !read_reg(kRegNameStatus2, &fault2)) {
        return (FaultType_e)0xffffffff;
    }

    return (FaultType_e)((uint32_t)fault1 | ((uint32_t)(fault2 & 0x0080) << 16));
}

bool Drv8301::read_reg(const RegName_e regName, uint16_t* data) {
    tx_buf_sync_ = build_ctrl_word(DRV8301_CtrlMode_Read, regName, 0);
    if (!spi_arbiter_->transfer(spi_task_.config, ncs_gpio_, (uint8_t *)(&tx_buf_sync_), nullptr, 1, 1000)) {
        return false;
    }

    // Datasheet says you don't have to pulse the nCS between transfers, (16
    // clocks should commit the transfer) but for some reason you actually need
    // to pulse it.
    delay_us(1);

    tx_buf_sync_ = build_ctrl_word(DRV8301_CtrlMode_Read, regName, 0);
    rx_buf_sync_ = 0xffff;
    if (!spi_arbiter_->transfer(spi_task_.config, ncs_gpio_, (uint8_t *)(&tx_buf_sync_), (uint8_t *)(&rx_buf_sync_), 1, 1000)) {
        return false;
    }

    delay_us(1);

    if (rx_buf_sync_ == 0xbeef) {
        return false;
    }

    if (data) {
        *data = rx_buf_sync_ & 0x07FF;
    }
    
    return true;
}

bool Drv8301::write_reg(const RegName_e regName, const uint16_t data) {
    // Do blocking write
    tx_buf_sync_ = build_ctrl_word(DRV8301_CtrlMode_Write, regName, data);
    if (!spi_arbiter_->transfer(spi_task_.config, ncs_gpio_, (uint8_t *)(&tx_buf_sync_), nullptr, 1, 1000)) {
        return false;
    }
    delay_us(1);

    return true;
}

void Drv8301::on_spi_complete(bool success) {
    if ((rx_buf_async_ & 0x7ff) != regs_.control_register_1) {
        success = false;
    }

    if (success) {
        last_liveness_timestamp_ = next_liveness_timestamp_;
    }

    spi_task_active_ = false;
}
