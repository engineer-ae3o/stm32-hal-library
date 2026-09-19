#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/spi_internals.h"
#include "utils/common.h"
#include "drivers/gpio.h"
#include "drivers/dma.h"
#include "drivers/i2s.h"
#include "test/i2s.hpp"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>


namespace test::i2s {

    namespace {

        constexpr const char* TAG = "I2S_Test";

        // Primary instance under test. I2S2 == SPI2 silicon, which is already known DMA-capable
        // from the SPI driver tests. Nothing needs to be connected to WS/SCLK/SD/MCLK for any
        // test in this file -- see the module comment in the PR/README for why.
        I2S_TypeDef* const TEST_INSTANCE = I2S2;

        GPIO_TypeDef* const  MCLK_PORT = GPIOC;
        constexpr gpio_pin_t MCLK_PIN  = GPIO_PIN_6;

        const i2s_master_config_t DEFAULT_TX_CONFIG = {
            .direction   = I2S_MASTER_TRANSMIT,
            .mode        = I2S_MODE_PHILIPS,
            .frequency   = I2S_FREQ_48kHz,
            .frame       = I2S_DATA_16_BITS_FRAME_16_BITS,
            .audio_clock = AUDIO_PLL_76_8MHz,
            .cpol        = false,
            .use_mck     = false,
            .ws_pin      = BOARD_I2S2_WS_PB12,
            .sd_pin      = BOARD_I2S2_SD_PB15,
            .mclk_pin    = BOARD_I2S2_MCLK_PC6,
            .sclk_pin    = BOARD_I2S2_SCLK_PB10,
        };

        const i2s_master_config_t DEFAULT_RX_CONFIG = [] {
            i2s_master_config_t cfg = DEFAULT_TX_CONFIG;
            cfg.direction           = I2S_MASTER_RECEIVE;
            return cfg;
        }();

        // Helpers
        volatile bool      s_tx_done = false;
        volatile hal_err_t s_tx_err  = HAL_OK;
        inline void        tx_done_callback(void*, hal_err_t err) {
            s_tx_err  = err;
            s_tx_done = true;
        }

        volatile bool      s_rx_done = false;
        volatile hal_err_t s_rx_err  = HAL_OK;
        inline void        rx_done_callback(void*, hal_err_t err) {
            s_rx_err  = err;
            s_rx_done = true;
        }

        volatile int s_dbm_completions = 0;
        inline void  dbm_callback(void*, hal_err_t) {
            s_dbm_completions += 1;
        }

        inline bool wait_for(volatile bool& flag) {
            uint32_t timeout = TIMEOUT;
            while (!flag && --timeout);
            return flag;
        }

        inline bool wait_for_count(volatile int& counter, int target) {
            uint32_t timeout = TIMEOUT;
            while (counter < target && --timeout);
            return counter >= target;
        }

        inline uint32_t get_gpio_moder(GPIO_TypeDef* port, gpio_pin_t pin) {
            return (port->MODER >> (std::to_underlying(pin) * 2U)) & 0b11U;
        }

        // TESTS
        void invalid_arg_guards() {
            std::array<uint16_t, 4> data{};
            std::array<uint16_t, 4> rx_buf{};
            uint8_t                 buffer_idx = 0;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2sx_clk_enable(reinterpret_cast<I2S_TypeDef*>(0x1), true));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_init(nullptr, &DEFAULT_TX_CONFIG));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_init(TEST_INSTANCE, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_deinit(nullptr));

            auto* const bogus_handle = reinterpret_cast<I2S_TypeDef*>(0x1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dma_init(bogus_handle, DMA_PRIORITY_LOW));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dma_deinit(bogus_handle));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_transmit_oneshot(bogus_handle, data.data(), data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_transmit_oneshot(TEST_INSTANCE, nullptr, data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_transmit_oneshot(TEST_INSTANCE, data.data(), 0, nullptr, nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_receive_oneshot(bogus_handle, rx_buf.data(), rx_buf.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_receive_oneshot(TEST_INSTANCE, nullptr, rx_buf.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_receive_oneshot(TEST_INSTANCE, rx_buf.data(), 0, nullptr, nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_init(bogus_handle, data.data(), rx_buf.data(), data.size(), dbm_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_init(TEST_INSTANCE, nullptr, rx_buf.data(), rx_buf.size(), dbm_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_init(TEST_INSTANCE, rx_buf.data(), nullptr, rx_buf.size(), dbm_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_init(TEST_INSTANCE, rx_buf.data(), rx_buf.data(), 0, dbm_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_init(TEST_INSTANCE, rx_buf.data(), rx_buf.data(), rx_buf.size(), nullptr, nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_deinit(bogus_handle));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_start(bogus_handle));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_stop(bogus_handle));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_get_filled_buffer(bogus_handle, &buffer_idx));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_dbm_get_filled_buffer(TEST_INSTANCE, nullptr));
        }

        void init_rejects_invalid_audio_clock_without_touching_registers() {
            i2s_master_config_t config = DEFAULT_TX_CONFIG;
            config.audio_clock         = static_cast<audio_clock_t>(0xFF);

            const uint32_t i2scfgr_before = TEST_INSTANCE->I2SCFGR;
            const uint32_t i2spr_before   = TEST_INSTANCE->I2SPR;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2s_master_init(TEST_INSTANCE, &config));

            // The switch on audio_clock is the very first thing init does, before any register
            // write, so an unrecognized value must leave the peripheral completely untouched.
            TEST_ASSERT_EQUAL_UINT32(i2scfgr_before, TEST_INSTANCE->I2SCFGR);
            TEST_ASSERT_EQUAL_UINT32(i2spr_before, TEST_INSTANCE->I2SPR);
        }

        void init_rejects_prescaler_below_the_supported_range() {
            // Forces i2sdiv < 2: a high nominal sample rate relative to the audio clock, with MCK
            // enabled (multiplier 256), pushes the divisor down below the minimum the hardware supports.
            i2s_master_config_t config = DEFAULT_TX_CONFIG;
            config.audio_clock         = AUDIO_PLL_172MHz;
            config.use_mck             = true;
            config.frequency           = static_cast<i2s_freq_t>(200'000); // Not a real audio rate, chosen to hit the boundary

            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, i2s_master_init(TEST_INSTANCE, &config));
        }

        void init_rejects_prescaler_above_the_supported_range() {
            // Forces i2sdiv > UINT8_MAX: a very low nominal sample rate against a fast audio clock
            // multiplier blows the divisor past what the 8-bit I2SDIV field can hold.
            i2s_master_config_t config = DEFAULT_TX_CONFIG;
            config.audio_clock         = AUDIO_PLL_76_8MHz;
            config.use_mck             = false;
            config.frame               = I2S_DATA_16_BITS_FRAME_16_BITS; // multiplier 32
            config.frequency           = static_cast<i2s_freq_t>(500);

            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, i2s_master_init(TEST_INSTANCE, &config));
        }

        void deinit_clears_control_registers() {
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &DEFAULT_TX_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));

            TEST_ASSERT_FALSE(TEST_INSTANCE->I2SCFGR & SPI_I2SCFGR_I2SE);
            TEST_ASSERT_EQUAL_UINT32(0,
                                     TEST_INSTANCE->I2SCFGR & (SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_I2SSTD |
                                                               SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_PCMSYNC));
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->I2SPR & (SPI_I2SPR_I2SDIV | SPI_I2SPR_ODD | SPI_I2SPR_MCKOE));

            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void init_programs_i2smod_direction_mode_and_polarity() {
            i2s_master_config_t config = DEFAULT_TX_CONFIG;
            config.direction           = I2S_MASTER_TRANSMIT;
            config.mode                = I2S_MODE_LEFT_JUSTIFIED;
            config.cpol                = true;

            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &config));

            TEST_ASSERT_TRUE(TEST_INSTANCE->I2SCFGR & SPI_I2SCFGR_I2SMOD);
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(I2S_MASTER_TRANSMIT),
                                     (TEST_INSTANCE->I2SCFGR & SPI_I2SCFGR_I2SCFG) >> SPI_I2SCFGR_I2SCFG_Pos);
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(I2S_MODE_LEFT_JUSTIFIED),
                                     (TEST_INSTANCE->I2SCFGR & SPI_I2SCFGR_I2SSTD) >> SPI_I2SCFGR_I2SSTD_Pos);
            TEST_ASSERT_TRUE(TEST_INSTANCE->I2SCFGR & SPI_I2SCFGR_CKPOL);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));

            // Direction flips and polarity clears
            config.direction = I2S_MASTER_RECEIVE;
            config.cpol      = false;
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &config));
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(I2S_MASTER_RECEIVE), (TEST_INSTANCE->I2SCFGR & SPI_I2SCFGR_I2SCFG) >> SPI_I2SCFGR_I2SCFG_Pos);
            TEST_ASSERT_FALSE(TEST_INSTANCE->I2SCFGR & SPI_I2SCFGR_CKPOL);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void init_programs_the_configured_frame_size() {
            constexpr auto FRAMES = std::array{
                I2S_DATA_16_BITS_FRAME_16_BITS, I2S_DATA_16_BITS_FRAME_32_BITS, I2S_DATA_24_BITS_FRAME_32_BITS, I2S_DATA_32_BITS_FRAME_32_BITS};

            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));

            for (const auto frame : FRAMES) {
                i2s_master_config_t config = DEFAULT_TX_CONFIG;
                config.frame               = frame;

                TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &config));
                TEST_ASSERT_EQUAL_UINT32(std::to_underlying(frame), TEST_INSTANCE->I2SCFGR & (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN));
                TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            }

            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void mclk_pin_only_programmed_when_use_mck_is_set() {
            const uint32_t mclk_mode_before = get_gpio_moder(MCLK_PORT, MCLK_PIN);

            i2s_master_config_t config = DEFAULT_TX_CONFIG;
            config.use_mck             = false;

            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &config));

            // use_mck == false: init must never touch the MCLK pin at all
            TEST_ASSERT_EQUAL_UINT32(mclk_mode_before, get_gpio_moder(MCLK_PORT, MCLK_PIN));
            TEST_ASSERT_FALSE(TEST_INSTANCE->I2SPR & SPI_I2SPR_MCKOE);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));

            config.use_mck = true;
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &config));

            // use_mck == true: MCLK pin goes to alternate function (0b10), and MCKOE gets set
            TEST_ASSERT_EQUAL_UINT32(0b10U, get_gpio_moder(MCLK_PORT, MCLK_PIN));
            TEST_ASSERT_TRUE(TEST_INSTANCE->I2SPR & SPI_I2SPR_MCKOE);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void clk_enable_toggles_the_correct_bus_bit() {
            i2sx_clk_enable(I2S1, false);
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_SPI1EN);
            i2sx_clk_enable(I2S1, true);
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_SPI1EN);
            i2sx_clk_enable(I2S1, false);

            i2sx_clk_enable(I2S2, false);
            TEST_ASSERT_FALSE(RCC->APB1ENR & RCC_APB1ENR_SPI2EN);
            i2sx_clk_enable(I2S2, true);
            TEST_ASSERT_TRUE(RCC->APB1ENR & RCC_APB1ENR_SPI2EN);
            // Leave I2S2 enabled: everything below needs it

            i2sx_clk_enable(I2S3, false);
            TEST_ASSERT_FALSE(RCC->APB1ENR & RCC_APB1ENR_SPI3EN);
            i2sx_clk_enable(I2S3, true);
            TEST_ASSERT_TRUE(RCC->APB1ENR & RCC_APB1ENR_SPI3EN);
            i2sx_clk_enable(I2S3, false);

            i2sx_clk_enable(I2S4, false);
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_SPI4EN);
            i2sx_clk_enable(I2S4, true);
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_SPI4EN);
            i2sx_clk_enable(I2S4, false);

            i2sx_clk_enable(I2S5, false);
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_SPI5EN);
            i2sx_clk_enable(I2S5, true);
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_SPI5EN);
            i2sx_clk_enable(I2S5, false);
        }

        void dma_init_rejects_unsupported_instances() {
            // I2S1 and I2S5 (== SPI1/SPI5) have no DMA streams mapped -- see the file-level assumption note
            i2sx_clk_enable(I2S1, true);
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, i2s_master_dma_init(I2S1, DMA_PRIORITY_LOW));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, i2s_master_dma_deinit(I2S1));
            i2sx_clk_enable(I2S1, false);

            i2sx_clk_enable(I2S5, true);
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, i2s_master_dma_init(I2S5, DMA_PRIORITY_LOW));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, i2s_master_dma_deinit(I2S5));
            i2sx_clk_enable(I2S5, false);
        }

        void transmit_oneshot_completes_and_invokes_callback() {
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &DEFAULT_TX_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_init(TEST_INSTANCE, DMA_PRIORITY_HIGH));

            constexpr std::array<uint16_t, 8> TX_DATA = {0, 1, 2, 3, 4, 5, 6, 7};

            s_tx_done = false;
            s_tx_err  = HAL_FAIL;

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_transmit_oneshot(TEST_INSTANCE, TX_DATA.data(), TX_DATA.size(), tx_done_callback, nullptr));
            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_tx_done), "I2S DMA oneshot TX never completed");
            TEST_ASSERT_EQUAL(HAL_OK, s_tx_err);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void receive_oneshot_completes_and_invokes_callback() {
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &DEFAULT_RX_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_init(TEST_INSTANCE, DMA_PRIORITY_HIGH));

            std::array<uint16_t, 8> rx_buf{};

            s_rx_done = false;
            s_rx_err  = HAL_FAIL;

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_receive_oneshot(TEST_INSTANCE, rx_buf.data(), rx_buf.size(), rx_done_callback, nullptr));
            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_rx_done), "I2S DMA oneshot RX never completed");
            TEST_ASSERT_EQUAL(HAL_OK, s_rx_err);
            // Content is meaningless with nothing driving SD -- only that it completed matters here

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void oneshot_rejects_a_call_while_a_transfer_is_still_in_flight() {
            // A deliberately slow bit clock (i2sdiv near mid-range, low nominal sample rate) plus a
            // large buffer keeps the DMA stream genuinely busy for milliseconds -- long enough to
            // reliably observe from software without faking any register state.
            i2s_master_config_t config = DEFAULT_TX_CONFIG;
            config.audio_clock         = AUDIO_PLL_172MHz;
            config.use_mck             = true;
            config.frequency           = static_cast<i2s_freq_t>(2'000);

            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &config));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_init(TEST_INSTANCE, DMA_PRIORITY_LOW));

            static std::array<uint16_t, 2000> tx_data{};

            s_tx_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_transmit_oneshot(TEST_INSTANCE, tx_data.data(), tx_data.size(), tx_done_callback, nullptr));

            // Re-entering immediately, before the first transfer has had any real time to finish
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, i2s_master_transmit_oneshot(TEST_INSTANCE, tx_data.data(), tx_data.size(), nullptr, nullptr));

            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_tx_done), "First I2S DMA oneshot TX never completed");

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void oneshot_doubles_transfer_length_for_24_and_32_bit_frames() {
            i2s_master_config_t config = DEFAULT_TX_CONFIG;
            config.frame               = I2S_DATA_24_BITS_FRAME_32_BITS;

            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &config));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_init(TEST_INSTANCE, DMA_PRIORITY_LOW));

            static std::array<uint16_t, 100> tx_data{};

            s_tx_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_transmit_oneshot(TEST_INSTANCE, tx_data.data(), tx_data.size(), tx_done_callback, nullptr));

            // NDTR is read immediately after starting: for a >16-bit frame the driver must have
            // programmed double the sample count into the DMA stream, since each sample takes two
            // half-word DMA beats
            dma_stream_map_t dma_map;
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_get_dma_stream_map(&dma_map, 1)); // idx 1 == I2S2
            TEST_ASSERT_TRUE(dma_map.tx.stream->NDTR <= tx_data.size() * 2);
            TEST_ASSERT_TRUE(dma_map.tx.stream->NDTR > tx_data.size()); // strictly more than 1x confirms doubling happened

            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_tx_done), "I2S DMA oneshot TX never completed");

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void oneshot_rejects_size_that_would_overflow_the_dma_transfer_length() {
            // At a 32-bit frame, actual_size = size * 2. Pick a size just over half of UINT16_MAX
            // so the doubled value overflows the field the driver packs it into.
            i2s_master_config_t config = DEFAULT_TX_CONFIG;
            config.frame               = I2S_DATA_32_BITS_FRAME_32_BITS;

            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &config));

            static std::array<uint16_t, 40'000> tx_data{};
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_SIZE, i2s_master_transmit_oneshot(TEST_INSTANCE, tx_data.data(), tx_data.size(), nullptr, nullptr));

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void dbm_start_and_stop_toggle_the_dma_stream_enable_bit() {
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &DEFAULT_RX_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_init(TEST_INSTANCE, DMA_PRIORITY_LOW));

            static std::array<uint16_t, 4> buf_0{};
            static std::array<uint16_t, 4> buf_1{};

            s_dbm_completions = 0;
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_init(TEST_INSTANCE, buf_0.data(), buf_1.data(), buf_0.size(), dbm_callback, nullptr));

            dma_stream_map_t dma_map;
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_get_dma_stream_map(&dma_map, 1));
            TEST_ASSERT_FALSE(dma_map.rx.stream->CR & DMA_SxCR_EN);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_start(TEST_INSTANCE));
            TEST_ASSERT_TRUE(dma_map.rx.stream->CR & DMA_SxCR_EN);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_stop(TEST_INSTANCE));
            TEST_ASSERT_FALSE(dma_map.rx.stream->CR & DMA_SxCR_EN);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

        void dbm_filled_buffer_index_alternates_across_completions() {
            // Small buffers at a moderate clock so a handful of half-buffer completions happen
            // within the wait budget below. Content is irrelevant -- only the CT/buffer-index
            // bookkeeping is under test.
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_init(TEST_INSTANCE, &DEFAULT_RX_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_init(TEST_INSTANCE, DMA_PRIORITY_HIGH));

            static std::array<uint16_t, 4> buf_0{};
            static std::array<uint16_t, 4> buf_1{};

            s_dbm_completions = 0;
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_init(TEST_INSTANCE, buf_0.data(), buf_1.data(), buf_0.size(), dbm_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_start(TEST_INSTANCE));

            TEST_ASSERT_TRUE_MESSAGE(wait_for_count(s_dbm_completions, 1), "First I2S DBM half-transfer never completed");
            uint8_t first_idx = 0xFFU;
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_get_filled_buffer(TEST_INSTANCE, &first_idx));

            TEST_ASSERT_TRUE_MESSAGE(wait_for_count(s_dbm_completions, 2), "Second I2S DBM half-transfer never completed");
            uint8_t second_idx = 0xFFU;
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_get_filled_buffer(TEST_INSTANCE, &second_idx));

            TEST_ASSERT_TRUE(first_idx == 0 || first_idx == 1);
            TEST_ASSERT_NOT_EQUAL(first_idx, second_idx);

            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_stop(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dbm_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2s_master_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, i2sx_clk_enable(TEST_INSTANCE, false));
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the I2S driver");
        UNITY_BEGIN();

        RUN_TEST(invalid_arg_guards);
        RUN_TEST(init_rejects_invalid_audio_clock_without_touching_registers);
        RUN_TEST(init_rejects_prescaler_below_the_supported_range);
        RUN_TEST(init_rejects_prescaler_above_the_supported_range);
        RUN_TEST(deinit_clears_control_registers);
        RUN_TEST(init_programs_i2smod_direction_mode_and_polarity);
        RUN_TEST(init_programs_the_configured_frame_size);
        RUN_TEST(mclk_pin_only_programmed_when_use_mck_is_set);
        RUN_TEST(clk_enable_toggles_the_correct_bus_bit);
        RUN_TEST(dma_init_rejects_unsupported_instances);
        RUN_TEST(transmit_oneshot_completes_and_invokes_callback);
        RUN_TEST(receive_oneshot_completes_and_invokes_callback);
        RUN_TEST(oneshot_rejects_a_call_while_a_transfer_is_still_in_flight);
        RUN_TEST(oneshot_doubles_transfer_length_for_24_and_32_bit_frames);
        RUN_TEST(oneshot_rejects_size_that_would_overflow_the_dma_transfer_length);
        RUN_TEST(dbm_start_and_stop_toggle_the_dma_stream_enable_bit);
        RUN_TEST(dbm_filled_buffer_index_alternates_across_completions);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the I2S driver");
    }

} // namespace test::i2s
