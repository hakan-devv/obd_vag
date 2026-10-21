#include "can_driver.h"

#include <string.h>

#include "board_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcp2515.h"
#include "sdkconfig.h"

#ifndef CONFIG_OBD_MCP2515_SPI_HZ
#define CONFIG_OBD_MCP2515_SPI_HZ 1000000
#endif

static const char *TAG = "can_driver";

static bool s_inited;
static bool s_mode_valid;
static can_mode_t s_current_mode = CAN_MODE_CONFIG;
static bool s_rx_configuration_verified;
static can_bus_config_t s_bus_config = {
    .bitrate = CAN_BITRATE_500_KBPS,
    .extended_ids = false,
};

static uint32_t spi_clock_hz(void)
{
    uint32_t hz = CONFIG_OBD_MCP2515_SPI_HZ;
    if (hz < 100000U) {
        hz = 100000U;
    }
    if (hz > 5000000U) {
        hz = 5000000U;
    }
    return hz;
}

static esp_err_t map_mcp_error(ERROR_t err)
{
    switch (err) {
    case ERROR_OK:
        return ESP_OK;
    case ERROR_NOMSG:
        return ESP_ERR_NOT_FOUND;
    case ERROR_ALLTXBUSY:
        return ESP_ERR_TIMEOUT;
    default:
        return ESP_FAIL;
    }
}

static bool bus_config_is_valid(const can_bus_config_t *config)
{
    return config != NULL &&
           (config->bitrate == CAN_BITRATE_250_KBPS ||
            config->bitrate == CAN_BITRATE_500_KBPS);
}

static esp_err_t write_and_verify_cnf(const can_bus_config_t *config)
{
    const uint8_t want1 = config->bitrate == CAN_BITRATE_500_KBPS
        ? BOARD_MCP2515_CNF1_500K_8MHZ : BOARD_MCP2515_CNF1_250K_8MHZ;
    const uint8_t want2 = config->bitrate == CAN_BITRATE_500_KBPS
        ? BOARD_MCP2515_CNF2_500K_8MHZ : BOARD_MCP2515_CNF2_250K_8MHZ;
    const uint8_t want3 = config->bitrate == CAN_BITRATE_500_KBPS
        ? BOARD_MCP2515_CNF3_500K_8MHZ : BOARD_MCP2515_CNF3_250K_8MHZ;

    ESP_LOGI(TAG,
             "CAN timing: crystal=8MHz bitrate=%lu CNF1=0x%02X CNF2=0x%02X CNF3=0x%02X",
             (unsigned long)config->bitrate, want1, want2, want3);

    esp_err_t err = MCP2515_setRegisterChecked(MCP_CNF1, want1);
    if (err != ESP_OK) return err;
    err = MCP2515_setRegisterChecked(MCP_CNF2, want2);
    if (err != ESP_OK) return err;
    err = MCP2515_setRegisterChecked(MCP_CNF3, want3);
    if (err != ESP_OK) return err;

    uint8_t got1 = 0, got2 = 0, got3 = 0;
    err = MCP2515_readRegisterChecked(MCP_CNF1, &got1);
    if (err != ESP_OK) return err;
    err = MCP2515_readRegisterChecked(MCP_CNF2, &got2);
    if (err != ESP_OK) return err;
    err = MCP2515_readRegisterChecked(MCP_CNF3, &got3);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "CNF write: 0x%02X 0x%02X 0x%02X", want1, want2, want3);
    ESP_LOGI(TAG, "CNF read:  0x%02X 0x%02X 0x%02X", got1, got2, got3);

    if (got1 != want1 || got2 != want2 || got3 != want3) {
        ESP_LOGE(TAG, "CNF readback mismatch");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Loopback and initial bench listening deliberately accept all valid CAN IDs. */
static esp_err_t configure_accept_all_rx(void)
{
    /* Loopback/bench listening intentionally disables response-ID filtering. */
    s_rx_configuration_verified = false;
    esp_err_t err = MCP2515_modifyRegisterChecked(
        MCP_RXB0CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_MASK);
    if (err != ESP_OK) return err;

    return MCP2515_modifyRegisterChecked(
        MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_MASK);
}

static esp_err_t set_hardware_mode(can_mode_t mode)
{
    ERROR_t mcp_err;
    switch (mode) {
    case CAN_MODE_NORMAL:
        mcp_err = MCP2515_setNormalMode();
        break;
    case CAN_MODE_LISTEN_ONLY:
        mcp_err = MCP2515_setListenOnlyMode();
        break;
    case CAN_MODE_LOOPBACK:
        mcp_err = MCP2515_setLoopbackMode();
        break;
    case CAN_MODE_CONFIG:
        mcp_err = MCP2515_setConfigMode();
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    if (mcp_err != ERROR_OK) {
        ESP_LOGE(TAG, "Hardware mode transition to %d failed", (int)mode);
        s_mode_valid = false;
        return ESP_FAIL;
    }

    s_current_mode = mode;
    s_mode_valid = true;
    ESP_LOGI(TAG, "CAN mode %d verified by MCP2515", (int)mode);
    return ESP_OK;
}

/* Abort queued TX requests before entering a mode in which sending is forbidden. */
static esp_err_t abort_pending_transmissions(void)
{
    const REGISTER_t tx_ctrl_regs[] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL};
    bool pending = false;

    for (size_t i = 0; i < sizeof(tx_ctrl_regs) / sizeof(tx_ctrl_regs[0]); ++i) {
        uint8_t ctrl = 0;
        esp_err_t err = MCP2515_readRegisterChecked(tx_ctrl_regs[i], &ctrl);
        if (err != ESP_OK) return err;
        if ((ctrl & TXB_TXREQ) != 0) pending = true;
    }

    if (!pending) return ESP_OK;

    esp_err_t err = MCP2515_modifyRegisterChecked(MCP_CANCTRL, CANCTRL_ABAT, CANCTRL_ABAT);
    if (err != ESP_OK) return err;

    bool still_pending = true;
    for (int attempt = 0; attempt < 10; ++attempt) {
        still_pending = false;
        for (size_t i = 0; i < sizeof(tx_ctrl_regs) / sizeof(tx_ctrl_regs[0]); ++i) {
            uint8_t ctrl = 0;
            err = MCP2515_readRegisterChecked(tx_ctrl_regs[i], &ctrl);
            if (err != ESP_OK) {
                (void)MCP2515_modifyRegisterChecked(MCP_CANCTRL, CANCTRL_ABAT, 0);
                return err;
            }
            if ((ctrl & TXB_TXREQ) != 0) still_pending = true;
        }
        if (!still_pending) break;
        vTaskDelay(1);
    }

    esp_err_t clear_err = MCP2515_modifyRegisterChecked(MCP_CANCTRL, CANCTRL_ABAT, 0);
    if (clear_err != ESP_OK) return clear_err;
    if (still_pending) {
        ESP_LOGE(TAG, "Could not abort pending MCP2515 transmissions before mode change");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGW(TAG, "Pending MCP2515 TX request(s) aborted before mode change");
    return ESP_OK;
}

esp_err_t can_init_with_config(const can_bus_config_t *config)
{
    if (!bus_config_is_valid(config)) return ESP_ERR_INVALID_ARG;
    if (s_inited) return ESP_OK;

    s_bus_config = *config;
    if (MCP2515_init() != ERROR_OK) {
        ESP_LOGE(TAG, "MCP2515_init failed");
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus_cfg = {
        .miso_io_num = BOARD_MCP2515_PIN_MISO,
        .mosi_io_num = BOARD_MCP2515_PIN_MOSI,
        .sclk_io_num = BOARD_MCP2515_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    esp_err_t err = spi_bus_initialize(BOARD_MCP2515_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .mode = BOARD_MCP2515_SPI_MODE,
        .clock_speed_hz = (int)spi_clock_hz(),
        .spics_io_num = BOARD_MCP2515_PIN_CS,
        .queue_size = 8,
    };

    err = spi_bus_add_device(BOARD_MCP2515_SPI_HOST, &dev_cfg, &MCP2515_Object->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "SPI host=%d SCK=%d MOSI=%d MISO=%d CS=%d hz=%lu mode=%d",
             (int)BOARD_MCP2515_SPI_HOST,
             BOARD_MCP2515_PIN_SCK,
             BOARD_MCP2515_PIN_MOSI,
             BOARD_MCP2515_PIN_MISO,
             BOARD_MCP2515_PIN_CS,
             (unsigned long)spi_clock_hz(),
             BOARD_MCP2515_SPI_MODE);

    if (MCP2515_reset() != ERROR_OK) {
        ESP_LOGE(TAG, "MCP2515_reset failed");
        return ESP_FAIL;
    }

    if (MCP2515_setConfigMode() != ERROR_OK) {
        ESP_LOGE(TAG, "setConfigMode failed");
        return ESP_FAIL;
    }

    err = write_and_verify_cnf(&s_bus_config);
    if (err != ESP_OK) return err;

    s_current_mode = CAN_MODE_CONFIG;
    s_mode_valid = true;
    s_rx_configuration_verified = false;
    s_inited = true;
    ESP_LOGI(TAG, "can_init OK; controller left in Configuration mode");
    return ESP_OK;
}

esp_err_t can_init(void)
{
    can_bus_config_t config = {
        .bitrate =
#if defined(CONFIG_OBD_CAN_BITRATE_250K) && CONFIG_OBD_CAN_BITRATE_250K
            CAN_BITRATE_250_KBPS,
#else
            CAN_BITRATE_500_KBPS,
#endif
        .extended_ids =
#if defined(CONFIG_OBD_CAN_ID_EXTENDED) && CONFIG_OBD_CAN_ID_EXTENDED
            true,
#else
            false,
#endif
    };
    return can_init_with_config(&config);
}

esp_err_t can_reconfigure(const can_bus_config_t *config)
{
    if (!s_inited || !s_mode_valid) return ESP_ERR_INVALID_STATE;
    if (!bus_config_is_valid(config)) return ESP_ERR_INVALID_ARG;
    if (s_current_mode != CAN_MODE_CONFIG) return ESP_ERR_INVALID_STATE;

    esp_err_t err = write_and_verify_cnf(config);
    if (err != ESP_OK) return err;
    s_bus_config = *config;
    s_rx_configuration_verified = false;
    ESP_LOGI(TAG, "CAN configuration updated: bitrate=%lu id-format=%s",
             (unsigned long)config->bitrate,
             config->extended_ids ? "29-bit" : "11-bit");
    return ESP_OK;
}

esp_err_t can_get_config(can_bus_config_t *config)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    *config = s_bus_config;
    return ESP_OK;
}

esp_err_t can_set_mode(can_mode_t mode)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (mode != CAN_MODE_NORMAL && mode != CAN_MODE_LISTEN_ONLY &&
        mode != CAN_MODE_LOOPBACK && mode != CAN_MODE_CONFIG) {
        return ESP_ERR_INVALID_ARG;
    }

    /* If the previous transition failed, first recover to a known safe state. */
    if (!s_mode_valid) {
        esp_err_t recover_err = set_hardware_mode(CAN_MODE_CONFIG);
        if (recover_err != ESP_OK) return recover_err;
    }
    if (mode == CAN_MODE_NORMAL && !s_rx_configuration_verified) {
        ESP_LOGE(TAG, "Refusing Normal mode: configure and verify RX acceptance mode first");
        return ESP_ERR_INVALID_STATE;
    }
    if (mode == s_current_mode) return ESP_OK;

    /* If leaving a transmit-capable mode, don't let a queued frame surprise us later. */
    if ((s_current_mode == CAN_MODE_NORMAL || s_current_mode == CAN_MODE_LOOPBACK) &&
        mode != s_current_mode) {
        esp_err_t err = abort_pending_transmissions();
        if (err != ESP_OK) return err;
    }

    /* RX configuration and one-shot TX settings are changed in Configuration mode. */
    if ((mode == CAN_MODE_LOOPBACK || mode == CAN_MODE_NORMAL) &&
        s_current_mode != CAN_MODE_CONFIG) {
        esp_err_t err = set_hardware_mode(CAN_MODE_CONFIG);
        if (err != ESP_OK) return err;
    }

    /*
     * Use normal CAN automatic retransmission. OBD requests can lose arbitration
     * against higher-priority vehicle traffic and must be retried by the controller.
     */
    if (mode == CAN_MODE_NORMAL) {
        esp_err_t err = MCP2515_modifyRegisterChecked(
            MCP_CANCTRL, CANCTRL_OSM, 0);
        if (err != ESP_OK) return err;
    }

    if (mode == CAN_MODE_LOOPBACK) {
        esp_err_t err = configure_accept_all_rx();
        if (err != ESP_OK) return err;
    }

    return set_hardware_mode(mode);
}

esp_err_t can_send(const can_frame_t *frame)
{
    if (!s_inited || !s_mode_valid) return ESP_ERR_INVALID_STATE;
    if (frame == NULL) return ESP_ERR_INVALID_ARG;
    if (frame->dlc > 8) return ESP_ERR_INVALID_ARG;
    if ((!frame->extended && frame->id > CAN_SFF_MASK) ||
        (frame->extended && frame->id > CAN_EFF_MASK)) return ESP_ERR_INVALID_ARG;

    /* Loopback is allowed for self-test; real transmission requires Normal mode. */
    if (s_current_mode != CAN_MODE_NORMAL && s_current_mode != CAN_MODE_LOOPBACK) {
        ESP_LOGW(TAG, "can_send rejected in mode %d", (int)s_current_mode);
        return ESP_ERR_INVALID_STATE;
    }

    CAN_FRAME_t mcp_frame;
    memset(mcp_frame, 0, sizeof(mcp_frame));
    if (frame->rtr) return ESP_ERR_NOT_SUPPORTED;
    mcp_frame->can_id = frame->extended
        ? (frame->id | CAN_EFF_FLAG) : (frame->id & CAN_SFF_MASK);
    mcp_frame->can_dlc = frame->dlc;
    memcpy(mcp_frame->data, frame->data, frame->dlc);

    return map_mcp_error(MCP2515_sendMessageAfterCtrlCheck(mcp_frame));
}

esp_err_t can_available(bool *available)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (available == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t stat = 0;
    esp_err_t err = MCP2515_getStatusChecked(&stat);
    if (err != ESP_OK) {
        *available = false;
        return err;
    }

    *available = (stat & STAT_RXIF_MASK) != 0;
    return ESP_OK;
}

esp_err_t can_receive(can_frame_t *frame)
{
    if (!s_inited || !s_mode_valid) return ESP_ERR_INVALID_STATE;
    if (frame == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t stat = 0;
    esp_err_t err = MCP2515_getStatusChecked(&stat);
    if (err != ESP_OK) return err;

    RXBn_t rx_buffer;
    if ((stat & STAT_RX0IF) != 0) {
        rx_buffer = RXB0;
    } else if ((stat & STAT_RX1IF) != 0) {
        rx_buffer = RXB1;
    } else {
        return ESP_ERR_NOT_FOUND;
    }

    CAN_FRAME_t mcp_frame;
    memset(mcp_frame, 0, sizeof(mcp_frame));
    ERROR_t mcp_err = MCP2515_readMessage(rx_buffer, mcp_frame);
    if (mcp_err != ERROR_OK) return map_mcp_error(mcp_err);
    if (mcp_frame->can_dlc > sizeof(frame->data)) return ESP_FAIL;

    memset(frame, 0, sizeof(*frame));
    frame->extended = (mcp_frame->can_id & CAN_EFF_FLAG) != 0;
    frame->rtr = (mcp_frame->can_id & CAN_RTR_FLAG) != 0;
    frame->id = mcp_frame->can_id & (frame->extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    frame->dlc = mcp_frame->can_dlc;
    memcpy(frame->data, mcp_frame->data, frame->dlc);
    return ESP_OK;
}

esp_err_t can_read_diagnostics(can_diagnostics_t *diagnostics)
{
    if (!s_inited || !s_mode_valid) return ESP_ERR_INVALID_STATE;
    if (diagnostics == NULL) return ESP_ERR_INVALID_ARG;

    memset(diagnostics, 0, sizeof(*diagnostics));
    esp_err_t err = MCP2515_readRegisterChecked(MCP_CANSTAT, &diagnostics->canstat);
    if (err != ESP_OK) return err;
    err = MCP2515_readRegisterChecked(MCP_EFLG, &diagnostics->eflg);
    if (err != ESP_OK) return err;
    err = MCP2515_readRegisterChecked(MCP_TEC, &diagnostics->tec);
    if (err != ESP_OK) return err;
    err = MCP2515_readRegisterChecked(MCP_REC, &diagnostics->rec);
    if (err != ESP_OK) return err;

    const REGISTER_t tx_regs[] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL};
    for (uint8_t i = 0; i < 3; ++i) {
        uint8_t ctrl = 0;
        err = MCP2515_readRegisterChecked(tx_regs[i], &ctrl);
        if (err != ESP_OK) return err;
        if ((ctrl & TXB_TXREQ) != 0) {
            diagnostics->txreq_mask |= (uint8_t)(1U << i);
        }
    }
    return ESP_OK;
}

esp_err_t can_read_and_clear_rx_overflow(uint8_t *overflow_flags)
{
    if (!s_inited || !s_mode_valid) return ESP_ERR_INVALID_STATE;
    if (overflow_flags == NULL) return ESP_ERR_INVALID_ARG;
    *overflow_flags = 0;

    uint8_t eflg = 0;
    esp_err_t err = MCP2515_readRegisterChecked(MCP_EFLG, &eflg);
    if (err != ESP_OK) return err;

    const uint8_t observed = (uint8_t)(eflg & ((uint8_t)EFLG_RX0OVR | (uint8_t)EFLG_RX1OVR));
    if (observed == 0) return ESP_OK;

    err = MCP2515_modifyRegisterChecked(MCP_EFLG, observed, 0);
    if (err != ESP_OK) return err;

    *overflow_flags = observed;
    return ESP_OK;
}

/*
 * Accept all valid standard and extended frames while the discovery state
 * machine is in Listen-Only mode. Active OBD probing installs response filters
 * before entering Normal mode.
 */
esp_err_t can_configure_diagnostic_accept_all_rx(void)
{
    if (!s_inited || !s_mode_valid) return ESP_ERR_INVALID_STATE;
    if (s_current_mode != CAN_MODE_CONFIG) {
        ESP_LOGE(TAG, "Accept-all RX mode can only be configured in Configuration mode");
        return ESP_ERR_INVALID_STATE;
    }

    s_rx_configuration_verified = false;

    /* Drain the two hardware RX buffers so startup Listen-Only frames are not
     * mistaken for frames captured after diagnostic Normal mode begins. */
    uint32_t stale_frames = 0;
    for (int i = 0; i < 2; ++i) {
        bool available = false;
        esp_err_t err = can_available(&available);
        if (err != ESP_OK) return err;
        if (!available) break;

        can_frame_t stale = {0};
        err = can_receive(&stale);
        if (err != ESP_OK) return err;
        ++stale_frames;
    }

    esp_err_t err = MCP2515_modifyRegisterChecked(
        MCP_RXB0CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_MASK);
    if (err != ESP_OK) return err;
    err = MCP2515_modifyRegisterChecked(
        MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_MASK);
    if (err != ESP_OK) return err;

    uint8_t rx0ctrl = 0;
    uint8_t rx1ctrl = 0;
    err = MCP2515_readRegisterChecked(MCP_RXB0CTRL, &rx0ctrl);
    if (err != ESP_OK) return err;
    err = MCP2515_readRegisterChecked(MCP_RXB1CTRL, &rx1ctrl);
    if (err != ESP_OK) return err;

    if ((rx0ctrl & RXBnCTRL_RXM_MASK) != RXBnCTRL_RXM_MASK ||
        (rx1ctrl & RXBnCTRL_RXM_MASK) != RXBnCTRL_RXM_MASK) {
        ESP_LOGE(TAG,
                 "Diagnostic accept-all RX readback mismatch RXB0CTRL=0x%02X RXB1CTRL=0x%02X",
                 rx0ctrl, rx1ctrl);
        return ESP_FAIL;
    }

    s_rx_configuration_verified = true;
    ESP_LOGI(TAG,
             "CAN discovery RX enabled: accepting standard + extended CAN IDs; flushed %lu stale frame(s)",
             (unsigned long)stale_frames);
    return ESP_OK;
}

esp_err_t can_configure_obd_response_filters(void)
{
    if (!s_inited || !s_mode_valid) return ESP_ERR_INVALID_STATE;
    if (s_current_mode != CAN_MODE_CONFIG) {
        ESP_LOGE(TAG, "OBD response filters can only be configured in Configuration mode");
        return ESP_ERR_INVALID_STATE;
    }

    s_rx_configuration_verified = false;

    const bool extended = s_bus_config.extended_ids;
    const uint32_t mask = extended ? 0x1FFFFF00U : 0x7F8U;
    const uint32_t filter_id = extended ? 0x18DAF100U : 0x7E8U;
    uint8_t expected_filter[4] = {0};
    uint8_t expected_mask[4] = {0};
    MCP2515_prepareId(expected_filter, extended, filter_id);
    MCP2515_prepareId(expected_mask, extended, mask);

    /*
     * For 11-bit OBD, 0x7F8 matches 0x7E8-0x7EF. For 29-bit OBD,
     * 0x1FFFFF00 matches the standard 0x18DAF1xx response family.
     */
    if (MCP2515_setFilterMask(MASK0, extended, mask) != ERROR_OK ||
        MCP2515_setFilterMask(MASK1, extended, mask) != ERROR_OK) {
        ESP_LOGE(TAG, "Failed to configure OBD response acceptance masks");
        return ESP_FAIL;
    }
    esp_err_t err = ESP_OK;

    const RXF_t filter_ids[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    const REGISTER_t filter_registers[] = {
        MCP_RXF0SIDH, MCP_RXF1SIDH, MCP_RXF2SIDH,
        MCP_RXF3SIDH, MCP_RXF4SIDH, MCP_RXF5SIDH
    };
    for (size_t i = 0; i < sizeof(filter_ids) / sizeof(filter_ids[0]); ++i) {
        if (MCP2515_setFilter(filter_ids[i], extended, filter_id) != ERROR_OK) {
            ESP_LOGE(TAG, "Failed to configure OBD response filter %u", (unsigned)i);
            return ESP_FAIL;
        }
        for (uint8_t offset = 0; offset < 4; ++offset) {
            uint8_t value = 0;
            err = MCP2515_readRegisterChecked(
                (REGISTER_t)((uint8_t)filter_registers[i] + offset), &value);
            if (err != ESP_OK) return err;
            if (value != expected_filter[offset]) {
                ESP_LOGE(TAG, "RXF%u readback mismatch at offset %u: got 0x%02X expected 0x%02X",
                         (unsigned)i, (unsigned)offset, value, expected_filter[offset]);
                return ESP_FAIL;
            }
        }
    }

    /* Verify both mask registers at register level. */
    const REGISTER_t mask_registers[] = {MCP_RXM0SIDH, MCP_RXM1SIDH};
    for (size_t i = 0; i < sizeof(mask_registers) / sizeof(mask_registers[0]); ++i) {
        for (uint8_t offset = 0; offset < 4; ++offset) {
            uint8_t value = 0;
            err = MCP2515_readRegisterChecked(
                (REGISTER_t)((uint8_t)mask_registers[i] + offset), &value);
            if (err != ESP_OK) return err;
            if (value != expected_mask[offset]) {
                ESP_LOGE(TAG, "RXM%u readback mismatch at offset %u: got 0x%02X expected 0x%02X",
                         (unsigned)i, (unsigned)offset, value, expected_mask[offset]);
                return ESP_FAIL;
            }
        }
    }

    /* RXM=00 enables the acceptance filters. RXB0 rollover allows RX1 to share capacity. */
    err = MCP2515_modifyRegisterChecked(
        MCP_RXB0CTRL,
        RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
        RXB0CTRL_BUKT);
    if (err != ESP_OK) return err;
    err = MCP2515_modifyRegisterChecked(
        MCP_RXB1CTRL,
        RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
        0);
    if (err != ESP_OK) return err;

    uint8_t rx0ctrl = 0;
    uint8_t rx1ctrl = 0;
    err = MCP2515_readRegisterChecked(MCP_RXB0CTRL, &rx0ctrl);
    if (err != ESP_OK) return err;
    err = MCP2515_readRegisterChecked(MCP_RXB1CTRL, &rx1ctrl);
    if (err != ESP_OK) return err;
    if ((rx0ctrl & RXBnCTRL_RXM_MASK) != 0 ||
        (rx1ctrl & RXBnCTRL_RXM_MASK) != 0 ||
        (rx0ctrl & RXB0CTRL_BUKT) == 0) {
        ESP_LOGE(TAG, "OBD filter mode readback mismatch RXB0CTRL=0x%02X RXB1CTRL=0x%02X",
                 rx0ctrl, rx1ctrl);
        return ESP_FAIL;
    }

    s_rx_configuration_verified = true;
    ESP_LOGI(TAG, "OBD RX filters verified: %s response family, mask=0x%08lX",
             extended ? "29-bit 0x18DAF1xx" : "11-bit 0x7E8-0x7EF",
             (unsigned long)mask);
    return ESP_OK;
}

esp_err_t can_run_self_test(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    if (s_current_mode != CAN_MODE_CONFIG) {
        esp_err_t err = can_set_mode(CAN_MODE_CONFIG);
        if (err != ESP_OK) return err;
    }

    esp_err_t err = write_and_verify_cnf(&s_bus_config);
    if (err != ESP_OK) return err;

    err = can_set_mode(CAN_MODE_LOOPBACK);
    if (err != ESP_OK) return err;

    can_frame_t tx = {
        .id = BOARD_CAN_LOOPBACK_TEST_ID,
        .dlc = 8,
        .extended = false,
        .data = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04},
    };

    /* There are only two RX buffers; drain stale frames without an unbounded loop. */
    for (int i = 0; i < 2; ++i) {
        bool available = false;
        err = can_available(&available);
        if (err != ESP_OK) return err;
        if (!available) break;
        can_frame_t dump;
        err = can_receive(&dump);
        if (err != ESP_OK) return err;
    }

    err = can_send(&tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "loopback TX failed: %s", esp_err_to_name(err));
        return err;
    }

    can_frame_t rx;
    bool got = false;
    for (int i = 0; i < 100; ++i) {
        err = can_receive(&rx);
        if (err == ESP_OK) {
            got = true;
            break;
        }
        if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "loopback RX error: %s", esp_err_to_name(err));
            return err;
        }
        vTaskDelay(1);
    }

    if (!got) {
        ESP_LOGE(TAG, "loopback RX timeout");
        return ESP_ERR_TIMEOUT;
    }

    if (rx.id != tx.id || rx.dlc != tx.dlc || rx.extended != tx.extended ||
        memcmp(rx.data, tx.data, tx.dlc) != 0) {
        ESP_LOGE(TAG, "loopback payload mismatch id=0x%lX dlc=%u",
                 (unsigned long)rx.id, rx.dlc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "loopback self-test OK id=0x%03lX", (unsigned long)tx.id);
    return ESP_OK;
}
