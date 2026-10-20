#include "onx_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define PCF8574_I2C_ADDRESS  0x38U
/* The vendor ONX2432G028 example names the LCD reset output EXPAND_IO_LCD_RST=6;
 * its PCF8574 helper uses 1-based pin numbering, so this is expander bit P5. */
#define PCF_LCD_RESET_BIT    (1U << 5)
#define PCF_I2C_SPEED_HZ     100000U
#define CST826_I2C_ADDRESS   0x15U
#define CST826_REG_TOUCH_NUM 0x02U
#define CST826_TOUCH_POLL_MS 25U
#define UI_NAV_Y             260
#define UI_NAV_HEIGHT        48
#define UI_NAV_LEFT_X        8
#define UI_NAV_RIGHT_X       124
#define UI_NAV_WIDTH         108
#define LCD_COLOR_BLACK      0x0000U
#define LCD_COLOR_WHITE      0xFFFFU
#define LCD_COLOR_CYAN       0x07FFU
#define LCD_COLOR_GREEN      0x07E0U
#define LCD_COLOR_NAVY       0x0010U
#define LCD_FILL_ROWS        8U
#define LCD_MAX_SCALE        5U
#define LCD_GLYPH_W          5U
#define LCD_GLYPH_H          7U
#define LCD_CELL_W           6U

static const char *TAG = "onx_display";
static bool s_initialized;
static SemaphoreHandle_t s_ui_mutex;
static esp_lcd_panel_io_handle_t s_lcd_io;
static esp_lcd_panel_handle_t s_panel;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_pcf8574;
static i2c_master_dev_handle_t s_touch;
static bool s_touch_ready;
static uint32_t s_last_rpm;
static bool s_rpm_valid;
static int32_t s_last_coolant_temp_c;
static bool s_coolant_valid;
static char s_last_status[64];

typedef enum {
    UI_PAGE_RPM = 0,
    UI_PAGE_COOLANT = 1,
} ui_page_t;

static ui_page_t s_page = UI_PAGE_RPM;
static uint16_t s_fill_buffer[BOARD_ONX_LCD_WIDTH * LCD_FILL_ROWS];
static uint16_t s_glyph_buffer[LCD_CELL_W * LCD_MAX_SCALE * LCD_GLYPH_H * LCD_MAX_SCALE];

/* 5-column, 7-row ASCII subset used by the diagnostic display. Bit 0 is top. */
static const uint8_t s_font_letters_digits[36][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x62,0x51,0x49,0x49,0x46}, /* 2 */
    {0x22,0x41,0x49,0x49,0x36}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x2F,0x49,0x49,0x49,0x31}, /* 5 */
    {0x3E,0x49,0x49,0x49,0x32}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x26,0x49,0x49,0x49,0x3E}, /* 9 */
};

static const uint8_t *glyph_columns(char ch)
{
    static const uint8_t blank[5] = {0,0,0,0,0};
    static const uint8_t dash[5] = {0,0x08,0x08,0x08,0};
    static const uint8_t slash[5] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t colon[5] = {0,0x36,0,0x36,0};
    static const uint8_t dot[5] = {0,0,0x60,0x60,0};
    static const uint8_t plus[5] = {0x08,0x08,0x3E,0x08,0x08};
    static const uint8_t underscore[5] = {0x40,0x40,0x40,0x40,0x40};
    ch = (char)toupper((unsigned char)ch);
    if (ch >= 'A' && ch <= 'Z') return s_font_letters_digits[(unsigned)(ch - 'A')];
    if (ch >= '0' && ch <= '9') return s_font_letters_digits[26U + (unsigned)(ch - '0')];
    switch (ch) {
    case '-': return dash;
    case '/': return slash;
    case ':': return colon;
    case '.': return dot;
    case '+': return plus;
    case '_': return underscore;
    case ' ': return blank;
    default: return blank;
    }
}

static esp_err_t pcf8574_write(uint8_t value)
{
    if (s_pcf8574 == NULL) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit(s_pcf8574, &value, 1, 100);
}

static esp_err_t init_reset_expander(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = BOARD_ONX_LCD_I2C_SCL,
        .sda_io_num = BOARD_ONX_LCD_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C0 init for onboard PCF8574 failed: %s", esp_err_to_name(err));
        return err;
    }

    const i2c_device_config_t pcf_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF8574_I2C_ADDRESS,
        .scl_speed_hz = PCF_I2C_SPEED_HZ,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &pcf_cfg, &s_pcf8574);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCF8574 at 0x38 not reachable: %s", esp_err_to_name(err));
        return err;
    }

    /* Release all expander pins, pulse LCD RST low on PCF8574 P5, then release it. */
    err = pcf8574_write(0xFFU);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(20));
    err = pcf8574_write((uint8_t)(0xFFU & (uint8_t)~PCF_LCD_RESET_BIT));
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(30));
    err = pcf8574_write(0xFFU);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "Onboard PCF8574 reset expander initialized; LCD RST pulsed via P5");
    return ESP_OK;
}

static esp_err_t init_touch_controller(void)
{
    const i2c_device_config_t touch_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CST826_I2C_ADDRESS,
        .scl_speed_hz = PCF_I2C_SPEED_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &touch_cfg, &s_touch);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not add CST826 I2C device at 0x%02X: %s",
                 CST826_I2C_ADDRESS, esp_err_to_name(err));
        s_touch = NULL;
        return err;
    }

    err = i2c_master_probe(s_i2c_bus, CST826_I2C_ADDRESS, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CST826 did not ACK at I2C address 0x%02X: %s",
                 CST826_I2C_ADDRESS, esp_err_to_name(err));
        s_touch = NULL;
        return err;
    }

    s_touch_ready = true;
    ESP_LOGI(TAG, "CST826 capacitive touch detected at I2C address 0x%02X on GPIO%d/GPIO%d",
             CST826_I2C_ADDRESS, BOARD_ONX_LCD_I2C_SCL, BOARD_ONX_LCD_I2C_SDA);
    return ESP_OK;
}

static esp_err_t init_lcd_panel(void)
{
    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = BOARD_ONX_LCD_PIN_SCLK,
        .mosi_io_num = BOARD_ONX_LCD_PIN_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = BOARD_ONX_LCD_WIDTH * LCD_FILL_ROWS * sizeof(uint16_t),
    };
    /* MCP2515 already owns SPI2_HOST. Keep the onboard LCD on SPI3_HOST. */
    esp_err_t err = spi_bus_initialize(BOARD_ONX_LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD SPI bus init failed (host=%d): %s",
                 (int)BOARD_ONX_LCD_SPI_HOST, esp_err_to_name(err));
        return err;
    }

    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = BOARD_ONX_LCD_PIN_DC,
        .cs_gpio_num = BOARD_ONX_LCD_PIN_CS,
        .pclk_hz = 40000000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 5,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BOARD_ONX_LCD_SPI_HOST,
                                   &io_cfg, &s_lcd_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LCD panel IO init failed: %s", esp_err_to_name(err));
        return err;
    }

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = GPIO_NUM_NC, /* board reset is controlled via PCF8574 */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7789(s_lcd_io, &panel_cfg, &s_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ST7789 panel create failed: %s", esp_err_to_name(err));
        return err;
    }
    if ((err = esp_lcd_panel_reset(s_panel)) != ESP_OK) return err;
    if ((err = esp_lcd_panel_init(s_panel)) != ESP_OK) return err;
    if ((err = esp_lcd_panel_swap_xy(s_panel, false)) != ESP_OK) return err;
    if ((err = esp_lcd_panel_mirror(s_panel, false, false)) != ESP_OK) return err;
    if ((err = esp_lcd_panel_invert_color(s_panel, false)) != ESP_OK) return err;
    if ((err = esp_lcd_panel_disp_on_off(s_panel, true)) != ESP_OK) return err;

    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << BOARD_ONX_LCD_PIN_BACKLIGHT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if ((err = gpio_config(&bl_cfg)) != ESP_OK) return err;
    if ((err = gpio_set_level(BOARD_ONX_LCD_PIN_BACKLIGHT, 1)) != ESP_OK) return err;

    ESP_LOGI(TAG, "ONX2432G028 ST7789 initialized: %ux%u, SPI host=%d SCLK=%d MOSI=%d DC=%d CS=%d BL=%d",
             (unsigned)BOARD_ONX_LCD_WIDTH, (unsigned)BOARD_ONX_LCD_HEIGHT,
             (int)BOARD_ONX_LCD_SPI_HOST, BOARD_ONX_LCD_PIN_SCLK, BOARD_ONX_LCD_PIN_MOSI,
             BOARD_ONX_LCD_PIN_DC, BOARD_ONX_LCD_PIN_CS, BOARD_ONX_LCD_PIN_BACKLIGHT);
    return ESP_OK;
}

static esp_err_t fill_rect(int x, int y, int width, int height, uint16_t color)
{
    if (s_panel == NULL) return ESP_ERR_INVALID_STATE;
    if (width <= 0 || height <= 0) return ESP_OK;
    if (x < 0) { width += x; x = 0; }
    if (y < 0) { height += y; y = 0; }
    if (x >= (int)BOARD_ONX_LCD_WIDTH || y >= (int)BOARD_ONX_LCD_HEIGHT) return ESP_OK;
    if (x + width > (int)BOARD_ONX_LCD_WIDTH) width = (int)BOARD_ONX_LCD_WIDTH - x;
    if (y + height > (int)BOARD_ONX_LCD_HEIGHT) height = (int)BOARD_ONX_LCD_HEIGHT - y;
    if (width <= 0 || height <= 0) return ESP_OK;

    for (int i = 0; i < width * (int)LCD_FILL_ROWS; ++i) s_fill_buffer[i] = color;
    for (int offset = 0; offset < height; offset += (int)LCD_FILL_ROWS) {
        int rows = height - offset;
        if (rows > (int)LCD_FILL_ROWS) rows = (int)LCD_FILL_ROWS;
        esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, x, y + offset,
                                                   x + width, y + offset + rows,
                                                   s_fill_buffer);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static esp_err_t draw_glyph(int x, int y, char ch, unsigned scale,
                            uint16_t foreground, uint16_t background)
{
    if (scale == 0U || scale > LCD_MAX_SCALE) return ESP_ERR_INVALID_ARG;
    const unsigned width = LCD_CELL_W * scale;
    const unsigned height = LCD_GLYPH_H * scale;
    if (width > LCD_CELL_W * LCD_MAX_SCALE || height > LCD_GLYPH_H * LCD_MAX_SCALE)
        return ESP_ERR_INVALID_SIZE;
    const uint8_t *columns = glyph_columns(ch);
    for (unsigned py = 0; py < height; ++py) {
        for (unsigned px = 0; px < width; ++px) {
            bool lit = false;
            const unsigned col = px / scale;
            const unsigned row = py / scale;
            if (col < LCD_GLYPH_W) lit = (columns[col] & (1U << row)) != 0U;
            s_glyph_buffer[py * width + px] = lit ? foreground : background;
        }
    }
    return esp_lcd_panel_draw_bitmap(s_panel, x, y, x + (int)width, y + (int)height,
                                     s_glyph_buffer);
}

static esp_err_t draw_text_box(int x, int y, int width, int height, unsigned scale,
                               uint16_t foreground, uint16_t background, const char *text)
{
    if (text == NULL) text = "";
    if (width <= 0 || height <= 0) return ESP_ERR_INVALID_ARG;
    size_t len = strnlen(text, 64U);
    while (len > 0U && len * LCD_CELL_W * scale > (size_t)width && scale > 1U) scale--;
    esp_err_t err = fill_rect(x, y, width, height, background);
    if (err != ESP_OK) return err;
    const int cell_width = (int)(LCD_CELL_W * scale);
    int cursor_x = x;
    for (size_t i = 0U; i < len && cursor_x + cell_width <= x + width; ++i) {
        err = draw_glyph(cursor_x, y, text[i], scale, foreground, background);
        if (err != ESP_OK) return err;
        cursor_x += cell_width;
    }
    return ESP_OK;
}

static esp_err_t draw_page_value(void)
{
    esp_err_t err;
    char value[16];
    const char *title;
    const char *unit;

    if (s_page == UI_PAGE_RPM) {
        title = "ENGINE RPM";
        unit = "RPM";
        if (s_rpm_valid) {
            (void)snprintf(value, sizeof(value), "%lu", (unsigned long)s_last_rpm);
        } else {
            (void)snprintf(value, sizeof(value), "----");
        }
    } else {
        title = "COOLANT TEMP";
        unit = "DEG C";
        if (s_coolant_valid) {
            (void)snprintf(value, sizeof(value), "%ld", (long)s_last_coolant_temp_c);
        } else {
            (void)snprintf(value, sizeof(value), "----");
        }
    }

    if ((err = draw_text_box(12, 60, 216, 22, 2, LCD_COLOR_CYAN, LCD_COLOR_BLACK,
                             title)) != ESP_OK) return err;
    if ((err = draw_text_box(12, 100, 216, 70, 5, LCD_COLOR_GREEN, LCD_COLOR_BLACK,
                             value)) != ESP_OK) return err;
    if ((err = draw_text_box(12, 177, 216, 22, 2, LCD_COLOR_WHITE, LCD_COLOR_BLACK,
                             unit)) != ESP_OK) return err;
    return ESP_OK;
}

static esp_err_t draw_nav_tab(int x, const char *label, bool active)
{
    const uint16_t bg = active ? LCD_COLOR_CYAN : LCD_COLOR_NAVY;
    const uint16_t fg = active ? LCD_COLOR_BLACK : LCD_COLOR_WHITE;
    const unsigned scale = 2U;
    const int text_width = (int)(strnlen(label, 20U) * LCD_CELL_W * scale);
    const int text_x = x + (UI_NAV_WIDTH - text_width) / 2;
    esp_err_t err;

    if ((err = fill_rect(x, UI_NAV_Y, UI_NAV_WIDTH, UI_NAV_HEIGHT, bg)) != ESP_OK) return err;
    /* Thin cyan outline also makes the tap target visible when it is inactive. */
    if ((err = fill_rect(x, UI_NAV_Y, UI_NAV_WIDTH, 2, LCD_COLOR_CYAN)) != ESP_OK) return err;
    if ((err = fill_rect(x, UI_NAV_Y + UI_NAV_HEIGHT - 2, UI_NAV_WIDTH, 2, LCD_COLOR_CYAN)) != ESP_OK) return err;
    if ((err = fill_rect(x, UI_NAV_Y, 2, UI_NAV_HEIGHT, LCD_COLOR_CYAN)) != ESP_OK) return err;
    if ((err = fill_rect(x + UI_NAV_WIDTH - 2, UI_NAV_Y, 2, UI_NAV_HEIGHT, LCD_COLOR_CYAN)) != ESP_OK) return err;
    return draw_text_box(text_x, UI_NAV_Y + 14, UI_NAV_WIDTH - 8, 20, scale, fg, bg, label);
}

static esp_err_t draw_current_screen(void)
{
    esp_err_t err = fill_rect(0, 0, BOARD_ONX_LCD_WIDTH, BOARD_ONX_LCD_HEIGHT, LCD_COLOR_BLACK);
    if (err != ESP_OK) return err;
    if ((err = fill_rect(0, 0, BOARD_ONX_LCD_WIDTH, 42, LCD_COLOR_NAVY)) != ESP_OK) return err;
    if ((err = draw_text_box(12, 10, 216, 22, 2, LCD_COLOR_WHITE, LCD_COLOR_NAVY,
                             "OBD / ENGINE")) != ESP_OK) return err;
    if ((err = fill_rect(12, 46, 216, 2, LCD_COLOR_CYAN)) != ESP_OK) return err;
    if ((err = draw_page_value()) != ESP_OK) return err;
    if ((err = fill_rect(12, 204, 216, 1, LCD_COLOR_CYAN)) != ESP_OK) return err;
    if ((err = draw_text_box(12, 213, 216, 24, 2, LCD_COLOR_WHITE, LCD_COLOR_BLACK,
                             s_last_status[0] != '\0' ? s_last_status : "WAITING FOR CAN")) != ESP_OK) return err;
    if ((err = draw_nav_tab(UI_NAV_LEFT_X, "RPM", s_page == UI_PAGE_RPM)) != ESP_OK) return err;
    if ((err = draw_nav_tab(UI_NAV_RIGHT_X, "COOLANT", s_page == UI_PAGE_COOLANT)) != ESP_OK) return err;
    return ESP_OK;
}

static void switch_page(ui_page_t page)
{
    if (s_ui_mutex == NULL || xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex for page switch");
        return;
    }
    if (s_page != page) {
        s_page = page;
        if (draw_current_screen() != ESP_OK) {
            ESP_LOGW(TAG, "LCD page redraw failed");
        } else {
            ESP_LOGI(TAG, "Touch selected page: %s", page == UI_PAGE_RPM ? "RPM" : "COOLANT TEMP");
        }
    }
    xSemaphoreGive(s_ui_mutex);
}

static void handle_touch(uint16_t raw_x, uint16_t raw_y)
{
    uint16_t x;
    uint16_t y;

    /* The glass/controller axes can differ by 90 degrees. Prefer direct coordinates,
     * and swap only when the point cannot fit the portrait LCD's coordinate range. */
    if (raw_x < BOARD_ONX_LCD_WIDTH && raw_y < BOARD_ONX_LCD_HEIGHT) {
        x = raw_x;
        y = raw_y;
    } else if (raw_x < BOARD_ONX_LCD_HEIGHT && raw_y < BOARD_ONX_LCD_WIDTH) {
        x = raw_y;
        y = raw_x;
    } else {
        ESP_LOGW(TAG, "Touch coordinates out of screen range: raw x=%u y=%u",
                 (unsigned)raw_x, (unsigned)raw_y);
        return;
    }

    ESP_LOGI(TAG, "CST826 touch down: raw=(%u,%u), screen=(%u,%u)",
             (unsigned)raw_x, (unsigned)raw_y, (unsigned)x, (unsigned)y);
    if (y < UI_NAV_Y || y >= UI_NAV_Y + UI_NAV_HEIGHT) return;

    if (x >= UI_NAV_LEFT_X && x < UI_NAV_LEFT_X + UI_NAV_WIDTH) {
        switch_page(UI_PAGE_RPM);
    } else if (x >= UI_NAV_RIGHT_X && x < UI_NAV_RIGHT_X + UI_NAV_WIDTH) {
        switch_page(UI_PAGE_COOLANT);
    }
}

static void cst826_touch_task(void *arg)
{
    bool finger_down = false;
    int64_t last_error_log_us = 0;
    (void)arg;

    while (true) {
        uint8_t reg = CST826_REG_TOUCH_NUM;
        uint8_t data[5] = {0}; /* count, XH, XL, YH, YL */
        esp_err_t err = i2c_master_transmit_receive(s_touch, &reg, 1, data,
                                                     sizeof(data), 30);
        if (err == ESP_OK) {
            const uint8_t touch_count = (uint8_t)(data[0] & 0x0FU);
            const uint8_t event = (uint8_t)(data[1] >> 6);
            const uint16_t raw_x = (uint16_t)(((uint16_t)(data[1] & 0x0FU) << 8) | data[2]);
            const uint16_t raw_y = (uint16_t)(((uint16_t)(data[3] & 0x0FU) << 8) | data[4]);

            if (touch_count > 0U && (event == 0U || event == 2U)) {
                if (!finger_down) handle_touch(raw_x, raw_y);
                finger_down = true;
            } else if (touch_count == 0U || event == 1U) {
                finger_down = false;
            }
        } else {
            const int64_t now_us = esp_timer_get_time();
            if (last_error_log_us == 0 || (now_us - last_error_log_us) >= 2000000LL) {
                last_error_log_us = now_us;
                ESP_LOGW(TAG, "CST826 touch read failed: %s", esp_err_to_name(err));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CST826_TOUCH_POLL_MS));
    }
}

void onx_display_set_status(const char *status)
{
    if (!s_initialized || status == NULL || s_ui_mutex == NULL) return;
    if (xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex for status update");
        return;
    }
    if (strncmp(s_last_status, status, sizeof(s_last_status)) == 0) {
        xSemaphoreGive(s_ui_mutex);
        return;
    }
    if (draw_text_box(12, 213, 216, 24, 2, LCD_COLOR_WHITE, LCD_COLOR_BLACK, status) == ESP_OK) {
        size_t n = strnlen(status, sizeof(s_last_status) - 1U);
        memcpy(s_last_status, status, n);
        s_last_status[n] = '\0';
    } else {
        ESP_LOGW(TAG, "LCD status update failed");
    }
    xSemaphoreGive(s_ui_mutex);
}

void onx_display_set_rpm(uint32_t rpm)
{
    if (!s_initialized || s_ui_mutex == NULL) return;
    if (xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex for RPM update");
        return;
    }
    if (!s_rpm_valid || rpm != s_last_rpm) {
        s_last_rpm = rpm;
        s_rpm_valid = true;
        if (s_page == UI_PAGE_RPM && draw_page_value() != ESP_OK) {
            ESP_LOGW(TAG, "LCD RPM update failed");
        }
    }
    xSemaphoreGive(s_ui_mutex);
}

void onx_display_clear_rpm(void)
{
    if (!s_initialized || s_ui_mutex == NULL) return;
    if (xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex to clear RPM");
        return;
    }
    s_rpm_valid = false;
    if (s_page == UI_PAGE_RPM && draw_page_value() != ESP_OK) {
        ESP_LOGW(TAG, "LCD RPM clear failed");
    }
    xSemaphoreGive(s_ui_mutex);
}

void onx_display_set_coolant_temp(int32_t temp_c)
{
    if (!s_initialized || s_ui_mutex == NULL) return;
    if (xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex for coolant update");
        return;
    }
    if (!s_coolant_valid || temp_c != s_last_coolant_temp_c) {
        s_last_coolant_temp_c = temp_c;
        s_coolant_valid = true;
        if (s_page == UI_PAGE_COOLANT && draw_page_value() != ESP_OK) {
            ESP_LOGW(TAG, "LCD coolant temperature update failed");
        }
    }
    xSemaphoreGive(s_ui_mutex);
}

void onx_display_clear_coolant_temp(void)
{
    if (!s_initialized || s_ui_mutex == NULL) return;
    if (xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex to clear coolant temperature");
        return;
    }
    s_coolant_valid = false;
    if (s_page == UI_PAGE_COOLANT && draw_page_value() != ESP_OK) {
        ESP_LOGW(TAG, "LCD coolant temperature clear failed");
    }
    xSemaphoreGive(s_ui_mutex);
}

void onx_display_show_waiting_for_rpm(const char *status)
{
    if (!s_initialized) return;
    onx_display_clear_rpm();
    onx_display_set_status(status != NULL ? status : "WAITING FOR RPM");
}

esp_err_t onx_display_init(void)
{
    if (s_initialized) return ESP_OK;
    if (s_ui_mutex == NULL) {
        s_ui_mutex = xSemaphoreCreateMutex();
        if (s_ui_mutex == NULL) {
            ESP_LOGE(TAG, "Could not create LCD/UI mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    esp_err_t err = init_reset_expander();
    if (err != ESP_OK) return err;

    /* CST826 shares GPIO7/GPIO8 I2C with the existing PCF8574 LCD-reset expander. */
    esp_err_t touch_err = init_touch_controller();
    if (touch_err != ESP_OK) {
        ESP_LOGW(TAG, "LCD will continue without touch navigation; verify the CST826 I2C connection");
    }

    err = init_lcd_panel();
    if (err != ESP_OK) return err;
    (void)snprintf(s_last_status, sizeof(s_last_status), "%s", "WAITING FOR CAN");
    s_page = UI_PAGE_RPM;
    s_rpm_valid = false;
    s_coolant_valid = false;
    err = draw_current_screen();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initial ST7789 UI draw failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    if (s_touch_ready) {
        if (xTaskCreate(cst826_touch_task, "cst826_touch", 3072, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Could not create CST826 touch task; page navigation is unavailable");
            s_touch_ready = false;
        } else {
            ESP_LOGI(TAG, "CST826 touch polling task started (%u ms period); tap the bottom tabs to switch pages",
                     (unsigned)CST826_TOUCH_POLL_MS);
        }
    }
    ESP_LOGI(TAG, "ONX2432G028 LCD ready: pages=RPM/COOLANT TEMP, 240x320; values update from OBD responses");
    return ESP_OK;
}
