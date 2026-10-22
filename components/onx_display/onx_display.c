#include "onx_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

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
#include "swipe_gesture.h"

#define PCF8574_I2C_ADDRESS  0x38U
/* The vendor ONX2432G028 example names the LCD reset output EXPAND_IO_LCD_RST=6;
 * its PCF8574 helper uses 1-based pin numbering, so this is expander bit P5. */
#define PCF_LCD_RESET_BIT    (1U << 5)
#define PCF_I2C_SPEED_HZ     100000U
#define CST826_I2C_ADDRESS   0x15U
#define CST826_REG_TOUCH_NUM 0x02U
#define CST826_TOUCH_POLL_MS 25U
#define UI_HEADER_HEIGHT     24
#define UI_CARD_X            6
#define UI_CARD_WIDTH        228
#define UI_CARD_HEIGHT       94
#define UI_CARD_GAP          4
#define UI_CARD_FIRST_Y      27
#define UI_TITLE_LINE_CHARS  17U
#define UI_MAX_ITEMS         64U
#define UI_STALE_MULTIPLIER  3LL
#define RGB565_SWAP(c) ((uint16_t)((((c) & 0xFFU) << 8) | ((c) >> 8)))
#define LCD_COLOR_CYAN   RGB565_SWAP(0x07FFU)
#define LCD_COLOR_GREEN  RGB565_SWAP(0x07E0U)
#define LCD_COLOR_NAVY   RGB565_SWAP(0x0010U)
#define LCD_COLOR_GRAY   RGB565_SWAP(0x8410U)
#define LCD_COLOR_RED    RGB565_SWAP(0xF800U)
#define LCD_COLOR_WHITE  RGB565_SWAP(0xFFFFU)
#define LCD_COLOR_BLACK  RGB565_SWAP(0x0000U)
#define LCD_FILL_ROWS        8U
#define LCD_MAX_SCALE        5U
#define LCD_GLYPH_W          5U
#define LCD_GLYPH_H          7U
#define LCD_CELL_W           6U
#define LCD_NUMBER_MAX_W     24U
#define LCD_NUMBER_MAX_H     40U
#define LCD_NUMBER_MIN_H     26U
#define LCD_CHAR_DEGREE     '\x7F' /* private code for the degree sign */
_Static_assert(UI_CARD_FIRST_Y + 3 * UI_CARD_HEIGHT + 2 * UI_CARD_GAP <=
               BOARD_ONX_LCD_HEIGHT, "Dashboard cards must fit the display");
_Static_assert(UI_CARD_FIRST_Y >= UI_HEADER_HEIGHT,
               "Dashboard cards must start below the page indicator");
_Static_assert(UI_CARD_WIDTH - 16 >= UI_TITLE_LINE_CHARS * LCD_CELL_W * 2U,
               "Dashboard title lines must fit at 2x text scale");

static const char *TAG = "onx_display";
static bool s_initialized;
static SemaphoreHandle_t s_ui_mutex;
static esp_lcd_panel_io_handle_t s_lcd_io;
static esp_lcd_panel_handle_t s_panel;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_pcf8574;
static i2c_master_dev_handle_t s_touch;
static bool s_touch_ready;
static char s_last_status[64];
static SemaphoreHandle_t s_trans_done; /* given by the LCD color-transfer-done ISR */
static TaskHandle_t s_ui_task;
static bool s_screen_dirty;
static bool s_header_dirty;

typedef struct {
    parameter_descriptor_t descriptor;
    float value;
    int64_t timestamp_us;
    parameter_validity_t validity;
    bool shown_live; /* card on screen is currently drawn as LIVE */
    bool value_dirty;
} dashboard_item_t;

static dashboard_item_t s_items[UI_MAX_ITEMS];
static size_t s_item_count;
static size_t s_page;
static uint16_t s_fill_buffer[BOARD_ONX_LCD_WIDTH * LCD_FILL_ROWS];
static uint16_t s_glyph_buffer[LCD_CELL_W * LCD_MAX_SCALE * LCD_GLYPH_H * LCD_MAX_SCALE];
static uint16_t s_number_glyph_buffer[LCD_NUMBER_MAX_W * LCD_NUMBER_MAX_H];

static esp_err_t draw_current_screen(void);
static void notify_ui_task(void)
{
    if (s_ui_task != NULL) xTaskNotifyGive(s_ui_task);
}

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
    static const uint8_t percent[5] = {0x23,0x13,0x08,0x64,0x62};
    static const uint8_t lparen[5] = {0x00,0x1C,0x22,0x41,0x00};
    static const uint8_t rparen[5] = {0x00,0x41,0x22,0x1C,0x00};
    static const uint8_t comma[5] = {0x00,0x50,0x30,0x00,0x00};
    static const uint8_t degree[5] = {0x00,0x06,0x09,0x09,0x06};
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
    case '%': return percent;
    case '(': return lparen;
    case ')': return rparen;
    case ',': return comma;
    case LCD_CHAR_DEGREE: return degree;
    case ' ': return blank;
    default: return blank;
    }
}

/* Converts UTF-8 / Latin-1 text into the display's 1-byte charset: the degree
 * sign becomes LCD_CHAR_DEGREE, any other non-ASCII sequence becomes ONE space.
 * dst is always NUL-terminated; returns the resulting length. Idempotent. */
static size_t normalize_text(const char *src, char *dst, size_t cap)
{
    size_t n = 0U;
    if (cap == 0U) return 0U;
    if (src == NULL) src = "";
    for (size_t i = 0U; src[i] != '\0' && n + 1U < cap; ++i) {
        const unsigned char c = (unsigned char)src[i];
        if (c < 0x80U) {
            dst[n++] = (char)c;
        } else if (c == 0xB0U) {                    /* Latin-1 degree sign */
            dst[n++] = LCD_CHAR_DEGREE;
        } else if (c >= 0xC0U) {                    /* UTF-8 lead byte */
            const size_t extra = c >= 0xF0U ? 3U : (c >= 0xE0U ? 2U : 1U);
            if (c == 0xC2U && (unsigned char)src[i + 1U] == 0xB0U) {
                dst[n++] = LCD_CHAR_DEGREE;
            } else {
                dst[n++] = ' ';
            }
            for (size_t k = 0U; k < extra && src[i + 1U] != '\0'; ++k) ++i;
        }
        /* stray continuation bytes (0x80..0xBF) are dropped */
    }
    dst[n] = '\0';
    return n;
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
    esp_err_t err = i2c_master_probe(s_i2c_bus, CST826_I2C_ADDRESS, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CST826 did not ACK at I2C address 0x%02X: %s",
                 CST826_I2C_ADDRESS, esp_err_to_name(err));
        return err;
    }

    const i2c_device_config_t touch_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CST826_I2C_ADDRESS,
        .scl_speed_hz = PCF_I2C_SPEED_HZ,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &touch_cfg, &s_touch);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not add CST826 I2C device at 0x%02X: %s",
                 CST826_I2C_ADDRESS, esp_err_to_name(err));
        s_touch = NULL;
        return err;
    }

    s_touch_ready = true;
    ESP_LOGI(TAG, "CST826 capacitive touch detected at I2C address 0x%02X on GPIO%d/GPIO%d",
             CST826_I2C_ADDRESS, BOARD_ONX_LCD_I2C_SCL, BOARD_ONX_LCD_I2C_SDA);
    return ESP_OK;
}

/* esp_lcd_panel_draw_bitmap() only QUEUES the transfer: the pixel buffer must not
 * be touched until the DMA is done. All drawing therefore goes through lcd_draw(),
 * which waits for the transfer-done callback before returning. */
static bool on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    (void)io;
    (void)edata;
    (void)user_ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_trans_done, &woken);
    return woken == pdTRUE;
}

static esp_err_t lcd_draw(int x_start, int y_start, int x_end, int y_end,
                          const void *pixels)
{
    if (s_panel == NULL || s_trans_done == NULL) return ESP_ERR_INVALID_STATE;
    (void)xSemaphoreTake(s_trans_done, 0); /* drop any stale completion */
    esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, x_start, y_start,
                                              x_end, y_end, pixels);
    if (err != ESP_OK) return err;
    if (xSemaphoreTake(s_trans_done, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGE(TAG, "LCD color transfer did not complete");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t init_lcd_panel(void)
{
    if (s_trans_done == NULL) {
        s_trans_done = xSemaphoreCreateBinary();
        if (s_trans_done == NULL) return ESP_ERR_NO_MEM;
    }
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
        .on_color_trans_done = on_color_trans_done,
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
    /* Backlight stays off until the first frame is drawn (GRAM is random after reset). */
    if ((err = gpio_set_level(BOARD_ONX_LCD_PIN_BACKLIGHT, 0)) != ESP_OK) return err;

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
        esp_err_t err = lcd_draw(x, y + offset, x + width, y + offset + rows, s_fill_buffer);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static esp_err_t draw_bitmap(int x, int y, int width, int height,
                             const uint16_t *pixels)
{
    if (pixels == NULL || width <= 0 || height <= 0) return ESP_ERR_INVALID_ARG;
    if (x < 0 || y < 0 || x + width > (int)BOARD_ONX_LCD_WIDTH ||
        y + height > (int)BOARD_ONX_LCD_HEIGHT) return ESP_ERR_INVALID_ARG;
    esp_err_t err = lcd_draw(x, y, x + width, y + height, pixels);
    if (err != ESP_OK) return err;
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
    return draw_bitmap(x, y, (int)width, (int)height, s_glyph_buffer);
}

static esp_err_t draw_text_box(int x, int y, int width, int height, unsigned scale,
                               uint16_t foreground, uint16_t background, const char *text)
{
    char norm[65];
    if (width <= 0 || height <= 0) return ESP_ERR_INVALID_ARG;
    size_t len = normalize_text(text, norm, sizeof(norm));
    text = norm;
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

static uint8_t numeric_segments(char ch)
{
    static const uint8_t digit_segments[10] = {
        0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
        0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU,
    };
    if (ch >= '0' && ch <= '9') return digit_segments[(unsigned)(ch - '0')];
    return ch == '-' ? 0x40U : 0U;
}

static esp_err_t draw_numeric_glyph(int x, int y, char ch, unsigned height,
                                    uint16_t foreground, uint16_t background)
{
    const unsigned thickness = height / 9U > 2U ? height / 9U : 3U;
    const unsigned digit_width = height * 3U / 5U;
    const bool decimal = ch == '.';
    const unsigned width = decimal ? height / 4U : digit_width;
    if (height > LCD_NUMBER_MAX_H || width > LCD_NUMBER_MAX_W) {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t pixels = (size_t)width * height;
    for (size_t i = 0; i < pixels; ++i) s_number_glyph_buffer[i] = background;

    if (decimal) {
        const unsigned dot_size = thickness;
        const unsigned dot_x = width > dot_size ? width - dot_size : 0U;
        const unsigned dot_y = height - dot_size;
        for (unsigned py = dot_y; py < height; ++py) {
            for (unsigned px = dot_x; px < width; ++px) {
                s_number_glyph_buffer[(size_t)py * width + px] = foreground;
            }
        }
    } else {
        const uint8_t segments = numeric_segments(ch);
        const unsigned middle_y = (height - thickness) / 2U;
        const unsigned vertical_height = middle_y > thickness
            ? middle_y - thickness : 1U;
        const unsigned lower_y = middle_y + thickness;
        const unsigned lower_height = height > thickness + lower_y
            ? height - thickness - lower_y : 1U;
        const unsigned horizontal_width = digit_width - 2U * thickness;
        const unsigned segment_x[7] = {
            thickness, digit_width - thickness, digit_width - thickness,
            thickness, 0U, 0U, thickness,
        };
        const unsigned segment_y[7] = {
            0U, thickness, lower_y, height - thickness,
            lower_y, thickness, middle_y,
        };
        const unsigned segment_w[7] = {
            horizontal_width, thickness, thickness, horizontal_width,
            thickness, thickness, horizontal_width,
        };
        const unsigned segment_h[7] = {
            thickness, vertical_height, lower_height, thickness,
            lower_height, vertical_height, thickness,
        };
        for (unsigned segment = 0; segment < 7U; ++segment) {
            if ((segments & (1U << segment)) == 0U) continue;
            for (unsigned py = segment_y[segment];
                 py < segment_y[segment] + segment_h[segment]; ++py) {
                for (unsigned px = segment_x[segment];
                     px < segment_x[segment] + segment_w[segment]; ++px) {
                    s_number_glyph_buffer[(size_t)py * width + px] = foreground;
                }
            }
        }
    }

    return draw_bitmap(x, y, (int)width, (int)height, s_number_glyph_buffer);
}

static unsigned numeric_glyph_width(char ch, unsigned height)
{
    return ch == '.' ? height / 4U : height * 3U / 5U;
}

static unsigned numeric_text_width(const char *text, unsigned height)
{
    const unsigned gap = height / 9U > 2U ? height / 9U : 3U;
    unsigned width = 0U;
    for (size_t i = 0U; text[i] != '\0'; ++i) {
        width += numeric_glyph_width(text[i], height);
        if (text[i + 1U] != '\0') width += gap;
    }
    return width;
}

static esp_err_t draw_numeric_text(int x, int y, const char *text, unsigned height,
                                   uint16_t foreground, uint16_t background, int max_x)
{
    const unsigned gap = height / 9U > 2U ? height / 9U : 3U;
    for (size_t i = 0U; text[i] != '\0'; ++i) {
        const int glyph_w = (int)numeric_glyph_width(text[i], height);
        if (x + glyph_w > max_x) break; /* never draw outside the card */
        esp_err_t err = draw_numeric_glyph(x, y, text[i], height,
                                           foreground, background);
        if (err != ESP_OK) return err;
        x += glyph_w;
        if (text[i + 1U] != '\0') x += (int)gap;
    }
    return ESP_OK;
}

static size_t page_count(void)
{
    const size_t pages = (s_item_count + 2U) / 3U;
    return pages == 0U ? 1U : pages;
}

static bool item_is_fresh(const dashboard_item_t *item, int64_t now_us)
{
    if (item->validity != PARAM_VALIDITY_VALID || item->timestamp_us <= 0) return false;
    return now_us - item->timestamp_us <
           (int64_t)item->descriptor.update_period_ms * UI_STALE_MULTIPLIER * 1000LL;
}

static void split_title(const char *title, char first[UI_TITLE_LINE_CHARS + 1U],
                        char second[UI_TITLE_LINE_CHARS + 1U])
{
    first[0] = '\0';
    second[0] = '\0';
    char *line = first;
    size_t line_length = 0U;
    const char *cursor = title;

    while (*cursor != '\0') {
        while (*cursor == ' ') ++cursor;
        if (*cursor == '\0') break;
        const char *word = cursor;
        while (*cursor != '\0' && *cursor != ' ') ++cursor;
        size_t word_length = (size_t)(cursor - word);
        if (line_length != 0U && line_length + 1U + word_length > UI_TITLE_LINE_CHARS) {
            if (line == second) break;
            line = second;
            line_length = 0U;
        }
        if (word_length > UI_TITLE_LINE_CHARS - line_length) {
            word_length = UI_TITLE_LINE_CHARS - line_length;
        }
        if (line_length != 0U) line[line_length++] = ' ';
        memcpy(line + line_length, word, word_length);
        line_length += word_length;
        line[line_length] = '\0';
    }
}

static esp_err_t draw_card_value(size_t item_index, size_t slot)
{
    if (item_index >= s_item_count) return ESP_OK;
    dashboard_item_t *item = &s_items[item_index];
    const int x = UI_CARD_X;
    const int y = UI_CARD_FIRST_Y + (int)slot * (UI_CARD_HEIGHT + UI_CARD_GAP);
    esp_err_t err = fill_rect(x + 8, y + 35, UI_CARD_WIDTH - 16, 58,
                              LCD_COLOR_NAVY);
    if (err != ESP_OK) return err;

    char status[12];
    const int64_t now_us = esp_timer_get_time();
    const bool finite_value = isfinite((double)item->value);
    const bool fresh = item_is_fresh(item, now_us) && finite_value;
    char number[16];
    char unit_text[17] = "";
    if (fresh) {
        (void)snprintf(number, sizeof(number), "%.1f", (double)item->value);
        const size_t length = strnlen(number, sizeof(number));
        if (length >= 2U && number[length - 2U] == '.' && number[length - 1U] == '0')
            number[length - 2U] = '\0';
        if (strcmp(number, "-0") == 0) { number[0] = '0'; number[1] = '\0'; }
        (void)normalize_text(item->descriptor.unit, unit_text, sizeof(unit_text));
        (void)snprintf(status, sizeof(status), "LIVE");
    } else {
        (void)snprintf(number, sizeof(number), "--");
        switch (item->validity) {
        case PARAM_VALIDITY_VALID:
            if (item->timestamp_us <= 0) (void)snprintf(status, sizeof(status), "NO DATA");
            else if (!finite_value) (void)snprintf(status, sizeof(status), "INVALID");
            else (void)snprintf(status, sizeof(status), "STALE");
            break;
        case PARAM_VALIDITY_STALE: (void)snprintf(status, sizeof(status), "STALE"); break;
        case PARAM_VALIDITY_TIMEOUT: (void)snprintf(status, sizeof(status), "TIMEOUT"); break;
        case PARAM_VALIDITY_INVALID: (void)snprintf(status, sizeof(status), "INVALID"); break;
        default: (void)snprintf(status, sizeof(status), "NO DATA"); break;
        }
    }

    const int value_area_width = UI_CARD_WIDTH - 16;
    const size_t unit_length = strnlen(unit_text, sizeof(unit_text));
    const unsigned unit_width = (unsigned)(unit_length * LCD_CELL_W * 2U);
    const unsigned unit_gap = 8U;
    bool show_unit = unit_length > 0U;
    unsigned number_height = LCD_NUMBER_MAX_H;
    for (;;) {
        const unsigned reserved = show_unit ? unit_width + unit_gap : 0U;
        if (numeric_text_width(number, number_height) + reserved <=
            (unsigned)value_area_width) break;
        if (number_height > LCD_NUMBER_MIN_H) {
            --number_height;
        } else if (show_unit) {
            show_unit = false;
            number_height = LCD_NUMBER_MAX_H;
        } else {
            break;
        }
    }
    const unsigned number_width = numeric_text_width(number, number_height);
    const unsigned group_width = number_width + (show_unit ? unit_width + unit_gap : 0U);
    int value_x = x + 8 + (value_area_width - (int)group_width) / 2;
    if (value_x < x + 8) value_x = x + 8;
    const int right_limit = x + UI_CARD_WIDTH - 8;
    const uint16_t value_color = fresh ? LCD_COLOR_GREEN : LCD_COLOR_GRAY;

    if ((err = draw_numeric_text(value_x, y + 39, number, number_height,
                                 value_color, LCD_COLOR_NAVY, right_limit)) != ESP_OK)
        return err;
    if (show_unit) {
        const int unit_x = value_x + (int)number_width + (int)unit_gap;
        if (unit_x + (int)unit_width <= right_limit &&
            (err = draw_text_box(unit_x, y + 53, (int)unit_width, 20, 2,
                                 value_color, LCD_COLOR_NAVY, unit_text)) != ESP_OK)
            return err;
    }
    err = draw_text_box(x + 8, y + 83, UI_CARD_WIDTH - 16, 9, 1,
                        fresh ? LCD_COLOR_GREEN : LCD_COLOR_RED,
                        LCD_COLOR_NAVY, status);
    if (err == ESP_OK) item->shown_live = fresh;
    return err;
}

static esp_err_t draw_card(size_t item_index, size_t slot)
{
    const int x = UI_CARD_X;
    const int y = UI_CARD_FIRST_Y + (int)slot * (UI_CARD_HEIGHT + UI_CARD_GAP);
    esp_err_t err;
    if ((err = fill_rect(x, y, UI_CARD_WIDTH, UI_CARD_HEIGHT, LCD_COLOR_NAVY)) != ESP_OK)
        return err;
    if ((err = fill_rect(x, y, UI_CARD_WIDTH, 2, LCD_COLOR_CYAN)) != ESP_OK) return err;
    if ((err = fill_rect(x, y + UI_CARD_HEIGHT - 2, UI_CARD_WIDTH, 2, LCD_COLOR_CYAN)) != ESP_OK)
        return err;

    if (item_index >= s_item_count) {
        return draw_text_box(x + 8, y + 20, UI_CARD_WIDTH - 16, 22, 2,
                             LCD_COLOR_GRAY, LCD_COLOR_NAVY, "--");
    }

    dashboard_item_t *item = &s_items[item_index];
    item->shown_live = false; /* set again only after a fully successful draw */

    char name_text[65];
    char title_first[UI_TITLE_LINE_CHARS + 1U];
    char title_second[UI_TITLE_LINE_CHARS + 1U];
    (void)normalize_text(item->descriptor.name, name_text, sizeof(name_text));
    split_title(name_text, title_first, title_second);
    if ((err = draw_text_box(x + 8, y + 4, UI_CARD_WIDTH - 16, 15, 2,
                             LCD_COLOR_CYAN, LCD_COLOR_NAVY,
                             title_first)) != ESP_OK) return err;
    if ((err = draw_text_box(x + 8, y + 19, UI_CARD_WIDTH - 16, 15, 2,
                             LCD_COLOR_CYAN, LCD_COLOR_NAVY,
                             title_second)) != ESP_OK) return err;

    err = draw_card_value(item_index, slot);
    return err;
}

static esp_err_t draw_page_header(void)
{
    char page_text[16];
    (void)snprintf(page_text, sizeof(page_text), "%u/%u",
                   (unsigned)(s_page + 1U), (unsigned)page_count());
    esp_err_t err = fill_rect(0, 0, BOARD_ONX_LCD_WIDTH, UI_HEADER_HEIGHT, LCD_COLOR_NAVY);
    if (err != ESP_OK) return err;
    err = draw_text_box(6, 8, (int)BOARD_ONX_LCD_WIDTH - 6 - 76, 9, 1,
                        LCD_COLOR_WHITE, LCD_COLOR_NAVY, s_last_status);
    if (err != ESP_OK) return err;
    return draw_text_box(BOARD_ONX_LCD_WIDTH - 70, 4, 66, 16, 2,
                         LCD_COLOR_CYAN, LCD_COLOR_NAVY, page_text);
}

static esp_err_t draw_current_screen(void)
{
    esp_err_t err = fill_rect(0, 0, BOARD_ONX_LCD_WIDTH, BOARD_ONX_LCD_HEIGHT, LCD_COLOR_BLACK);
    if (err != ESP_OK) return err;
    if ((err = draw_page_header()) != ESP_OK) return err;
    const size_t first = s_page * 3U;
    for (size_t slot = 0; slot < 3U; ++slot) {
        if ((err = draw_card(first + slot, slot)) != ESP_OK) return err;
        if (first + slot < s_item_count) s_items[first + slot].value_dirty = false;
    }
    return ESP_OK;
}

static void onx_ui_task(void *arg)
{
    (void)arg;
    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (s_ui_mutex == NULL ||
            xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(100)) != pdTRUE) continue;

        esp_err_t err = ESP_OK;
        if (s_screen_dirty) {
            err = draw_current_screen();
            if (err == ESP_OK) {
                s_screen_dirty = false;
                s_header_dirty = false;
            }
        } else if (s_header_dirty) {
            err = draw_page_header();
            if (err == ESP_OK) s_header_dirty = false;
        } else {
            const size_t first = s_page * 3U;
            const size_t end = first + 3U < s_item_count ? first + 3U : s_item_count;
            for (size_t i = first; i < end; ++i) {
                if (!s_items[i].value_dirty) continue;
                err = draw_card_value(i, i - first);
                if (err != ESP_OK) break;
                s_items[i].value_dirty = false;
            }
        }
        if (err != ESP_OK) ESP_LOGW(TAG, "Asynchronous LCD redraw failed: %s",
                                    esp_err_to_name(err));
        xSemaphoreGive(s_ui_mutex);
    }
}

static void switch_page(int direction)
{
    if (s_ui_mutex == NULL || xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex for page switch");
        return;
    }
    const size_t pages = page_count();
    if (pages <= 1U) {
        xSemaphoreGive(s_ui_mutex);
        return;
    }
    size_t next = s_page;
    if (direction > 0 && next + 1U < pages) ++next;
    if (direction < 0 && next > 0U) --next;
    if (next != s_page) {
        s_page = next;
        s_screen_dirty = true;
        s_header_dirty = false;
        ESP_LOGI(TAG, "Dashboard page %u/%u",
                 (unsigned)(s_page + 1U), (unsigned)pages);
    }
    xSemaphoreGive(s_ui_mutex);
    notify_ui_task();
}

static bool normalize_touch(uint16_t raw_x, uint16_t raw_y, int16_t *x, int16_t *y)
{
    if (raw_x < BOARD_ONX_LCD_WIDTH && raw_y < BOARD_ONX_LCD_HEIGHT) {
        *x = (int16_t)raw_x;
        *y = (int16_t)raw_y;
        return true;
    }
    if (raw_x < BOARD_ONX_LCD_HEIGHT && raw_y < BOARD_ONX_LCD_WIDTH) {
        *x = (int16_t)raw_y;
        *y = (int16_t)raw_x;
        return true;
    }
    return false;
}

static void cst826_touch_task(void *arg)
{
    bool finger_down = false;
    int64_t last_error_log_us = 0;
    swipe_gesture_t gesture = {0};
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

            int16_t x = 0;
            int16_t y = 0;
            const bool coordinate_valid = normalize_touch(raw_x, raw_y, &x, &y);
            if (touch_count > 0U && (event == 0U || event == 2U)) {
                if (coordinate_valid && !finger_down) {
                    swipe_gesture_begin(&gesture, x, y);
                    finger_down = true;
                } else if (coordinate_valid) {
                    swipe_gesture_update(&gesture, x, y);
                }
            } else if (event == 1U && finger_down) {
                if (!coordinate_valid) {
                    x = gesture.last_x;
                    y = gesture.last_y;
                }
                const swipe_direction_t direction = swipe_gesture_end(&gesture, x, y);
                if (direction == SWIPE_LEFT) switch_page(1);
                else if (direction == SWIPE_RIGHT) switch_page(-1);
                finger_down = false;
            } else if (touch_count == 0U && finger_down) {
                const swipe_direction_t direction =
                    swipe_gesture_end(&gesture, gesture.last_x, gesture.last_y);
                if (direction == SWIPE_LEFT) switch_page(1);
                else if (direction == SWIPE_RIGHT) switch_page(-1);
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
    if (strncmp(s_last_status, status, sizeof(s_last_status) - 1U) == 0) {
        xSemaphoreGive(s_ui_mutex);
        return;
    }
    size_t n = strnlen(status, sizeof(s_last_status) - 1U);
    memcpy(s_last_status, status, n);
    s_last_status[n] = '\0';
    s_header_dirty = true;
    xSemaphoreGive(s_ui_mutex);
    notify_ui_task();
}

static int find_item(uint8_t pid)
{
    for (size_t i = 0; i < s_item_count; ++i) {
        if (s_items[i].descriptor.pid == pid) return (int)i;
    }
    return -1;
}

void onx_display_set_catalog(const parameter_descriptor_t *catalog, size_t count)
{
    static const uint8_t preferred[] = {0x0CU, 0x05U, 0x04U, 0x0DU, 0x0BU, 0x0FU};
    if (catalog == NULL && count != 0U) return;
    if (count > UI_MAX_ITEMS) {
        ESP_LOGE(TAG, "Dashboard catalog exceeds supported capacity (%u > %u)",
                 (unsigned)count, (unsigned)UI_MAX_ITEMS);
        return;
    }
    const bool can_draw = s_initialized && s_ui_mutex != NULL;
    if (can_draw && xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex to configure dashboard");
        return;
    }

    memset(s_items, 0, sizeof(s_items));
    s_item_count = 0U;
    for (size_t preferred_index = 0;
         preferred_index < sizeof(preferred) / sizeof(preferred[0]);
         ++preferred_index) {
        for (size_t i = 0; i < count; ++i) {
            if (catalog[i].pid == preferred[preferred_index]) {
                s_items[s_item_count++].descriptor = catalog[i];
                break;
            }
        }
    }
    for (size_t i = 0; i < count; ++i) {
        bool already_added = false;
        for (size_t j = 0; j < s_item_count; ++j) {
            if (s_items[j].descriptor.pid == catalog[i].pid) {
                already_added = true;
                break;
            }
        }
        if (!already_added) s_items[s_item_count++].descriptor = catalog[i];
    }

    if (!can_draw) return;
    s_page = 0U;
    s_screen_dirty = true;
    xSemaphoreGive(s_ui_mutex);
    notify_ui_task();
}

void onx_display_update_parameter(uint8_t pid, float value,
                                  parameter_validity_t validity,
                                  int64_t timestamp_us)
{
    if (!s_initialized || s_ui_mutex == NULL) return;
    if (xSemaphoreTake(s_ui_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not acquire LCD mutex for PID 0x%02X update", pid);
        return;
    }
    const int index = find_item(pid);
    if (index < 0) {
        xSemaphoreGive(s_ui_mutex);
        return;
    }
    dashboard_item_t *item = &s_items[index];
    /* Redraw also when valid data arrives but the card is not currently shown as
     * LIVE (e.g. it was drawn "--" while off-screen), even if the value is equal. */
    const bool changed = item->validity != validity || item->value != value ||
                         (validity == PARAM_VALIDITY_VALID && !item->shown_live);
    item->value = value;
    item->validity = validity;
    item->timestamp_us = timestamp_us;
    if (changed) item->value_dirty = true;
    xSemaphoreGive(s_ui_mutex);
    if (changed) notify_ui_task();
}

void onx_display_poll(int64_t now_us)
{
    if (!s_initialized || s_ui_mutex == NULL ||
        xSemaphoreTake(s_ui_mutex, 0) != pdTRUE) return;
    const size_t first = s_page * 3U;
    const size_t end = first + 3U < s_item_count ? first + 3U : s_item_count;
    bool changed = false;
    for (size_t i = first; i < end; ++i) {
        dashboard_item_t *item = &s_items[i];
        if (item->validity == PARAM_VALIDITY_VALID &&
            now_us - item->timestamp_us >=
                (int64_t)item->descriptor.update_period_ms * UI_STALE_MULTIPLIER * 1000LL) {
            item->validity = PARAM_VALIDITY_STALE;
            item->value_dirty = true;
            changed = true;
        }
    }
    xSemaphoreGive(s_ui_mutex);
    if (changed) notify_ui_task();
}

void onx_display_show_waiting_for_rpm(const char *status)
{
    if (!s_initialized) return;
    onx_display_set_status(status != NULL ? status : "WAITING FOR OBD DATA");
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
    s_item_count = 0U;
    s_page = 0U;
    s_screen_dirty = false;
    s_header_dirty = false;
    err = draw_current_screen();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initial ST7789 UI draw failed: %s", esp_err_to_name(err));
        return err;
    }
    (void)gpio_set_level(BOARD_ONX_LCD_PIN_BACKLIGHT, 1);

    s_initialized = true;
    if (xTaskCreate(onx_ui_task, "onx_ui", 4096, NULL, 4, &s_ui_task) != pdPASS) {
        s_ui_task = NULL;
        s_initialized = false;
        ESP_LOGE(TAG, "Could not create asynchronous LCD UI task");
        return ESP_ERR_NO_MEM;
    }
    if (s_touch_ready) {
        if (xTaskCreate(cst826_touch_task, "cst826_touch", 3072, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Could not create CST826 touch task; page navigation is unavailable");
            s_touch_ready = false;
        } else {
            ESP_LOGI(TAG, "CST826 touch polling task started (%u ms period); swipe horizontally to switch pages",
                     (unsigned)CST826_TOUCH_POLL_MS);
        }
    }
    ESP_LOGI(TAG, "ONX2432G028 LCD ready: waiting for ECU-supported PID catalog, 240x320");
    return ESP_OK;
}
