#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_MCP2515_SPI_HOST SPI2_HOST
#define BOARD_MCP2515_SPI_MODE 0
#define BOARD_MCP2515_PIN_SCK  38
#define BOARD_MCP2515_PIN_MOSI 39
#define BOARD_MCP2515_PIN_MISO 40
#define BOARD_MCP2515_PIN_CS   42

#define BOARD_MCP2515_OSC_HZ   8000000
#define BOARD_CAN_BITRATE_BPS  500000

/*
 * Datasheet CNF for 8 MHz / 500 kbps, 8 TQ, 75% sample point:
 *   TQ = 2 * (BRP+1) / Fosc; BRP=0 => 250 ns
 *   CNF1 0x00: SJW=1 TQ, BRP=0
 *   CNF2 0x91: BTLMODE=1, SAM=0, PS1=3 TQ, PropSeg=2 TQ
 *   CNF3 0x01: PS2=2 TQ
 *   1 + 2 + 3 + 2 = 8 TQ; sample point (1+2+3)/8 = 75%
 *
 * Library preset MCP_8MHz_500kBPS is 0x00/0x90/0x82 (62.5% SP). Not used.
 */
#define BOARD_MCP2515_CNF1_500K_8MHZ  0x00
#define BOARD_MCP2515_CNF2_500K_8MHZ  0x91
#define BOARD_MCP2515_CNF3_500K_8MHZ  0x01

/* 8 MHz / 250 kbit/s: 16 TQ at 250 ns/TQ, 62.5% sample point. */
#define BOARD_MCP2515_CNF1_250K_8MHZ  0x00
#define BOARD_MCP2515_CNF2_250K_8MHZ  0xB1
#define BOARD_MCP2515_CNF3_250K_8MHZ  0x85

#define BOARD_CAN_LOOPBACK_TEST_ID    0x123U

/* ONX2432G028 onboard ST7789 LCD; keep it off SPI2_HOST used by MCP2515. */
#define BOARD_ONX_LCD_SPI_HOST       SPI3_HOST
#define BOARD_ONX_LCD_PIN_SCLK       5
#define BOARD_ONX_LCD_PIN_MOSI       1
#define BOARD_ONX_LCD_PIN_DC         3
#define BOARD_ONX_LCD_PIN_CS         2
#define BOARD_ONX_LCD_PIN_BACKLIGHT  6
#define BOARD_ONX_LCD_I2C_SCL        7
#define BOARD_ONX_LCD_I2C_SDA        8
#define BOARD_ONX_LCD_WIDTH          240U
#define BOARD_ONX_LCD_HEIGHT         320U

#ifdef __cplusplus
}
#endif

#endif
