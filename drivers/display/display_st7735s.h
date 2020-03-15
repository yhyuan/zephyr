/*
 * Copyright (c) 2019 Marc Reilly
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ST7735S_DISPLAY_DRIVER_H__
#define ST7735S_DISPLAY_DRIVER_H__

#include <zephyr.h>

#define ST7735S_CMD_NOP		0x00
#define ST7735S_CMD_SWRESET	0x01
#define ST7735S_CMD_RDDID	0x04
#define ST7735S_CMD_RDDST	0x09
#define ST7735S_CMD_RDDPM	0x0A
#define ST7735S_CMD_RDDMADCTL	0x0B
#define ST7735S_CMD_RDDCOLMOD	0x0C
#define ST7735S_CMD_RDDIM	0x0D
#define ST7735S_CMD_RDDSM	0x0E
#define ST7735S_CMD_RDDSDR	0x0F
#define ST7735S_CMD_SLPIN	0x10
#define ST7735S_CMD_SLPOUT	0x11
#define ST7735S_CMD_PTLON	0x12
#define ST7735S_CMD_NORON	0x13
#define ST7735S_CMD_INVOFF	0x20
#define ST7735S_CMD_INVON	0x21
#define ST7735S_CMD_GAMSET	0x26
#define ST7735S_CMD_DISPOFF	0x28
#define ST7735S_CMD_DISPON	0x29
#define ST7735S_CMD_CASET	0x2A
#define ST7735S_CMD_RASET	0x2B
#define ST7735S_CMD_RAMWR	0x2C
#define ST7735S_CMD_RGBSET	0x2D
#define ST7735S_CMD_RAMRD	0x2E
#define ST7735S_CMD_PTLAR	0x30
#define ST7735S_CMD_SCRLAR	0x33
#define ST7735S_CMD_TEOFF	0x34
#define ST7735S_CMD_TEON	0x35
#define ST7735S_CMD_MADCTL	0x36
#define ST7735S_CMD_VSCSAD	0x37
#define ST7735S_CMD_IDMOFF	0x38
#define ST7735S_CMD_IDMON	0x39
#define ST7735S_CMD_COLMOD	0x3A
#define ST7735S_CMD_RDID1	0xDA
#define ST7735S_CMD_RDID2	0xDB
#define ST7735S_CMD_RDID3	0xDC
#define ST7735S_CMD_FRMCTR1	0xB1
#define ST7735S_CMD_FRMCTR2	0xB2
#define ST7735S_CMD_FRMCTR3	0xB3
#define ST7735S_CMD_INVCTR	0xB4
#define ST7735S_CMD_PWCTR1	0xC0
#define ST7735S_CMD_PWCTR2	0xC1
#define ST7735S_CMD_PWCTR3	0xC2
#define ST7735S_CMD_PWCTR4	0xC3
#define ST7735S_CMD_PWCTR5	0xC4
#define ST7735S_CMD_VMCTR1	0xC5
#define ST7735S_CMD_VMOFCTR	0xC7
#define ST7735S_CMD_WRID2	0xD1
#define ST7735S_CMD_WRID3	0xD2
#define ST7735S_CMD_NVCTR1	0xD9
#define ST7735S_CMD_NVCTR2	0xDE
#define ST7735S_CMD_NVCTR3	0xDF
#define ST7735S_CMD_GAMCTRP1	0xE0
#define ST7735S_CMD_GAMCTRN1	0xE1
#define ST7735S_CMD_GCV		0xFC

#define ST7735S_SW_RESET ST7735S_CMD_SWRESET
#define ST7735S_READ_DISPLAY_ID ST7735S_CMD_RDDID
#define ST7735S_READ_DISPLAY_STATUS ST7735S_CMD_RDDST
#define ST7735S_READ_DISPLAY_POWER_MODE ST7735S_CMD_RDDPM
#define ST7735S_READ_DISPLAY_MEMORY_ACCESS_DATA_CONTROL ST7735S_CMD_RDDMADCTL
#define ST7735S_READ_DISPLAY_COLOR_MODE ST7735S_CMD_RDDCOLMOD
#define ST7735S_READ_DISPLAY_IMAGE_MODE ST7735S_CMD_RDDIM
#define ST7735S_READ_DISPLAY_SIGNAL_MODE ST7735S_CMD_RDDSM
#define ST7735S_READ_DISPLAY_SELF_DIAGNOSTIC_RESULT ST7735S_CMD_RDDSDR
#define ST7735S_SLEEP_IN ST7735S_CMD_SLPIN
#define ST7735S_SLEEP_OUT ST7735S_CMD_SLPOUT
#define ST7735S_PARTIAL_MODE_ON ST7735S_CMD_PTLON
#define ST7735S_NORMAL_MODE_ON ST7735S_CMD_NORON
#define ST7735S_INVERT_OFF ST7735S_CMD_INVOFF
#define ST7735S_INVERT_ON ST7735S_CMD_INVON
#define ST7735S_GAMMA_CURVE_SET ST7735S_CMD_GAMSET
#define ST7735S_DISPLAY_OFF ST7735S_CMD_DISPOFF
#define ST7735S_DISPLAY_ON ST7735S_CMD_DISPON
#define ST7735S_COLUMN_ADDRESS_SET ST7735S_CMD_CASET
#define ST7735S_ROW_ADDRESS_SET ST7735S_CMD_RASET
#define ST7735S_RAM_WRITE ST7735S_CMD_RAMWR
#define ST7735S_RGB_SET ST7735S_CMD_RGBSET
#define ST7735S_RAM_READ ST7735S_CMD_RAMRD
#define ST7735S_PARTIAL_START_END_ADDRESS_SET ST7735S_CMD_PTLAR
#define ST7735S_SCROLL_AREA_SET ST7735S_CMD_SCRLAR
#define ST7735S_TERAING_EFFECT_OFF ST7735S_CMD_TEOFF
#define ST7735S_TEARING_EFFECT_ON ST7735S_CMD_TEON
#define ST7735S_MEMORY_ACCESS_DATA_CONTROL ST7735S_CMD_MADCTL
#define ST7735S_SCROLL_START_ADDRESS ST7735S_CMD_VSCSAD
#define ST7735S_IDLE_MODE_OFF ST7735S_CMD_IDMOFF
#define ST7735S_IDLE_MODE_ON ST7735S_CMD_IDMON
#define ST7735S_COLOR_MODE ST7735S_CMD_COLMOD
#define ST7735S_READ_ID1 ST7735S_CMD_RDID1
#define ST7735S_READ_ID2 ST7735S_CMD_RDID2
#define ST7735S_READ_ID3 ST7735S_CMD_RDID3
#define ST7735S_FRAME_CONTROL1 ST7735S_CMD_FRMCTR1
#define ST7735S_FRAME_CONTROL2 ST7735S_CMD_FRMCTR2
#define ST7735S_FRAME_CONTROL3 ST7735S_CMD_FRMCTR3
#define ST7735S_INVERSION_CONTROL ST7735S_CMD_INVCTR
#define ST7735S_POWER_CONTROL1 ST7735S_CMD_PWCTR1
#define ST7735S_POWER_CONTROL2 ST7735S_CMD_PWCTR2
#define ST7735S_POWER_CONTROL3 ST7735S_CMD_PWCTR3
#define ST7735S_POWER_CONTROL4 ST7735S_CMD_PWCTR4
#define ST7735S_POWER_CONTROL5 ST7735S_CMD_PWCTR5
#define ST7735S_VCOM_CONTROL1 ST7735S_CMD_VMCTR1
#define ST7735S_VCOM_OFFSET ST7735S_CMD_VMOFCTR
#define ST7735S_WRITE_ID2 ST7735S_CMD_WRID2
#define ST7735S_WRITE_ID3 ST7735S_CMD_WRID3
#define ST7735S_NVM_CONTROL1 ST7735S_CMD_NVCTR1
#define ST7735S_NVM_CONTROL2 ST7735S_CMD_NVCTR2
#define ST7735S_NVM_CONTROL3 ST7735S_CMD_NVCTR3
#define ST7735S_GAMMA_ADJUST_P ST7735S_CMD_GAMCTRP1
#define ST7735S_GAMMA_ADJUST_N ST7735S_CMD_GAMCTRN1
#define ST7735S_GATE_CLOCK_VARIABLE ST7735S_CMD_GCV

#define ST7735S_MADCTL_MY_TOP_TO_BOTTOM		0x00
#define ST7735S_MADCTL_MY_BOTTOM_TO_TOP		0x80
#define ST7735S_MADCTL_MX_LEFT_TO_RIGHT		0x00
#define ST7735S_MADCTL_MX_RIGHT_TO_LEFT		0x40
#define ST7735S_MADCTL_MV_REVERSE_MODE		0x20
#define ST7735S_MADCTL_MV_NORMAL_MODE		0x00
#define ST7735S_MADCTL_ML			0x10
#define ST7735S_MADCTL_RBG			0x00
#define ST7735S_MADCTL_BGR			0x08
#define ST7735S_MADCTL_MH_LEFT_TO_RIGHT		0x00
#define ST7735S_MADCTL_MH_RIGHT_TO_LEFT		0x04

#define ST7735S_COLMOD_RGB_65K			(0x5 << 4)
#define ST7735S_COLMOD_RGB_262K			(0x6 << 4)
#define ST7735S_COLMOD_FMT_12bit		(3)
#define ST7735S_COLMOD_FMT_16bit		(5)
#define ST7735S_COLMOD_FMT_18bit		(6)

struct st7735s_data;

void st7735s_set_lcd_margins(struct st7735s_data *data,
			     u16_t x_offset, u16_t y_offset);

void st7735s_lcd_init(struct st7735s_data *data);

void st7735s_transmit(struct st7735s_data *data, u8_t cmd,
		      u8_t *tx_data, size_t tx_count);

#endif
