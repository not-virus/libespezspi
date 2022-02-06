/**
 * libespezspi.h
 * 
 * Written by Cameron Krueger
 * 8 August 2021
 * 
 * Header file for the libespili9341 display driver library
 */

#ifndef __LIBESPEZSPI_H__
#define __LIBESPEZSPI_H__

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/spi.h"

// These values must be defined for the library to work.
/*#ifndef EZSPI_PIN_MISO
#error EZSPI_PIN_MISO not defined
#endif

#ifndef EZSPI_PIN_MOSI
#error EZSPI_PIN_MOSI not defined
#endif

#ifndef EZSPI_PIN_CLK
#error EZSPI_PIN_CLK not defined
#endif

#ifndef EZSPI_PIN_DC
#error EZSPI_PIN_DC not defined
#endif*/

esp_err_t init_spi();
esp_err_t spi_send_command_with_data(uint8_t cmd, const uint8_t* data, uint32_t len);
esp_err_t spi_send_command(uint8_t cmd);
esp_err_t spi_send_raw(const uint8_t* data, uint32_t len);
void spi_read(unsigned char* buf, uint16_t len);


#endif