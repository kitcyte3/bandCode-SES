#include <app_uart.h>
#include <stdbool.h>
#include <stdint.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h" 
//
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

#define NRF_LOG_MODULE_NAME "APP"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */


void uart_error_handle(app_uart_evt_t * p_event);
static void show_error(void);
static void uart_loopback_test(void);

void MUX_init(void);

void twi_init (void);

void twi1_init (void);
void writei2c(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write_high, uint8_t data_to_write_low);
void writei2cOneByte(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write);
uint16_t readi2cHighLow(uint8_t deviceAddr, uint8_t read_reg_addr);
uint8_t readi2cOneByte(uint8_t deviceAddr, uint8_t read_reg_addr);

bool GPIOEXP1_init(void);
uint8_t GPIOEXP1_readPort0(void);

bool LED_BT_on(void);
bool LED_BT_off(void);
void MUX_set(bool s3, bool s2, bool s1, bool s0);
bool RGBW_on(void);
bool RGBW_off(void);

//returns an array of doubles, max value is 255 for each color, skips matrix multiplications
double* RGBW_get(void);
