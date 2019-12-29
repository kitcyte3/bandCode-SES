// INSTRUCTIONS:
// Call the following functions as needed:
// Basics: 
//	twi_init(); //call this first, enables NRF TWI drivers
//
// RGBW: (no special init function required)
//	turn on: RGBW_on(); returns 1 if failed, 0 if ok
//	turn off: RGBW_off(); returns 1 if failed, 0 if ok
//	get colors: RGBW_get(); returns a pointer to a static array of doubles [double R, doubleG, doubleB]
//
// GPIO expander 1 (main board right angle LEDs:
//	init: GPIOEXP1_init();
//	LED_BT_on();
//	LED_BT_off();
//
// MUX (both at the same time):
// Set the MUX to the sensor you want to read, then use the RGBW sensor code or Prox code to get data
//	MUX_init(); //sets output status on NRF GPIO pins
//	MUX_set(s3,s2,s1,s0); //s3 to s0 sets which sensor you get data from





/** @file
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h" 
//
#include <stdio.h>
//#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

//#define NRF_LOG_MODULE_NAME "APP"

//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"


 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0); //TWI instance 0
static const nrf_drv_twi_t m_twi1 = NRF_DRV_TWI_INSTANCE(1); //TWI instance 1
uint8_t readi2cOneByte1(uint8_t deviceAddr, uint8_t read_reg_addr);

// UART code copied over
//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t * p_event){
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void MUX_init(){
		nrf_gpio_cfg_output(30); //S0
		nrf_gpio_cfg_output(0); //S1
		nrf_gpio_cfg_output(1); //S2
		nrf_gpio_cfg_output(2); //S3
}


void twi_init (void){
		//make sure TWO0 and TWI1 are both enabled in config text file somewhere else in project
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 16, //16
       .sda                = 15, //15
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void twi1_init (void){
		//make sure TWO0 and TWI1 are both enabled in config text file somewhere else in project
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi1_config = {
       .scl                = 18, //18
       .sda                = 17, //17
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi1, &twi1_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi1);
}

//write to twi0
void writei2c(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write_high, uint8_t data_to_write_low){
	//power on spectrum sensor
	uint8_t dataToWrite[2] = {data_to_write_high, data_to_write_low};
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}
void writei2cSingle(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write_high){
	//power on spectrum sensor
	uint8_t dataToWrite[2] = {reg_addr, data_to_write_high};
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}

//write to twi1
void writei2c1(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write_high, uint8_t data_to_write_low){
	//power on spectrum sensor
	uint8_t dataToWrite[2] = {data_to_write_high, data_to_write_low};
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi1, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}
void writei2cOneByte(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write){
	//power on spectrum sensor
	uint8_t dataToWrite[1] = {data_to_write};
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}

//use twi1
void writei2cOneByte1(uint8_t deviceAddr, uint8_t reg_addr, uint8_t data_to_write){
	//power on spectrum sensor
	uint8_t dataToWrite[1] = {data_to_write};
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi1, deviceAddr, &dataToWrite[0], sizeof(dataToWrite), false);
}

uint16_t readi2cHighLow(uint8_t deviceAddr, uint8_t read_reg_addr){
	//ret_code_t err_code; //to hold return code, currently do nothing with it
	
	//READ 1st byte
	uint16_t data_both;
	//first half of read
	uint8_t dataToSend4[1] = {read_reg_addr};
	nrf_drv_twi_tx(&m_twi, deviceAddr, &dataToSend4[0], sizeof(dataToSend4), true);


	uint8_t read_data2[2];
	nrf_drv_twi_rx(&m_twi, deviceAddr, &read_data2[0], 2);
	//printf("addr: %x, data: 0x%x 0x%x \r\n",read_reg_addr ,read_data2[1], read_data2[0]);
	
	data_both = read_data2[0] | (read_data2[1] << 8);
	//printf("both: 0x%x \r\n",data_both);
	
	return data_both;
}

uint8_t readi2cOneByte(uint8_t deviceAddr, uint8_t read_reg_addr){
	//ret_code_t err_code; //to hold return code, currently do nothing with it
	
	//READ 1st byte
	//first half of read
	uint8_t dataToSend4[1] = {read_reg_addr};
	nrf_drv_twi_tx(&m_twi, (deviceAddr), &dataToSend4[0], sizeof(dataToSend4), true);
	
	uint8_t read_data2[1];
	nrf_drv_twi_rx(&m_twi, (deviceAddr), &read_data2[0], 1);
	return read_data2[0];
}


//use twi1
uint8_t readi2cOneByte1(uint8_t deviceAddr, uint8_t read_reg_addr){
	//ret_code_t err_code; //to hold return code, currently do nothing with it
	
	//READ 1st byte
	//first half of read
	uint8_t dataToSend4[1] = {read_reg_addr};
	nrf_drv_twi_tx(&m_twi1, (deviceAddr<<1)|0x0, &dataToSend4[0], sizeof(dataToSend4), true);
	
	uint8_t read_data2[1];
	nrf_drv_twi_rx(&m_twi1, (deviceAddr<<1)|0x1, &read_data2[0], 1);
	
	return read_data2[0];
}

bool GPIOEXP0_init(){
	//make sure twi_init() was run before this function.
	//AND make sure MUX is set to 1111
				//adr,R/!W		config hi low
	writei2c((0x22<<1), 0x0c, 0x0,0x0); //config port0 to output
	writei2c((0x22<<1), 0x0d, 0x0,0x0); //config port1 to output
	writei2c((0x22<<1), 0x0e, 0x0,0x0); //config port2 to output
	
	return 0;
}

bool GPIOEXP1_init(){
	//make sure twi_init() was run before this function.
				//adr,R/!W		config hi low
	writei2c1((0x23<<1), 0x0c, 0x0,0x0); //config port0 to output
	writei2c1((0x23<<1), 0x0d, 0x0,0x0); //config port1 to output
	writei2c1((0x23<<1), 0x0e, 0x0,0x0); //config port2 to output
	
	return 0;
}

uint8_t GPIOEXP1_readPort0(){
	return readi2cOneByte((0x23<<1)|0x1, 0x4);
}

uint8_t GPIOEXP0_readPort0(){
//	return readi2cOneByte((0x22<<1)|0x1, 0x4);
	return readi2cOneByte((0x22<<1)|0x1, 0x4);
}

bool LED_BT_blue(){
	//make sure GPIOEXP1_init was run before this function.
	uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
	res = (res & ~0x38) | (0x30 & 0x38); //0banything & 0b000[fix]111
	writei2cSingle(0x23, 0x04, res); //set Blue light closer to USB on
	return 0;
}
bool LED_BT_off(){
	//make sure GPIOEXP1_init was run before this function.
	uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
	res = (res & ~0x38) | (0x38 & 0x38); //0banything & 0b000[fix]111
	writei2cSingle(0x23, 0x04, res); //set Blue light closer to USB on
	return 0;
}
bool LED_PWR_red(){
	//make sure GPIOEXP1_init was run before this function.
	uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
	res = (res & ~0x07) | (0x3 & 0x07); //0banything & 0b000000[011]
	writei2cSingle(0x23, 0x04, res); //set red light away from USB ON
	return 0;
}
bool LED_PWR_green(){
	//make sure GPIOEXP1_init was run before this function.
	//see explanation in LED_BT_blue and LED_PWR_red
	uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
	res = (res & ~0x07) | (0x05 & 0x07);; //0banything & 0b000000[011]
	writei2cSingle(0x23, 0x04, res); //set green light away from USB on
	return 0;
}
bool LED_PWR_off(){
	//make sure GPIOEXP1_init was run before this function.
	//see explanation in LED_BT_blue and LED_PWR_red
	uint8_t res = readi2cOneByte(0x23, 0x04); //read LED register
	res = (res & ~0x07) | (0x07 & 0x07);; //0banything & 0b000000[011]
	writei2cSingle(0x23, 0x04, res); //set green light away from USB on
	return 0;
}

bool LED_indicator_init(){
		//configuration registers set all outputs to outputs instead of default input
		writei2cSingle(0x23, 0x0C, 0x0); 
		writei2cSingle(0x23, 0x0D, 0x0); 
		writei2cSingle(0x23, 0x0E, 0x0); 
}
bool LED_vertBoard_init(){
		//configuration registers set all outputs to outputs instead of default input
		writei2cSingle(0x22, 0x0C, 0x0); 
		writei2cSingle(0x22, 0x0D, 0x0); 
		writei2cSingle(0x22, 0x0E, 0x0); 
}

bool LED_WHITE_on(){
	//make sure GPIOEXP1_init was run before this function.
	//read current GPIOEXP1 settings:
	
	//LEDS WORK, BUT THIS DRIVER FUNCTION HASNT BEEN IMPLEMENTED CORRECTLY YET
	
	uint8_t port0 = 7;
	port0 = GPIOEXP0_readPort0();
	printf("port0: %x\r\n",port0);
	port0 |= 0x03;
	writei2cOneByte((0x22<<1),0x04, port0);
	
	return 0;
}

void MUX_set(bool s3, bool s2, bool s1, bool s0){
	s3?nrf_gpio_pin_set(2):nrf_gpio_pin_clear(2);
	s2?nrf_gpio_pin_set(1):nrf_gpio_pin_clear(1);
	s1?nrf_gpio_pin_set(0):nrf_gpio_pin_clear(0);
	s0?nrf_gpio_pin_set(30):nrf_gpio_pin_clear(30);
}

bool RGBW_on(){
		//turns RGBW sensor on, returns 0 if success, 1 if falied
					//addr,  reg,  hi, low
		writei2c(0x10,0x00,0x00,0x00); //enable RGBW sensors with 80ms integration time

		//read back settings
		uint16_t i2cResult = readi2cHighLow(0x10, 0x00);
	
		return i2cResult != 0x00;
}
bool RGBW_off(){
		//turns RGBW sensor on, returns 0 if success, 1 if falied
					//addr,  reg,  hi, low
		writei2c(0x10,0x00,0x00,0x01); //enable RGBW sensors with 80ms integration time

		//read back settings
		uint16_t i2cResult = readi2cHighLow(0x10, 0x00);
	
		return i2cResult != 0x01;
}
double* RGBW_get(){
	//returns an array of doubles, max value is 255 for each color, skips matrix multiplications
	bool printResult = true;
	static double RGBreturn[3] = {0,0,0};
	double r,g,b, w;
	
	//read colors
	r = readi2cHighLow(0x10, 0x08);// *100 / 96.0;	
	g = readi2cHighLow(0x10, 0x09);// *100 / 74.0;
	b = readi2cHighLow(0x10, 0x0A);// *100 / 56.0;
	w = readi2cHighLow(0x10, 0x0B);
	
	//do some math
	double normalizedR = r;
	double normalizedG = g;
	double normalizedB = b;
	normalizedR = normalizedR/255;
	normalizedG = normalizedG/255;
	normalizedB = normalizedB/255;
	int hexR, hexB, hexG;
	hexR = normalizedR;
	hexB = normalizedB;
	hexG = normalizedG;
	
	if(printResult){
		printf("  raw red: 0x%x\r\n",r );
		printf("raw green: 0x%x\r\n",g );
		printf(" raw blue:  0x%x\r\n",b );
		printf("raw white:  0x%x\r\n",w );
		printf("RGB: %x %x %x\r\n\r\n",hexR,hexG,hexB);
	}
	RGBreturn[0]= hexR;
	RGBreturn[1]= hexB;
	RGBreturn[2]= hexG;
	
	return RGBreturn;
	
}
 
void band_uart_init(){
uint32_t err_code;
	const app_uart_comm_params_t comm_params =
      {
          //RX_PIN_NUMBER,
          //TX_PIN_NUMBER,
					9,
					10,
					12,
					11,
          //RTS_PIN_NUMBER,
          //CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
	printf("\r\nUART works\r\n");
}

void scan_twi(){
	//check for TWI dvices
	uint8_t sample_data;
	uint8_t address;
	uint32_t err_code;
	for (address = 1; address <= 127; address++){
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            printf("TWI device detected at address 0x%x.\r\n", address);
        }
    }
}

int main(void){
	//These are direct NRF GPIO
	MUX_init();						//setup nrf GPIO to output
	MUX_set(1,1,1,1); 		//1111 for SDA -> LEDs	(s3 s2 s1 s0)
	band_uart_init(); 		// setup UART
		
	//then do twi init, since it tends to get stuck here
	twi_init();
	twi1_init();
	
	nrf_delay_ms(1000);
	scan_twi(); // do a twi scan
	
	//indicator LED stuff on 0x23 expander
	LED_indicator_init();
	LED_BT_blue();
	LED_PWR_green();

	//indicator LED stuff on 0x22 expander
	LED_vertBoard_init();
	writei2cSingle(0x22, 0x04, 0x77); //set Blue light closer to USB on
	writei2cSingle(0x22, 0x05, 0x0); //set Blue light closer to USB on
	writei2cSingle(0x22, 0x06, 0x20); //set Blue light closer to USB on
	
	//RGBW setup
	MUX_set(0,0,1,1);
	RGBW_on();


	//===========EVERYTHING UP TO HERE WORKS==========================
	
	/*
	//read some registers from the gpio expanders 0x22
		uint8_t address;	
		uint8_t res = 0;
		address = 0x0C; //address to read
		for(address; address < 0xc+3; address++){
			res = readi2cOneByte(0x22, address); //read config refister
			printf("gpioexp0 %x: %x\r\n",address,res);
			res = readi2cOneByte1(0x22, address); //read config refister
			printf("gpioexp0 %x: %x\r\n",address,res);
		}
		address = 0x04; //address to read
		for(address; address < 0x4+3; address++){
			res = readi2cOneByte(0x22, address); //read config refister
			printf("gpioexp0 %x: %x\r\n",address,res);
			res = readi2cOneByte1(0x22, address); //read config refister
			printf("gpioexp0 %x: %x\r\n",address,res);
		}
		*/	
		
		

		//set mux to read one of the 
		MUX_set(0,0,1,1);
    /*while (true) //PROX CODE
    {
						ret_code_t err_code;
					//read VCNL prox snesor data
					uint8_t address = 0x60; //0x60  for prox					
					uint8_t dataToSend2[3] = {0x03,0x00,0x00};
					err_code = nrf_drv_twi_tx(&m_twi, address, &dataToSend2[0], sizeof(dataToSend2), false);
					
					//first half of read
					uint8_t dataToSend[1] = {0xF2};
					err_code = nrf_drv_twi_tx(&m_twi, address, &dataToSend[0], sizeof(dataToSend), true);
					uint8_t read_data[2];
      		err_code = nrf_drv_twi_rx(&m_twi, address, &read_data[0], 2);
					printf("recieved: 0x%x 0x%x\r\n",read_data[1], read_data[0]);
					if (err_code == NRF_SUCCESS){}
						nrf_delay_ms(200);
		}	*/
		for(int i = 1; i< 100000; i++){
			
			RGBW_get();
			nrf_delay_ms(200);
		}
		
		
		
		
		
		/*
					//read spectrum snesor data
					uint8_t address2 = 0x39; //0x60  for prox					
					//uint8_t dataToSend2[3] = {0x03,0x00,0x00};
					//err_code = nrf_drv_twi_tx(&m_twi, address, &dataToSend2[0], sizeof(dataToSend2), false);
					
					
					printf("spectrum start=====>\r\n");
						
					//power on spectrum sensor
					uint8_t dataToSend3[2] = {0x80, 0x01};
					err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend3[0], sizeof(dataToSend3), false);
					
					//confirm above write
						uint8_t dataToSend5[1] = {0x80};
						err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend5[0], sizeof(dataToSend5), true);
						uint8_t read_data3[1];
						err_code = nrf_drv_twi_rx(&m_twi, address2, &read_data3[0], 1);
						printf("addr: 0x80, data: 0x%x \r\n",read_data3[0]);
						
						//start measurement
						dataToSend3[0] = 0x80;
						dataToSend3[1] = 0x03;
						err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend3[0], sizeof(dataToSend3), false);
					
						//confirm above write
						dataToSend5[1] = 0x80;
						err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend5[0], sizeof(dataToSend5), true);
						err_code = nrf_drv_twi_rx(&m_twi, address2, &read_data3[0], 1);
						printf("addr: 0x80, data: 0x%x \r\n",read_data3[0]);
						
					for(uint8_t reg = 0x95; reg < 0xA0; reg += 1){
						//first half of read
						uint8_t dataToSend4[1] = {reg};
						err_code = nrf_drv_twi_tx(&m_twi, address2, &dataToSend4[0], sizeof(dataToSend4), true);
						uint8_t read_data2[1];
						err_code = nrf_drv_twi_rx(&m_twi, address2, &read_data2[0], 1);
						printf("addr: %x, data: 0x%x \r\n",reg,read_data2[0]);
						if (err_code == NRF_SUCCESS){}
						
						NRF_LOG_FLUSH();
					}
					printf("<====== spectrum end\r\n");
					*/
					
					
					//RGBW_get(); //returns pointer to static [double R, doubleB, doubleG]
					
			//nrf_delay_ms(250);
    //}
	
	//above added UART code
	
}

/** @} */