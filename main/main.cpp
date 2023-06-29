#include "Arduino.h"
#include <Adafruit_MCP23X17.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#include "pin_def.h"

#include <inttypes.h>

static Adafruit_MCP23X17 xunit;

uart_port_t uart_num = UART_NUM_1;

extern "C" int sample()
{
	uint8_t buff[] = {0x01, 0x06, 0x01, 0x10, 0x00, 0xFF, 0xC9, 0xB3};	
	int l = uart_write_bytes(uart_num, buff, 8);

	l = uart_read_bytes(uart_num, buff, 8, pdMS_TO_TICKS(500));
	
	if (l == 8) {
		ESP_LOGI("sample res", "read succes from sensor");
		for (int i = 0; i < 8; i++) {
			ESP_LOGI("res", "%x", buff[i]);
		}
	}

	return 0;
}


extern "C" int uart_setup()
{
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};

	esp_err_t ret = uart_driver_install(uart_num, 512, 0, 0, NULL, 0); 
	ret = uart_param_config(uart_num, &uart_config);
	ret = uart_set_pin(uart_num, GPIO_NUM_7, GPIO_NUM_6, -1, -1);

	return ret;
}

static int mcp_expander_setup()
{
	if(!xunit.begin_I2C()) {
		ESP_LOGE("MCP", "failed to init");	
		return -1;
	}
	

	for (int i = 0; i < 15; i++) {
		xunit.pinMode(i, OUTPUT);
		xunit.digitalWrite(i, 0);
	}
	return 0;
}


static int sample_sen(int num)
{
	xunit.digitalWrite(EN_3V3_SEN, 1);
	xunit.digitalWrite(MUX_EN_0, 1);
	sample();	

	return 0;
}

extern "C" void app_main(void)
{
	if (mcp_expander_setup()) {
		ESP_LOGE("main", "ERROE mcp setup");	
		while (1) vTaskDelay(pdMS_TO_TICKS(200));

	}
	
	uart_setup();
	sample_sen(1);
	
	while (1) vTaskDelay(pdMS_TO_TICKS(200));

}
