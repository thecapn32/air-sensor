#include "Arduino.h"
#include <Adafruit_MCP23X17.h>
#include <SoftwareSerial.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#include "pin_def.h"

#include <inttypes.h>

static Adafruit_MCP23X17 xunit;

uart_port_t uart_num = UART_NUM_1;

EspSoftwareSerial::UART swSer1;

extern "C" int sample()
{
	 uint8_t buff[] = {0xFF, 0x03, 0x10, 0x20, 0x00, 0x01, 0x94, 0xDE};
	
	while (1) {
		int l = uart_write_bytes(uart_num, buff, 8);
		
		//swSer1.write(buff, 8);
		l = 0;
		while (l == 0) {
		l = uart_read_bytes(uart_num, buff, 8, pdMS_TO_TICKS(40));
		//int l = swSer1.read(buff, 8);
		ESP_LOGI("sample res", "reeeeeeeead l value %d", l);
		if (l > 0) {
			for (int i = 0; i < 8; i++) {
				ESP_LOGI("res", "%x", buff[i]);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	return 0;
}


 int uart_setup()
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
	ret = uart_set_pin(uart_num, GPIO_NUM_16, GPIO_NUM_15, -1, -1);

	//swSer1.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, 15, 16, false, 256);



	return 0;
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
	xunit.digitalWrite(0, 1);
	xunit.digitalWrite(1, 0);
	xunit.digitalWrite(2, 1);
	xunit.digitalWrite(3, 1);
	ESP_LOGI("ss", "%d %d", xunit.digitalRead(EN_3V3_SEN), xunit.digitalRead(EN_3V3_SEN));
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
