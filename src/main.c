//
// CANable firmware - a fork of CANtact by Eric Evenchick
//


#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "can.h"
#include "slcan.h"
#include "system.h"
#include "led.h"

#ifdef Ollie
#define OUTPUT_EN_5V_Pin 			GPIO_PIN_13
#define OUTPUT_EN_5V_GPIO_Port		GPIOC
#define OUTPUT_EN_3V3_Pin 			GPIO_PIN_14
#define OUTPUT_EN_3V3_GPIO_Port 	GPIOC
#define OUTPUT_EN_1V8_Pin 			GPIO_PIN_15
#define OUTPUT_EN_1V8_GPIO_Port 	GPIOC
#define SW1_Pin 					GPIO_PIN_2
#define SW1_GPIO_Port 				GPIOA
#define LED_5V_Pin 					GPIO_PIN_5
#define LED_5V_GPIO_Port 			GPIOA
#define LED_3V3_Pin 				GPIO_PIN_6
#define LED_3V3_GPIO_Port 			GPIOA
#define LED_1V8_Pin 				GPIO_PIN_7
#define LED_1V8_GPIO_Port 			GPIOA
#define VS_5V_Pin 					GPIO_PIN_6
#define VS_5V_GPIO_Port 			GPIOB
#define VS_3V3_Pin 					GPIO_PIN_5
#define VS_3V3_GPIO_Port 			GPIOB
#define VS_1V8_Pin 					GPIO_PIN_4
#define VS_1V8_GPIO_Port 			GPIOB
//static void ollie_gpio_init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct;
//
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//	__HAL_RCC_GPIOC_CLK_ENABLE();
//
//	/*Configure GPIO pin Output Level */
//	HAL_GPIO_WritePin(GPIOC, OUTPUT_EN_5V_Pin|OUTPUT_EN_3V3_Pin|OUTPUT_EN_1V8_Pin, GPIO_PIN_RESET);
//
//	/*Configure GPIO pin Output Level */
//	HAL_GPIO_WritePin(GPIOA, LED_5V_Pin|LED_3V3_Pin|LED_1V8_Pin, GPIO_PIN_RESET);
//
//	/*Configure GPIO pins : OUTPUT_EN_5V_Pin OUTPUT_EN_3V3_Pin OUTPUT_EN_1V8_Pin */
//	GPIO_InitStruct.Pin = OUTPUT_EN_5V_Pin|OUTPUT_EN_3V3_Pin|OUTPUT_EN_1V8_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
////
//	/*Configure GPIO pin : SW1_Pin */
//	GPIO_InitStruct.Pin = SW1_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);
//
//	/*Configure GPIO pins : LED_5V_Pin LED_3V3_Pin LED_1V8_Pin */
//	GPIO_InitStruct.Pin = LED_5V_Pin|LED_3V3_Pin|LED_1V8_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//	/*Configure GPIO pins : VS_5V_Pin VS_3V3_Pin VS_1V8_Pin */
//	GPIO_InitStruct.Pin = VS_5V_Pin|VS_3V3_Pin|VS_1V8_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//}

#endif


int main(void)
{
    // Initialize peripherals
    system_init();

//    ollie_gpio_init();

    can_init();
	#ifndef Ollie
    led_init();
	#endif
    usb_init();

	#ifndef Ollie
    led_blue_blink(2);
	#endif

    // Storage for status and received message buffer
    CAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[8] = {0};
    uint8_t msg_buf[SLCAN_MTU];

	#ifdef Ollie
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, OUTPUT_EN_5V_Pin|OUTPUT_EN_3V3_Pin|OUTPUT_EN_1V8_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_5V_Pin|LED_3V3_Pin|LED_1V8_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : OUTPUT_EN_5V_Pin OUTPUT_EN_3V3_Pin OUTPUT_EN_1V8_Pin */
	GPIO_InitStruct.Pin = OUTPUT_EN_5V_Pin|OUTPUT_EN_3V3_Pin|OUTPUT_EN_1V8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//	/*Configure GPIO pin : SW1_Pin */
	GPIO_InitStruct.Pin = SW1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_5V_Pin LED_3V3_Pin LED_1V8_Pin */
	GPIO_InitStruct.Pin = LED_5V_Pin|LED_3V3_Pin|LED_1V8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : VS_5V_Pin VS_3V3_Pin VS_1V8_Pin */
	GPIO_InitStruct.Pin = VS_5V_Pin|VS_3V3_Pin|VS_1V8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	if(HAL_GPIO_ReadPin(VS_5V_GPIO_Port, VS_5V_Pin) == GPIO_PIN_RESET)
	{
		HAL_GPIO_WritePin(LED_5V_GPIO_Port, LED_5V_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUTPUT_EN_5V_GPIO_Port, OUTPUT_EN_5V_Pin, GPIO_PIN_SET);
	}
	else if(HAL_GPIO_ReadPin(VS_3V3_GPIO_Port, VS_3V3_Pin) == GPIO_PIN_RESET)
	{
		HAL_GPIO_WritePin(LED_3V3_GPIO_Port, LED_3V3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUTPUT_EN_3V3_GPIO_Port, OUTPUT_EN_3V3_Pin, GPIO_PIN_SET);
	}
	else if(HAL_GPIO_ReadPin(VS_1V8_GPIO_Port, VS_1V8_Pin) == GPIO_PIN_RESET)
	{
		HAL_GPIO_WritePin(LED_1V8_GPIO_Port, LED_1V8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(OUTPUT_EN_1V8_GPIO_Port, OUTPUT_EN_1V8_Pin, GPIO_PIN_SET);
	}

	#endif

    while(1)
    {
        // Block until a CAN message is received
        while (!is_can_msg_pending(CAN_RX_FIFO0))
        {
			#ifndef Ollie
            led_process();
			#endif
        }

        uint32_t status = can_rx(&rx_msg_header, rx_msg_data);

        // If message received from bus, parse the frame
        if (status == HAL_OK)
        {
        	uint16_t msg_len = slcan_parse_frame((uint8_t *)&msg_buf, &rx_msg_header, rx_msg_data);

            // Transmit message via USB-CDC 
            if(msg_len)
            {
                CDC_Transmit_FS(msg_buf, msg_len);
            }
        }
		#ifndef Ollie
        led_process();
		#endif
    }
}

