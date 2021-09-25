#ifndef WIFI_H
#define WIFI_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stdlib.h"
#include <stdarg.h>
#include <string.h>
#include "stdio.h"

#define ESP8266_RX_BUFFER_LEN 8192	

extern char esp8266RxBuffer[ESP8266_RX_BUFFER_LEN];

void wifi_init(void);
void esp8266ClearBuffer(void);
void uart_send(const char *tx_buf, ...) ;

#endif /* WIFI_H */
