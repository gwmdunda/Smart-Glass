#ifndef __WIFI_H
#define	__WIFI_H

#include "stm32f10x.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include <stdarg.h>

#define ESP8266_RX_BUFFER_LEN 8192	

extern char esp8266RxBuffer[ESP8266_RX_BUFFER_LEN];
extern volatile unsigned int bufferHead;

void wifi_init(void);
void wifi_clear_buffer(void);
void usart_send(const char *tx_buf, ...);


#endif 
