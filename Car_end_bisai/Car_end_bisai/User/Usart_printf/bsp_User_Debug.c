#include "bsp_User_Debug.h"

#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "usart.h"

void usart_printf(const char *fmt,...)
{
  static uint8_t tx_buf[256] = {0};
  static va_list ap;
  static uint16_t len;
		
  va_start(ap, fmt);
  len = vsprintf((char *)tx_buf, fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&huart3,tx_buf,len,100);
}

