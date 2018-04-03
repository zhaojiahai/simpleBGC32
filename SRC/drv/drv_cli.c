/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

tRingBuffer RingBufferUSBTX;

static int USBCallBackCalled = 0;

///////////////////////////////////////////////////////////////////////////////

void printUSART(const char *fmt, ...)
{
	char buf[256];

	va_list vlist;
	va_start(vlist, fmt);

	vsnprintf(buf, sizeof(buf) - 1, fmt, vlist);
	USART_PutString((unsigned char *)buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////

void printDirect(const char *fmt, ...)
{
	char buf[256];

	va_list vlist;
	va_start(vlist, fmt);

	vsnprintf(buf, sizeof(buf) - 1, fmt, vlist);
	USART_PutStringDirect((unsigned char *)buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////

int usbOverrun(void)
{
	return (RingBufferUSBTX.Overrun);
}

///////////////////////////////////////////////////////////////////////////////

void USBPushTXData(void)
{
	tRingBuffer *rb = &RingBufferUSBTX;
	uint8_t *p = rb->Buffer + rb->Read;
	int len = rb->Write - rb->Read;

	if (len != 0)
	{
		if (len < 0)
		{
			len = RingBufferSize(rb) - rb->Read;
		}

		if (len > 64)
		{
			len = 64;
		}

		packetSent = len;

		// move read pointer before sending data to avoid race condition with endpoint interrupt
		rb->Read = (rb->Read + len) % RingBufferSize(rb);
		CDC_Send_DATA(p, len);
	}

	else
	{
		packetSent = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////

void USBPushTX(void)
{
	if (packetSent)
	{
		return; // transfer will be handled by next callback
	}

	//__disable_irq_nested();
	USBPushTXData();
	//__enable_irq_nested();
}

///////////////////////////////////////////////////////////////////////////////
// CLI Initialization
///////////////////////////////////////////////////////////////////////////////

void EP1_IN_Callback(void)
{
	USBCallBackCalled = 1;
	//__disable_irq_nested();
	USBPushTXData();
	//__enable_irq_nested();
}

///////////////////////////////////////////////////////////////////////////////

void cliInit(void)
{
	Set_System();
	Usart4Init();

	printUSART("USART4 ready\n");

	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();

	RingBufferInit(&RingBufferUSBTX, &USBPushTX);
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print
///////////////////////////////////////////////////////////////////////////////

uint32_t cliPrint(const uint8_t *str, uint32_t len)
{
	if (usbIsConnected())
	{
		RingBufferPutBlock(&RingBufferUSBTX, (uint8_t *)str, len, 0);
	}

	return len;
}

///////////////////////////////////////////////////////////////////////////////
// CLI Available
///////////////////////////////////////////////////////////////////////////////

uint32_t cliAvailable(void)
{
	return receiveLength;
}

///////////////////////////////////////////////////////////////////////////////
// CLI Read
///////////////////////////////////////////////////////////////////////////////

uint32_t cliRead(uint8_t *recvBuf, uint32_t len)
{
	int newBytes = cliAvailable();

	if ((int)len > newBytes)
	{
		len = newBytes;
	}

	CDC_Receive_DATA(recvBuf, len);

	return len;
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print Formatted - Print formatted string to USB VCP
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void cliPrintF(const char *fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start(vlist, fmt);

	vsnprintf(buf, sizeof(buf) - 1, fmt, vlist);
	cliPrint((uint8_t *)buf, strlen(buf));
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////

void usbDsbISR(void)
{
	;
}

///////////////////////////////////////////////////////////////////////////////

static int lastChar = -1;

int getChar(void)
{
	if (lastChar < 0)
	{
		if (cliAvailable())
		{
			uint8_t c;
			cliRead(&c, 1);
			return (c);
		}

		else
		{
			return -1;
		}
	}

	else
	{
		int c = lastChar;
		lastChar = -1;
		return (c);
	}
}

///////////////////////////////////////////////////////////////////////////////

