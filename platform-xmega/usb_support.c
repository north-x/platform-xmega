/*
 * Copyright (c) 2014, Manuel Vetterli
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */ 

#include "sys/process.h"
#include "usb_support.h"
#include "loconet.h"
#include "ln_interface.h"
#include "usb/usb.h"
#include "usb/usb_pipe.h"

PROCESS(usb_process, "USB Handler");

USB_PIPE(ep_in,  0x81 | USB_EP_PP, USB_EP_TYPE_BULK_gc, 64, 4, PIPE_ENABLE_FLUSH);
USB_PIPE(ep_out, 0x02 | USB_EP_PP, USB_EP_TYPE_BULK_gc, 64, 4, 0);

void usb_init(void)
{
	//USB_ConfigureClock();
	
	USB.INTCTRLA = USB_BUSEVIE_bm | USB_INTLVL_MED_gc;
	USB.INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;
	
	USB_Init();
	process_start(&usb_process, NULL);
}

void sendLocoNetPacketUSB(lnMsg *LnPacket)
{
	uint8_t index, size;
	
	if (usb_pipe_can_write(&ep_in))
	{
		size = getLnMsgSize(LnPacket);
		
		for (index=0;index<size;index++)
		{
			usb_pipe_write_byte(&ep_in, LnPacket->data[index]);
		}
		
		usb_pipe_flush(&ep_in);
	}
}

PROCESS_THREAD(usb_process, ev, data)
{
	lnMsg *LnPacket;
	
	PROCESS_BEGIN();
	
	while (1)
	{
		if (usb_pipe_can_read(&ep_out))
		{
			LnPacket = (lnMsg *) usb_pipe_read_ptr(&ep_out);
		
			if (getLnMsgSize(LnPacket))
			{
				uint8_t tmp = lnTxEcho;
				lnTxEcho = 1;
				sendLocoNetPacket(LnPacket);
				lnTxEcho = tmp;
			}
			usb_pipe_done_read(&ep_out);
		}
		
		PROCESS_PAUSE();
	}
	
	PROCESS_END();
}

void EVENT_USB_Device_ConfigurationChanged(uint8_t configuration)
{
	usb_pipe_init(&ep_in);
	usb_pipe_init(&ep_out);
}

ISR(USB_BUSEVENT_vect)
{
	if (USB.INTFLAGSACLR & USB_SOFIF_bm)
	{
		USB.INTFLAGSACLR = USB_SOFIF_bm;
	}
	else if (USB.INTFLAGSACLR & (USB_CRCIF_bm | USB_UNFIF_bm | USB_OVFIF_bm))
	{
		USB.INTFLAGSACLR = (USB_CRCIF_bm | USB_UNFIF_bm | USB_OVFIF_bm);
	}
	else if (USB.INTFLAGSACLR & USB_STALLIF_bm)
	{
		USB.INTFLAGSACLR = USB_STALLIF_bm;
	}
	else
	{
		USB.INTFLAGSACLR = USB_SUSPENDIF_bm | USB_RESUMEIF_bm | USB_RSTIF_bm;
		USB_Evt_Task();
	}
}

ISR(USB_TRNCOMPL_vect)
{
	USB.FIFOWP = 0;
	USB.INTFLAGSBCLR = USB_SETUPIF_bm | USB_TRNIF_bm;
	USB_Task();
	usb_pipe_handle(&ep_in);
	usb_pipe_handle(&ep_out);
}

/** Event handler for the library USB Control Request reception event. */
bool EVENT_USB_Device_ControlRequest(USB_Request_Header_t* req)
{
	if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_VENDOR)
	{
		switch (req->bRequest)
		{
			case 0xBB: // disconnect from USB, jump to bootloader
				USB_enter_bootloader();
				return true;
		}
	}
	
	return false;
}
