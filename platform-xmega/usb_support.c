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
#include "usb/usb_cdc.h"

PROCESS(usb_process, "USB Handler");

void usb_init(void)
{
	//USB_ConfigureClock();
	
	USB.INTCTRLA = USB_BUSEVIE_bm | USB_INTLVL_MED_gc;
	USB.INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;
	
	usb_cdc_init();
	process_start(&usb_process, NULL);
}

void sendLocoNetPacketUSB(lnMsg *LnPacket)
{
	uint8_t index, size;
	
	if (cdc_txb.flag==0)
	{
		size = getLnMsgSize(LnPacket);
		
		for (index=0;index<size;index++)
		{
			cdc_txb.data[index] = LnPacket->data[index];
		}
		
		cdc_txb.len = size;
		cdc_txb.flag = 1;
	}
	
}

PROCESS_THREAD(usb_process, ev, data)
{
	lnMsg *LnPacket;
		
	PROCESS_BEGIN();
	
	cdc_rxb.flag = 1;
	cdc_txb.flag = 0;
	
	while (1)
	{
		EP_DEF_in(ep_out);
		EP_DEF_out(ep_in);
		EP_DEF_out(ep_note);
				
		if (cdc_rxb.flag==0)
		{
			LnPacket = (lnMsg *) cdc_rxb.data;
		
			if (getLnMsgSize(LnPacket))
			{
				uint8_t tmp = lnTxEcho;
				lnTxEcho = 1;
				sendLocoNetPacket(LnPacket);
				lnTxEcho = tmp;
			}
			cdc_rxb.flag = 1;
		}
		PROCESS_PAUSE();
	}
	
	PROCESS_END();
}

