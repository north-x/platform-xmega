/****************************************************************************
    Copyright (C) 2002 Alex Shepherd

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

 Title :   LocoNet Software UART Access library
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     13-Aug-2002
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device

 DESCRIPTION
  Basic routines for interfacing to the LocoNet via any output pin and
  either the Analog Comparator pins or the Input Capture pin

  The receiver uses the Timer1 Input Capture Register and Interrupt to detect
  the Start Bit and then the Compare A Register for timing the subsequest
  bit times.

  The Transmitter uses just the Compare A Register for timing all bit times
       
 USAGE
  See the C include ln_interface.h file for a description of each function
       
*****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#include "ln_sw_uart.h"    // prototypes of this module
#include "ln_interface.h"  // hardware independent prototypes and definitions
#include "sysdef.h"        // board definition

// Board defines for WA2
#define LN_TMR					TCE0

#define LN_TMR_COUNT_REG TCE0.CNT
#define LN_TMR_INP_CAPT_REG TCE0.CCA
#define LN_TMR_OUTP_CAPT_REG TCE0.CCB
#define LN_SB_SIGNAL ACA_AC0_vect
#define LN_TMR_SIGNAL TCE0_CCC_vect

#define LN_SW_UART_SET_TX_HIGH PORTE.OUTCLR = (1<<3);
#define LN_SW_UART_SET_TX_LOW PORTE.OUTSET = (1<<3);

#define LN_RX_PORT            ACA.STATUS
#define LN_RX_BIT             AC_AC0STATE_bp

#define LN_TX_PORT            PORTE.OUT
#define LN_TX_DDR             PORTE.DIR
#define LN_TX_BIT             3

#define LN_TIMER_TX_RELOAD_ADJUST	10		// 3 us delay


#define LN_ST_IDLE            0   // net is free for anyone to start transmission
#define LN_ST_CD_BACKOFF      1   // timer interrupt is counting backoff bits
#define LN_ST_TX_COLLISION    2   // just sending break after creating a collision
#define LN_ST_TX              3   // transmitting a packet
#define LN_ST_RX              4   // receiving bytes
#define LN_ST_SB			  128 // check start bit


#define   LN_COLLISION_TICKS 15

#define   LN_TX_RETRIES_MAX  25

          // The Start Bit period is a full bit period + half of the next bit period
          // so that the bit is sampled in middle of the bit
#define LN_TIMER_RX_START_PERIOD    LN_BIT_PERIOD + (LN_BIT_PERIOD / 2)
#define LN_TIMER_RX_RELOAD_PERIOD   LN_BIT_PERIOD 
#define LN_TIMER_TX_RELOAD_PERIOD   LN_BIT_PERIOD 

// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// LN_TIMER_TX_RELOAD_ADJUST is a value for an error correction. This is needed for 
// every start of a byte. The first bit is to long. Therefore we need to reduce the 
// reload value of the bittimer.
// The following value depences highly on used compiler, optimizationlevel and hardware.
// Define the value in sysdef.h. This is very project specific.
// For the FREDI hard- and software it is nearly a quarter of a LN_BIT_PERIOD.
// Olaf Funke, 19th October 2007
// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#ifndef LN_TIMER_TX_RELOAD_ADJUST
	#define LN_TIMER_TX_RELOAD_ADJUST   0
	#error detect value by oszilloscope
#endif

// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//#define RX_MONITOR


//#define COLLISION_MONITOR
#ifdef COLLISION_MONITOR
#define COLLISION_MONITOR_PORT PORTB
#define COLLISION_MONITOR_DDR DDRB
#define COLLISION_MONITOR_BIT PB4
#endif

//#define STARTBIT_MONITOR
#ifdef STARTBIT_MONITOR
#define STARTBIT_MONITOR_PORT PORTB
#define STARTBIT_MONITOR_DDR DDRB
#define STARTBIT_MONITOR_BIT PB4
#endif

#if 0
volatile byte    		lnState ;
volatile byte				lnBitCount ;
volatile byte				lnCurrentByte ;
#else
#define lnState GPIOR0
#define lnBitCount GPIOR1
#define lnCurrentByte GPIOR2
#endif
volatile uint16_t       lnCompareTarget ;

LnBuf               * lnRxBuffer ;
volatile lnMsg      * volatile lnTxData ;
volatile byte				lnTxIndex ;
volatile byte				lnTxLength ;
volatile byte       lnTxSuccess ;   // this boolean flag as a message from timer interrupt to send function
uint8_t ln_ac_buf[2];
volatile uint8_t lnTxEcho;

/**************************************************************************
*
* Start Bit Interrupt Routine
*
* DESCRIPTION
* This routine is executed when a falling edge on the incoming serial
* signal is detected. It disables further interrupts and enables
* timer interrupts (bit-timer) because the UART must now receive the
* incoming data.
*
**************************************************************************/

ISR(LN_SB_SIGNAL)
{
	// Disable startbit detection
	ACA.AC0CTRL &= ~AC_INTLVL_HI_gc;
	TCE0.CTRLD = TC_EVACT_OFF_gc;

    // Clear the current compare interrupt status bit and enable the compare interrupt
	TCE0.INTFLAGS = TC0_CCAIF_bm;
	TCE0.INTCTRLB |= TC_CCAINTLVL_HI_gc;

    // Set the state to indicate that we have started to receive
	lnState |= LN_ST_SB ;
}

ISR(TCE0_CCA_vect)
{
	// Disable further interrupts
	TCE0.INTCTRLB &= ~TC_CCAINTLVL_HI_gc;
	
	// Sanity check of start bit
	// If LN is 1 again, then this was just noise
	if (ACA.STATUS&AC_AC0STATE_bm)
	{
		// Restore previous state
		lnState &= ~LN_ST_SB;
		// Clear the Start Bit Interrupt Status Flag and Enable ready to
		// detect the next Start Bit
		// Clear pending interrupts and enable SB detection
		ACA.STATUS = AC_AC0IF_bm;
		ACA.AC0CTRL |= AC_INTLVL_HI_gc;
		TCE0.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH0_gc;
		lnRxBuffer->Stats.RxErrors++;
	}
	else
	{
		// Prepare ISR to receive byte
		TCE0.INTFLAGS = TC0_CCCIF_bm;
		TCE0.INTCTRLB |= TC_CCCINTLVL_HI_gc;
	}
}
/**************************************************************************
*
* Timer Tick Interrupt
*
* DESCRIPTION
* This routine coordinates the transmition and reception of bits. This
* routine is automatically executed at a rate equal to the baud-rate. When
* transmitting, this routine shifts the bits and sends it. When receiving,
* it samples the bit and shifts it into the buffer.
*
**************************************************************************/

ISR(LN_TMR_SIGNAL)     /* signal handler for timer0 overflow */
{
	uint8_t filter_cnt = 0;
	lnBitCount++;                         //Increment bit_counter

	// Sanity check of start bit
	if ( lnState&LN_ST_SB )
	{
		lnState = LN_ST_RX;
		lnBitCount = 0;
		return;
	}
	
	if (ACA.STATUS&AC_AC0STATE_bm)
		filter_cnt++;
	
	if (ln_ac_buf[0]&AC_AC0STATE_bm)
		filter_cnt++;
	
	if (ln_ac_buf[1]&AC_AC0STATE_bm)
		filter_cnt++;
		
	filter_cnt >>= 1;
	
    // Are we in the RX State
	if( lnState == LN_ST_RX )                // Are we in RX mode
	{
		if( lnBitCount < 9)               // Are we in the Stop Bits phase
		{
			lnCurrentByte >>= 1;
			
			if (filter_cnt)
				lnCurrentByte |= 0x80;
			return ;
		}

		// Clear the Start Bit Interrupt Status Flag and Enable ready to 
		// detect the next Start Bit
		// Clear pending interrupts and enable SB detection
		ACA.STATUS = AC_AC0IF_bm;
		ACA.AC0CTRL |= AC_INTLVL_HI_gc;
		TCE0.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH0_gc;
				
		// If the Stop bit is not Set then we have a Framing Error
		if (!(filter_cnt))
			lnRxBuffer->Stats.RxErrors++ ;
		else
			// Put the received byte in the buffer
			addByteLnBuf( lnRxBuffer, lnCurrentByte ) ;

		lnBitCount = 0 ;
		lnState = LN_ST_CD_BACKOFF ;
	}


    // Are we in the TX State
	if( lnState == LN_ST_TX )
	{
		// To get to this point we have already begun the TX cycle so we need to 
		// first check for a Collision. For now we will simply check that TX and RX
		// ARE NOT THE SAME as our circuit requires the TX signal to be INVERTED
		// If they are THE SAME then we have a Collision
		if( ( ( LN_TX_PORT >> LN_TX_BIT ) & 0x01 ) == ( ( filter_cnt ) & 0x01 ) )
		{
			lnBitCount = 0 ;
			lnState = LN_ST_TX_COLLISION ;
		}
		// Send each Bit
		else if( lnBitCount < 9)
		{
			if( lnCurrentByte & 0x01 )
				LN_SW_UART_SET_TX_HIGH
			else
				LN_SW_UART_SET_TX_LOW

			lnCurrentByte >>= 1;
		}
		// When the Data Bits are done, generate stop-bit
		else if( lnBitCount ==  9)
			LN_SW_UART_SET_TX_HIGH

		// Any more bytes in buffer
		else if( ++lnTxIndex < lnTxLength )
		{
			// Setup for the next byte
			lnBitCount = 0 ;
			lnCurrentByte = lnTxData->data[ lnTxIndex ] ;

			// Begin the Start Bit
			LN_SW_UART_SET_TX_LOW

			// Get the Current Timer1 Count and Add the offset for the Compare target
			// added adjustment value for bugfix (Olaf Funke)
			//lnCompareTarget = LN_TMR_COUNT_REG + LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST; 
			//LN_TMR_OUTP_CAPT_REG = lnCompareTarget ;
		}
		else
		{
			// Successfully Sent all bytes in the buffer
			// so set the Packet Status to Done
			lnTxSuccess = 1 ;

			// Now copy the TX Packet into the RX Buffer
			if (lnTxEcho)
				addMsgLnBuf( lnRxBuffer, lnTxData );

			// Begin CD Backoff state
			lnBitCount = 0 ;
			lnState = LN_ST_CD_BACKOFF ;      
		}
	}

	// Note we may have got here from a failed TX cycle, if so BitCount will be 0
	if( lnState == LN_ST_TX_COLLISION )
	{
		if( lnBitCount == 0 )
		{
			// Pull the TX Line low to indicate Collision
			LN_SW_UART_SET_TX_LOW
	    }
	    else if( lnBitCount >= LN_COLLISION_TICKS )
	    {
			// Release the TX Line
			LN_SW_UART_SET_TX_HIGH
			lnBitCount = 0 ;
			lnState = LN_ST_CD_BACKOFF ;

			lnRxBuffer->Stats.Collisions++ ;
		}
	}

	if( lnState == LN_ST_CD_BACKOFF )
	{
		if( lnBitCount == 0 )
		{
			// Even though we are waiting, other nodes may try and transmit early
			// so Clear the Start Bit Interrupt Status Flag and Enable ready to 
			// detect the next Start Bit
			// Clear the Start Bit Interrupt Status Flag and Enable ready to
			// detect the next Start Bit
			ACA.STATUS = AC_AC0IF_bm;
			ACA.AC0CTRL |= AC_INTLVL_HI_gc;
			TCE0.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH0_gc;
		}
		else if( lnBitCount >= LN_BACKOFF_MAX )
		{ // declare network to free after maximum backoff delay
			lnState = LN_ST_IDLE ;
			TCE0.INTCTRLB &= ~TC_CCCINTLVL_HI_gc;
		}
	}
}

void initLocoNetHardware( LnBuf *RxBuffer )
{
	lnRxBuffer = RxBuffer;
	
	// PORTE 2:RX 3:TX
	PORTE.PIN3CTRL = PORT_OPC_TOTEM_gc;
	PORTE.OUTCLR = (1<<3);
	PORTE.DIRSET = (1<<3);
	
	// Set voltage reference
	ACA.CTRLB = ((2500*10/78*64/3300)-1)&0x3F; // 2.5 V Threshold

	// connect comparators to pins
	ACA.AC0MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_SCALER_gc;
	ACA.AC1MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_SCALER_gc;

	// and enable them
	//AC0 generates event on falling edge
	ACA.AC0CTRL = AC_ENABLE_bm | AC_INTMODE_FALLING_gc | AC_INTLVL_OFF_gc | AC_HYSMODE_LARGE_gc;
	//AC1 generates event on rising edge
	ACA.AC1CTRL = AC_ENABLE_bm | AC_INTMODE_RISING_gc | AC_INTLVL_OFF_gc | AC_HYSMODE_LARGE_gc;

	// clear pending interrupt flags (net free detection)
	ACA.STATUS = AC_AC0IF_bm | AC_AC1IF_bm;

	EVSYS.CH0MUX = EVSYS_CHMUX_ACA_CH0_gc;
	EVSYS.CH1MUX = EVSYS_CHMUX_ACA_CH1_gc;

	// Timer E0: Prescaler 8
	// Event 0 triggers capture used for start bit detection
	TCE0.PER = LN_BIT_PERIOD;
	TCE0.CTRLB = TC_WGMODE_NORMAL_gc;
	TCE0.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH0_gc;
	TCE0.CTRLA = TC_CLKSEL_DIV8_gc;
	
	TCE0.CCA = LN_BIT_PERIOD/4;
	TCE0.CCB = LN_BIT_PERIOD/2;
	TCE0.CCC = 3*LN_BIT_PERIOD/4;
	
	// Clear pending interrupts and enable SB detection
	ACA.STATUS = AC_AC0IF_bm;
	ACA.AC0CTRL |= AC_INTLVL_HI_gc;

	lnState = LN_ST_IDLE ;
	lnTxEcho = 1;
	
	DMA.CTRL = 0;
	DMA.CTRL = DMA_RESET_bm;
	
	DMA.CH0.ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;
	DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_TCE0_CCA_gc;
	DMA.CH0.TRFCNT = 1; // 1 Bytes per Block
	DMA.CH0.REPCNT = 0; // unlimited repeat
	DMA.CH0.DESTADDR0 = (((uint16_t) (&ln_ac_buf[0]))>>0*8) & 0xFF;
	DMA.CH0.DESTADDR1 = (((uint16_t) (&ln_ac_buf[0]))>>1*8) & 0xFF;
	DMA.CH0.DESTADDR2 = 0;
	DMA.CH0.SRCADDR0 = (((uint16_t)(&ACA_STATUS))>>0*8) & 0xFF;
	DMA.CH0.SRCADDR1 = (((uint16_t)(&ACA_STATUS))>>1*8) & 0xFF;
	DMA.CH0.SRCADDR2 = 0;
	DMA.CH0.CTRLA = DMA_CH_BURSTLEN_1BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;
	DMA.CH0.CTRLB = 0;
	
	DMA.CH1.ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;
	DMA.CH1.TRIGSRC = DMA_CH_TRIGSRC_TCE0_CCB_gc;
	DMA.CH1.TRFCNT = 1; // 1 Bytes per Block
	DMA.CH1.REPCNT = 0; // unlimited repeat
	DMA.CH1.DESTADDR0 = (((uint16_t) (&ln_ac_buf[1]))>>0*8) & 0xFF;
	DMA.CH1.DESTADDR1 = (((uint16_t) (&ln_ac_buf[1]))>>1*8) & 0xFF;
	DMA.CH1.DESTADDR2 = 0;
	DMA.CH1.SRCADDR0 = (((uint16_t)(&ACA_STATUS))>>0*8) & 0xFF;
	DMA.CH1.SRCADDR1 = (((uint16_t)(&ACA_STATUS))>>1*8) & 0xFF;
	DMA.CH1.SRCADDR2 = 0;
	DMA.CH1.CTRLA = DMA_CH_BURSTLEN_1BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;
	DMA.CH1.CTRLB = 0;
	
	DMA.CTRL = DMA_ENABLE_bm;
	
	DMA.CH0.CTRLA |= DMA_CH_ENABLE_bm;
	DMA.CH1.CTRLA |= DMA_CH_ENABLE_bm;
}


LN_STATUS sendLocoNetPacketTry(lnMsg *TxData, unsigned char ucPrioDelay)
{
	byte  CheckSum ;
	byte  CheckLength ;

	lnTxLength = getLnMsgSize( TxData ) ;

	// First calculate the checksum as it may not have been done
	CheckLength = lnTxLength - 1 ;
	CheckSum = 0xFF ;

	for( lnTxIndex = 0; lnTxIndex < CheckLength; lnTxIndex++ )
		CheckSum ^= TxData->data[ lnTxIndex ] ;
  
	TxData->data[ CheckLength ] = CheckSum ; 

	// clip maximum prio delay
	if (ucPrioDelay > LN_BACKOFF_MAX)
		ucPrioDelay = LN_BACKOFF_MAX;

	// if priority delay was waited now, declare net as free for this try
	cli();  // disabling interrupt to avoid confusion by ISR changing lnState while we want to do it
	if (lnState == LN_ST_CD_BACKOFF)
	{
		if (lnBitCount >= ucPrioDelay)  // Likely we don't want to wait as long as
		{                               // the timer ISR waits its maximum delay.
			lnState = LN_ST_IDLE ;
			TCE0.INTCTRLB &= ~TC_CCCINTLVL_HI_gc;
		}
	}
	sei();  // a delayed start bit interrupt will happen now,
          // a delayed timer interrupt was stalled

	// If the Network is not Idle, don't start the packet
	if (lnState == LN_ST_CD_BACKOFF)
	{
		if (lnBitCount < LN_CARRIER_TICKS)   // in carrier detect timer?
			return LN_CD_BACKOFF;
		else
			return LN_PRIO_BACKOFF;
	}

	if( lnState != LN_ST_IDLE )
		return LN_NETWORK_BUSY;  // neither idle nor backoff -> busy

	// We need to do this with interrupts off.
	// The last time we check for free net until sending our start bit
	// must be as short as possible, not interrupted.
	cli() ;
	// Before we do anything else - Disable StartBit Interrupt
	ACA.AC0CTRL &= ~AC_INTLVL_HI_gc;
	if (ACA.STATUS&AC_AC0IF_bm)
	{
		// first we disabled it, than before sending the start bit, we found out
		// that somebody was faster by examining the start bit interrupt request flag
		ACA.AC0CTRL |= AC_INTLVL_HI_gc;
		sei() ;  // receive now what our rival is sending
		return LN_NETWORK_BUSY;
	}
	
	TCE0.CTRLD = TC_EVACT_OFF_gc;
	LN_SW_UART_SET_TX_LOW        // Begin the Start Bit

	// Get the Current Timer1 Count and Add the offset for the Compare target
	// added adjustment value for bugfix (Olaf Funke)
	TCE0.CNT = TCE0.CCC + LN_TIMER_TX_RELOAD_ADJUST;

	sei() ;

	lnTxData = TxData ;
	lnTxIndex = 0 ;
	lnTxSuccess = 0 ;

    // Load the first Byte
	lnCurrentByte = TxData->data[ 0 ] ;

    // Set the State to Transmit
	lnState = LN_ST_TX ;                      

    // Reset the bit counter
	lnBitCount = 0 ;                          

    // Clear the current Compare interrupt status bit and enable the Compare interrupt
	TCE0.INTFLAGS = TC0_CCCIF_bm;
	TCE0.INTCTRLB |= TC_CCCINTLVL_HI_gc;
	
	// now busy waiting until the interrupts did the rest
	while (lnState == LN_ST_TX) {}

	if (lnTxSuccess)
	{
		lnRxBuffer->Stats.TxPackets++ ;
		return LN_DONE;
	}

	if (lnState == LN_ST_TX_COLLISION)
	{
		return LN_COLLISION;
	}

	return LN_UNKNOWN_ERROR; // everything else is an error
}

