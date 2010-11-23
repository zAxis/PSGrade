/*
    PSGrade

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

#include <LUFA/Version.h>
#include <LUFA/Drivers/Board/LEDs.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/USB/Class/CDC.h>

#include "key.h"
#include "hmac.h"

#define RED	(LEDS_LED1)
#define GREEN	(LEDS_LED2)
#define BOTH	(RED|GREEN)
#define NONE	(LEDS_NO_LEDS)
#define LED(x) LEDs_SetAllLEDs(x)

#define PORT_EMPTY 0x0100   /* powered only */
#define PORT_FULL 0x0103    /* connected, enabled, powered, full-speed */
#define C_PORT_CONN  0x0001 /* connection */
#define C_PORT_RESET 0x0010 /* reset */
#define C_PORT_NONE  0x0000 /* no change */
#define CHALLENGE_INDEX	7

uint16_t port_status[6] = { PORT_EMPTY, PORT_EMPTY, PORT_EMPTY, PORT_EMPTY, PORT_EMPTY, PORT_EMPTY };
uint16_t port_change[6] = { C_PORT_NONE, C_PORT_NONE, C_PORT_NONE, C_PORT_NONE, C_PORT_NONE, C_PORT_NONE };

enum { 
	init,
	wait_hub_ready,
	hub_ready,
	p5_wait_reset,
	p5_wait_enumerate,
	p5_challenged,
	p5_responded,
	p5_wait_disconnect,
	p5_disconnected,
	done,
} state = init;


uint8_t hub_int_response = 0x00;
uint8_t hub_int_force_data0 = 0;
int last_port_conn_clear = 0;
int last_port_reset_clear = 0;

int8_t port_addr[7] = { -1, -1, -1, -1, -1, -1, -1 };
int8_t port_cur = -1;

void USB_Device_SetDeviceAddress(uint8_t Address)
{
	port_addr[port_cur] = Address & 0x7f;
	UDADDR = Address & 0x7f;
	UDADDR |= (1 << ADDEN);
}

void switch_port(int8_t port)
{
	if (port_cur == port) return;
	port_cur = port;
	if (port_addr[port] < 0)
		port_addr[port] = 0;
	UDADDR = port_addr[port] & 0x7f;
	UDADDR |= (1 << ADDEN);
}

volatile uint8_t expire = 0; /* counts down every 10 milliseconds */
ISR(TIMER1_OVF_vect) 
{ 
	uint16_t rate = (uint16_t) -(F_CPU / 64 / 100);
	TCNT1H = rate >> 8;
	TCNT1L = rate & 0xff;
	if (expire > 0)
		expire--;
}



void panic(int led1, int led2)
{
	for(;;) {
		_delay_ms(100);
		LED(led1);
		_delay_ms(100);
		LED(led2);
	}		
}

void HUB_Task(void)
{
	Endpoint_SelectEndpoint(1);

	if (Endpoint_IsReadWriteAllowed())
	{
		if (hub_int_response) {
			if (hub_int_force_data0) {
				Endpoint_ResetDataToggle();
				hub_int_force_data0 = 0;
			}
			Endpoint_Write_Byte(hub_int_response);
			Endpoint_ClearIN();
			hub_int_response = 0x00;
		}
	}
}


void JIG_Task(void)
{
	static int bytes_out = 0, bytes_in = 0;

        Endpoint_SelectEndpoint(2);
        if (Endpoint_IsReadWriteAllowed())
        {
		Endpoint_Read_Stream_LE  (&jig_challenge_res[bytes_out], 8, NO_STREAM_CALLBACK) ;		
        Endpoint_ClearOUT();
		bytes_out += 8;
		if (bytes_out >= 64) {			
			
			//prepare the response
			jig_challenge_res[1]--;
			jig_challenge_res[3]++;
			jig_challenge_res[6]++;
			HMACBlock(&jig_challenge_res[CHALLENGE_INDEX],20);
			
			USB_USBTask(); //just in case
			HMACDone();
			jig_challenge_res[7] = jig_id[0];
			jig_challenge_res[8] = jig_id[1];
			int i;
			for( i = 0; i < 20; i++)
				jig_challenge_res[9+i] = hmacdigest[i];
			state = p5_challenged;
			expire = 50; // was 90
		}
	}

        Endpoint_SelectEndpoint(1);
        if (Endpoint_IsReadWriteAllowed() && state == p5_challenged && expire == 0) 
	{
		if ( bytes_in < 64) {
			Endpoint_Write_Stream_LE(&jig_challenge_res[bytes_in], 8, NO_STREAM_CALLBACK);
			Endpoint_ClearIN();
			bytes_in += 8;
			if ( bytes_in >= 64) {
			
				state = p5_responded;
				expire = 50;
			}
		}
	}
}

void connect_port(int port)
{
	last_port_reset_clear = 0;
	hub_int_response = (1 << port);
	port_status[port - 1] = PORT_FULL;
	port_change[port - 1] = C_PORT_CONN;

}

void disconnect_port(int port)
{
	last_port_conn_clear = 0;
	hub_int_response = (1 << port);
	port_status[port - 1] = PORT_EMPTY;
	port_change[port - 1] = C_PORT_CONN;
}



void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Setup timer */
	TCCR1B = 0x03;  /* timer rate clk/64 */
	TIMSK1 = 0x01;

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
	sei(); 
}


int main(void)
{
	SetupHardware();
	HMACInit(jig_key,20);

	LED(~GREEN);
	state = init;
	switch_port(0);

	for (;;)
	{
		if (port_cur == 0)
			HUB_Task();

		if (port_cur == 5)
			JIG_Task();

		USB_USBTask();
		
			//connect 5
		if (state == hub_ready && expire == 0)
		{
			connect_port(5);
			state = p5_wait_reset;
		}
		

		if (state == p5_wait_reset && last_port_reset_clear == 5)
		{
			switch_port(5);
			state = p5_wait_enumerate;
		}

		// disconnect 5
		if (state == p5_responded && expire == 0)
		{			
			switch_port(0);
			disconnect_port(5);
			state = p5_wait_disconnect;
		}
		
		if (state == p5_wait_disconnect && last_port_conn_clear == 5)
		{
			state = p5_disconnected;
			expire = 20;
		}
		
		// done
		if (state == p5_disconnected && expire == 0)
		{
			LED(GREEN);
			break;
		}
	}
}

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);

	void*          Address = NULL;
	uint16_t       Size    = NO_DESCRIPTOR;

	switch (DescriptorType)
	{
	case DTYPE_Device:
		switch (port_cur) {
		case 0:
			Address = (void *) HUB_Device_Descriptor;
			Size    = sizeof(HUB_Device_Descriptor);
			break;
		case 5:
			Address = (void *) port5_device_descriptor;
			Size    = sizeof(port5_device_descriptor);
			break;
		}
		break;
	case DTYPE_Configuration: 
		switch (port_cur) {
		case 0:
			Address = (void *) HUB_Config_Descriptor;
			Size    = sizeof(HUB_Config_Descriptor);
			break;
		case 5:
			// 1 config
			Address = (void *) port5_config_descriptor;
			Size    = sizeof(port5_config_descriptor);
			break;
		}
		break;
	case 0x29: // HUB descriptor (always to port 0 we'll assume)
		switch (port_cur) {
		case 0:
			Address = (void *) HUB_Hub_Descriptor;
			Size    = sizeof(HUB_Hub_Descriptor);
			break;
		}
		break;
	}
	
	*DescriptorAddress = Address;
	return Size;
}

void EVENT_USB_Device_Connect(void) { }
void EVENT_USB_Device_Disconnect(void) { }

void EVENT_USB_Device_UnhandledControlRequest(void)
{		
	
	if (port_cur == 5 && USB_ControlRequest.bRequest == REQ_SetInterface)
	{
		/* can ignore this */
		Endpoint_ClearSETUP();
		Endpoint_ClearIN();
		Endpoint_ClearStatusStage();
		return;
	}

	if (port_cur == 0 &&
	    USB_ControlRequest.bmRequestType == 0xA0 &&
	    USB_ControlRequest.bRequest == 0x00 &&  // GET HUB STATUS
	    USB_ControlRequest.wValue == 0x00 &&
	    USB_ControlRequest.wIndex == 0x00 &&
	    USB_ControlRequest.wLength == 0x04) {
		Endpoint_ClearSETUP();
		Endpoint_Write_Word_LE(0x0000); // wHubStatus
		Endpoint_Write_Word_LE(0x0000); // wHubChange
		Endpoint_ClearIN();
		Endpoint_ClearStatusStage();
		return;
	}

	if (port_cur == 0 &&
	    USB_ControlRequest.bmRequestType == 0xA3 &&  
	    USB_ControlRequest.bRequest == 0x00 &&   //  GET PORT STATUS
	    USB_ControlRequest.wValue == 0x00 &&
	    USB_ControlRequest.wLength == 0x04) {
		uint8_t p = USB_ControlRequest.wIndex;
		if (p < 1 || p > 6) return;

		Endpoint_ClearSETUP();
		Endpoint_Write_Word_LE(port_status[p - 1]); // wHubStatus
		Endpoint_Write_Word_LE(port_change[p - 1]); // wHubChange
		Endpoint_ClearIN();
		Endpoint_ClearStatusStage();
		return;
	}

	if (port_cur == 0 &&
	    USB_ControlRequest.bmRequestType == 0x23 &&
	    USB_ControlRequest.bRequest == 0x03 && // SET_FEATURE
	    USB_ControlRequest.wLength == 0x00) {
		uint8_t p = USB_ControlRequest.wIndex;
		if (p < 1 || p > 6) return;
		
		Endpoint_ClearSETUP();
		Endpoint_ClearIN();
		Endpoint_ClearStatusStage();

		switch(USB_ControlRequest.wValue) {
		case 0x0008: // PORT_POWER
			if (p == 6 && state == init) {
				/* after the 6th port is powered, wait a bit and continue */
				state = hub_ready;
				expire = 15;
			}
			break;
		case 0x0004: // PORT_RESET
			hub_int_response = (1 << p);
			port_change[p - 1] |= C_PORT_RESET;
			break;
		}
		return;
	}

	if (port_cur == 0 &&
	    USB_ControlRequest.bmRequestType == 0x23 &&
	    USB_ControlRequest.bRequest == 0x01 && // CLEAR_FEATURE
	    USB_ControlRequest.wLength == 0x00) {
		uint8_t p = USB_ControlRequest.wIndex;
		if (p < 1 || p > 6) return;
		
		Endpoint_ClearSETUP();
		Endpoint_ClearIN();
		Endpoint_ClearStatusStage();

		switch(USB_ControlRequest.wValue) {
		case 0x0010: // C_PORT_CONNECTION
			last_port_conn_clear = p;
			port_change[p - 1] &= ~C_PORT_CONN;
			break;
		case 0x0014: // C_PORT_RESET
			last_port_reset_clear = p;
			port_change[p - 1] &= ~C_PORT_RESET;
			break;
		}
		return;
	}

	panic(RED, GREEN);
}

void EVENT_USB_Device_ConfigurationChanged(void)
{ 
	/* careful with endpoints: we don't reconfigure when "switching ports"
	   so we need the same configuration on all of them */
	if (!Endpoint_ConfigureEndpoint(1, EP_TYPE_INTERRUPT, ENDPOINT_DIR_IN, 8, ENDPOINT_BANK_SINGLE))
		panic(GREEN, BOTH);
	if (!Endpoint_ConfigureEndpoint(2, EP_TYPE_INTERRUPT, ENDPOINT_DIR_OUT, 8, ENDPOINT_BANK_SINGLE))
		panic(GREEN, BOTH);
}

void EVENT_USB_Device_Suspend(void) { }
void EVENT_USB_Device_WakeUp(void) { }
void EVENT_USB_Device_Reset(void) { }
void EVENT_USB_Device_StartOfFrame(void) { }
void EVENT_USB_InitFailure(const uint8_t ErrorCode) { }
void EVENT_USB_UIDChange(void) {}
