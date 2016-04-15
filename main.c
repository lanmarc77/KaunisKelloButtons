/*
Copyright (C) 2016  Marcel Langner (langner.marcel@myiq.de)

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
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "irmp.h"
/*
1=back
2=ok
4=up
8=d0wn
*/
unsigned char button_state=0;


unsigned char TWIByteCounter=0;
unsigned char debounce_counter=0;
IRMP_DATA   irmp_data;
#define TWI_MASTER_ADDR 0x32

ISR(TWI_vect)
{
	unsigned char stopCond=0;
    switch (TWSR&(0xFC))
    {
        //master transmitter

        case 0x08: //A START condition has been transmitted
                    TWDR=TWI_MASTER_ADDR;
                    break;
        case 0x10: //A repeated START condition has been transmitted
                    break;
        case 0x18: //SLA+W has been transmitted; ACK has been received
                    TWIByteCounter=0;
                    TWDR=0x01;//type key code
                    break;
        case 0x20: //SLA+W has been transmitted; NOT ACK has been received
                    stopCond=1;
                    break;
        case 0x28: //Data byte has been transmitted; ACK has been received
                    TWIByteCounter++;
                    if(TWIByteCounter==2){
                        stopCond=1;
                    }else{
                        TWDR=button_state&0x0F;
                    }
                    break;
        case 0x30: //Data byte has been transmitted; NOT ACK has been received
                    stopCond=1;
                    break;
        case 0x38: //Arbitration lost in SLA+W or data bytes
					TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
						(1<<TWIE)|(1<<TWINT)|                      // Enable Interupts and clear flag
						(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests
						(0<<TWWC);                                 //
                    break;
		default:
					break;
	}
	if(stopCond){
		TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
		(1<<TWIE)|(1<<TWINT)|                      // Enable Interupts and clear flag
		(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // No Signal requests
		(0<<TWWC);                                 //
	}else{
		TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
		(1<<TWIE)|(1<<TWINT)|                      // Enable Interupts and clear flag
		(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests
		(0<<TWWC);                                 //
	}
}	

void I2C_send(unsigned char c){
	TWBR = 18;                                  // Set bit rate register (Baudrate).
	TWSR = 0x01;                                        //prescaler = 4
	TWDR = 0xFF;                                      // Default content = SDA released.
	TWCR = (1<<TWEN)|                             // TWI Interface enabled.
		 (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
		 (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
		 (0<<TWWC);
	TWIByteCounter=0;
	button_state=c;
}

unsigned char ir_buttons=0;
unsigned char real_buttons=0;

ISR(TIMER0_COMPA_vect){
	irmp_ISR();
	irmp_data.command=0;
	irmp_data.flags=0;
	irmp_data.protocol=0;
	if (irmp_get_data (&irmp_data))
	{
		if((irmp_data.flags & IRMP_FLAG_REPETITION) ==  0){
			if(	((irmp_data.protocol==IRMP_NEC_PROTOCOL)&&(irmp_data.command == 0x12))||
			((irmp_data.protocol==IRMP_RC5_PROTOCOL)&&(irmp_data.command == 12))
			){
				ir_buttons=0x01;
			}else
			if(	((irmp_data.protocol==IRMP_NEC_PROTOCOL)&&(irmp_data.command == 0x1E))||
			((irmp_data.protocol==IRMP_RC5_PROTOCOL)&&(irmp_data.command == 32))
			){
				ir_buttons=0x04;
			}else
			if(	((irmp_data.protocol==IRMP_NEC_PROTOCOL)&&(irmp_data.command == 0x03))||
			((irmp_data.protocol==IRMP_RC5_PROTOCOL)&&(irmp_data.command == 33))
			){
				ir_buttons=0x08;
			}else
			if(	((irmp_data.protocol==IRMP_NEC_PROTOCOL)&&(irmp_data.command == 0x01))||
			((irmp_data.protocol==IRMP_RC5_PROTOCOL)&&(irmp_data.command == 13))
			){
				ir_buttons=0x02;
			}
		}else{
			ir_buttons=0;
		}
	}else{
		ir_buttons=0;
	}
	if(debounce_counter){
		debounce_counter++;
		if(debounce_counter>=250){
			debounce_counter=0;
			real_buttons=(~PIND>>4)&0x0F;
		}
	}
	if(ir_buttons|real_buttons){
		I2C_send(ir_buttons|real_buttons);
		real_buttons=0;
		ir_buttons=0;
	}
	
}



ISR(PCINT2_vect){
	debounce_counter=1;
}


int main (void)
{

	DDRD&=~0x80;//PD7/PCINT23
	PORTD|=0x80;//pullup enable

	DDRD&=~0x40;//PD6/PCINT22
	PORTD|=0x40;//pullup enable

	DDRD&=~0x20;//PD5/PCINT21
	PORTD|=0x20;//pullup enable

	DDRD&=~0x10;//PD4/PCINT20
	PORTD|=0x10;//pullup enable

	sei();
	PCICR|=0x04;//PCIE2 enable
	PCMSK2|=0xF0;//PCINT23..20 enable trigger
	
	OCR0A=100-1;		//resulting in 10Khz call frequency
	TCCR0A=0x02;	//CTC mode
	TCCR0B=0x02;	//prescaler 8
	TIMSK0|=0x02;	//OC0IE enabled
	
	irmp_init();
	
	set_sleep_mode(SLEEP_MODE_IDLE);
	while(1){
		sleep_mode();
	}
}
