#include <atmel_start.h>
#include <util/delay.h>

//Pin number definitions
#define RED_LED   6 //PORT A6
#define GREEN_LED 7 //PORT A5
#define BLUE_LED  5 //PORT A7
#define PG        2 //Port A2
#define SW        0 //Port C0
#define STAT1     3 //Port C3
#define STAT2     1 //Port C1

typedef struct  
{
	uint8_t id;
	uint8_t port;
	uint8_t port_pin;
	uint8_t pin_type;
}peripheral_t;

typedef enum{
	PORT_A,
	PORT_B,
	PORT_C,
}port_num_t;

typedef enum{
	INPUT,
	OUTPUT,
}pin_type_t;

typedef enum{
	red_led,
	green_led,
	blue_led,
	pg,
	stat1,
	stat2,
	sw,
	num_periphs,
}peripheral_id;

peripheral_t periphs[(uint8_t) num_periphs] = {
	{red_led, PORT_A, RED_LED, OUTPUT},
	{green_led, PORT_A, GREEN_LED, OUTPUT},
	{blue_led, PORT_A, BLUE_LED, OUTPUT},
	{pg, PORT_A, PG, INPUT},
	{stat1, PORT_C, STAT1, INPUT},
	{stat2, PORT_C, STAT2, INPUT},
	{sw, PORT_C, SW, INPUT},
	};

//Variables to track ISR state changes
uint8_t sw_state_change = 0;
uint8_t pg_state_change = 0;

//main.c function declarations
void check_for_UART_msg(uint8_t * cur_msg_size, char * buffer);
void process_uart_message(char *buffer);
uint8_t validate_uart_checksum(char *buffer);
void process_uart_request(char *buffer);
void process_uart_command(char *buffer);
void set_led_by_id(peripheral_id id, char val);
uint8_t is_vaild_start_byte(uint8_t new_val);
void send_data_uart_msg(char *buffer, peripheral_id id, uint16_t data);
uint8_t calculate_uart_checksum(char *buffer);
uint16_t get_data_from_periph(peripheral_id id);

/**
 * \brief main function for battery box operation.
 *
 * Function is called when the pull switch (Port C Pin 0) sees a rising or 
 * falling edge.
 */
int main(void)
{
	/* Initializes MCU */
	atmel_start_init();
	USART_0_enable();
	char buffer[UART_MSG_SIZE];
	uint8_t cur_msg_size = 0;
	
	//set PortA Outputs
	PORTA.DIR |= (  1 << RED_LED
	              | 1 << GREEN_LED
				  | 1 << BLUE_LED);
    //set PortA Inputs
	PORTA.DIR &= ~(1 << PG);
				  
	//set PortC Inputs
	PORTC.DIR &= ~( 1 << SW
	              | 1 << STAT1
				  | 1 << STAT2);
	
	//set up interrupts for the switch and pg
	PORTC.PIN0CTRL |= (1); //enable rising and falling edge detection PortCPin0
	PORTA.PIN2CTRL |= (1); //enable rising and falling edge detection PortAPin2
	sei();

	while (1) {
		check_for_UART_msg(&cur_msg_size, buffer);
		if(pg_state_change)
		{
			//send updated pg message
			buffer[1] = pg;
			process_uart_request(buffer);
			pg_state_change = 0;
		}
		if(sw_state_change)
		{
			//send updated sw message, 
			buffer[1] = sw;
			process_uart_request(buffer);
			sw_state_change = 0;
		}
	}
}

/**
 * \brief ISR function for pull switch PORTC interrupt
 *
 * Function is called when the pull switch (Port C Pin 0) sees a rising or 
 * falling edge.
 */
ISR(PORTC_PORT_vect)
{
	cli();
	sw_state_change = 1;
	//clear the interrupt flag
	PORTC.INTFLAGS |= (1 << SW);
	sei();
}

/**
 * \brief ISR function for PG (charging) PORTA interrupt
 *
 * Function is called when the pg pin (Port A Pin 2) sees a rising or 
 * falling edge. This indicates that charging has started or stopped.
 */
ISR(PORTA_PORT_vect)
{
	cli();
	pg_state_change = 1;
	//clear the interrupt flag
	PORTA.INTFLAGS |= (1 << PG);
	sei();
}

/**
 * \brief Checks for new characters from the USART port.
 * Checks for new characters and reads them in if present. Only accepts valid
 * start bits for the first character to fix communication mismatches. Calls
 * the appropriate processing function if a full message has been received.
 *
 * \param[in] cur_msg_size a pointer to the current message size stored
 * in buffer
 * \param[in] buffer a buffer to store fixed length USART messages 
 *
 * \return Nothing
 */
void check_for_UART_msg(uint8_t *cur_msg_size, char *buffer)
{
	uint8_t new_val = 0;
	if(USART_0_is_rx_ready())
	{	
		while(USART_0_is_rx_ready())
		{
			//read the new character to verify 
			new_val = USART_0_read();
			if(*cur_msg_size == 0)
			{
				//verify the first character of a message is a valid start bit
				if(is_vaild_start_byte(new_val))
				{
					buffer[*cur_msg_size] = new_val;
					*cur_msg_size += 1;
				}
			}
			else
			{
				buffer[*cur_msg_size] = new_val;
				*cur_msg_size += 1;
			}
		}
		if(*cur_msg_size >= UART_MSG_SIZE)
		{
			//A complete message has been read
			process_uart_message(buffer);
			*cur_msg_size = 0;
		}
	}
}

/**
 * \brief Process a received UART message.
 * Calls different sub processing functions based on the message type.
 *
 * \param[in] buffer a buffer to store fixed length UART messages 
 *
 * \return Nothing
 */
void process_uart_message(char *buffer)
{
	//Only process if the checksum is valid
	if(validate_uart_checksum(buffer))
	{
		uint8_t msg_type = buffer[0];
		if(msg_type == CMD_TYPE)
		{
			process_uart_command(buffer);
		}
		else if(msg_type == REQ_TYPE)
		{
			process_uart_request(buffer);
		}
	}
	else
	{
		//flash the red led for debugging purposes
		//PORTA.OUT |= (1 << RED_LED);
	}
}

/**
 * \brief Validates a simple UART message using the checksum.
 * Sums the entire fixed length UART message.
 *
 * \param[in] buffer a buffer to store fixed length UART messages 
 *
 * \return A boolean value. 1 indicates a valid checksum. Otherwise 0.
 */
uint8_t validate_uart_checksum(char *buffer)
{
	uint8_t sum = 0, i = 0;
	for(i=0; i<UART_MSG_SIZE; i++)
	{
		sum += buffer[i];
	}
	return (sum == 0xFF);
}

/**
 * \brief Processes a UART command message.
 * Performs the action indicated by the message. This function is simplified
 * because all current commands are limited to changing the LED status.
 *
 * \param[in] buffer a buffer to store fixed length UART messages 
 *
 * \return Nothing
 */
void process_uart_command(char *buffer)
{
	//perform the command
	peripheral_id id;
	id = buffer[1];
	set_led_by_id(id, buffer[3]);
	//send a data packet as an ACK
	send_data_uart_msg(buffer, id, (uint16_t)buffer[3]);
}

/**
 * \brief Processes a UART request message.
 * Gets and then sends the data requested by the message.
 *
 * \param[in] buffer a buffer to store fixed length UART messages 
 *
 * \return Nothing
 */
void process_uart_request(char *buffer)
{
	uint16_t new_data;
	//get the data from the peripheral
	new_data = get_data_from_periph(buffer[1]);
	//send the data
	send_data_uart_msg(buffer, buffer[1], new_data);
}

/**
 * \brief Set an LED using its peripheral ID
 * Looks up the port and pin assigned to the peripheral id and sets it
 * according to val.
 *
 * \param[in] id the peripheral id of the LED to set
 * \param[in] val the binary value to set the led
 *
 * \return Nothing
 */
void set_led_by_id(peripheral_id id, char val)
{
	if(val)
	{
		//turn on the led defined by id
		PORTA.OUT |= (1 << periphs[id].port_pin);
	}
	else
	{
		//turn off the led defined by id
		PORTA.OUT &= ~(1 << periphs[id].port_pin);
	}
}

/**
 * \brief Checks if the value is a valid UART starting byte
 * Returns a boolean value according to the UART messaging specification. 
 * This prevents communication errors due to message shifting.
 *
 * \param[in] new_val a received UART character
 *
 * \return a boolean value to represent the start byte status. 1 indicates 
 * a valid byte.
 */
uint8_t is_vaild_start_byte(uint8_t new_val)
{
	return ((new_val == CMD_TYPE) | (new_val == REQ_TYPE) | (new_val == DAT_TYPE));
}

/**
 * \brief Send a data type UART message
 * Stages the buffer and writes the data to the USART_0 peripheral
 *
 * \param[in] buffer a buffer for staging a UART data message
 * \param[in] id the peripheral id to be used in the data message
 *
 * \return Nothing
 */
void send_data_uart_msg(char *buffer, peripheral_id id, uint16_t data)
{
	buffer[0] = DAT_TYPE;
	buffer[1] = id;
	buffer[2] = (data >> 8);
	buffer[3] = (data & 0x00FF);
	buffer[4] = calculate_uart_checksum(buffer);
	
	USART_0_write_data(buffer, UART_MSG_SIZE);
}

/**
 * \brief Calculate a UART checksum given a staged message in a buffer
 *
 * \param[in] buffer a buffer containing a staged UART message
 *
 * \return the calculated 8 bit checksum
 */
uint8_t calculate_uart_checksum(char *buffer)
{
	uint8_t i, sum = 0;
	//Stop before summing the checksum field
	for(i=0; i<UART_MSG_SIZE-1; i++)
	{
		sum += buffer[i];
	}
	//return the compliment of the sum
	return ~sum;
}

/**
 * \brief Get the data from a peripheral given a peripheral id
 *
 * \param[in] id a peripheral id
 *
 * \return the 16 bit (only 8 bits used currently) value of the peripheral
 */
uint16_t get_data_from_periph(peripheral_id id)
{
	peripheral_t new_periph;
	new_periph = periphs[id];
	if(new_periph.pin_type == INPUT)
	{
		if(new_periph.port == PORT_A)
		{
			return (PORTA.IN & (1 << new_periph.port_pin));
		}
		else if(new_periph.port == PORT_C)
		{
			return (PORTC.IN & (1 << new_periph.port_pin));
		}
		return 255;
	}
	else if(new_periph.pin_type == OUTPUT)
	{
		//read from the output port- all outputs are on PORTA, return the data
		return (PORTA.OUT & (1 << new_periph.port_pin));
	}
	return 0;
}