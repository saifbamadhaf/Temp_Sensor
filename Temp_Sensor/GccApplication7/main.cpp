#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

// Define clock speed and baud rate
#define F_CPU 16000000UL // 16 MHz clock speed
#define BAUD 9600
#define UBRRn ((F_CPU/(16UL*BAUD)) - 1) // 179

// Define I2C settings
#define SCL_CLOCK 100000L
#define TEMP_SENSOR_ADDR 0x48 // DS1621 address with A2, A1, A0 connected to GND
#define IO_EXPANDER_ADDR 0x20 // MCP23008 address with A2, A1, A0 connected to GND
#define IODIR 0x00  // IODIR register
#define GPIO 0x09  // GPIO register
#define WRITE 0x00
#define READ 0x01

// Initialize USART
void USART_Init(void) {
	UBRR0H = (UBRRn >> 8);
	UBRR0L = UBRRn;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Set format: 8 data bits, 1 stop bit between each 8 bits
}

// Transmit data via USART
void USART_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0))); // Wait for  until u can send data
	UDR0 = data; // Put data into buffer, sends the data
}

// Print string via USART
void USART_Print(const char *s) {
	while (*s) {
		USART_Transmit(*s++); // iterates through each char till null. when null is reached it send the character to transmission.
	}
}

// Initialize I2C interface
void i2c_init(void) {
// Set prescaler to 1 (TWPS0=0 TWPS1 = 0)
// TWSR &= ~(1 << TWPS0); // not needed as TWPS0 TWPS1 are initially 0
// TWSR &= ~(1 << TWPS1);
 // TWBR = ((F_CPU) / (SCL_CLOCK) * 2 + 16);
	TWBR = ((F_CPU / 100000UL) / 2) - 8; // Set SCL frequency p.222
}

// Start I2C communication
void i2c_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Start condition. TWINT Flag, TWSTA Start, TWEN Enable TWI.
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT to clear
}

// Stop I2C communication
void i2c_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // Stop condition
}

// Write data to I2C
void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

// Read data with ACK from I2C
uint8_t i2c_read_ack(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// Read data with NACK from I2C
uint8_t i2c_read_nack(void) {
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// Initialize DS1621 temperature sensor
void init_TempSensor(void) {
	i2c_start();
	i2c_write(TEMP_SENSOR_ADDR << 1 | WRITE); // Write mode
	i2c_write(0xEE); // Start Convert T command
	i2c_stop();
}

// Initialize MCP23008 IO expander
void init_IOEXPANDER(void) {
	i2c_start();
	i2c_write(IO_EXPANDER_ADDR << 1);
	i2c_write(IODIR); // IODIR register address, configure all pins as output
	i2c_write(0x00); // Set all pins as output
	i2c_stop();
}

// Write data to MCP23008 GPIO register
void write_IOEXPANDER(uint8_t data) {
	i2c_start();
	i2c_write(IO_EXPANDER_ADDR << 1 | WRITE); // Write mode
	i2c_write(GPIO); // GPIO register address
	i2c_write((uint8_t)data); // Write the temperature value directly to GPIO register
	i2c_stop();
}

// Setup function
void setup(void) {
	USART_Init(); // Initialize USART
	i2c_init(); // Initialize I2C interface
	init_IOEXPANDER(); // Initialize MCP23008
}

// Main loop
void loop(void) {
	char buffer[30]; // Buffer for temperature string
	int temp = 0; // Whole part of the temperature
	int tempFraction = 0; // Fractional part of the temperature

	_delay_ms(1000); // Give time for measurement

	init_TempSensor(); // Initialize DS1621 temperature sensor

	i2c_start();
	i2c_write(TEMP_SENSOR_ADDR << 1 | WRITE); // Write mode
	i2c_write(0xAA); // Read temperature command
	i2c_stop();

	i2c_start();
	i2c_write(TEMP_SENSOR_ADDR << 1 | READ); // Read mode
	temp = (int8_t)i2c_read_ack(); // Read temperature as signed int
	tempFraction = i2c_read_nack() & 0b10000000; // Extract MSB (9th bit) of the LSB
	i2c_stop();

	// Display temperature on LEDs in binary format
	write_IOEXPANDER(temp);
	
	// int tempF = temp * 9 / 5 + 32; for F

	// Print temperature value to serial monitor
	snprintf(buffer, sizeof(buffer), "Temperature: %d.%d Celsius\n", temp, tempFraction ? 5 : 0);

	USART_Print(buffer);
}


int main(void) {
	setup();
	while (1) {
		loop();
	}
	return 0;
}
