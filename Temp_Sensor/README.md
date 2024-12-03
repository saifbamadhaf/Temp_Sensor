Saif Ba Madhaf

### Circuit Design:

#### Temperature sensor `DS1621` pin layout :



`SDA`: is connected t0 a pull_up 10k Ohm resistor to the SDA A4 pin of the Arduino UNO.

`SCL`: is connected to a pull_up 10k Ohm resistor to the SCL A5 pin of the Arduino UNO.

`Tout`: Is not connected to anything.

`GND`: is connected to Arduino ground.

`A0` `A1` `A2`: Address pins are all connected to ground.

`VDD`: is connected to Arduino's 5v.

----------------------------------------------------------------------------------------------------------------


#### I/O expander `MCP23008` pin layout:



`SCL`: is connected to the same pull_up 10k Ohm resistor used in the ``DS1621`` sensor to the SCL A5 pin of the Arduino UNO.

`SDA`: is connected to the same pull_up 10k Ohm resistor used in the ``DS1621`` sensor to the SDA A4 pin of the Arduino UNO.

`A0` `A1` `A2`: Address pins are all connected to ground.

`Reset` & `interrupt`: are not used.

`Vss`: is connected to Arduino's ground.

`GP0` - `GP7`: are connected to LEDs.

`VDD`: is connected to Arduino's 5v.


----------------------------------------------------------------------------------------------------------------------
### Two Wire Interface:


This is the functions needed to implement the `TWI` or Two Wires Interface:

```c
// Initialize I2C interface
void i2c_init(void) {
// Set prescaler to 1 (TWPS0=0 TWPS1 = 0)
// TWSR &= ~(1 << TWPS0); // not needed as TWPS0 TWPS1 are initially 0
// TWSR &= ~(1 << TWPS1);
TWBR = ((F_CPU) / (SCL_CLOCK) * 2 + 16); // Set SCL frequency p.222
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
```

To begin with to transfer data through the two wires there is 9 bits sent. The first 7 bits of the `MSB` are for the slave addresses 
and 1 bit is for the `READ` `1` or `WRITE` `0`  operation and the last bit the acknowledge `ACK` and non acknowledge `NACK` bit.

An acknowledge ``ACK`` is when the Receiver accepts a data packet. It does this by pulling the data line (SDA) low.
If the Receiver cannot accept the data or the communication is ending, it signals a non-acknowledge (NACK) by leaving the SDA line high.


----------------------------------------------------------------------------------------------------------


The code:
```c
#define F_CPU 16000000UL // 16 MHz clock speed
#define SCL_CLOCK 100000L


void i2c_init(void) {
// Set prescaler to 1 (TWPS0=0 TWPS1 = 0)
// TWSR &= ~(1 << TWPS0); // not needed as TWPS0 TWPS1 are initially 0
// TWSR &= ~(1 << TWPS1);
TWBR = ((F_CPU) / (SCL_CLOCK) * 2 + 16); // Set SCL frequency p.222
}

```

We create a function to initialize the `TWI`.
In this function, we begin by setting the prescaler to 1 which means setting `TWPS0` and `TWPS1` both to zero. In this
part I did not as it is initially zero. Then we begin by calculating the `TWBR` which is the two wire bit rate, which could 
be found in the atmega328p datasheet.


---------------------------------------------------------------------------------------------------

```c
void TWI_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); //Send START condition
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT FLag set. This indicates that the START condition has been transmitted
}
```
We the go to create a function that start the `TWI`

This function starts by:
`TWEN` must be set to enable the 2-wire Serial Interface, `TWSTA` must be written to one to transmit a START condition and 
`TWINT` must be written to one to clear the `TWINT` Flag.

```c
void TWI_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}
```
This is the function of how to STOP the condition

This function is similar to the start function, however we write one to `TWSTO` to stop the transmission.

```c

void i2c_write(uint8_t data) {
TWDR = data;
TWCR = (1 << TWINT) | (1 << TWEN);
while (!(TWCR & (1 << TWINT)));
}
```
This function is for transmitting a single byte of `data` over the `TWI`

```TWDR = data;``` This loads the data u want to send to the two wire data register. The `TWDR` holds the data you want 
to send.

```TWCR = (1 << TWINT) | (1 << TWEN);``` Here we clear the `TWINT` flag, then we enable the two wire interface using `TWEN`.
so that we can start sending the data.

```while (!(TWCR & (1 << TWINT))); ``` Loop that waits for the `TWINT` flag to set and when its set the transmission can stop.


----------------------------------------------------------------------------------------------------------------------

This function for the acknowledge bit
```c
uint8_t i2c_read_ack(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}
```
We begin by clearing the `TWINT` flag, then we enable the two wire interface `TWEN`, and we enable the acknowledge bit by
setting the `TWEA` to 1.

```while (!(TWCR & (1 << TWINT)));``` loop that doesnt stop until the `TWINT` flag is 1 again, which means the operation is done.

Finally, we return the function to the `TWDR` register.

----------------------------------------------------------------------------------------------------------------------

This function for the non acknowledge bit
```c

uint8_t i2c_read_nack(void) {
TWCR = (1 << TWINT) | (1 << TWEN);
while (!(TWCR & (1 << TWINT)));
return TWDR;
}
```
The code similar to the acknowledge bit, however we do not enable the `TWEA` which is for the acknowledge bit.

### What is the difference between an acknowledge and non acknowledge?
 
The difference is that the acknowledge means that the data byte transfer was recieved, it usually gives an acknowledge 
after each byte transfer. Non acknowledge means that you want to tell the master device that it will not recieve anymore bytes transfers.

-------------------------------------------------------------------------------------------------------------------------

After implementing the necessary `TWI` function we can now initialize the I2C ICs.

#### Initializing the temperature sensor `DS1621`

```c
#define TEMP_SENSOR_ADDR 0x48 // DS1621 address with A2, A1, A0 connected to GND
#define WRITE 0x00

void init_TempSensor(void) {
	i2c_start();
	i2c_write(TEMP_SENSOR_ADDR << 1 | WRITE); // Write mode
	i2c_write(0xEE); // Start Convert T command
	i2c_stop();
}
```

We start with the function `i2c_start();` to send the START condition prepare the I2C bus for the communication. Then we
use the `i2c_write()` function to write to the device. How to write to i2c device? You begin by getting the devices address
in this case it is 0x48 which is in hexadecimal, then we use our write function to shift the address 7 bits to the left
and we write a zero at the end which means we are in writing mode. This because we know that i2c line takes 9 bits as mentioned
earlier first 7 bits are for the slave address and the 8th bit is for the read/write. Then we write `0xEE` the temp sensor to start
conversion using the address taken from the `DS1621` datasheet. Finally we use `i2c_stop()` function to stop transmission.

-----------------------------------------------------------------------------------------------------------------------

#### Initializing the I/O expander `MCP23008`

```c
#define IO_EXPANDER_ADDR 0x20 // MCP23008 address with A2, A1, A0 connected to GND
#define IODIR 0x00  // IODIR register


void init_IOEXPANDER(void) {
i2c_start();
i2c_write(IO_EXPANDER_ADDR << 1);
i2c_write(IODIR); // IODIR register address
i2c_write(0x00); // Set all pins as output
i2c_stop();
}
```

In the function `init_IOEXPANDER`, we start with the function `i2c_start();` to send the START condition.
Then we use the `i2c_write()` function to write to the io expander. Then we write to the address of the `IODIR` register
to write to it that we want to set all are pins as outputs using this line `i2c_write(0x00);`. Finally we use `i2c_stop()` 
function to stop transmission.



--------------------------------------------------------------------------------------------------------------------------
#### Writing the I/O expander `MCP23008`
```c
#define IO_EXPANDER_ADDR 0x20 // MCP23008 address with A2, A1, A0 connected to GND
#define GPIO 0x09  // GPIO register
#define WRITE 0x00


void write_IOEXPANDER(uint8_t data) {
i2c_start();
i2c_write(IO_EXPANDER_ADDR << 1 | WRITE); // Write mode
i2c_write(GPIO); // GPIO register address
i2c_write((uint8_t)data); // Write the temperature value directly to GPIO register
i2c_stop();
}

```
In this function we basically just write to the `GPIO`  register address which is already set as an output earlier.
Then we write the data bytes from the temp sensor passed in the function here `write_IOEXPANDER(uint8_t data)` to the GPIO pins
so that it can display the temp values.

--------------------------------------------------------------------------------------------------------------------------

#### main Loop

```c
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>


#define F_CPU 16000000UL // 16 MHz clock speed
#define BAUD 9600
#define UBRRn ((F_CPU/(16UL*BAUD)) - 1) // 179


#define SCL_CLOCK 100000L
#define TEMP_SENSOR_ADDR 0x48 // DS1621 address with A2, A1, A0 connected to GND
#define IO_EXPANDER_ADDR 0x20 // MCP23008 address with A2, A1, A0 connected to GND
#define IODIR 0x00  // IODIR register
#define GPIO 0x09  // GPIO register
#define WRITE 0x00
#define READ 0x01



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


```

Finally is the main loop which I will explain the crucial parts. We begin by initializing the io expander, i2c, and temp sensor.

```c
	i2c_start();
	i2c_write(TEMP_SENSOR_ADDR << 1 | WRITE); // Write mode
	i2c_write(0xAA); // Read temperature command
	i2c_stop();
```

Then we start by writing to the temp sensor address and making it in write mode. Afterwards we instruct the temp sensor to read
the temp values using the register `0xAA` found in the datasheet.

```c
	i2c_start();
	i2c_write(TEMP_SENSOR_ADDR << 1 | READ); // Read mode
	temp = (int8_t)i2c_read_ack(); // Read temperature as signed int
	tempFraction = i2c_read_nack() & 0b10000000; // Extract MSB of the LSB in 16 bits
	i2c_stop();
```
This part basically we start by shifting the 7 bit address of the temp sensor 1 bit to the left and make it in read mode `0` using a bitwise `OR`.
In the temp sensor it sends 2 bytes each 8 bits the first byte is temperature and the second byte is the fraction part.
This is why we store the first byte read from the temp sensor to a variable `temp` we do `ACK` since we still need to read the second byte
basically saying reading first byte was successful and there is a second byte to be sent. Afterwards, we extract MSB 
of the LSB since it is 16 bits all together which stores our fractional part to a variable `tempFraction`. Here we use  `i2c_read_nack()` 
becuase we are telling the master device that there is no data bytes to be read anymore.

```c
write_IOEXPANDER(temp);
```

Then we write the `temp` to the io expander so that it can write he values to the GPIO pins which we already set as outputs.

```c
	snprintf(buffer, sizeof(buffer), "Temperature: %d.%d Celsius\n", temp, tempFraction ? 5 : 0);
```
Finally we print the values to the serial monitor. `tempFraction ? 5 : 0` this part means that if the temp fraction recieved
from the temp sensor is on add a .5 and if not add a 0.

--------------------------------------------------------------------------------------------------------------------------------

#### Main loop flowchart:

![Flowchart](C:\Users\saifb\Downloads\aaaab\MicroFinal.jpg)




--------------------------------------------------------------------------------------------------------------------------------


## Code:

```c
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
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Set frame format: 8 data bits, 1 stop bit
}

// Transmit data via USART
void USART_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
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
	TWBR = ((F_CPU) / (SCL_CLOCK) * 2 + 16); // Set SCL frequency p.222
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
	tempFraction = i2c_read_nack() & 0b10000000; // Extract MSB of the LSB
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

```