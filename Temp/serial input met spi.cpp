#include <Arduino.h>
#include <SPI.h>
/*How It Works:
External SPI Master:
An external SPI master device sends data to the Arduino via the SPI bus.
The SPI master also provides the clock signal (SCK) and selects the Arduino as the slave using the SS (Slave Select) pin.

Data Reception:
When the Arduino receives data via SPI, the ISR(SPI_STC_vect) function is triggered, and the data is stored in the shiftRegister variable.

Latch Signal:
The external device sends a latch signal to the Arduino via the latchPin.
When the latchPin goes high, the Arduino updates the output based on the received data.

Output:
The latched data can be used or displayed on other pins or printed to the Serial Monitor for debugging.

Wiring:
Connect the external SPI master's MOSI (Master Out Slave In) to the Arduino's MISO (Pin 12).
Connect the external SPI master's SCK (Serial Clock) to the Arduino's SCK (Pin 13).
Connect the external SPI master's SS (Slave Select) to the Arduino's SS (Pin 10).
Connect the external latch signal to the Arduino's latchPin (e.g., D4).
Optionally, connect LEDs to pins D5 to D12 to visualize the output.

Debugging:
Use the Serial.println(shiftRegister, BIN) statement to print the received data to the Serial Monitor in binary format.
Use the Serial.print("Latched Data: ") statement to print the latched data to the Serial Monitor.*/

// Declare funtions:
void updateOutput();

// Pin definitions
const int latchPin = 4;  // STCP (Storage Register Clock Input) - Latch signal

// Array to store the 8-bit output
byte shiftRegister = 0b00000000; // Initialize all bits to 0

void setup() {
  // Set pin modes
  pinMode(latchPin, INPUT); // Latch pin is an INPUT

  // Initialize SPI in slave mode
  SPCR |= _BV(SPE); // Enable SPI in slave mode
  SPCR |= _BV(SPIE); // Enable SPI interrupt

  // Initialize Serial Monitor for debugging (optional)
  Serial.begin(9600);
}

void loop() {
  // Check for a rising edge on the latch pin
  if (digitalRead(latchPin) == HIGH) {
    // Update the output (simulate latching the data)
    updateOutput();
  }
}

// SPI interrupt service routine
ISR(SPI_STC_vect) {
  // Read the received data from the SPI Data Register (SPDR)
  shiftRegister = SPDR;

  // Print the received data to the Serial Monitor for debugging (optional)
  Serial.print("Received Data: ");
  Serial.println(shiftRegister, BIN);
}

// Function to update the output (simulate latching the data)
void updateOutput() {
  // In a real 74HC595, this is where the data would be sent to the output pins.
  // For simulation, we can print the latched data to the Serial Monitor.
  Serial.print("Latched Data: ");
  Serial.println(shiftRegister, BIN);

  // Optionally, you can output the data to other pins here.
  // For example:
  // for (int i = 0; i < 8; i++) {
  //   digitalWrite(i + 5, bitRead(shiftRegister, i)); // Pins D5 to D12
  // }
}