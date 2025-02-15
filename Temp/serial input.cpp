#include <Arduino.h>

// Pin definitions
const int dataPin = 2;   // DS (Serial Data Input) - Arduino reads data here
const int clockPin = 3;  // SHCP (Shift Register Clock Input) - Arduino reads clock here
const int latchPin = 4;  // STCP (Storage Register Clock Input) - Arduino reads latch here

// Array to store the 8-bit output
byte shiftRegister = 0b00000000; // Initialize all bits to 0

// Variables to track the state of the clock and latch pins
int lastClockState = LOW;
int lastLatchState = LOW;

void setup() {
  // Set pin modes
  pinMode(dataPin, INPUT);      // Data pin is an INPUT
  pinMode(clockPin, INPUT);     // Clock pin is an INPUT
  pinMode(latchPin, INPUT);     // Latch pin is an INPUT

  // Initialize Serial Monitor for debugging (optional)
  Serial.begin(9600);
}

void loop() {
  // Read the current state of the clock and latch pins
  int currentClockState = digitalRead(clockPin);
  int currentLatchState = digitalRead(latchPin);

  // Check for a rising edge on the clock pin
  if (currentClockState == HIGH && lastClockState == LOW) {
    // Shift in a new bit from the data pin
    shiftInBit();
  }

  // Check for a rising edge on the latch pin
  if (currentLatchState == HIGH && lastLatchState == LOW) {
    // Update the output (simulate latching the data)
    updateOutput();
  }

  // Save the current state of the clock and latch pins for the next loop iteration
  lastClockState = currentClockState;
  lastLatchState = currentLatchState;
}

// Function to shift in a single bit from the data pin
void shiftInBit() {
  // Read the data pin
  int bitValue = digitalRead(dataPin);

  // Shift the shiftRegister left and add the new bit
  shiftRegister = (shiftRegister << 1) | bitValue;

  // Print the current state of the shift register for debugging (optional)
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