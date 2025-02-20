/*
With this code, I try to attempted to program a Arduino nano, to use in older transceivers, where the CTCCC board is no longer available.
The goal is to let the Arduino communicate with the transceiver, as it would with a factory CTCSS board.
I found out that Icom, yaesu, and Kenwood, all use a 8 bit shift register IC, and a CTCSS encoder/decoder IC like the FX365, that are obsolete.

I used the library written by PE1CID,that I slightly modified, to generate the sub-audio tone. (CtcssTone)

Pins used:
PIN 2 = PTT_BUTTON to switch from receive to transmit
PIN 3 = PWM_OUTPUT output that generates the sub-audio tone
PIN 4 = latchPin Input of the latch from the transceiver
PIN 8 = DECODE_INDICATOR goes high if CTCSS code is decoded
PIN 11 = MOSI SPI data from transceiver
PIN 13 = SCK SPI clock serial clock from transceiver
PIN 14 = A0 Sub-audio tone input, needs 260Hz lowpass filter
*/

#include <Arduino.h>
#include <SPI.h>
#include <arduinoFFT.h>
#include "CtcssToneId.h"
#include "CtcssTone.h"

// Declare funtions:
int readShiftRegister();
void generateSineWave(int frequency);
bool decodeCTCSS(float targetFrequency);

// PTT Button
#define PTT_BUTTON 2  // Pin connected to the PTT button
const int latchPin = 4;  // STCP (Storage Register Clock Input) - Latch signal

// Decode Indicator Output
#define DECODE_INDICATOR 8  // Pin for decode indicator (e.g., LED)

// PWM Output for Sine Wave
#define PWM_OUTPUT 3  // Pin for PWM output (must be PWM-capable)

// TIA-603-E CTCSS Frequencies (Hz)
const float ctcssFrequencies[] = {
  00.0, 67.0, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100.0, 103.5, 107.2, 110.9, 114.8
  , 118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8
  , 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3
};

const uint8_t ctcssCodes[] = {
  0, 29, 28, 27, 26, 25, 24, 23, 22, 21, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42
  , 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30
};

// Initialize all bits to 0
byte shiftRegister = 0b00000000;


// FFT Settings
#define SAMPLES 128             // Must be a power of 2
#define SAMPLING_FREQUENCY 1000 // Hz, must be at least twice the highest frequency in the signal

ArduinoFFT<double> FFT = ArduinoFFT<double>();

unsigned int samplingPeriod;
unsigned long microSeconds;

double vReal[SAMPLES];
double vImag[SAMPLES];

int selectedCode = 0;  // Stores the selected CTCSS code (0 = off)
int code = 0;
int prefCode = 0;
int numCodes = sizeof(ctcssCodes) / sizeof(ctcssCodes[0]);

void setup() {
  // Latch pin is an INPUT
  pinMode(latchPin, INPUT_PULLUP); 

  // Initialize SPI in slave mode
  SPCR |= _BV(SPE); // Enable SPI in slave mode
  SPCR |= _BV(SPIE); // Enable SPI interrupt

  TCCR1A = _BV(COM1A1) | _BV(WGM10);  // Set PWM mode and output
  TCCR1B = _BV(CS12);  // Set prescaler to 1 (no prescaling)

  // Initialize Serial Monitor for debugging (optional)
  Serial.begin(9600);

  // Initialize PTT button as input
  pinMode(PTT_BUTTON, INPUT);

  // Initialize decode indicator as output
  pinMode(DECODE_INDICATOR, OUTPUT);

  // Initialize PWM output for sine wave
  pinMode(PWM_OUTPUT, OUTPUT);

  // Calculate sampling period
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY)); // Period in microseconds

  CtcssTone.init();
}

void loop() {
  prefCode = selectedCode;
  // Read the serial input and get the selected CTCSS code from ctcssCodes array
  selectedCode = readShiftRegister();

  // Check if the PTT button is pressed
  if (digitalRead(PTT_BUTTON) == HIGH && selectedCode != 0 ) {
    // PTT button is pressed - transmit selected CTCSS tone as a sine wave
    digitalWrite(DECODE_INDICATOR, LOW);  // Turn off decode indicator
    generateSineWave(selectedCode);
    if (selectedCode != prefCode) {
      Serial.print("Transmitting CTCSS tone: ");
      Serial.println(ctcssFrequencies[selectedCode]);
    }
  } else {
    // PTT button is not pressed - start decode incoming audio
    if (selectedCode != 0) {
      bool isDecoded = decodeCTCSS(ctcssFrequencies[selectedCode - 1]);
      digitalWrite(DECODE_INDICATOR, isDecoded ? HIGH : LOW);  // Turn on/off decode indicator
    } else {
      digitalWrite(DECODE_INDICATOR, LOW);  // Turn off decode indicator if no code is selected
    }
  }
}

// SPI interrupt service routine
ISR(SPI_STC_vect) {
  // Read the received data from the SPI Data Register (SPDR)
  shiftRegister = SPDR;

  // Print the received data to the Serial Monitor for debugging (optional)
  //Serial.print("Received Data: ");
  //Serial.println(shiftRegister, DEC);
}

int readShiftRegister() {
  if (digitalRead(latchPin) == LOW) {
    for (int i =0; i < numCodes; i++) {
      if (shiftRegister == ctcssCodes[i]) {
        code = i;
        //Serial.print("Code: ");
        //Serial.println(code, DEC);
        break;
      }
    }
  }
  return code;  // Returns a frequentie value between 0 and 250.3 Hz
}

void generateSineWave(int toneout) {
  CtcssTone.tone_on(toneout);
}


bool decodeCTCSS(float targetFrequency) {
  // Sample the audio signal
  for (int i = 0; i < SAMPLES; i++) {
    microSeconds = micros();    // Returns the number of microseconds since the Arduino board began running the current script

    vReal[i] = analogRead(A0);  // Read the audio signal from analog pin A0
    vImag[i] = 0;               // Imaginary part must be zeroed for FFT

    while (micros() < (microSeconds + samplingPeriod)) {
      // Wait for the next sample
    }
  }

  // Perform FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Find the peak frequency
  double peakFrequency = FFT.majorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  // Check if the peak frequency matches the target frequency (within a small margin of error)
  if (abs((peakFrequency - 1) - targetFrequency) < 1) {
    //Serial.print("CTCSS tone detected: ");
    //Serial.println(targetFrequency);
    return true;
  } else {
    //Serial.println("No CTCSS tone detected.");
    return false;
  }
}