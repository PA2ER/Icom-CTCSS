/*
With this code, I try to attempted to program a Arduino nano, to use in older transceivers, where the CTCCC board is no longer available.
The goal is to let the Arduino communicate with the transceiver, as it would with a factory CTCSS board.
I found out that Icom, yaesu, and Kenwood, all use a 8 bit shift register IC, and a CTCSS encoder/decoder IC like the FX365, that are obsolete.

I used the library written by PE1CID,that I slightly modified, to generate the sub-audio tone. (CtcssTone)

Pins used:
PIN 4 = PTT_BUTTON to switch from receive to transmit
PIN 3 = PWM_OUTPUT output that generates the sub-audio tone
PIN 8 = DECODE_INDICATOR goes high if CTCSS code is decoded
PIN 2 = ChipSelect Input for TSTB from the transceiver
PIN 11 = MOSI SPI DATA from transceiver
PIN 13 = SCK SPI clock serial clock for CK from transceiver
PIN 14 = A0 Sub-audio tone input, needs 260Hz lowpass filter

selectedCode:
Bit7 = PTT_on when low
Bit6 = Tone_off when high
Bit5-0 = CTCSS code
*/

#include <Arduino.h>
#include <SPI.h>
#include <arduinoFFT.h>
#include "CtcssToneId.h"
#include "CtcssTone.h"

// Declare funtions:
void readShiftRegister();
void generateSineWave(int frequency);
bool decodeCTCSS(float targetFrequency);

// PTT Button
#define PTT_BUTTON 4  // Pin connected to the PTT button
#define CS_PIN    2  // D10 = pin16 = PortB.4
#define SCK_PIN   13  // D13 = pin19 = PortB.5
#define MOSI_PIN  11  // D11 = pin17 = PortB.3

// Decode Indicator Output
#define DECODE_INDICATOR 8  // Pin for decode indicator (e.g., LED)

// PWM Output for Sine Wave
#define PWM_OUTPUT 3  // Pin for PWM output (must be PWM-capable)

// TIA-603-E CTCSS Frequencies (Hz)
const float ctcssFrequencies[] = {
  00.0, 67.0, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 100.0, 103.5, 107.2, 110.9, 114.8
  , 118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8
  , 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3
};

const uint8_t ctcssCodes[] = {
  0, 29, 28, 27, 26, 25, 24, 23, 22, 21, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42
  , 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30
};

/*const uint8_t ctcssCodes[] = {
  192, 157, 156, 155, 154, 153, 152, 151, 150, 149, 185, 184, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169
  , 168, 167, 166, 165, 164, 163, 162, 161, 160, 159, 158
};
*/

// Initialize all bits to 0
volatile byte shiftRegister = 0b00000000;

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

// Variables for state tracking
bool isWaitingForCSHigh = true;
bool isWaitingForCSLow = false;
bool isReadingData = false;

void setup() {
  // Set SPI pins as input
  pinMode(CS_PIN, INPUT);
  pinMode(SS, INPUT);
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);

  // Initialize SPI in slave mode
  SPCR = (1<<SPE)|(0<<DORD)|(0<<MSTR)|(1<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0); // SPI on
  SPCR |= _BV(SPIE); // Enable SPI interrupt

  // Attach an interrupt to the clock pin (rising edge)
  attachInterrupt(digitalPinToInterrupt(CS_PIN), readShiftRegister, RISING);

  // Initialize Timer 1 for PWM output
  TCCR1A = _BV(COM1A1) | _BV(WGM10);  // Set PWM mode and output
  TCCR1B = _BV(CS12);  // Set prescaler to 1 (no prescaling)

  // Initialize Serial Monitor for debugging (optional)
  Serial.begin(115200);

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
  // Check if the PTT button is pressed
  if (digitalRead(PTT_BUTTON) == HIGH) {  // PTT button is pressed - transmit selected CTCSS tone as a sine wave
    digitalWrite(DECODE_INDICATOR, LOW);  // Turn off decode indicator
    if (selectedCode == 0) {
      CtcssTone.tone_off();
      Serial.println("Tone is off");
    } else {
      generateSineWave(selectedCode);
      if (selectedCode != prefCode) {
        Serial.print("Transmitting CTCSS tone: ");
        Serial.println(ctcssFrequencies[selectedCode]);
      }
    } 
  } else {
    // PTT button is not pressed - start decode incoming audio
    if (selectedCode != 0) {
      bool isDecoded = decodeCTCSS(ctcssFrequencies[selectedCode]);
      digitalWrite(DECODE_INDICATOR, isDecoded ? HIGH : LOW);  // Turn on/off decode indicator
    } else {
      digitalWrite(DECODE_INDICATOR, LOW);  // Turn off decode indicator if no code is selected
    }
  }
  //currentState = WAIT_FOR_CS_HIGH;
}

// SPI interrupt service routine
ISR(SPI_STC_vect) {
  // Read the received data from the SPI Data Register (SPDR)
  shiftRegister = SPDR;
}

void readShiftRegister() {
  if (bitRead(shiftRegister, 7) == 0) {
    Serial.println("PTT Button pressed");
  }
  if (bitRead(shiftRegister, 6) == 0) {
    Serial.println("Tone on");
  }
  
  int code = 0;
  for (int i =0; i < numCodes; i++) {
    if ((shiftRegister & 0b111111) == ctcssCodes[i]) {
      code = i;
      //Serial.print("Code: ");
      //Serial.println(code, DEC);
      break;
    }
  }
  selectedCode = code;
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