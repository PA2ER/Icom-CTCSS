/*
With this code, I try to attempted to program a Arduino nano, to use in older transceivers, where the CTCCC board is no longer available.
The goal is to let the Arduino communicate with the transceiver, as it would with a factory CTCSS board.
I found out that Icom, yaesu, and Kenwood, all use a 8 bit shift register IC, and a CTCSS encoder/decoder IC like the FX365, that are obsolete.

I used the library written by PE1CID,that I slightly modified, to generate the sub-audio tone. (CtcssTone)

Pins used:
PIN 2 = ChipSelect Input for TSTB from the transceiver
PIN 3 = PWM_OUTPUT output that generates the sub-audio tone
PIN 4 = PTT_INDICATOR output indicator with led -for testing purposes
PIN 5 = TONE_OFF_INDICATOR output indicator with led for testing purposes
PIN 8 = DECODE_INDICATOR goes high if CTCSS code is decoded
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

#define PTT_INDICATOR 4  // D4 Pin connected to the PTT Led--------for testing purposes
#define TONE_OFF_INDICATOR 5  // D5 Pin connected to the Tone Off Led--------for testing purposes
#define CS_PIN    2  // D2 Pin connected to the Chip Select (TSTB) pin of the transceiver
#define SCK_PIN   13  // D13 Pin connected to the Serial Clock (CK) pin of the transceiver
#define MOSI_PIN  11  // D11 Pin connected to the Master Out Slave In (DATA) pin of the transceiver

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

bool pttOn = false;
bool toneOff = false;

void setup() {
  // Set SPI pins as input
  pinMode(CS_PIN, INPUT); //D2 Chip Select (TSTB) from transceiver
  pinMode(SS, INPUT); //not used
  pinMode(SCK_PIN, INPUT); //D13 Serial Clock (CK) from transceiver
  pinMode(MOSI_PIN, INPUT); //D11 Master Out Slave In (DATA) from transceiver

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

  // Initialize PTT indicator as output
  pinMode(PTT_INDICATOR, OUTPUT);

  // Initialize Tone Off indicator as output
  pinMode(TONE_OFF_INDICATOR, OUTPUT);

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
  if (pttOn) {  // PTT button is pressed - transmit selected CTCSS tone as a sine wave
    digitalWrite(DECODE_INDICATOR, LOW);  // Turn off decode indicator
    digitalWrite(PTT_INDICATOR, HIGH);  // Turn on PTT indicator
    if (!toneOff) {
      CtcssTone.tone_off();
      digitalWrite(TONE_OFF_INDICATOR, HIGH);  // Turn on tone off indicator
      Serial.println("Tone is off");
    } else {
      generateSineWave(selectedCode);
      if (selectedCode != prefCode) {
        digitalWrite(TONE_OFF_INDICATOR, LOW);  // Turn off tone off indicator
        Serial.print("Transmitting CTCSS tone: ");
        Serial.println(ctcssFrequencies[selectedCode]);
      }
    } 
  } else {
    // PTT button is not pressed - start decode incoming audio
    digitalWrite(PTT_INDICATOR, LOW);  // Turn off PTT indicator
    if (!toneOff) {
      bool isDecoded = decodeCTCSS(ctcssFrequencies[selectedCode]);
      digitalWrite(TONE_OFF_INDICATOR, LOW);  // Turn off tone off indicator
      digitalWrite(DECODE_INDICATOR, isDecoded ? HIGH : LOW);  // Turn on/off decode indicator
    } else {
      digitalWrite(TONE_OFF_INDICATOR, HIGH);  // Turn on tone off indicator
      digitalWrite(DECODE_INDICATOR, LOW);  // Turn off decode indicator if no code is selected
    }
  }
  //currentState = WAIT_FOR_CS_HIGH;
}

// SPI interrupt service routine, reads 8 bit data from the shift register store in shiftRegister
ISR(SPI_STC_vect) {
  // Read the received data from the SPI Data Register (SPDR)
  shiftRegister = SPDR;
}

// Read the shift register and update the selected CTCSS code, and PTT and tone status
void readShiftRegister() {
  if (bitRead(shiftRegister, 7) == 0) { //if PTT button is pressed switch boolean pttOn
    pttOn = true;
    Serial.println("PTT Button pressed");
  }
  else {
    pttOn = false;
  }
  if (bitRead(shiftRegister, 6) == 0) { //if T-Sql is off switch boolean toneOff
    toneOff = false;
    Serial.println("Tone on");
    CtcssTone.prev_tone_on();
  }
  else {
    toneOff = true;
    Serial.println("Tone off");
    CtcssTone.tone_off();
  }
  int code = 0;
  for (int i =0; i < numCodes; i++) {
    if ((shiftRegister & 0b111111) == ctcssCodes[i]) {
      code = i;
      Serial.print("Code: ");
      Serial.println(shiftRegister & 0b111111, DEC);
      break;
    }
  }
  selectedCode = code;
}

// Generate a sine wave with the specified frequency, with arrays ctcssCodes and ctcssFrequencies
// toneout is the code send by transceiver
void generateSineWave(int toneout) {
  CtcssTone.tone_on(toneout);
}

// Decode the incoming audio signal and check if it matches the target CTCSS frequency
// targetFrequency is the CTCSS frequency to decode
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