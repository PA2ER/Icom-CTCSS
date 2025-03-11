# Arduino CTCSS (de)coder for Icom IC-275

## Description:
With this code, I try to attempted to program a Arduino nano, to use in older transceivers, where the CTCCC board is no longer available.<br>
The goal is to let the Arduino communicate with the transceiver, as it would with a factory CTCSS board.<br>
I found out that Icom, yaesu, and Kenwood, all use a 8 bit shift register IC, and a CTCSS encoder/decoder IC like the FX365, that are obsolete.</p>
## acknowledgment
I used the library written by PE1CID,that I slightly modified, to generate the sub-audio tone. (CtcssTone)</p>
## Pins used:<br>
PIN 2 = ChipSelect Input for TSTB (P47-3) from the transceiver<br>
PIN 3 = PWM_OUTPUT output that generates the sub-audio tone, via filter network to TSTN (P46-5) to the transceiver<br>
PIN 4 = PTT_INDICATOR output indicator with led -for testing purposes<br>
PIN 5 = TONE_OFF_INDICATOR output indicator with led for testing purposes<br>
PIN 8 = DECODE_INDICATOR goes high if CTCSS code is decoded<br>
PIN 11 = MOSI SPI DATA (P47-4) from transceiver<br>
PIN 13 = SCK SPI clock serial clock for CK (P47-5) from transceiver<br>
PIN 14 = A0 Sub-audio tone input, needs 260Hz lowpass filter<p>
## Transceiver conections:
P47-1 = 5V<br>
P47-2 = GND<br>
P47-3 = TSTB (Chip Select) to D2<br>
P47-4 = DATA (MOSI) to D11<br>
P47-5 = CK (SCK) to D13<p>

P46-1 = TSFL Squelch control<br>
P46-2 = AFMT AF mute<br>
P46-3 = TSAO Receive OUTPUT<br>
P46-4 = GND<br>
P46-5 = TSTN Transmit Input to D3 via filter network<br>
P46-6 = GND<p>
## Serial data from transceiver
Channel 1 = Yellow = Data<br>
Channel 2 = Purple = Clock<br>
Channel 3 = Blue = ChipSelect (CS)<p>
Clock period (Fig1) = 4.4 uSec<br>
Data Pulse (Fig2) = 2.2 uSec.<br>
Latch after last clock pulse (Fig3) = 14.8 uSec.<br>
Latch time (Fig4) = 3.8 µSec.<br>
Latch to next bust of always 192 (Fig5) = 22.2 uSec.<p>
### Clock period (Fig1) = 4.4 uSec<br>
<img src="pics/CTCSS_Clock_period1.png" alt="Fig1"><p>

### Data Pulse (Fig2) = 2.2 uSec.<br>
<img src="pics/CTCSS_Data_pulse.png" alt="Fig2"><p>

### Latch after last clock pulse (Fig3) = 14.8 uSec.<br>
<img src="pics/CTCSS_Latch_after_last_clock.png" alt="Fig3"><p>

### Latch time (Fig4) = 3.8 µSec.<br>
<img src="pics/CTCSS_latch_time.png" alt="Fig4"><p>

### Latch to next bust of always 192 (Fig5) = 22.2 uSec.<br>
<img src="pics/CTCSS_latch_to_next_burst.png" alt="Fig5"><p>
## Basic Schema<br>
<img src="pics/CTCSS_Schema.png" alt="Basic Schema"><p>
## UT34-Board<br>
<img src="pics/CTCSS_Board.png" alt="Basic Board"><p>

