# Arduino CTCSS (de)coder for Icom IC-275

## Description:
With this code, I try to attempted to program a Arduino nano, to use in older transceivers, where the CTCCC board is no longer available.<br>
The goal is to let the Arduino communicate with the transceiver, as it would with a factory CTCSS board.<br>
I found out that Icom, yaesu, and Kenwood, all use a 8 bit shift register IC, and a CTCSS encoder/decoder IC like the FX365, that are obsolete.</p>
## acknowledgment
I used the library written by PE1CID,that I slightly modified, to generate the sub-audio tone. (CtcssTone)</p>
## Pins used:<br>
PIN 2 = ChipSelect Input for TSTB from the transceiver<br>
PIN 3 = PWM_OUTPUT output that generates the sub-audio tone<br>
PIN 4 = PTT_INDICATOR output indicator with led -for testing purposes<br>
PIN 5 = TONE_OFF_INDICATOR output indicator with led for testing purposes<br>
PIN 8 = DECODE_INDICATOR goes high if CTCSS code is decoded<br>
PIN 11 = MOSI SPI DATA from transceiver<br>
PIN 13 = SCK SPI clock serial clock for CK from transceiver<br>
PIN 14 = A0 Sub-audio tone input, needs 260Hz lowpass filter<p>
## Serial data from transceiver
Clock period (Fig1) = 4.4 uSec<br>
Data Pulse (Fig2) = 2.2 uSec.<br>
Latch after last clock pulse (Fig3) = 14.8 uSec.<br>
Latch time (Fig4) = 3.8 µSec.<br>
Latch to next bust of always 192 (Fig5) = 22.2 uSec.<p>
### Fig 1
<img src="pics/CTCSS_Clock_period1.png" alt="Fig1"><p>

### Fig 2
<img src="pics/CTCSS_Data_pulse.png" alt="Fig2"><p>

### Fig 3
<img src="pics/CTCSS_Latch_after_last_clock.png" alt="Fig3"><p>

### Fig 4
<img src="pics/CTCSS_latch_time.png" alt="Fig4"><p>

### Fig 5
<img src="pics/CTCSS_latch_to_next_burst.png" alt="Fig5"><p>