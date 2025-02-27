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
Clock period (Fig1) = 4.4 uSec<BR>
Data Pulse (Fig2) = 2.2 uSec.<BR>
Latch after last clock pulse (Fig3) = 14.8 uSec.<BR>
Latch time (Fig4) = 3.8 µSec.<BR>
Latch to next bust of always 192 (Fig5) = 22.2 uSec.<BR>
### Fig1
![Alt Fig1] (https://github.com/PA2ER/Icom-CTCSS/tree/main/pics/CTCSS Clock period1.png)
### Fig2
![Alt Fig2] (https://github.com/PA2ER/Icom-CTCSS/tree/main/pics/CTCSS Data pulse.png)
### Fig3
![Alt Fig3] (https://github.com/PA2ER/Icom-CTCSS/tree/main/pics/CTCSS Latch after last clock.png)
### Fig4
![Alt Fig4] (https://github.com/PA2ER/Icom-CTCSS/tree/main/pics/CTCSS latch time.png)
### Fig5
![Alt Fig5] (https://github.com/PA2ER/Icom-CTCSS/tree/main/pics/CTCSS latch to next burst.png)