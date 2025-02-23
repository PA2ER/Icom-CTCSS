<p class="has-line-data" data-line-start="0" data-line-end="3">With this code, I try to attempted to program a Arduino nano, to use in older transceivers, where the CTCCC board is no longer available.<br>
The goal is to let the Arduino communicate with the transceiver, as it would with a factory CTCSS board.<br>
I found out that Icom, yaesu, and Kenwood, all use a 8 bit shift register IC, and a CTCSS encoder/decoder IC like the FX365, that are obsolete.</p>
<p class="has-line-data" data-line-start="4" data-line-end="5">I used the library written by PE1CID,that I slightly modified, to generate the sub-audio tone. (CtcssTone)</p>
<p class="has-line-data" data-line-start="6" data-line-end="13">Pins used:<br>
PIN 2 = PTT_BUTTON to switch from receive to transmit<br>
PIN 3 = PWM_OUTPUT output that generates the sub-audio tone<br>
PIN 8 = DECODE_INDICATOR goes high if CTCSS code is decoded<br>
PIN 10 = latchPin Input of the latch from the transceiver<br>
PIN 11 = MOSI SPI data from transceiver<br>
PIN 13 = SCK SPI clock serial clock from transceiver<br>
PIN 14 = A0 Sub-audio tone input, needs 260Hz lowpass filter</p>