# arduino_wow_and_flutter_meter
Arduino-based wow-and-flutter meter for testing tape recorders per DIN 45507  / IEC-386 / GOST 11948-78 and historical Japanese WRMS metric. 
Requires 8-bit 16-MHz AVR board (memory sizes aren't important but clock speed is critical). Tested with genuine Arduino Duemilanove, 
noname code of Sparkfun Nano and Amperka-Iskra Nano (another clone of Arduino Nano). Output uses  generic 4x20 alphanumeric LCD with 
HD44780 controller and 4-bit parallel data bus.

The Arduino's 16-bit Timer1, configured in input-capture mode, watches the rising edges of incoming audio frequency (3,150 Hz standard). 
It skips the first seven periods of the pack; after eight complete periods, it performs calculations starting with the value of ICP1 register.
The computing power of Arduino is more than enough for the required simple DSP task; the main limiting factors are the clock speed and the
16-bit resolution of the available timers. A single 3,150 Hz pulse equals 5,080 clock cycles; if processed at this rate, resolution would be
an unacceptably coarse 0.04%. Thus, the timer count window is extended over 8 input cycles, or around 40,635 system clock cycles, or around 2.5 ms.
This improves resolution to less than 0.01% - a bare minimum for the job. A further increase in sampling window length would compromise the 
high-frequency response which, according to DIN45507, must extend to at least 300 Hz.

The test frequency need not be precise; the acceptable range is limited to 2100-4725 Hz (toofast_pack_clocks=27090, tooslow_pack_clocks=60952). 
Note, however, that the filter frequencies and time constants are not fixed - they are tied to the test frequency. If it deviates considerably
from the reference 3150 Hz, the filter curves would move up or down from the standard curves. I originally planned to add real-time adjustment of
filter constants to test frequency, so that the curves would stay more or less fixed, but it's just not needed in real use. 

8-bit Timer2 is configured to output ca. 3,150 Hz square-wave test signal (as-built, it is filtered and trimmed to be more like trapezoid). Actual
frequency, due to 8-bit resolution, is more like 3,130 Hz.

8-bit Timer0 is configured as an 8 kHz PWM and is currently unused. It was reserved for driving an external analogue voltmeter.

* photographs in operaton - https://commons.wikimedia.org/wiki/Category:Arduino-based_wow-and-flutter_meter
* schematic - WFmeter schematic.png - nothing special, really. A plain input amp/limiter loaded into a plain amp/Schmitt-trigger loaded into the
arduino's analogue pin via a zener safety clamp. Two diodes, D4 and D3, raise the "ground" of the first and the second opamp stages respectively.
The choice of opamp (here, LM358) is unimportant as long as it can reliably work in low-voltage environment and its common-mode input voltage 
extends down to its negative rail.
* DSP flowchart - WFmeter schematic.png - again, nothing special, a simple approximation of DIN 45507 curves. Note that there's another filter at
the input, not shown here - a rectangular-window sampling FIR with sampling frequency of around 400 Hz (2.5 ms period).
