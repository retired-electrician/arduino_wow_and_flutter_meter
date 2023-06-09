# arduino_wow_and_flutter_meter
Arduino-based wow-and-flutter meter for testing tape recorders per DIN 45507  / GOST 11948-78 and historical Japanese WRMS metric
Requires 8-bit 16-MHz AVR board (memory sizes aren't important but clock speed is critical)
Tested with genuine Arduino Duemilanove, noname code of Sparkfun Nano and Amperka-Iskra Nano (another clone of Arduino Nano). Output uses
a generic 4x20 alphanumeric LCD with HD44780 controller and 4-bit parallel data bus.

The Arduino's 16-bit Timer1, configured in input-capture mode, watches the rising edges of incoming audio frequency (3,150 Hz standard). 
It skips the first seven periods of the pack; after eight complete periods, it performs calculations starting with the value of ICP1 register.
The computing power of Arduino is more than enough for the required simple DSP task; the main limiting factors are the clock speed and the
16-bit resolution of the available timers. A single 3,150 Hz pulse equals 5,080 clock cycles; if processed at this rate, resolution would be
an unacceptably coars 0.04%. Thus, the timer count window is extended over 8 input cycles, or around 40,635 system clock cycles, or around 2.5 ms
This improves resolution to less than 0.01% - a bare minimum for the job. A further increase in sampling window length would compromise the 
high-frequency response which, according to DIN45507, must extend to at least 300 Hz.
