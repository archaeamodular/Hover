# Hover
Hover beam controller firmware. 

Copyright Archaea Modular Synthesis 2022.

Requirements:
- avr-gcc AVR C compiler installed.
- avrdude AVR programmer installed.
- AVR ISP programmer with 6-pin SPI cable.

Build commands using the Makefile:
- make            : Builds and downloads the firmware.
- make clean      : Clean the build directory. Use this after changing Makefile build options!
- make reset      : Reset the Hover connected to the programmer.

IMPORTANT: if you want to use the Makefile to download the firmware you must configure your programmer type in the Makefile before building!
