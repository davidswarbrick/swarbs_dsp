# swarbs DSP
This repository contains my DSP experiments with a [PÚCA DSP](https://www.ohmic.net/puca-dsp) board, based on the [ESP32](https://en.wikipedia.org/wiki/ESP32) RISC-V processor.

The code is based on v5.0 of the ESP-IDF from Espressif, documented [here](https://docs.espressif.com/projects/esp-idf/en/v5.0/esp32/index.html).

After the IDF is installed and in your path, building and flashing is simply `idf.py flash` in the root of this repository.

As of 2/1/23, I've implemented two example functions, a sine wave output (adapted from the PÚCA DSP examples but updated to v5.0 of the IDF) and passthrough from ADC -> DAC. I hope to implement a configurable delay and/or eventually a reverb, with the help of filters extracted from boilerplate code from the [Faust project](https://faust.grame.fr/).
