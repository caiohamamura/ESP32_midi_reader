# ESP32 MIDI Parser

This is a MIDI parser project, it contains:

1. A MIDI parser from LittleFS MIDI.
1. A timer set to fire ~16KHz sampling rate.
1. The timer will then update the `dac_output_voltage` of GPIO25 in ESP32.
1. The sample wave is calculated differently based on instrument and drums is only random noise.