# HL2-PA70

## 70W Power Amplifier for the Hermes-Lite 2.x

This is a work in progress to create a 70W power amplifier to be used together with the Hermes-Lite 2.x using the commercial DIY Kit 70W SSB linear HF Power Amplifier, a Low Pass Filter, and an Arduino board, to control it all.

The amplifier is based on an "70W SSB Linear HF Power Amplifier" you can find on eBay, Banggood, AliExpress, etc. This is the schematic:

![70W SSB Linear HF Power Amplifier](https://github.com/ea3igt/HL2-PA70/blob/main/70W%20SSB%20Amplifier%20circuit.png?raw=true)


I did some modifications on the original board to adapt to the new transistor and my requirements:

1. I changed the two included IRF640 Single Mosfets, for one MRF-9120 (Double RF Power Mosfet) that fits perfectly in the PCB and is more reliable because the heat transference is more convenient due to the form factor (the PCB accept both formats, the TO-220 package for the IRF640 ant the NI-860 package for the [MRF-9120](https://www.nxp.com/docs/en/data-sheet/MRF9120.pdf))
2. I changed the R7 to a lower value (100 Ohm) to get the proper Bias level (~4.2 volt)
3. I added the R6 (15 Ohm) for better input SWR adaptation (modification suggested by OE1CGS)

This 70W SSB linear HF Power Amplifier needs a Low Pass Filter (LPF) to be compliant, removing the harmonics (there is an Input-Output connector in the PCB to use an external LPF) and I choose a very simple "HF low pass LPF 3.5M-30Mhz for Ham Radio CW FM" that fits perfectly with the power amplifier and does its job. This LPF has 4 bandpass filters for 80m, 40m, 20-17m and 15-10m, selectable connecting the specific pin to ground.

I wanted to use this Power Amplifier for my Hermes Lite 2 (HL2) and I wanted a fully automatic operation, so I decided to use an Arduino Nano to control it all:

1. Reading the I2C bus from the HL2 to decode the band
2. Selecting the appropiate filter band on the LPF board
3. Using the EXTTR signal to activate the amplifier (PPT)
4. Sensing the temperatura of the tgransistor using an NTC
5. Drive the speed of the fan using Pulse-width modulation (PWM) depending on the transistor temperature
6. And display Power Amplifier basic information using an OLED mini-screen

This is the complete block diagram of the amplifier:

![PA70 Block Diagram](https://github.com/ea3igt/HL2-PA70/blob/main/PA70%20Block%20Diagram%20v2.1.2.JPG?raw=true)

And this is the schematic of the first version of the Control Board:

![PA70 Control Board Schematic](https://github.com/ea3igt/HL2-PA70/blob/main/Control%20Board%20v2.1.1.JPG?raw=true)
