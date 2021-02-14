# HL2-PA70
70W Power Amplifier for the Hermes-Lite 2.x

This is a work in progress to create a 70W power amplifier to be used together with the Hermes-Lite 2.x using the commercial DIY Kit 70W SSB linear HF Power Amplifier, a Low Pass Filter, and an Arduino board, to control it all.

The amplifier is based on an "70W SSB Linear HF Power Amplifier" you can find on eBay, Banggood, AliExpress, etc. This is the schematic:

![70W SSB Linear HF Power Amplifier](https://github.com/ea3igt/HL2-PA70/blob/main/70W%20SSB%20Amplifier%20circuit.png?raw=true)

I changed the two included IRF640 Single Mosfets, for one MRF9120 (Double RF Power Mosfet) that fits perfectly in the PCB and is more reliable because the heat transference is more convenient due to the form factor (the PCB accept both formats, the TO-220 package for the IRF640 ant the NI-860 package for the MRF-9120)

![MRF9120 Package](https://github.com/ea3igt/HL2-PA70/blob/main/MRF9120.JPG?raw=true)

This 70W SSB linear HF Power Amplifier needs a Low Pass Filter (LPF) to be compliant removing the harmonics (in fact there is an Input-Output connector in the PCB to use an external LPF) and I choose a very simple "HF low pass LPF 3.5M-30Mhz for Ham Radio CW FM" that fits perfectly with the power amplifier and does its job. This LPF has 4 bandpass filters for 80m, 40m, 20-17m and 15-10m, selectable connecting the specific pin to ground.

I wanted to use this Power Amplifier for my Hermes Lite 2 (HL2) and I wanted a fully automatic operation, so I decided to use an Arduino Nano to control it all, reading the I2C bus from the HL2 to select the band, and using the EXTTR signal to activate the amplifier. This is the complete block diagram of the aemplifier:

![PA70 Block Diagram](https://github.com/ea3igt/HL2-PA70/blob/main/PA70%20Block%20Diagram%20v2.1.2.JPG?raw=true)
