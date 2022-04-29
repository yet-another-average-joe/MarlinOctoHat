# MarlinOctoHat
 
How to get rid of the Reprap Discount Full Graphics Smart Controller, and display Marlin on the OctoPi LCD.

The easiest and most efficient way to fetch the Marlin UI frames is to enable the MKS_12864OLED display in Configuration.h.

The MKS OLED 1.3" 128×64 Full Graphics Controller is based on the SSD1306, a very common OLED module. This display basically is an SPI slave. But the RaspBerry Foundation didn't think or didn't want to enable/support the SPI slave mode...

The Marlin graphics driver (a optimized u8glib fork) sends bitmaps @1Mbbs. A STM32 bluepill reads these frames, reencodes them, and makes them available to the RaspBerry Pi SPI master interface.

The RaspBerry runs a service (see xxxxxxx), and displays the Marlin UI on the RaspBerry LCD.

This repo contains everything needed to build a hat (two PCBs and a firmware) that replaces the 12864 display. This hat is connected to the printer motherboard through the EXP1 and EXP2 connectors, and breaks out the rotary encoder, the reset button for the STM32 and the motherboard, the buzzer, a "Marlin Mode" button, a 5V screw terminal to power RassPi and the STM32. It also breaks out the RasPi UART for direct connection to the motherboard (if supported...).

The two PCBs are inexpensive, easy to populate (THT components only), and require components that every tinkerer has at hand. The KiCad projects and the Gerber files are provided. Just order the PCBs from a manufacturer (JLCPCB, PCBWay, etc.)

For the RasPi softwares (service and OctoPrint plugin), see xxxxxxxx

**The SSD1306 (MKS OLED 1.3" 128×64 Full Graphics Controller) is not enabled for every motherboard, even if the option is available in Configuration.h ("#define MKS_12864OLED_SSD1306") ; enabling support is not difficult, but requires sone basic knowledge ! The file to be edited is somwhere under /src/HAL : some "#define..." to add. The motherboard schematics and pinouts could be required ; forums and sicussion groups could be helpfull !**



