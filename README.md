# MarlinOctoHat
 
How to get rid of the Reprap Discount Full Graphics Smart Controller, and display Marlin on the OctoPi LCD.

The easiest and most efficient way to fetch the Marlin UI frames is to enable the MKS_12864OLED display in Configuration.h.

The MKS OLED 1.3" 128×64 Full Graphics Controller is based on the SSD1306, a very common OLED module. This display basically is a SPI slave. But the RaspBerry Foundation didn't think or didn't want to enable/support the SPI slave mode...

The Marlin graphics driver (a optimized u8glib fork) sends bitmaps @1MBbs. A STM32 bluepill reads these frames, transcodes them, and makes them available to the RaspBerry Pi SPI master interface.

The RaspBerry runs a service (see xxxxxxx), and displays the Marlin UI on the RaspBerry LCD.

This repo contains everything needed to build a hat (two PCBs and a firmware) that replaces the 12864 display. This hat is connected to the printer motherboard through the EXP1 and EXP2 connectors, and breaks out the rotary encoder, the reset button for the STM32 and the motherboard, the buzzer, a button for displaying Marlin, a 5V screw terminal for the RassPi and the STM32 power. It also breaks out the RasPi UART for a direct connection to the motherboard (if supported...).

The two PCBs are inexpensive, easy to populate (THT components only), and require components that every tinkerer has by hand. The KiCad projects and the Gerber files are provided. Just order the PCBs from a manufacturer (JLCPCB, PCBWay, etc.)

For the RasPi softwares (service and OctoPrint plugin), see xxxxxxxx

**The SSD1306 (MKS OLED 1.3" 128×64 Full Graphics Controller) is not enabled for every motherboard, even if the option is in Configuration.h ; enabling support is not difficult, but requires sone basic knowledge ! The file to be edited is somwhrtr uder /src/HAL : some #define to add. The motherboard schematics and pinouts could be required**



