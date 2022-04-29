# MarlinOctoHat
 
How to get rid of the Reprap Discount Full Graphics Smart Controller, and display Marlin on the OctoPi LCD. The easiest and most efficient way to fetch the Marlin UI frames is to enable the MKS_12864OLED display in Configuration.h. This display basically is a SPI slave. But the RaspBerry Fundation didn't think to enable/support the SPI slave mode...

The MKS OLED 1.3" 128×64 Full Graphics Controller is based on the SSD1306, a very common OLED module. The protocol basiacally is SPI, the printer being the master, and the SSD1306 being the slave. The Marlin graphics driver (a optimized u8glib fork) sends bitmaps @1MBbs. A STM32 bluepill reads these frames, transcodes them, and make them available to the RaspBerry Pi SPI master interface.

The RaspBerry runs a service (se xxxxxxx), and displays the Marlin UI on the RaspBerry LCD.

This repo contains evrything needed to build a shield (two PCBs and a firmware) that replaces the 12864 display.

The RaspBerry Fundation didn't think or didn't want to enable/support the SPI slave mode...

**All 3D models should align properly with the KiCad and FreeCad coordinates systems. It wasn't the case with the previous version. The problem is now solved.**
