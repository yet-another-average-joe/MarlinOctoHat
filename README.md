# MarlinOctoHat
 
How to get rid of the Reprap Discount Full Graphics Smart Controller, and display Marlin on the OctoPi LCD.

The easiest and most efficient way to fetch the Marlin UI frames is to enable the MKS_12864OLED display in Configuration.h.

The MKS OLED 1.3" 128x64 Full Graphics Controller is based on the SSD1306, a very common OLED module. This display basically is an SPI slave. But the RaspBerry Foundation didn't think or didn't want to enable/support the SPI slave mode, which is required for emulation....

The Marlin graphics driver (a optimized u8glib fork) sends bitmaps @1Mbps. The STM32 BluePill gets these frames, reencodes them, and makes them available to the RaspBerry Pi SPI master interface. When a new frame is ready, the STM32 sends a pulse to the RasPi. The RasPi then reads the STM32 at up to 28Mbps (adjustable in the OctoPrint plugin).

As a result, the printer motherboard "sees" a SSD1306 SPI slave, and the RasPi "sees" the printer motherboard as a SPI slave peripheral ! Thanks to the STM32F103 dual SPI, and to the developpers that implemented the SPI DMA mode in the Roger's core !

The RaspBerry runs a service that overlays the Marlin UI onto the RaspBerry display using dispmanx, and an OctoPrint plugin allows for easy display customization.

This repo contains everything needed to build the hat (two PCBs and a firmware) that replaces the 12864 display. The hat is connected to the printer motherboard through the EXP1 and EXP2 connectors, and breaks out the rotary encoder, the reset button for the STM32 and the motherboard, the speaker, the "Marlin Mode" button, and a 5V screw terminal to for the RasPi and the STM32 power. It also breaks out the RasPi UART for direct connection to the motherboard (if supported...). Both 3.3V and 5V printer motherboards are supported.

The two PCBs are inexpensive, easy to populate using a decent soldering iron, and require components that every tinkerer has at hand (through hole only) : a BluePill (legit or fake), a couple of NPN tranistors and resistors, a small diode, 4 jumpers and some pinheaders and sockets (including a long RaspBerry 2x20 PC104-style pin header/socket). Everything can be easily sourced from Amazon, eBay, Ali...

The KiCad projects and the Gerber files are provided. Just order the PCBs from a manufacturer (JLCPCB, PCBWay, etc.)

Hat firmware : see https://github.com/yet-another-average-joe/MarlinOctoHat/tree/main/MarlinOctoHat_Firmware

RasPi software (service and OctoPrint plugin) :

service : https://github.com/yet-another-average-joe/OctoPrint_MarlinOSD_Service

plugin : https://github.com/yet-another-average-joe/OctoPrint_MarlinOSD_Plugin

**The SSD1306 (MKS OLED 1.3" 128x64 Full Graphics Controller) is not supported for all motherboards, even though the option is available in Configuration.h ; enabling support is not difficult, but requires sone basic knowledge about Marlin and microcontrollers ! The pins files are somewhere under Marlin/src/pins : some "#define..." to add if needed. The motherboard schematics and pinouts might be needed ; forums and dicussion groups could be helpfull !**

First, enable the "MKS OLED 1.3" 128x64 Full Graphics Controller" in Configurattion.h :

#define MKS_12864OLED

If the OLED is not supported, you will get a compile error similar to :

In file included from Marlin\src\lcd\dogm\marlinui_DOGM.cpp:42:

Marlin\src\lcd\dogm\marlinui_DOGM.cpp: In static member function 'static void MarlinUI::init_lcd()':

Marlin\src\lcd\dogm\marlinui_DOGM.h:223:48: error: 'DOGLCD_CS' was not declared in this scope; did you mean 'DOGLCD_SCK'?

You'll have to add such a block in your pins definition file :

      #elif ENABLED(MKS_12864OLED_SSD1306)
        #define DOGLCD_CS            EXP1_05_PIN
        #define DOGLCD_A0            EXP1_04_PIN
        #define DOGLCD_SCK           EXP2_09_PIN
        #define DOGLCD_MOSI          EXP2_05_PIN
        #define LCD_PINS_DC          LCD_PINS_D4
        //#define FORCE_SOFT_SPI                      // Use this if default of hardware SPI causes display problems
      #endif

You could have to figure wich microcontroller pins go to the EXP1 and EXP2 pin headers. May be done without the schematics, but easier with.

**THIS IS AN EXAMPLE !** : lines I had to add to /Marlin/src/pins/lpc1768/pins_BTT_SKR_V1_4.h for a SKR 1.4 Turbo.


![](https://github.com/yet-another-average-joe/MarlinOctoHat/blob/main/_pictures/DSC_8491.JPG)

![](https://github.com/yet-another-average-joe/MarlinOctoHat/blob/main/_pictures/DSC_8492.JPG)

![](https://github.com/yet-another-average-joe/MarlinOctoHat/blob/main/_pictures/DSC_8485.JPG)

![](https://github.com/yet-another-average-joe/MarlinOctoHat/blob/main/_pictures/DSC_8486.JPG)

![](https://github.com/yet-another-average-joe/MarlinOctoHat/blob/main/_pictures/DSC_8487.JPG)
