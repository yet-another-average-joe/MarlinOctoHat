/*
 Name:        MarlinOctoHat.ino
 Created:    31/08/2021 11:16:18
 Author:    Y@@J
    TODO : pins PB10/PB11 = input for emulator selection OLED SSD1306 / OLED SSD1309 / OLED SSH1106 / LCD ST7920
            DISPLAY_TYPE : PB1
    TODO : power management
            support for monostable relay throught PIN_RELAY_ON
    /////////////////////////////////////////////////////////////////////////////////////
    SPI Slave Sniffer for the SSD1306/SSD1309/SSH1106 OLED ; for marlin UI, u8glib
    tested Ok with SPI SSD1309 OLED and Marlin v2.0.x on genuine and fake STM32F103C6 ("BluePill")
    /////////////////////////////////////////////////////////////////////////////////////
    thread on stm32duino.com, and special thanks for the help about SPI DMA tranfers ! :
    https://www.stm32duino.com/viewtopic.php?t=825
    SSD1306 datasheet :
    https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
    SPI data :
    1 SPI frame = 8 pages
    1 page = 8lines/128 pixels, vertical macropixels
    1 page : 3 command bytes + 128 bytes (= 8 lines on the display)
    1 command byte = 0x10 0x00 0XBn , n = page # (0...7)
    total : 8 pages ; 8 * 131 = 1048 bytes, CLK @ 1MHz
    /////////////////////////////////////////////////////////////////////////////////////
    // OLED wirings
        SSD1306-9        SPI
        SH1106
        SCL                SCLK
        SDA                MOSI
        RES                N/A
        DC                N/A
        SS                SS
    SS : LOW for pages, including commands ; HIGH between pages and between frames
    DC : LOW for commands only
    // BluePill / RaspBerry Pi wirings
                                              ----------
                        (LCD_5)  [NSS2] PB12 |            | GND
                        (SCK)    [SCK2] PB13 |            | GND
                                        PB14 |            | 3V3
                        (MOSI)  [MOSI2] PB15 |            | RESET
                                         PA8 |            | PB11 -> GPIO2  (AUX4)
                                         PA9 |            | PB10 -> GPIO3  (AUX3)
                                        PA10 |            | PB1
                                        PA11 |            | PB0
                                        PA12 |            | PA7  [MOSI1] -> GPIO20 (SPI1 MOSI)
                                        PA15 |            | PA6  [MISO1] -> GPIO19 (SPI1 MISO)
                             (LCD_E)    PB3  |            | PA5  [SCK1]  -> GPIO21 (SPI1 SCLK)
                             (LCD_4)    PB4  |            | PA4  [NSS1]  -> GPIO16 (SPI1 CS0)
                             (LCD_7)    PB5  |            | PA3
        GPIO5  <- DATA_READY            PB6  |            | PA2  PIN_RELAY_OFF
        GPIO17 <- PIN_SOFT_SHUTDOWN     PB7  |            | PA1  PIN_RELAY_ON
        GPIO3  <- PIN_HARD_SHUTDOWN     PB8  |            | PA0  PIN_BTN_POWER
        GPIO2  <- PIN_POWEROFF          PB9  |            | PC15
                                         5V  |            | PC14
                                        GND  |            | PC13
                                        3V3  |            | VBAT
                                             |            |
                                              ----------
    LCD_E, LCD_4, LCD_7 : variant with ST7920 emulator (not implemented)
    SPI2 : SPI "input" (MOSI), from printer motherboard
    ---------------------------------------------------
        STM32 Pins : 5V tolerant
            MoBo        OLED    BluePill
            XP1_5        CS        PB12    PIN_SS_2
            XP2-9        SCL       PB13    PIN_SCK_2
                         N/C       PB14
            EXP2-5       SDA       PB15    PIN_MOSI_2
    SPI1 :    SPI "output" (MISO), to RasPi
    -------------------------------------
        STM32 Pins : /!\ NOT 5V TOLERANT /!\
            PIN_READY : high for READY_PULSE_DURATION ms when a new bitmap is available
                        tells the RasPi it's time to get data and refresh display
                        drastically reduces RasPi CPU usage
                    BluePill            PasPi
             PA7    PIN_MOSI1            [38]    GPIO20    MOSI1
            (PA6    PIN_MISO1            [35]    GPIO19    MISO1)
             PA5    PIN_SCK1             [40]    GPIO21    SCK1
             PA4    PIN_NSS1             [36]    GPIO16    NSS1
             PB6    PIN_READY            [29]    GPIO5     data ready to send
             PB7    PIN_SOFT_SHUTDOWN    [11]    GPIO17    output, default = LOW, HIGH = short press -> OctoPrint Shutdown
             PB8    PIN_HARD_SHUTDOWN    [5]     GPIO3     output, default = LOW, HIGH triggered by PIN_BTN_POWER long press -> OS Shutdown
             PB9    PIN_POWEROFF         [3]     GPIO2     input, interrupt
             PA0    PIN_BTN_POWER        [33]    GPIO13    input, pullup,    interrupt, sends 1 sec LOW pulse to PIN_HARD_SHUTDOWN
             PA1    PIN_RELAY_ON         [32]    GPIO12    output, default = LOW, HIGH for 1 sec at boot
             PA2    PIN_RELAY_OFF        [31]    GPIO6     output, default = LOW, HIGH triggerd by PIN_POWEROFF

*/
lkjhsqdgfklhjgbqskdhjgbksdqjhgdksjhkqgqsdkjfhgqsdf
#include <SPI.h>
//#include "Bitmaps.h"

//#define __SERIAL_DEBUG // costs 2328 bytes

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// pins : pullups ? pulldowns ?

// SPI_1 : input (MOSI), OLED sniffing ; /!\ *NOT* 5V tolerant
// SPI_2 : output (MISO), to RaspBerry Pi
// pins are set by the library

#define READY_PULSE_DURATION    1        // 1ms

#define PIN_READY        PB6

//#define PIN_HARD_SHUTDOWN    PB8        // to RasPi Shutdown pin (5 / GPIO3)
//#define PIN_POWEROFF         PB9        // from RasPi PowerOff pin (2 / GPIO2)
#define PIN_BTN_POWER        PA0        // Power button
#define PIN_RELAY_ON         PA1        // to PSU module ; ON bistable coil
#define PIN_RELAY_OFF        PA2        // to PSU module ; OFF bistable coil
#define PIN_SOFT_SHUTDOWN    PB7        // to RasPi (11 / GPIO17)

#define LONG_PRESS_TIME            1500    // milliseconds
#define DEBOUNCE_TIME                25    // milliseconds
#define HARD_SHUTDOWN_PULSE         500    // 100ms debounce by default on RasPi ; min pulse width 210ms (measured)
#define SOFT_SHUTDOWN_PULSE         500    // milliseconds
#define RELAY_PULSE                 500    // milliesconds ; Schrack RT314F12 : min 30 ms

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OLED    data

// data (SSD1306/SSD1309) : 1 frame = 8 pages, 1 page = 3 command bytes + 128 bytes, 1 page = 8 graphic lines
#define OLED_PAGE_COUNT          8
#define    OLED_LINES_PER_PAGE   8
#define OLED_CMD_SIZE            3
#define OLED_PAGE_SIZE           128
#define OLED_SPI_PAGE_SIZE       (OLED_CMD_SIZE + OLED_PAGE_SIZE)        // 1048
#define OLED_SPI_FRAME_SIZE      (OLED_PAGE_COUNT * OLED_SPI_PAGE_SIZE)  // 1024

// bmpOled with vertical 8bit monochrome macropixels...
volatile uint8_t bmpOled[OLED_PAGE_COUNT][OLED_PAGE_SIZE] = { 0 };

// converts bmpOled to horizontal 8bit monochrome macropixels
// will be used as Tx buffer for SPI_1
// allways keeps the last captured screen until a new one arrives
volatile uint8_t bmpOut[OLED_PAGE_COUNT * OLED_PAGE_SIZE] = { 0 };

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPI 1&2 : DMA channels

#define SPI_1_TX_DMA_CH        DMA_CH3
#define SPI_2_RX_DMA_CH        DMA_CH4

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPI_2  : MOSI (Rx) only

SPIClass SPI_2(2);

#define PIN_NSS_2    PB12

volatile uint8_t SPI_2_Rx_Buffer[OLED_SPI_FRAME_SIZE] = { 0 };
volatile bool SPI_2_DMA_transfer_complete = false;
volatile bool SPI_2_DMA_error = false;

// DMA tube configuration for SPI2 Rx
dma_tube_config SPI_2_DMA_RxTubeCfg =
{
    &SPI2->regs->DR,           // Source of data
    DMA_SIZE_8BITS,            // Source transfer size
    &SPI_2_Rx_Buffer,          // Destination of data
    DMA_SIZE_8BITS,            // Destination transfer size
    sizeof(SPI_2_Rx_Buffer),   // Number of bytes to receive
                               // Flags :
    DMA_CFG_DST_INC |          // auto increment destination address
    DMA_CFG_CIRC |             // circular buffer
    DMA_CFG_CMPLT_IE |         // set tube full IRQ
    DMA_CFG_ERR_IE |           // want messages in the DMA irq
    DMA_CCR_PL_VERY_HIGH,      // very high priority
    0,                         // reserved
    DMA_REQ_SRC_SPI2_RX        // Hardware DMA request source
};

void setup_SPI_2();
void setup_SPI_2_DMA();
void SPI_2_DMA_IRQ();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPI_1 : MISO (Tx) only

#define SPI_1        SPI

// DMA tube configuration for SPI1 Tx ; data = bmpOut
dma_tube_config SPI_1_DMA_TxTubeCfg =
{
    bmpOut,                     // Data source
    DMA_SIZE_8BITS,             // Source transfer size
    &SPI1->regs->DR,            // Destination of data
    DMA_SIZE_8BITS,             // Destination transfer size
    sizeof(bmpOut),             // Number of bytes to transfer
                                // Flags :
    DMA_CFG_SRC_INC |           // auto increment source address
    DMA_CFG_CIRC |              // circular buffer
    DMA_CFG_CMPLT_IE |          // set tube full IRQ
    DMA_CFG_ERR_IE |            // want messages in the DMA irq
    DMA_CCR_PL_VERY_HIGH,       // very high priority
    0,                          // reserved
    DMA_REQ_SRC_SPI1_TX         // Hardware DMA request source
};

void setup_SPI_1();
void setup_SPI_1_DMA();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ISR : NSS_2
// RISING : if not a payload frame, reset SPI + DMA
// pages begin with 0x10 0x00 0xB6 nn, 0 <= n <= 7)

void ISR_NSS_2()
{
    volatile uint8_t* p = SPI_2_Rx_Buffer;

    // keep pages only : 0x10 0x00 0xBn (0 <= n < 8)
    if ((*p++ != 0x10) || (*p++ != 0x00) || ((*p++ & 0xF0) != 0xB0) || ((*p & 0x0F) > 7)) // not a page or not in sync
    {
        // reset
        dma_disable(DMA1, SPI_2_RX_DMA_CH);
        dma_tube_cfg(DMA1, SPI_2_RX_DMA_CH, &SPI_2_DMA_RxTubeCfg);
        dma_enable(DMA1, SPI_2_RX_DMA_CH);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ISR : PIN_POWEROFF
// "poweroff" from RasPi

bool flagPowerOff = false;

void ISR_POWEROFF()
{
    flagPowerOff = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : "Arduino" setup()

void setup()
{
#ifdef __SERIAL_DEBUG
    Serial.begin(115200);
#endif

//    pinMode(PIN_HARD_SHUTDOWN, OUTPUT);
//    digitalWrite(PIN_HARD_SHUTDOWN, HIGH);
    pinMode(PIN_SOFT_SHUTDOWN, OUTPUT);
    digitalWrite(PIN_SOFT_SHUTDOWN, HIGH);

    pinMode(LED_BUILTIN, OUTPUT); // blinks when display is updated

    pinMode(PIN_READY, OUTPUT);
    digitalWrite(PIN_READY, LOW);
    pinMode(PIN_NSS_2, INPUT);    // needed ?

    //pinMode(PIN_POWEROFF, INPUT);
    pinMode(PIN_BTN_POWER, INPUT_PULLUP);
    pinMode(PIN_RELAY_ON, OUTPUT);
    pinMode(PIN_RELAY_OFF, OUTPUT);

    // turn bistable relay on
    digitalWrite(PIN_RELAY_OFF, LOW);
    digitalWrite(PIN_RELAY_ON, HIGH);
    delay(RELAY_PULSE); // bistable relay pulse
    digitalWrite(PIN_RELAY_ON, LOW);

    attachInterrupt(digitalPinToInterrupt(PIN_NSS_2), ISR_NSS_2, RISING);
    //attachInterrupt(digitalPinToInterrupt(PIN_POWEROFF), ISR_POWEROFF, FALLING);

    setup_SPI_1();
    setup_SPI_1_DMA();
    setup_SPI_2();
    setup_SPI_2_DMA();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : "Arduino" loop()

void loop()
{
    if (SPI_2_DMA_transfer_complete && stripCmdBytes())
    {
        bmpOledToBmpOut();
        SPI_2_DMA_transfer_complete = false; // reset flag

        // tells the RasPi new bitmap is ready : 1ms pulse
        // and blinks buitibn LED
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(PIN_READY, HIGH);
        delay(READY_PULSE_DURATION);
        digitalWrite(PIN_READY, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
    }

    if (flagPowerOff)
    {
#ifdef __SERIAL_DEBUG
        Serial.println("POWER OFF");
#endif
        //detachInterrupt(PIN_POWEROFF); // 2 falling edges ; ignore the second one
        //digitalWrite(PIN_RELAY_OFF, HIGH);
        //flagPowerOff = false; // otherwise : will be called until power is off
        // no further processing : power is now off...
    }

    // see https://myhomethings.eu/en/arduino-long-short-button-press/
    static uint32 holdingTime;
    static uint32 prevHoldingTime;
    static uint32 firstButtonPressTime;
    static uint32 btnState;
    static uint32 prevBtnState = HIGH;
    static bool done = false;
    //digitalWrite(PIN_HARD_SHUTDOWN, digitalRead(PIN_BTN_POWER));
    digitalWrite(PIN_SOFT_SHUTDOWN, digitalRead(PIN_BTN_POWER));
/*
    btnState = digitalRead(PIN_BTN_POWER);

    if (btnState == LOW && prevBtnState == HIGH && (millis() - firstButtonPressTime) > DEBOUNCE_TIME)
        firstButtonPressTime = millis();

    holdingTime = (millis() - firstButtonPressTime);

    if (holdingTime > DEBOUNCE_TIME)
    {
        if (btnState == LOW && holdingTime > prevHoldingTime)
        {
            if (holdingTime > LONG_PRESS_TIME && !done)
            {
#ifdef __SERIAL_DEBUG
                Serial.println("LONG PRESS");
                //delay(HARD_SHUTDOWN_PULSE);
                digitalWrite(PIN_HARD_SHUTDOWN, LOW);
                delay(HARD_SHUTDOWN_PULSE);
                digitalWrite(PIN_HARD_SHUTDOWN, HIGH);
#else
                digitalWrite(PIN_HARD_SHUTDOWN, LOW);
                delay(HARD_SHUTDOWN_PULSE);
                digitalWrite(PIN_HARD_SHUTDOWN, HIGH);
#endif
                done = true;
            }
        }

        if (btnState == HIGH && prevBtnState == LOW)
        {
            done = false;

            if (holdingTime <= LONG_PRESS_TIME)
            {
#ifdef __SERIAL_DEBUG
                Serial.println("SHORT PRESS");
                //                delay(SOFT_SHUTDOWN_PULSE);
                digitalWrite(PIN_SOFT_SHUTDOWN, HIGH);
                delay(SOFT_SHUTDOWN_PULSE);
                digitalWrite(PIN_SOFT_SHUTDOWN, LOW);
#else
                digitalWrite(PIN_SOFT_SHUTDOWN, HIGH);
                delay(SOFT_SHUTDOWN_PULSE);
                digitalWrite(PIN_SOFT_SHUTDOWN, LOW);
#endif
            }
        }
    }
*/
    prevBtnState = btnState;
    prevHoldingTime = holdingTime;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : setup SPI_2 ; slave ; Rx only

void setup_SPI_2()
{
    SPI_2.setModule(2); // STM32F103C8 : low density 1 or 2
    SPISettings spiSettings(0, MSBFIRST, SPI_MODE0, DATA_SIZE_8BIT);  // 0 Hz : set by master
    SPI_2.beginTransactionSlave(spiSettings);
    spi_rx_reg(SPI_2.dev()); // Clear Rx register in case we already received SPI data
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : setup SPI_1 ; slave, Rx only

void setup_SPI_1()
{
    SPI_1.setModule(1); // STM32F103C8 : low density 1 or 2
    SPISettings spiSettings(0, MSBFIRST, SPI_MODE0, DATA_SIZE_8BIT); // 0 Hz : set by master
    SPI_1.beginTransactionSlave(spiSettings);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : setup SPI_2 DMA ; Rx (MOSI) only

void setup_SPI_2_DMA()
{
    dma_init(DMA1);

    int ret_rx = dma_tube_cfg(DMA1, SPI_2_RX_DMA_CH, &SPI_2_DMA_RxTubeCfg);

    if (ret_rx != DMA_TUBE_CFG_SUCCESS)
    {
        // if the tube creation failed, no hope the code will work !
        while (1)
        {
#ifdef __SERIAL_DEBUG
            Serial.print("setup SPI_2 Rx DMA configuration error: ");
            Serial.println(ret_rx, HEX);
            Serial.println("Reset is needed!");
            delay(100);
#else
            // hope it will trigger the watchdog...
#endif
        }
    }

    spi_rx_reg(SPI_2.dev()); // Clear RX register in case we already received SPI data
    dma_attach_interrupt(DMA1, SPI_2_RX_DMA_CH, SPI_2_DMA_IRQ); // Attach interrupt to catch end of DMA transfer
    dma_enable(DMA1, SPI_2_RX_DMA_CH); // Rx : Enable DMA configurations
    spi_rx_dma_enable(SPI_2.dev()); // SPI DMA requests for Rx 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : setup SPI_1 DMA ; Tx (MISO) only
// http://stm32duinoforum.com/forum/viewtopic_f_14_t_3527_start_10.html

void setup_SPI_1_DMA()
{
    dma_init(DMA1);

    int ret_tx = dma_tube_cfg(DMA1, SPI_1_TX_DMA_CH, &SPI_1_DMA_TxTubeCfg);

    if (ret_tx != DMA_TUBE_CFG_SUCCESS)
    {
        // if the tube creation failed, no hope the code will work !
        while (1)
        {
#ifdef __SERIAL_DEBUG
            Serial.print("setup SPI_1 Tx DMA configuration error: ");
            Serial.println(ret_tx, HEX);
            Serial.println("Reset is needed!");
            delay(100);
#else
            // hope it will trigger the watchdog...
#endif
        }
    }

    dma_enable(DMA1, SPI_1_TX_DMA_CH); // Tx : Enable DMA configurations
    spi_tx_dma_enable(SPI_1.dev()); // SPI DMA requests for Tx 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : SPI_2 Rx IRQ

void SPI_2_DMA_IRQ()
{
    dma_irq_cause cause = dma_get_irq_cause(DMA1, SPI_2_RX_DMA_CH);

#ifdef __SERIAL_DEBUG
    switch (cause)
    {
    case DMA_TRANSFER_COMPLETE:            // Transfer is complete
        SPI_2_DMA_transfer_complete = true;
        //Serial.println("DMA_2 : DMA_TRANSFER_COMPLETE");
        break;
    case DMA_TRANSFER_ERROR:            // Error occurred during transfer
        Serial.println("DMA_2 : DMA_TRANSFER_ERROR");
        SPI_2_DMA_error = true;
        break;
    case DMA_TRANSFER_DME_ERROR:        // Direct mode error occurred during transfer
        Serial.println("DMA_2 : DMA_TRANSFER_DME_ERROR");
        SPI_2_DMA_error = true;
        break;
    case DMA_TRANSFER_FIFO_ERROR:        // FIFO error occurred during transfer
        Serial.println("DMA_2 : DMA_TRANSFER_FIFO_ERROR");
        SPI_2_DMA_error = true;
        break;
    case DMA_TRANSFER_HALF_COMPLETE:    // Transfer is half complete
        Serial.println("DMA_2 : DMA_TRANSFER_HALF_COMPLETE");
        SPI_2_DMA_error = true;
        break;
    default:
        Serial.println("DMA_2 : UNKNOWN ERROR");
        SPI_2_DMA_error = true;
        break;
    }
#else
    if (cause == DMA_TRANSFER_COMPLETE)
        SPI_2_DMA_transfer_complete = true;
    else
        SPI_2_DMA_error = true;
#endif

    dma_clear_isr_bits(DMA1, SPI_2_RX_DMA_CH);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : copies the SPI/DMA buffer to bmpOut, stripping the command bytes ; duration  30µs

bool stripCmdBytes()
{
    for (int i = 0; i < OLED_PAGE_COUNT; i++)
    {
        uint8_t* pRxBuf = (uint8_t*)SPI_2_Rx_Buffer + i * OLED_SPI_PAGE_SIZE;
        uint16_t cmd = *(uint16_t*)pRxBuf; // command bytes
        uint8_t pageNum = *(uint8_t*)(pRxBuf + 2);

        // test 1st 3 bytes (= command bytes) of each frame ; 0x10 0x00 0xBn , n = frame #
        // shouldn't happen, see /CS ISR...
        if (cmd != 0x10 || pageNum != i + 0xB0)
        {
#ifdef __SERIAL_DEBUG
            Serial.println("stripCmdBytes() : ERROR");
#endif
            return false; // error, not in sync, etc.
        }

        // copy pages to bmpOled, drop command bytes
        memcpy((uint8_t*)&bmpOled + i * OLED_PAGE_SIZE, pRxBuf + OLED_CMD_SIZE, OLED_PAGE_SIZE);
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function : converts the OLED bitmap into bmpOut bitmap
//
//    SSD1306/SSD1309/SSH1106 : 8 pages with 128 8bit vertical macropixels
//    bmpOut : 1024 8bit monochrome macropixels (128 lines 64 columns)
//    The operation is similar to a matrix transposition, each elementaty matrix being 8x8
//    duration : 3.9ms

void bmpOledToBmpOut()
{
    uint16_t i = 0; // bytes counter for output bitmap
    uint16_t j = 0; // bits counter for the current byte

    for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++)
        for (uint8_t line = 0; line < OLED_LINES_PER_PAGE; line++)
            for (uint8_t col = 0; col < OLED_PAGE_SIZE; col++)
            {
                // vertical byte to horizontal byte

                uint8_t val = bmpOled[page][col];

                if (val & (1 << line))
                    bmpOut[i] |= (1 << (7 - j));
                else
                    bmpOut[i] &= ~(1 << (7 - j));


                if (val & (1 << line))
                    bmpOut[i] |= (1 << (7 - j));
                else
                    bmpOut[i] &= ~(1 << (7 - j));

                j++;
                j %= 8;

                if (j == 0) // byte complete
                    i++; // next byte
            }
}

// END