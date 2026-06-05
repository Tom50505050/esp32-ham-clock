# Konfiguracja pinów ESP32-HAM-CLOCK

## Wyświetlacz TFT ILI9488 (480x320)

| Pin TFT | Pin ESP32 | Opis |
|---------|-----------|------|
| VCC | 5V | Zasilanie wyświetlacza |
| GND | GND | Masa |
| CS | 15 | Chip Select |
| DC/RS | 2 | Data/Command |
| SD/MOSI | 23 | Master Out Slave In |
| SCK | 18 | Serial Clock |
| SDO/MISO | 19 | Master In Slave Out |
| LED | 32 | Podświetlenie (PWM) |
| RESET | EN | Reset (podłączone do EN ESP32) |

## Dotyk XPT2046

| Pin Touch | Pin ESP32 | Opis |
|-----------|-----------|------|
| T.CLK | 18 | Serial Clock (wspólne z TFT) |
| T.CS | 21 | Chip Select |
| T.DIN/MOSI | 23 | Master Out Slave In (wspólne z TFT) |
| T.DO/MISO | 19 | Master In Slave Out (wspólne z TFT) |
| T.IRQ | 22 | Interrupt Request |

## Uwagi

- SPI TFT i touch korzystają z tego samego magistrali SPI (VSPI)
- Częstotliwość SPI TFT: 20 MHz
- Częstotliwość SPI touch: 2.5 MHz
- Podświetlenie sterowane przez PWM na pinie 32
- Reset wyświetlacza podłączony do EN (reset całego ESP32)

## Konfiguracja w platformio.ini

```ini
-DILI9488_DRIVER=1
-DTFT_MISO=19
-DTFT_MOSI=23
-DTFT_SCLK=18
-DTFT_CS=15
-DTFT_DC=2
-DTFT_RST=-1
-DTFT_WIDTH=480
-DTFT_HEIGHT=320
-DSPI_FREQUENCY=20000000
-DTFT_BL_PIN=32
-DTFT_ESPI_TOUCH_DISABLED=0
-DSPI_TOUCH_FREQUENCY=2500000
```

## Konfiguracja w kodzie (main.cpp)

```cpp
#define TOUCH_CS 21
#define TOUCH_IRQ 22
#define TOUCH_MOSI TFT_MOSI  // 23
#define TOUCH_MISO TFT_MISO  // 19
#define TOUCH_CLK TFT_SCLK   // 18
```
