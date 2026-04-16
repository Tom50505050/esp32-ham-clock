# Instrukcja Montażu i Wgrania ESP32-HAM-CLOCK

## Informacje o projekcie

**Wersja firmware:** 1.2b

**Autor oryginalnego projektu:** Konrad Wiśniewski SP3KON  
**Oryginalny projekt:** https://github.com/SP3KON/ESP32-HAM-CLOCK  
**Modyfikacje w tej wersji:**
- Rozszerzona obsługa PSK Reporter z filtrowaniem
- Interfejs dotykowy dla ekranu TFT
- Dodatkowe informacje systemowe w interfejsie web

**Licencja:** MIT License  
Kod można używać, modyfikować i rozpowszechniać pod warunkiem zachowania informacji o autorze oryginalnym. Pełna treść licencji w pliku CHANGELOG.md.

## Spis treści
1. [Montaż - Schemat połączeń](#montaż)
2. [Wgrywanie firmware](#wgrywanie-firmware)
3. [Troubleshooting](#znane-problemy)

---

# MONTAŻ

## Wymagane komponenty

| Element | Opis | Alternatywa |
|---------|------|-------------|
| ESP32 | DevKit v1 (38 pinów) | WROOM-32 + płytka |
| Wyświetlacz | TFT 3.5" ILI9488 480x320 | ILI9341 2.8" |
| Zasilanie | Moduł 18650 z ochroną DW01A | 3xAA lub USB |
| Przyciski | 2x tact switch (BOOT, RST) | - |
| Rezystory | 2x 100kΩ (dzielnik napięcia baterii) | - |
| Kondensatory | 100nF + 10μF (stabilizacja) | - |

## Schemat połączeń ESP32 z TFT

```
ESP32 DevKit v1         TFT ILI9488 (3.5" 480x320)
═══════════════════════════════════════════════════════
GPIO 18 (SCK)    ──────→  SCK / SCL        [ żółty ]
GPIO 23 (MOSI)   ──────→  SDI / MOSI       [ zielony ]
GPIO 19 (MISO)   ──────→  SDO / MISO       [ niebieski ] (opcjonalnie)
GPIO 15 (CS)     ──────→  CS               [ pomarańczowy ]
GPIO 2  (DC/RS)  ──────→  DC / RS           [ biały ]

ZASILANIE:
3.3V             ──────→  VCC              [ czerwony ]
GND              ──────→  GND              [ czarny ]
3.3V             ──────→  LED (podświetlenie) [ przez rezystor 100Ω ]
```

## Pomiar napięcia baterii (dzielnik)

```
Bateria 18650 (+) ────┬─── [R1 100kΩ] ───┬───→ GPIO 34 (ADC)
                      │                 │
                     GND              [R2 100kΩ] ───┬─── GND
                                                   │
                                              Kondensator 100nF
```

**Formuła:** Vbat = Vadc × 2  (przy R1 = R2 = 100kΩ)

## Podłączenie dotyku XPT2046 (opcjonalnie)

```
ESP32                   XPT2046 (dotyk)
══════════════════════════════════════════
GPIO 4   ──────→  T_IRQ
GPIO 12  ──────→  T_DO
GPIO 13  ──────→  T_DIN
GPIO 14  ──────→  T_CS
GPIO 25  ──────→  T_CLK
```

## Pełny pinout ESP32 DevKit v1 (38 pinów)

```
          EN (RST)  [ ]   [ ]  3.3V
          GPIO 23   [ ]   [ ]  GPIO 22
          GPIO 22   [ ]   [ ]  GPIO 1 (TX0)
          GPIO 1    [ ]   [ ]  GPIO 3 (RX0)
          GPIO 3    [ ]   [ ]  GPIO 21
          GPIO 21   [ ]   [ ]  GPIO 19 (MISO)  ← TFT_MISO
          GPIO 19   [ ]   [ ]  GPIO 18 (SCK)   ← TFT_SCLK
          GPIO 18   [ ]   [ ]  GPIO 5
          GPIO 5    [ ]   [ ]  GPIO 17 (TX2)
          GPIO 17   [ ]   [ ]  GPIO 16 (RX2)
          GPIO 16   [ ]   [ ]  GPIO 4
          GPIO 4    [ ]   [ ]  GPIO 0 (BOOT)   ← Przycisk BOOT
          GPIO 0    [ ]   [ ]  GPIO 2           ← TFT_DC
          GPIO 2    [ ]   [ ]  GPIO 15 (CS)      ← TFT_CS
          GPIO 15   [ ]   [ ]  GPIO 13          ← T_DIN
          GPIO 13   [ ]   [ ]  GPIO 12          ← T_DO
          GPIO 12   [ ]   [ ]  GPIO 14          ← T_CS
          GPIO 14   [ ]   [ ]  GPIO 27
          GPIO 27   [ ]   [ ]  GPIO 26
          GPIO 26   [ ]   [ ]  GPIO 25          ← T_CLK
                    [ ]   [ ]
          GPIO 33   [ ]   [ ]  GPIO 32
          GPIO 32   [ ]   [ ]  GPIO 35
          GPIO 35   [ ]   [ ]  GPIO 34          ← BATTERY_ADC
          GPIO 34   [ ]   [ ]  GPIO 39
          GPIO 39   [ ]   [ ]  GPIO 36
          GPIO 36   [ ]   [ ]  VIN (5V)
          GND       [ ]   [ ]  GND
```

---

# WGRYWANIE FIRMWARE

## Pliki w folderze `pliki.bin/`

| Plik | Adres | Opis |
|------|-------|------|
| `bootloader.bin` | 0x1000 | Bootloader ESP32 |
| `partitions.bin` | 0x8000 | Tablica partycji |
| `firmware.bin` | 0x10000 | Główny program |
| `littlefs.bin` | 0x290000 | System plików (ikony, fonty) |

---

## Sposób 1: Wgranie przez ESP Flash Download Tool (Windows)

1. Pobierz **ESP Flash Download Tool** z:
   https://www.espressif.com/en/support/download/other-tools

2. Podłącz ESP32 do komputera przez USB

3. Uruchom ESP Flash Download Tool:
   - Wybierz chip: **ESP32**
   - COM port: Twój port (np. COM3, COM4)
   - BAUD: **921600** (lub 115200)

4. Dodaj pliki z odpowiednimi adresami:
   ```
   bootloader.bin  @ 0x1000
   partitions.bin  @ 0x8000
   firmware.bin    @ 0x10000
   littlefs.bin    @ 0x290000
   ```

5. Zaznacz checkbox przy każdym pliku

6. Kliknij **START** (przedtem przytrzymaj BOOT/IO0 i naciśnij RESET na ESP32)

---

## Sposób 2: Wgranie przez esptool.py (Windows/Linux/Mac)

### Instalacja esptool:
```bash
pip install esptool
```

### Komenda wgrania (wszystkie pliki na raz):
```bash
esptool.py --chip esp32 --port COM7 --baud 921600 write_flash \
  0x1000 bootloader.bin \
  0x8000 partitions.bin \
  0x10000 firmware.bin \
  0x290000 littlefs.bin
```

### Jeśli ESP32 nie wchodzi w tryb flash:
Przed wgraniem przytrzymaj przycisk **BOOT** (IO0), naciśnij i puść **RST** (EN), puść **BOOT**.

---

## Sposób 3: Wgranie przez PlatformIO (VS Code)

Jeśli masz całe źródło projektu:
```bash
pio run --target upload
```

---

## Sposób 4: Wgranie przez WWW (OTA - Over-The-Air)

### Wymagania:
- ESP32 musi być podłączony do sieci WiFi
- Znany adres IP ESP32 (wyświetlany na ekranie lub w serial monitor)
- Plik `firmware.bin` (tylko aplikacja, bez bootloadera i partycji)

### Przez web interface (jeśli zaimplementowano):
1. Wejdź w przeglądarce na adres: `http://[IP_ESP32]/`
2. Jeśli dostępna jest opcja "Update" lub "OTA" - wybierz plik `firmware.bin`
3. Kliknij "Upload" i poczekaj na zakończenie

### Przez ArduinoOTA (jeśli dodano do kodu):

1. Dodaj bibliotekę do `platformio.ini`:
```ini
lib_deps = 
    bblanchon/ArduinoJson @ ^7.0.0
    bodmer/TFT_eSPI @ 2.4.79
    https://github.com/PaulStoffregen/XPT2046_Touchscreen.git
    me-no-dev/AsyncTCP @ ^1.1.1
    me-no-dev/ESP Async WebServer @ ^1.2.3
    ayushsharma82/AsyncElegantOTA @ ^2.2.7
```

2. Dodaj w `main.cpp` (w sekcji setup lub init):
```cpp
#include <AsyncElegantOTA.h>

// W setupWebServer() lub setup()
AsyncElegantOTA.begin(server);
```

3. Po restarcie ESP32 wejdź na: `http://[IP_ESP32]/update`
4. Wybierz plik `firmware.bin` i kliknij "Update"

### Przez ESP32 OTA Web Browser (alternatywnie):
```bash
# Użyj narzędzia ota-web-server
pip install esptool
python -m esptool --chip esp32 --port "socket://[IP_ESP32]:3232" write_flash 0x10000 firmware.bin
```

**Uwaga:** OTA wymaga, aby firmware był skompilowany z odpowiednim schematem partycji z obsługą OTA.

---

## Sposób 5: Wgranie przez WebSerial ESPTool (przeglądarka)

**Najprostsza metoda - bez instalowania żadnych programów!**

### Wymagania:
- Przeglądarka obsługująca Web Serial API: **Chrome**, **Edge** lub **Opera**
- ESP32 podłączony przez USB do komputera

### Link:
🔗 **https://jason2866.github.io/WebSerial_ESPTool/**

### Krok po kroku:

1. **Otwórz stronę** w przeglądarce Chrome/Edge/Opera:
   ```
   https://jason2866.github.io/WebSerial_ESPTool/
   ```

2. **Podłącz ESP32** przez USB

3. **Kliknij "Connect"** na stronie i wybierz port COM z listy (np. COM7)

4. **Przytrzymaj BOOT** (GPIO0) na ESP32, **naciśnij i puść RST**, **puść BOOT**
   - ESP32 wejdzie w tryb bootloadera
   - Strona powinna pokazać "Connected" i informacje o chipie

5. **Wybierz pliki do wgrania** (przycisk "Add Files") w kolejności:

   | Plik | Adres | Rozmiar |
   |------|-------|---------|
   | `bootloader.bin` | 0x1000 | ~17KB |
   | `partitions.bin` | 0x8000 | ~3KB |
   | `firmware.bin` | 0x10000 | ~1250KB |
   | `littlefs.bin` | 0x290000 | ~1984KB |

6. **Kliknij "Program"** i poczekaj na zakończenie (ok. 2-3 minuty)

7. Po zakończeniu **naciśnij RST** na ESP32 aby uruchomić nowy firmware

### Zalety tej metody:
- ✅ **Brak instalacji** - działa w przeglądarce
- ✅ **Działa na Windows, Mac, Linux, Chromebook**
- ✅ **Prosty interface** - wybierasz pliki i klikasz "Program"
- ✅ **Automatyczne wykrywanie** ESP32

### W razie problemów:
- Jeśli nie widzisz portu COM → zainstaluj sterowniki CH340/CP2102
- Jeśli połączenie się nie udaje → spróbuj niższej prędkości (115200 zamiast 921600)
- Jeśli wgrywanie się nie udaje → sprawdź czy użyłeś trybu bootloadera (BOOT+RST)

---

## Podłączenie ESP32 DevKit v1 (38 pinów)

| Funkcja | GPIO | Pin na PCB |
|---------|------|------------|
| **TFT_SCLK** (SCK) | GPIO 18 | 18 |
| **TFT_MISO** | GPIO 19 | 19 |
| **TFT_MOSI** (SDI) | GPIO 23 | 23 |
| **TFT_CS** | GPIO 15 | 15 |
| **TFT_DC** (RS) | GPIO 2 | 2 |
| **TFT_RST** | -1 (nieużywane) | - |

### Zasilanie TFT 3.5" ILI9488:
- **VCC** → 3.3V lub 5V (zależy od modułu)
- **GND** → GND
- **LED** → 3.3V (podświetlenie) lub rezystor 100Ω do 5V

### Dotyk XPT2046 (opcjonalnie):
- **T_IRQ** → GPIO 4
- **T_DO** → GPIO 12
- **T_DIN** → GPIO 13
- **T_CS** → GPIO 14
- **T_CLK** → GPIO 25

---

## Pełny pinout ESP32 DevKit v1 (38 pinów)

```
          EN (RST)  [ ]   [ ]  3.3V
          GPIO 23   [ ]   [ ]  GPIO 22
          GPIO 22   [ ]   [ ]  GPIO 1 (TX0)
          GPIO 1    [ ]   [ ]  GPIO 3 (RX0)
          GPIO 3    [ ]   [ ]  GPIO 21
          GPIO 21   [ ]   [ ]  GPIO 19 (MISO)  ← TFT_MISO
          GPIO 19   [ ]   [ ]  GPIO 18 (SCK)   ← TFT_SCLK
          GPIO 18   [ ]   [ ]  GPIO 5
          GPIO 5    [ ]   [ ]  GPIO 17 (TX2)
          GPIO 17   [ ]   [ ]  GPIO 16 (RX2)
          GPIO 16   [ ]   [ ]  GPIO 4
          GPIO 4    [ ]   [ ]  GPIO 0 (BOOT)   ← Przycisk BOOT
          GPIO 0    [ ]   [ ]  GPIO 2           ← TFT_DC
          GPIO 2    [ ]   [ ]  GPIO 15 (CS)     ← TFT_CS
          GPIO 15   [ ]   [ ]  GPIO 13
          GPIO 13   [ ]   [ ]  GPIO 12
          GPIO 12   [ ]   [ ]  GPIO 14
          GPIO 14   [ ]   [ ]  GPIO 27
          GPIO 27   [ ]   [ ]  GPIO 26
          GPIO 26   [ ]   [ ]  GPIO 25
                    [ ]   [ ]
          GPIO 33   [ ]   [ ]  GPIO 32
          GPIO 32   [ ]   [ ]  GPIO 35
          GPIO 35   [ ]   [ ]  GPIO 34
          GPIO 34   [ ]   [ ]  GPIO 39
          GPIO 39   [ ]   [ ]  GPIO 36
          GPIO 36   [ ]   [ ]  VIN (5V)
          GND       [ ]   [ ]  GND
```

---

## Schemat połączenia TFT ILI9488 z ESP32

```
ESP32                    ILI9488 (3.5" 480x320)
──────────────────────────────────────────────
GPIO 18 (SCK)     →      SCK / SCL
GPIO 23 (MOSI)    →      SDI / MOSI
GPIO 19 (MISO)    →      SDO / MISO (opcjonalnie)
GPIO 15 (CS)      →      CS
GPIO 2  (DC)      →      DC / RS
3.3V              →      VCC
GND               →      GND
3.3V (lub 5V+100Ω) →     LED (podświetlenie)
```

---

## Znane problemy

1. **"Failed to connect to ESP32"** - Przytrzymaj BOOT i naciśnij RST przed wgraniem
2. **Biały ekran** - Sprawdź połączenie zasilania i podświetlenia LED
3. **Brak obrazu** - Sprawdź czy masz dobre połączenie SPI (CS, DC, MOSI, SCK)
4. **Nie działa touch** - XPT2046 używa osobnego SPI (może być software SPI)

---

## Przydatne komendy esptool

### Wyczyść całą pamięć flash:
```bash
esptool.py --chip esp32 --port COM7 erase_flash
```

### Odczytaj flash do pliku (backup):
```bash
esptool.py --chip esp32 --port COM7 read_flash 0x00000 0x400000 backup.bin
```

### Sprawdź informacje o chipie:
```bash
esptool.py --chip esp32 --port COM7 chip_id
```

---

## Uwaga dotycząca schematu partycji

Projekt używa niestandardowego podziału:
- **2MB** na aplikację (firmware)
- **2MB** na LittleFS (dane, ikony, fonty)

Jeśli wgrywasz tylko `firmware.bin`, musisz wcześniej wgrać `littlefs.bin` razem z `partitions.bin` i `bootloader.bin`.

---

Przygotowano dla: **ESP32 HAM CLOCK**  
Wersja: 2024-04-10
