# ESP32-HAM-CLOCK v1.4

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Display: TFT ILI9488](https://img.shields.io/badge/Display-TFT_480x320-green.svg)]()

Zaawansowany zegar stacji amatorskiej oparty na ESP32 z 14 ekranami TFT, obsługą dotykową i interfejsem WWW. Modyfikacja oryginalnego projektu [SP3KON/ESP32-HAM-CLOCK](https://github.com/SP3KON/ESP32-HAM-CLOCK).

## Ekran glowny

| Ekran | Opis |
|-------|------|
| **HAM Clock** | Zegar UTC, kalendarz, fazy Ksiezyca, DX info |
| **DX Cluster** | Spoty z calego swiata w czasie rzeczywistym |
| **Sun Spots** | Aktywnosc sloneczna i warunki propagacyjne |
| **Band Info** | Informacje o pasmach amatorskich |
| **Weather DSP** | Pogoda z OpenMeteo |
| **APRS IS** | Monitoring pakietow APRS |
| **POTA Cluster** | Aktywacje Parkow na Powietrzu |
| **Ham Alert** | Powiadomienia o spotach |
| **APRS Radar** | Radar APRS wokol QTH |
| **Matrix Clock** | Zegar w stylu Matrix |
| **Unlis Hunter** | Logowanie stacji |
| **Weather Forecast** | Prognoza pogody na 7 dni |
| **PSK Map** | Mapa aktywnosci cyfrowych (FT8/FT4/JS8) |
| **ISS Tracking** | Sledzenie ISS z predykcja przelotow |

## Cechy v1.4

### Stabilnosc i niezawodnosc
- **Mutex HTTPS** — serializacja zadan TLS eliminujaca race condition
- **SPI race fix** — rysowanie ISS przeniesione do UI task na Core 1, eliminujac konflikt magistrali
- **client.stop()** — naprawiony wyciek polaczen TLS we WSZYSTKICH funkcjach HTTP
- **Heap guards** — auto-skip fetch przy malo pamieci (< 50KB heap)
- **Auto-restart** — restart przy krytycznie malo pamieci (< 30KB)
- **Circuit breaker z WiFi reconnect** — poprawiony reset WiFi po przekroczeniu progu bledow

### ISS (International Space Station)
- **Predykcja przelotow n2yo.com** — nastepny przelot z czasem AOS/LOS i MAX EL
- **SGP4 fallback** — obliczenia orbity gdy n2yo niedostepne
- **Flagi krajow (16bpp BMP)** — automatyczne wyswietlanie flagi kraju nad ISS
- **Menu ISS** — restart danych ISS z poziomu ekranu dotykowego
- **Mapa swiata** — pozycja ISS na mapie z kalkulacja odleglosci

### Web UI
- **Tlumaczenie EN/PL** — pelne tlumaczenie interfejsu web (200+ wpisow)
- **Kolejnosc ekranow** — konfiguracja 14 ekranow z nazwami w czasie rzeczywistym
- **Dynamiczne stringi** — tlumaczenie wszystkich tekstow generowanych przez JS

### LittleFS
- **Partycja 2304KB** — 90 flag krajow (16bpp BMP), splash, mapa swiata, fonty, ikony
- **BMP failure cache** — cache nieudanych sciezek BMP przez 60s
- **DX/POTA reconnect cooldown** — 10s cooldown po rozlaczeniu

## Sprzet

| Komponent | Specyfikacja |
|-----------|-------------|
| Mikrokontroler | ESP32 (ESP32-2432S028R lub podobny) |
| Wyswietlacz | TN3.5 Cal ILI9488 480x320 |
| Touchscreen | XPT2046 |
| Pamiec | 4MB Flash (partycja: 1792KB APP + 2304KB LittleFS) |
| Zasilanie | 5V USB lub akumulator 18650 + TP4056 |

### Mapa pinow

```
TFT:
  MISO: GPIO 19    MOSI: GPIO 23    SCLK: GPIO 18
  CS:   GPIO 15    DC:   GPIO 2     RST:  NC (-1)

Touchscreen XPT2046:
  CS: GPIO 21      IRQ: GPIO 27

Bateria (opcjonalnie):
  ADC: GPIO 34
```

## Instalacja

### Metoda 1: PlatformIO (zalecana)

1. Zainstaluj [VS Code](https://code.visualstudio.com/) + [PlatformIO](https://platformio.org/)
2. Sklonuj repozytorium:
   ```
   git clone https://github.com/Tom50505050/esp32-ham-clock.git
   ```
3. Podlacz ESP32 przez USB
4. Kliknij **Upload** w PlatformIO
5. Wgraj LittleFS: **Project Tasks > Upload File System Image**

### Metoda 2: Gotowe pliki bin (bez kompilacji)

1. Pobierz pliki z folderu `pliki.bin/`
2. Wejdz na: https://jason2866.github.io/WebSerial_ESPTool/
3. Podlacz ESP32
4. Wgraj pliki:
   - `0x1000-bootloader.bin` -> `0x1000`
   - `0x8000-partitions.bin` -> `0x8000`
   - `0x10000-firmware.bin` -> `0x10000`
   - `0x1C0000-littlefs.bin` -> `0x1C0000`
5. Kliknij **Program**

## Konfiguracja

Po pierwszym uruchomieniu ESP32 utworzy siec WiFi `ESP32-HAM-CLOCK`:

1. Polacz sie z siecia `ESP32-HAM-CLOCK`
2. Otworz przegladarke: `http://192.168.4.1`
3. Skonfiguruj:
   - WiFi (SSID + haslo)
   - Callsign (znak amatorski, np. SP9TNV)
   - Locator (np. JO90HF)
   - DX Cluster (host + port)
   - APRS-IS (host + port + filter)
   - HamAlert (login + haslo)
   - PSK Reporter (znak, pasmo, tryb, max spotow)
   - ISS (wybor zrodla danych: n2yo.com)
   - POTA (automatyczne odswiezanie)

## Struktura projektu

```
esp32-ham-clock/
├── src/
│   └── main.cpp                  # Glowny kod (~21700 linii)
├── littlefs_data/
│   ├── index.html                # Interfejs WWW z i18n EN/PL
│   ├── splash.bmp                # Ekran startowy
│   ├── mapa.bmp                  # Baza mapy (16bit BMP)
│   ├── Mapa swiata.bmp           | Mapa swiata (24bit BMP)
│   ├── flags/                    # Flagi krajow (90x 16bpp BMP)
│   ├── icon50/                   # Ikony ekranow
│   └── fonts/                    # Fonty VLW
├── pliki.bin/                    # Gotowe pliki bin do wgrania
├── flags_png/                    # Zrodlowe PNG flag
├── partitions.csv                # Schemat partycji
├── platformio.ini                # Konfiguracja PlatformIO
├── CHANGELOG.md                  # Historia zmian
└── README.md                     # Ten plik
```

## REST API

| Endpoint | Opis |
|----------|------|
| `GET /api/config` | Pobierz konfiguracje |
| `POST /api/config` | Zapisz konfiguracje |
| `GET /api/spots` | Pobierz spoty DX Cluster |
| `GET /api/psk` | Pobierz dane PSK Reporter |
| `GET /api/system` | Informacje systemowe |
| `GET /api/pota` | Aktywacje POTA |
| `GET /api/propagation` | Warunki propagacyjne |
| `GET /instruction` | Instrukcja obslugi |

## Zmiana splash screen

Wymagania pliku BMP:
- Rozdzielczosc: 480x320
- Format: BMP 24-bit (RGB) lub 16-bit (RGB565)
- Kompresja: brak
- Nazwa: `splash.bmp`

Zamien plik w `littlefs_data/splash.bmp` i wgraj LittleFS.

## Zmiany v1.4 vs v1.3

### Poprawki krytyczne
- **SPI race condition** — `updateIssDynamicDisplay()` przeniesiona do UI task, eliminujac konflikt magistrali SPI miedzy Core 0 a Core 1
- **xQueueGenericSend crash** — `handleScreenIssMenuTouch()` uzywa `requestUiScreenRedraw()` zamiast `drawScreen()`
- **WiFi circuit breaker** — dodano `WiFi.disconnect()` + `WiFi.begin()` + `wifiConnected=false`
- **PSK Reporter crash** — globalny `WiFiClientSecure` z try/catch

### Stabilnosc sieci
- **Weather/Propagation/Forecast HTTP -1** — `client.stop()` + timeout 5000ms
- **HTTPS mutex** — `SemaphoreHandle_t httpsMutex` serializujace zadania HTTPS
- **ISS task na Core 1** — nie wywoluje `httpsFailCount++`, circuit breaker nie blokowany
- **Heap guards** — skip fetch przy < 50KB heap, forecast/air < 40KB
- **Callook/POTA heap guards** — skip < 50KB, timeout 3-8s
- **Auto-restart** — restart przy < 30KB free heap

### ISS
- **Flag BMP (16bpp)** — `drawBmp16FromFS()` z obsluga ujemnej wysokosci i walidacja wymiarow
- **n2yo visualpasses** — poprawiony URL, fallback na radiopasses
- **SGP4 fallback** — bestPeakEl/bestAosT gdy n2yo niedostepne
- **Max EL display** — "MAX EL: XX.X" w kolorze CYAN
- **ISS menu** — hamburger (5,7) z opcja "RESTART ISS DATA"
- **Startup delay** — 15s, natychmiastowy fetch n2yo

### Web UI
- **EN/PL i18n** — slownik ~200+ wpisow, `setLang('en')`, `localStorage` persistencja
- **Screen order labels** — `updateScreenSlotLabels()` z `SCREEN_NAME_MAP`
- **Dynamiczne stringi** — `I18N_JS` dictionary + funkcja `t()`

### LittleFS
- **Partycja 2304KB** — 90 flag (16bpp), usuniety splash.png
- **BMP failure cache** — 60s cache nieudanych sciezek
- **DX/POTA cooldown** — 10s po rozlaczeniu

## Licencja

**MIT License**

Autor oryginalnego projektu: [Krzysztof Blaszczyk SP3KON](https://github.com/SP3KON/ESP32-HAM-CLOCK)

Modyfikacje: Tomasz [SP9TNV](https://github.com/Tom50505050)

## Podziekowania

- **SP3KON** — oryginalny projekt i baza kodowa
- **OpenMeteo** — API pogodowe
- **PSK Reporter** — API monitoringu cyfrowych trybow
- **HamAlert** — system powiadomien
- **POTA** — program Parkow na Powietrzu
- **N2YO** — predykcja przelotow ISS
- **Celestrak** — TLE ISS

## Kontakt

- **GitHub:** [Tom50505050/esp32-ham-clock](https://github.com/Tom50505050/esp32-ham-clock)
- **Oryginalny projekt:** [SP3KON/ESP32-HAM-CLOCK](https://github.com/SP3KON/ESP32-HAM-CLOCK)

---

**73! de SP9TNV**
