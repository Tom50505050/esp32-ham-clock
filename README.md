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

## Zmiany v1.4 vs v1.3 — Pelna lista

### Poprawki krytyczne (crash / zawieszanie)

- **SPI race condition** — `updateIssDynamicDisplay()` wywolywana z Core 0 (main loop) kolindowala z UI task na Core 1 po magistrali SPI. Przeniesiono rysowanie ISS do flagi `uiPendingIssDynamicRedraw` obslugiwanej przez UI task. Eliminuje zawieszanie ekranow po kilku minutach pracy.
- **xQueueGenericSend crash** — `handleScreenIssMenuTouch()` wywolywal `drawScreen()` z Core 0, powodujac deadlock kolejki UI task. Zamieniono na `requestUiScreenRedraw()` ktore ustawia flage bezposrednio.
- **WiFi circuit breaker** — po przekroczeniu progu `HTTPS_FAIL_THRESHOLD` resetowal WiFi bez `WiFi.disconnect()` + `WiFi.begin()`, przez co ESP32 nie mogl sie polaczyc ponownie. Dodano `wifiConnected = false` + wymuszenie reconnect.
- **PSK Reporter crash** — globalny `WiFiClientSecure` bez try/catch przy `loadCACert` — dodano obsluge wyjatkow.

### Stabilnosc sieci

- **Weather / Propagation / Forecast HTTP -1** — `static WiFiClientSecure` nie wywolywal `client.stop()` przed kolejnym zadaniem, zostawaly stany TLS powodujace bledy -1. Dodano `client.stop()` + timeout 5000ms do WSZYSTKICH funkcji HTTP: `fetchWeatherData()`, `fetchWeatherForecast()`, `fetchAirPollutionData()`, `fetchPropagationData()`, `fetchCallookCallsignInfo()`, `fetchPotaApi()`, `ensureQrzSession()`.
- **HTTPS mutex (splot)** — `SemaphoreHandle_t httpsMutex` tworzony w `setup()` serializuje wszystkie zadania HTTPS, eliminujac race condition na poziomie TLS/WiFi miedzy Core 0 a Core 1.
- **ISS task na Core 1** — `issTask()` dziala na Core 1, nie wywoluje juz `httpsFailCount++`, wiec circuit breaker nie jest blokowany przez ISS.
- **Weather heap guards** — skip fetch jesli `ESP.getFreeHeap() < 50000`, skip forecast/air jesli `< 40000`.
- **Callook/POTA heap guards** — skip jesli `heap < 50000`, timeout 3-8s.
- **Auto-restart** — restart jesli `ESP.getFreeHeap() < 30000` i `loopCounter > 100`.

### ISS (International Space Station)

- **Flag BMP (16bpp)** — `drawBmpFromFS()` obslugiwal tylko 24-bit BMP, a flagi w `/flags/` sa 16-bit (822B). Dodano `drawBmp16FromFS()` z obsluga ujemnej wysokosci (top-down BMP) i walidacja wymiarow. `updateIssDynamicDisplay()` probuje najpierw 16-bit, potem fallback na 24-bit.
- **n2yo visualpasses** — poprawiony format URL na `/5?apiKey=`, dodano fallback na `radiopresses` jesli visual zwraca 0.
- **SGP4 fallback** — jesli n2yo nie ma danych, SGP4 oblicza najlepszy pass (`bestPeakEl`/`bestAosT`). Zmienna `issNextPassFromN2yo` chroni przed nadpisaniem dobrych danych n2yo przez SGP4.
- **Max EL display** — "MAX EL: XX.X" w kolorze CYAN na ekranie ISS, nowa zmienna globalna `issNextPassEstimateMaxEl`.
- **ISS menu** — hamburger (5,7) w `drawIssStaticInterface()` z opcjami "RESTART ISS DATA" i "CLOSE". Resetuje timery i ustawia `issForceRefresh = true`.
- **Startup delay** — 15s zamiast dluzszego, natychmiastowy fetch n2yo po starcie.

### Web UI

- **EN/PL i18n** — slownik `I18N` (~200+ wpisow PL->EN) dla statycznego HTML + `I18N_JS` dla dynamicznych stringow. Funkcja `setLang('en'/'pl')` uzywa DOM tree walker. Zapis w `localStorage('hamclock_lang')`, przywracany przy starcie strony.
- **EN/PL buttons** — zmieniono z `<a href="/indexEN.html">` na `onclick="setLang('pl'/'en')"` z klasami `.pl-btn`/`.en-btn`.
- **Screen order labels** — `updateScreenSlotLabels()` synchronizuje selecty `screen_slot_1..14` z `SCREEN_NAME_MAP` i `onchange` handlers.
- **Dynamiczne stringi** — funkcja `t()` owija JS-generated text przez `I18N_JS` dictionary.

### LittleFS

- **Partycja 2304KB** — zmniejszono liczbe flag z 254 do 90 (16bpp BMP), usuniety `splash.png`. Zajete ~1778KB, wolne ~582KB.
- **BMP failure cache** — `drawBmpFromFS()` cache'uje nieudane sciezki przez 60s, zapobiegajac spamowi logow.
- **DX/POTA cluster reconnect cooldown** — 10s cooldown po rozlaczeniu, zapobiegajac natychmiastowemu reconnect loop.

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
