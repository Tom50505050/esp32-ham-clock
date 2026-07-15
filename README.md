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
- **Mutex HTTPS** — `SemaphoreHandle_t httpsMutex` tworzony w `setup()` serializuje wszystkie zadania HTTPS, eliminujac race condition na poziomie TLS/WiFi miedzy Core 0 a Core 1
- **SPI race fix** — `updateIssDynamicDisplay()` wywolywana z Core 0 (main loop) kolindowala z UI task na Core 1 po magistrali SPI. Przeniesiono rysowanie ISS do flagi `uiPendingIssDynamicRedraw` obslugiwanej przez UI task. Eliminuje zawieszanie ekranow po kilku minutach pracy
- **xQueueGenericSend crash** — `handleScreenIssMenuTouch()` wywolywal `drawScreen()` z Core 0, powodujac deadlock kolejki UI task. Zamieniono na `requestUiScreenRedraw()` ktore ustawia flage bezposrednio
- **WiFi circuit breaker** — po przekroczeniu progu `HTTPS_FAIL_THRESHOLD` resetowal WiFi bez `WiFi.disconnect()` + `WiFi.begin()`, przez co ESP32 nie mogl sie polaczyc ponownie. Dodano `wifiConnected = false` + wymuszenie reconnect
- **PSK Reporter crash** — globalny `WiFiClientSecure` bez try/catch przy `loadCACert` — dodano obsluge wyjatkow
- **client.stop()** — naprawiony wyciek polaczen TLS: `client.stop()` + timeout 5000ms do WSZYSTKICH funkcji HTTP (`fetchWeatherData()`, `fetchWeatherForecast()`, `fetchAirPollutionData()`, `fetchPropagationData()`, `fetchCallookCallsignInfo()`, `fetchPotaApi()`, `ensureQrzSession()`)
- **Heap guards** — skip fetch Weather/Propagation jesli `ESP.getFreeHeap() < 50000`, skip Forecast/Air jesli `< 40000`, skip Callook/POTA jesli `< 50000`, timeout 3-8s
- **Auto-restart** — restart jesli `ESP.getFreeHeap() < 30000` i `loopCounter > 100`
- **ISS task na Core 1** — `issTask()` dziala na Core 1, nie wywoluje `httpsFailCount++`, circuit breaker nie jest blokowany przez ISS

### ISS (International Space Station)
- **Predykcja przelotow n2yo.com** — nastepny przelot z czasem AOS/LOS i MAX EL, format URL `/5?apiKey=`, fallback na `radiopresses` jesli visual zwraca 0
- **SGP4 fallback** — jesli n2yo nie ma danych, SGP4 oblicza najlepszy pass (`bestPeakEl`/`bestAosT`). Zmienna `issNextPassFromN2yo` chroni przed nadpisaniem dobrych danych n2yo przez SGP4
- **Flagi krajow (16bpp BMP)** — `drawBmpFromFS()` obslugiwal tylko 24-bit BMP, a flagi w `/flags/` sa 16-bit (822B). Dodano `drawBmp16FromFS()` z obsluga ujemnej wysokosci (top-down BMP) i walidacja wymiarow. `updateIssDynamicDisplay()` probuje najpierw 16-bit, potem fallback na 24-bit
- **ISS menu** — hamburger (5,7) w `drawIssStaticInterface()` z opcjami "RESTART ISS DATA" i "CLOSE". Resetuje timery i ustawia `issForceRefresh = true`
- **Mapa swiata** — pozycja ISS na mapie z kalkulacja odleglosci do QTH
- **Max EL display** — "MAX EL: XX.X" w kolorze CYAN na ekranie ISS, zmienna globalna `issNextPassEstimateMaxEl`
- **Startup delay** — 15s zamiast dluzszego, natychmiastowy fetch n2yo po starcie

### Web UI
- **EN/PL i18n** — slownik `I18N` (~200+ wpisow PL->EN) dla statycznego HTML + `I18N_JS` dla dynamicznych stringow. Funkcja `setLang('en'/'pl')` uzywa DOM tree walker. Zapis w `localStorage('hamclock_lang')`, przywracany przy starcie strony
- **EN/PL buttons** — zmieniono z `<a href="/indexEN.html">` na `onclick="setLang('pl'/'en')"` z klasami `.pl-btn`/`.en-btn`
- **Screen order labels** — `updateScreenSlotLabels()` synchronizuje selecty `screen_slot_1..14` z `SCREEN_NAME_MAP` i `onchange` handlers
- **Dynamiczne stringi** — funkcja `t()` owija JS-generated text przez `I18N_JS` dictionary

### LittleFS
- **Partycja 2304KB** — zmniejszono liczbe flag z 254 do 90 (16bpp BMP), usuniety `splash.png`. Zajete ~1778KB, wolne ~582KB
- **BMP failure cache** — `drawBmpFromFS()` cache'uje nieudane sciezki przez 60s, zapobiegajac spamowi logow
- **DX/POTA cluster reconnect cooldown** — 10s cooldown po rozlaczeniu, zapobiegajac natychmiastowemu reconnect loop

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

---

## Historia wersji

### v1.4 (najnowsza)

**Poprawki krytyczne:**
- SPI race condition — rysowanie ISS przeniesione do UI task na Core 1
- xQueueGenericSend crash — `requestUiScreenRedraw()` zamiast `drawScreen()`
- WiFi circuit breaker — dodano `WiFi.disconnect()` + `WiFi.begin()`
- PSK Reporter crash — globalny `WiFiClientSecure` z try/catch

**Stabilnosc sieci:**
- client.stop() + timeout 5000ms do WSZYSTKICH funkcji HTTP
- HTTPS mutex — serializacja zadan TLS miedzy Core 0 a Core 1
- Heap guards — skip fetch przy malo pamieci
- Auto-restart przy krytycznie malo pamieci

**ISS:**
- Predykcja przelotow n2yo.com z AOS/LOS/MAX EL
- SGP4 fallback gdy n2yo nedostepne
- Flagi krajow (16bpp BMP) z drawBmp16FromFS()
- Menu ISS — restart danych z poziomu ekranu
- Mapa swiata z pozycja ISS i odlegloscia do QTH

**Web UI:**
- Tlumaczenie EN/PL (200+ wpisow) z localStorage
- Kolejnosc ekranow — 14 slotow z nazwami
- Dynamiczne stringi — I18N_JS dictionary

**LittleFS:**
- Partycja 2304KB — 90 flag (16bpp), splash, mapa, fonty
- BMP failure cache 60s
- DX/POTA cooldown 10s

---

### v1.3

**Naprawa kolejnosci ekranow TFT:**
- Usunieto blad nadpisujacy ustawienia uzytkownika domyslna kolejnoscia przy restarcie
- Dodano 12 slotow dla ekranow TFT w interfejsie WWW
- Zsynchronizowano typy ekranow — wszystkie 14 typow dostepnych w firmware

**Aktualizacja dokumentacji i licencji:**
- Dodano plik LICENSE z pelna trescia licencji MIT
- Poprawiono autora — Konrad Wisniewski SP3KON
- Zaktualizowano instrukcje

---

### v1.2b

**Rozszerzony PSK Reporter:**
- Filtrowanie spotow po pasmie, trybie, znaku odbiornika/nadawcy
- Interfejs dotykowy na ekranie TFT — menu, klawiatura ekranowa, selektory
- Zapis ustawien do pamieci NVS (nieulotnej)
- Automatyczne odswiezanie z konfigurowalnym interwalem

**Informacje systemowe w WWW:**
- Adres IP, SSID, RSSI
- Wolna pamiec RAM i LittleFS
- Temperatura ESP32 i napiecie baterii
- Uptime systemu i wersja firmware

**Interfejs dotykowy TFT:**
- Przycisk menu (≡) na mapie PSK
- Klawiatura ekranowa dla wprowadzania znakow i liczb
- Selektory pasm (160m-6m) i trybow (FT8, FT4, CW, itp.)
- Potwierdzenie zapisu ustawien

**PSK Reporter backend:**
- Dynamiczny URL API z parametrami
- Obsluga filtrow (znak, pasmo, tryb, max spots)
- Okno czasowe — X godzin lub Y dni wstecz
- Automatyczny timer odswiezania

---

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
