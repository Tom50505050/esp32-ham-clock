# Zmiany w projekcie ESP32-HAM-CLOCK (modyfikacje oryginału SP3KON)

## Wersja firmware: 1.4

---

## ZMIANY v1.4 vs v1.3

### 1. POPRAWKI KRYTYCZNE (crash / zawieszanie)

#### 1.1 SPI race condition
`updateIssDynamicDisplay()` wywoływana z Core 0 (main loop) kolidowała z UI task na Core 1 po magistrali SPI. Przeniesiono rysowanie ISS do flagi `uiPendingIssDynamicRedraw` obsługiwanej przez UI task. Eliminuje zawieszanie ekranów po kilku minutach pracy.

#### 1.2 xQueueGenericSend crash
`handleScreenIssMenuTouch()` wywoływał `drawScreen()` z Core 0, powodując deadlock kolejki UI task. Zamieniono na `requestUiScreenRedraw()` które ustawia flagę bezpośrednio.

#### 1.3 WiFi circuit breaker
Po przekroczeniu progu `HTTPS_FAIL_THRESHOLD` resetował WiFi bez `WiFi.disconnect()` + `WiFi.begin()`, przez co ESP32 nie mógł się połączyć ponownie. Dodano `wifiConnected = false` + wymuszenie reconnect.

#### 1.4 PSK Reporter crash
Globalny `WiFiClientSecure` bez try/catch przy `loadCACert` — dodano obsługę wyjątków.

---

### 2. STABILNOŚĆ SIECI

#### 2.1 Weather / Propagation / Forecast HTTP -1
`static WiFiClientSecure` nie wywoływał `client.stop()` przed kolejnym żądaniem, zostawały stany TLS powodujące błędy -1. Dodano `client.stop()` + timeout 5000ms do WSZYSTKICH funkcji HTTP:
- `fetchWeatherData()`
- `fetchWeatherForecast()`
- `fetchAirPollutionData()`
- `fetchPropagationData()`
- `fetchCallookCallsignInfo()`
- `fetchPotaApi()`
- `ensureQrzSession()`

#### 2.2 HTTPS mutex (splot)
`SemaphoreHandle_t httpsMutex` tworzony w `setup()` serializuje wszystkie żądania HTTPS, eliminując race condition na poziomie TLS/WiFi między Core 0 a Core 1.

#### 2.3 ISS task na Core 1
`issTask()` działa na Core 1, nie wywołuje już `httpsFailCount++`, więc circuit breaker nie jest blokowany przez ISS.

#### 2.4 Heap guards
- Skip fetch Weather/Propagation jeśli `ESP.getFreeHeap() < 50000`
- Skip Forecast/Air jeśli `< 40000`
- Skip Callook/POTA jeśli `< 50000`, timeout 3-8s

#### 2.5 Auto-restart
Restart jeśli `ESP.getFreeHeap() < 30000` i `loopCounter > 100`.

---

### 3. ISS (International Space Station)

#### 3.1 Predykcja przelotów n2yo.com
Następny przelot z czasem AOS/LOS i MAX EL. Format URL `/5?apiKey=`, fallback na `radiopresses` jeśli visual zwraca 0.

#### 3.2 SGP4 fallback
Jeśli n2yo nie ma danych, SGP4 oblicza najlepszy pass (`bestPeakEl`/`bestAosT`). Zmienna `issNextPassFromN2yo` chroni przed nadpisaniem dobrych danych n2yo przez SGP4.

#### 3.3 Flagi krajów (16bpp BMP)
`drawBmpFromFS()` obsługiwał tylko 24-bit BMP, a flagi w `/flags/` są 16-bit (822B). Dodano `drawBmp16FromFS()` z obsługą ujemnej wysokości (top-down BMP) i walidacją wymiarów. `updateIssDynamicDisplay()` próbuje najpierw 16-bit, potem fallback na 24-bit.

#### 3.4 ISS menu
Hamburger (5,7) w `drawIssStaticInterface()` z opcjami "RESTART ISS DATA" i "CLOSE". Resetuje timery i ustawia `issForceRefresh = true`.

#### 3.5 Mapa świata
Pozycja ISS na mapie z kalkulacją odległości do QTH.

#### 3.6 Max EL display
"MAX EL: XX.X" w kolorze CYAN na ekranie ISS, zmienna globalna `issNextPassEstimateMaxEl`.

#### 3.7 Startup delay
15s zamiast dłuższego, natychmiastowy fetch n2yo po starcie.

---

### 4. WEB UI

#### 4.1 EN/PL i18n
Słownik `I18N` (~200+ wpisów PL->EN) dla statycznego HTML + `I18N_JS` dla dynamicznych stringów. Funkcja `setLang('en'/'pl')` używa DOM tree walker. Zapis w `localStorage('hamclock_lang')`, przywracany przy starcie strony.

#### 4.2 EN/PL buttons
Zmieniono z `<a href="/indexEN.html">` na `onclick="setLang('pl'/'en')"` z klasami `.pl-btn`/`.en-btn`.

#### 4.3 Screen order labels
`updateScreenSlotLabels()` synchronizuje selecty `screen_slot_1..14` z `SCREEN_NAME_MAP` i `onchange` handlers.

#### 4.4 Dynamiczne stringi
Funkcja `t()` owija JS-generated text przez `I18N_JS` dictionary.

---

### 5. LittleFS

#### 5.1 Partycja 2304KB
Zmniejszono liczbę flag z 254 do 90 (16bpp BMP), usunięty `splash.png`. Zajęte ~1778KB, wolne ~582KB.

#### 5.2 BMP failure cache
`drawBmpFromFS()` cache'uje nieudane ścieżki przez 60s, zapobiegając spamowi logów.

#### 5.3 DX/POTA cluster reconnect cooldown
10s cooldown po rozłączeniu, zapobiegając natychmiastowemu reconnect loop.

---

## Wersja firmware: 1.2b

---

## INFORMACJE O PROJEKCIE

### Autor oryginalnego projektu
**Krzysztof Błaszczyk, SP3KON**
- Oryginalny projekt: https://github.com/SP3KON/ESP32-HAM-CLOCK
- Projekt bazowy zawiera funkcjonalność: zegar, kalendarz, fazy księżyca, dane DX Cluster, APRS, POTA, propagacja, itp.

### Autor modyfikacji
**Modyfikacje dodane w tej wersji:**
- Rozszerzona obsługa PSK Reporter z filtrowaniem
- Interfejs dotykowy dla ekranu TFT
- Dodatkowe informacje systemowe w interfejsie web

### Licencja
**MIT License**

Projekt jest objęty licencją MIT, co oznacza że możesz:
- Używać go do celów prywatnych i komercyjnych
- Modyfikować i rozpowszechniać kod
- Tworzyć na jego podstawie własne projekty

Warunki:
- Zachowaj informację o autorze oryginalnym (SP3KON)
- Dołącz treść licencji MIT do swoich kopii kodu

Treść licencji MIT:
```
Copyright (c) 2024 Krzysztof Błaszczyk SP3KON
Copyright (c) 2024 Modyfikacje: [nazwa modyfikatora]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
```

---

## 1. INTERFEJS WEB (WWW)

### 1.1 Nowa zakładka "System"
Dodano zakładkę z informacjami o systemie:
- **Adres IP** urządzenia
- **Nazwa sieci WiFi** (SSID)
- **Szybkość połączenia** (RSSI w dBm)
- **Wolna pamięć RAM** (heap w kB)
- **Użycie pamięci LittleFS** (wolna/zajęta w kB)
- **Częstotliwość CPU** (MHz)
- **Temperatura ESP32** (°C)
- **Napięcie baterii** (V i % naładowania)
- **Wersja firmware** (1.2b)
- **Czas pracy urządzenia** (uptime: dni, godziny, minuty)

### 1.2 Rozszerzone ustawienia PSKReporter
Dodano nowe pola konfiguracyjne:
- **Znak do monitorowania** - znak nadawcy (stations) do śledzenia
- **Raport od X dni** - zakres danych (0-62 dni)
- **Automatyczne odświeżanie** - interwał (5-60 minut)

### 1.3 Pobieranie firmware
Dodano przycisk **"Pobierz firmware"** w zakładce System dla łatwej aktualizacji.

---

## 2. EKRAN TFT - PSK REPORTER MAP

### 2.1 Przycisk menu (≡)
W lewym górnym rogu ekranu PSK Reporter Map dodano przycisk menu (hamburger).

### 2.2 Menu filtrów PSK (dotykowe)
Po dotknięciu przycisku menu otwiera się panel z filtrami:
- **Znak** (receiver callsign) - filtr dla stacji odbiorczej
- **Pasmo** - wybór pasma (160m, 80m, 40m, 30m, 20m, 17m, 15m, 12m, 10m, 6m, ALL)
- **Tryb** - wybór trybu (FT8, FT4, JS8, PSK31, PSK63, RTTY, CW, SSB, ALL)
- **Max spots** - maksymalna liczba spotów (10-200)

### 2.3 Klawiatura ekranowa
Dla pól **Znak** i **Max** dodano klawiaturę ekranową z dotykiem:
- Litery A-Z (układ QWERTY, 3 wiersze)
- Cyfry 0-9
- Znaki specjalne: / (slash), - (minus), spacja
- Backspace (←)
- OK - zatwierdź wprowadzone dane
- X (Anuluj) - zamknij klawiaturę bez zapisywania

### 2.4 Selektor pasm
Po dotknięciu pola **Pasmo** lub **Tryb** otwiera się menu z kafelkami wyboru:
- 11 kafelków z pasma amatorskimi
- Zaznaczenie na zielono wybranego pasma
- Przycisk "ZAMKNIJ" - powrót do głównego menu

### 2.5 Selektor trybów
Podobnie jak dla pasm - 9 kafelków z trybami cyfrowymi i analogowymi.

### 2.6 Zapisywanie ustawień
- Przycisk **ZAPISZ** - zapisuje filtry do pamięci NVS (nieulotnej)
- Przycisk **ANULUJ** - zamyka menu bez zapisywania
- Potwierdzenie **"ZAPISANO!"** na ekranie po zapisaniu

### 2.7 Automatyczne odświeżanie mapy
Po zapisaniu ustawień mapa automatycznie odświeża się z nowymi filtrami.

---

## 3. BACKEND / LOGIKA PSK REPORTER

### 3.1 Dynamiczny URL API
URL do PSKReporter jest budowany dynamicznie z parametrami:
```
https://retrieve.pskreporter.info/query?flowStartSeconds=-X&maxrows=Y&receiverCallsign=Z&...
```

### 3.2 Obsługa filtrów
- **Filtr znaka odbiornika** - pokazuje tylko spoty dla konkretnego odbiornika
- **Filtr znaka nadawcy** - monitorowanie konkretnej stacji (transmitter)
- **Filtr pasma** - filtrowanie po paśmie (np. 20m)
- **Filtr trybu** - filtrowanie po trybie (np. FT8)
- **Max spots** - limit liczby spotów (domyślnie 50)

### 3.3 Okno czasowe
- **Standardowe** - X godzin wstecz (konfigurowalne)
- **Raport dniowy** - Y dni wstecz (0-62 dni)

### 3.4 Automatyczny timer
Dodano timer odświeżający dane PSK Reporter co skonfigurowany interwał (5-60 minut).

---

## 4. ZMIANY W KODZIE

### 4.1 Nowe zmienne globalne
```cpp
String pskMonitorCallsign;      // Znak do monitorowania
int pskReportDays;              // Liczba dni raportu (0-62)
int pskAutoRefreshMinutes;      // Interwał odświeżania (5-60)
String pskCustomUrl;            // Własny URL API

// Menu PSK Map
bool pskMapMenuOpen;
String pskTempReceiver, pskTempBand, pskTempMode;
int pskTempMaxSpots;
String pskKeyboardBuffer;
int pskKeyboardTarget;
enum PskMenuField { ... };
```

### 4.2 Nowe funkcje
```cpp
void drawPskMenuButton();           // Rysuje przycisk menu (≡)
void drawPskSettingsMenu();       // Rysuje panel filtrów
void drawPskKeyboard();            // Rysuje klawiaturę ekranową
void drawPskBandSelector();       // Rysuje selektor pasm
void drawPskModeSelector();       // Rysuje selektor trybów
void handlePskKeyboardTouch();    // Obsługa dotyku klawiatury
void handlePskBandSelectorTouch(); // Obsługa wyboru pasm
void handlePskModeSelectorTouch(); // Obsługa wyboru trybów
void handlePskMapTouch();         // Główna obsługa dotyku
```

### 4.3 Modyfikacje API REST
Dodano nowe endpointy i pola JSON:
- `psk_monitor` - znak monitorowany
- `psk_report_days` - dni raportu
- `psk_autorefresh` - minuty odświeżania

---

## 5. NAPRAWIONE BŁĘDY

### 5.1 Brakujące deklaracje funkcji
Dodano forward declaration dla `handlePskMapTouch()` na początku pliku.

### 5.2 Routing dotyku
Dodano wywołanie `handlePskMapTouch()` w głównej funkcji obsługi dotyku `handleTouchNavigation()`.

### 5.3 Wyśrodkowanie nagłówka
Napis "PSK Reporter Map" w nagłówku ekranu jest teraz wyśrodkowany.

---

## 6. PLIKI KONFIGURACYJNE

### 6.1 platformio.ini
Bez zmian - używany standardowy env `esp32dev`.

### 6.2 data/index.html
Rozszerzony o:
- Sekcję System z informacjami o sprzęcie
- Dodatkowe pola PSK Reporter (monitorowanie, dni, odświeżanie)
- JavaScript do wyświetlania uptime i dynamicznych pól

---

## PODSUMOWANIE

Projekt został rozbudowany o:
1. **Zaawansowane filtry PSK Reporter** z interfejsem dotykowym
2. **Klawiaturę ekranową** dla wygodnego wprowadzania danych
3. **Informacje systemowe** w interfejsie web
4. **Automatyczne odświeżanie** danych z konfigurowalnym interwałem
5. **Lepszą obsługę pamięci NVS** dla trwałego zapisywania ustawień

Wszystkie zmiany są kompatybilne wstecz z oryginalnym projektem SP3KON.
