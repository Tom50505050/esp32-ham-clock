# Skrypt pobierania LittleFS z ESP32-HAM-CLOCK
# Użycie: .\download_littlefs.ps1 [COM_PORT]
# Domyślnie: COM7

# Ustaw ścieżkę do Pythona z esptool
$PYTHON_ESPTOOL = "C:\Users\tomas\scoop\apps\python313\current\python.exe"

param(
    [string]$Port = "COM7"
)

Write-Host "=== Pobieranie LittleFS z ESP32-HAM-CLOCK ===" -ForegroundColor Green
Write-Host "Port: $Port"
Write-Host ""

# Sprawdź czy platformio jest zainstalowane
$pio = Get-Command pio -ErrorAction SilentlyContinue
if (-not $pio) {
    Write-Host "ERROR: PlatformIO CLI nie jest zainstalowane!" -ForegroundColor Red
    Write-Host "Zainstaluj: pip install platformio" -ForegroundColor Yellow
    exit 1
}

Write-Host "1. Pobieranie obrazu LittleFS z ESP32 ($Port)..." -ForegroundColor Cyan
Write-Host "   Upewnij się, że ESP32 jest podłączone" -ForegroundColor Yellow
Write-Host ""

# Metoda 1: PlatformIO (zalecana)
$pioSuccess = $true
try {
    pio run --target downloadfs --environment esp32dev --upload-port $Port
    if ($LASTEXITCODE -ne 0) {
        $pioSuccess = $false
    }
} catch {
    $pioSuccess = $false
}

if (-not $pioSuccess) {
    Write-Host "PlatformIO nie powiodło się, próbuję esptool..." -ForegroundColor Yellow
    Write-Host ""

    # Metoda 2: esptool (fallback)
    $outputDir = "build"
    if (-not (Test-Path $outputDir)) {
        New-Item -ItemType Directory -Path $outputDir | Out-Null
    }

    # Offset i rozmiar z partitions.csv: spiffs @ 0x1C0000, size 0x240000
    & $PYTHON_ESPTOOL -m esptool --port $Port --baud 115200 read-flash 0x1C0000 0x240000 "$outputDir\littlefs_backup.bin"

    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Pobieranie LittleFS nie powiodło się!" -ForegroundColor Red
        exit 1
    }
}

Write-Host ""
Write-Host "=== SUKCES! LittleFS pobrane ===" -ForegroundColor Green

# Kopiuj do folderu głównego projektu
$mainDir = "."
if ($pioSuccess) {
    # PlatformIO zapisuje w build/, skopiuj stamtąd
    $sourceFile = "build\littlefs.bin"
    if (Test-Path $sourceFile) {
        Copy-Item $sourceFile "$mainDir\littlefs.bin" -Force
        Write-Host "Skopiowano do: $mainDir\littlefs.bin" -ForegroundColor Cyan
    }
} else {
    # esptool zapisał jako littlefs_backup.bin
    $sourceFile = "build\littlefs_backup.bin"
    if (Test-Path $sourceFile) {
        Copy-Item $sourceFile "$mainDir\littlefs.bin" -Force
        Write-Host "Skopiowano do: $mainDir\littlefs.bin" -ForegroundColor Cyan
    }
}

# Kopiuj do folderu clon
$clonDir = "clon"
if (-not (Test-Path $clonDir)) {
    New-Item -ItemType Directory -Path $clonDir | Out-Null
    Write-Host "Utworzono folder: $clonDir" -ForegroundColor Cyan
}

if (Test-Path "$mainDir\littlefs.bin") {
    Copy-Item "$mainDir\littlefs.bin" "$clonDir\littlefs.bin" -Force
    Write-Host "Skopiowano do: $clonDir\littlefs.bin" -ForegroundColor Cyan
}

Write-Host ""
Write-Host "Pliki littlefs.bin zapisane w:" -ForegroundColor Green
Write-Host "  - $mainDir\littlefs.bin" -ForegroundColor White
Write-Host "  - $clonDir\littlefs.bin" -ForegroundColor White
