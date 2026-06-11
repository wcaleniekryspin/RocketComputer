#ifndef RAKIETA_H
#define RAKIETA_H

#include <sys/_stdint.h>
#include <stdlib.h>
#include <cstring>
#include <cstdio>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_FlashTransport.h>

#include <BitStorage.h>
#include "config.h"


class Rakieta
{
private:
  // === Zmienne stanu ===
  enum class SystemMode {
    DEBUG = 0,
    FLIGHT = 1,
    DUMP = 2,
    SLEEP = 3
  } currentMode;

  enum class FlightState {
    IDLE = 0,
    BOOST = 1,
    COAST = 2,
    APOGEE = 3,
    DESCENT = 4,
    LANDED = 5
  } currentFlightState;

  enum class ParachuteType { DROGUE, MAIN };

  struct {
    struct {
      uint8_t valid;
      float lat;
      float lng;
      float alti;
    } gps;
    struct {
      uint8_t valid;
      float ax;
      float ay;
      float az;
      float gx;
      float gy;
      float gz;
    } lsm;
    struct {
      uint8_t valid;
      float ax;
      float ay;
      float az;
    } adxl;
    struct {
      uint8_t valid;
      float alti;
    } bmp1, bmp2;
  } offsets;

  struct {
    struct {
      float lat;
      float lng;
      float alti;
      uint8_t h;
      uint8_t m;
      uint8_t s;
      uint8_t centi;
      float speed;
      float course;
      uint8_t satNum;
      uint8_t hdop;
      float maxAlti;
    } gps;
    struct {
      float ax;
      float ay;
      float az;
      float gx;
      float gy;
      float gz;
      float temp;
      float lastTotalAlti;
      float lastTotalSpeed;
      float lastTotalAccel;
      uint32_t lastTime;
    } lsm;
    struct {
      float ax;
      float ay;
      float az;
      float lastTotalAlti;
      float lastTotalSpeed;
      float lastTotalAccel;
      uint32_t lastTime;
    } adxl;
    struct {
      float pressure;
      float altitude;
      float temp;
      float lastAltitude;
      float lastVerticalSpeed;
      float maxAltitude;
      uint32_t lastTime;
    } bmp1, bmp2;
    struct {
      float voltage;
    } battery;
    struct {
      float ax;
      float ay;
      float az;
      float gx;
      float gy;
      float gz;
      float alti;
      float speed;
      float accel;
      float maxAlti;
      float roll;
      float pitch;
    } filtered;
  } data;

  // === Operacyjne ===
  bool offsetsSet;
  bool drogueDeployed;
  bool mainDeployed;
  uint16_t errorFlags;

  // === Urządzenia On/Off ===
  bool led1IsOn;
  uint32_t led1OffTime;
  bool led2IsOn;
  uint32_t led2OffTime;
  bool buzzerIsOn;
  uint32_t buzzerOffTime;
  bool solenoid1IsOn;
  uint32_t solenoid1OffTime;
  bool solenoid2IsOn;
  uint32_t solenoid2OffTime;

  // Czasy dla detekcji
  uint32_t launchDetectTime;
  uint32_t burnoutDetectTime;
  uint32_t apogeeDetectTime;
  uint32_t descentDetectTime;
  uint32_t landedDetectTime;

  // === Obiekty SPI ===
  SPIClass spiFlash;  // SPI1: W25Q128
  SPIClass spiLora;   // SPI3: SX1262
  SPIClass spiFast;   // SPI4: LSM, ADXL375, BMP388

  // === Czujniki i peryferia ===
  Adafruit_LSM6DS lsm;
  Adafruit_BMP3XX bmp1;
  Adafruit_BMP3XX bmp2;
  Adafruit_ADXL343 adxl;
  TinyGPSPlus gps;
  HardwareSerial gpsSerial;

  // === LoRa ===
  static volatile bool operationDone;
  uint32_t packet;
  SPISettings loraSettings;
  Module loraModule;
  SX1262 lora;
  BitStorage message;

  // === Flash ===
  uint8_t summarySend;
  uint16_t flashWriteCount;
  // String currentFileName;
  File32 flashDataFile;
  File32 dumpFile;
  Adafruit_FlashTransport_SPI flashTransport;
  Adafruit_SPIFlash flash;
  FatVolume fatfs;

  // === Zarządzanie czasem ===
  uint32_t lastFlightModeLoop;

  void initGPIO();  // Inicjalizacja pinów GPIO, diod, buzzerów, solenoidów, DIP switch
  void initSPI();  // Inicjalizacja trzech magistral SPI (flash, LoRa, sensory)
  bool initLSM();  // Inicjalizacja czujnika LSM6DS (akcelerometr + żyroskop)
  bool initADXL();  // Inicjalizacja czujnika ADXL375 (akcelerometr)
  bool initBMP1();  // Inicjalizacja czujnika BMP388 (BAROMETR)
  bool initBMP2();  // Inicjalizacja czujnika BMP388 (BAROMETR)
  bool initGPS();  // Inicjalizacja GPS przez UART

  void updateLeds(const uint32_t);  // Wyłącza diody po upływie czasu (nieblokujące)
  void updateBuzzer(const uint32_t);  // Wyłącza buzzera po upływie czasu (nieblokujące)
  void updateSolenoid(const uint32_t);  // Wyłącza solenoidu po upływie czasu (nieblokujące)

  void systemReset();  // Sprzętowy reset mikrokontrolera
  void systemSleep(const uint32_t);  // Uśpienie systemu na podany czas
  void setSystemMode();  // Odczyt stanu DIP switch i ustawienie trybu pracy (DEBUG/FLIGHT/DUMP/SLEEP)
  void printSystemMode() const;  // Wypisanie aktualnego trybu systemowego przez Serial
  void printFlightMode() const;  // Wypisanie aktualnego trybu lotu przez Serial

  void handleBattery();  // Odczyt napięcia baterii
  void handleLsm();  // Odczyt i walidacja danych z LSM6DS
  void handleAdxl();  // Odczyt i walidacja danych z ADXL375
  void handleBmp1();  // Odczyt i walidacja danych z BMP388
  void handleBmp2();  // Odczyt i walidacja danych z BMP388
  void handleGPS();  // Odczyt i parsowanie danych z GPS
  void readSensorsData();  // Zbiorczy odczyt wszystkich sensorów

  void printData() const;  // Wypisanie wszystkich danych przez Serial
  void resetOffsets();  // Resetowanie offsetów dla sensorów i GPS
  void setOffsets();  // Wyznaczenie offsetów dla sensorów i GPS
  void prepareGpsOffset(char*, const size_t);  // Przygotowanie komunikatu z offsetami GPS
  void prepareOffsetsMsg(char*, const size_t);  // Przygotowanie komunikatu z offsetami
  void prepareDataLineMsg(char*, const size_t);  // Przygotowanie linii danych telemetrycznych (CSV) do zapisu/flash/LoRa
  void filterGyro();  // Filtracja dolnoprzepustowa danych z żyroskopu
  void calculateOrientation();  // Obliczenie kąta nachylenia (pitch) i przechylenia (roll) z przefiltrowanego przyspieszenia
  void filterAccel();  // Fuzja danych akcelerometrów z LSM i ADXL
  void filterSpeed();  // Fuzja prędkości z BMP, LSM, ADXL i GPS
  void filterAlti();  // Fuzja wysokości z BMP, LSM, ADXL i GPS

  bool initLora();  // Inicjalizacja modułu LoRa (SX1262)
  static void setOperationFlag();  // Flaga ustawiana przez przerwanie DIO1 (koniec nadawania/odbioru)
  void prepareLoraStatusMsg(char*, const size_t);  // Przygotowanie statusu LoRa do debugu
  void preparePacket();  // Zapisanie wszystkich danych do pakietu binarnego (BitStorage)
  void sendPacket();  // Wysłanie pakietu przez LoRa
  void transmit(const uint8_t*, const size_t);  // Transmisja bufora przez LoRa
  void transmit(const char*, const size_t);  // Transmisja bufora przez LoRa
  void startListening();  // Przełączenie LoRa w tryb nasłuchiwania
  void handleCommand(const char*);  // Parsowanie odebranej komendy i wykonanie akcji
  void checkRadio();  // Sprawdzenie, czy przyszła nowa wiadomość LoRa

  bool initFlash();  // Inicjalizacja zewnętrznej pamięci Flash (W25Q128)
  bool flashFindNextFileNumber(char*, size_t);  // Znalezienie wolnego numeru pliku "data_XXXX.csv"
  uint16_t findMaxFileNumber();  // Znalezienie ostatniego numeru pliku
  bool flashOpenNewFile();  // Otwarcie nowego pliku do zapisu danych
  void flashWriteString(const char*);  // Zapisanie ciągu znaków do pliku na Flash
  void writeLogToFlash(const char*);
  void flashFlushBuffer();  // Wymuszenie opróżnienia bufora zapisu
  void flashCloseFile();  // Zamknięcie pliku danych
  void flashDumpFileList();  // Wypisanie listy plików .csv na konsolę
  void flashDumpFileData(const uint16_t);  // Wypisanie zawartości wskazanego pliku
  void flashDumpLastFile();  // Wypisanie ostatniego pliku danych

  bool detectLaunch();  // Detekcja startu
  bool detectBurnout();  // Detekcja swobodnego wznoszenia
  bool detectApogee();  // Detekcja apogeum
  bool detectLanding();  // Detekcja uderzenia w ziemię
  bool checkDeploymentConditions(const ParachuteType) const;  // Sprawdzenie warunków do otwarcia spadochronu

  void sendFlightSummary();  // Wysłanie podsumowania faz lotu przez LoRa i do flash
  void drogueParashuteOpen();  // Otwarcie spadochronu drogue
  void mainParashuteOpen();  // Otwarcie spadochronu głównego
  void updateFlightState();  // Automat maszyny stanów lotu (IDLE -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED)

  void initWatchdog();  // Inicjalizacja niezależnego watchdog (IWDG)
  void watchdog();  // Odświeżenie watchdog
  void reinitComponent(bool (Rakieta::*initFunc)());  // Próba reinicjalizacji uszkodzonego komponentu
  void handleErrors();  // Cykliczna obsługa błędów
  void handleSerialCommands();  // Odczytkomend z Serial

  void handleDebugMode();  // Główna pętla dla trybu DEBUG
  void handleFlightMode();  // Główna pętla dla trybu FLIGHT
  void handleDumpMode(const uint32_t);  // Główna pętla dla trybu DUMP
  void handleSleepMode(const uint32_t);  // Główna pętla dla trybu SLEEP + możliwość wybudzenia przez DIP switch
  void handleMode(const uint32_t);  // Dystrybutor wywołujący odpowiednią funkcję `handleXXXMode()` z odpowiednim interwałem

public:
  Rakieta();
  ~Rakieta();
  
  void init();  // Inicjalizacja całego systemu
  void loop();  // Główna pętla programu
};


#endif  // RAKIETA_H