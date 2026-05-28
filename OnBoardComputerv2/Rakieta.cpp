#include "Rakieta.h"


/*
  1. Ograniczone użycie const   -    W wielu metodach, które nie modyfikują stanu obiektu, brakuje deklaracji const. Na przykład printData(), prepareDataLineMsg(), prepareOffsetsMsg(), sendFlightSummary() – wszystkie powinny być const, ponieważ tylko odczytują dane. Brak const utrudnia kompilatorowi optymalizacje i wprowadza w błąd programistę co do intencji funkcji. W systemie krytycznym każda metoda, która nie zmienia obiektu, powinna być jawnie oznaczona jako const.
  2. Użycie String   -    Kod intensywnie używa String do budowania komunikatów (np. prepareDataLineMsg(), prepareOffsetsMsg(), sendFlightSummary(), handleCommand()). Klasa String z Arduino dynamicznie alokuje pamięć na stercie, co jest zabronione w standardach MISRA/JSF. W locie może to prowadzić do fragmentacji, nieprzewidywalnych opóźnień lub wyczerpania pamięci. Wszystkie operacje na napisach powinny być zastąpione statycznymi buforami char[] i snprintf().
  4. Brak pełnego użycia constexpr   -    W config.h wszystkie stałe zdefiniowano za pomocą #define, a w Rakieta.cpp nie ma ani jednego constexpr. Makra preprocesora nie mają typu, nie podlegają sprawdzeniu składniowemu i nie są widoczne w debuggerze. Zamiast #define FREQUENCY 868 powinno być constexpr int FREQUENCY = 868. constexpr daje typowanie i możliwość sprawdzenia stałych w czasie kompilacji.
  5. Ograniczone użycie list inicjalizacyjnych   -    W konstruktorze Rakieta::Rakieta() lista inicjalizacyjna jest długa, ale niektóre pola (np. errorFlags) nie są na niej umieszczone, mimo że mają być inicjalizowane. Co prawda errorFlags ma inicjalizator w klasie (uint16_t errorFlags = 0), co jest dobre, ale w innych miejscach (np. operationDone jako zmienna globalna) brakuje inicjalizacji. W systemie krytycznym preferuje się inicjalizację na liście konstruktora zamiast późniejszego przypisania.
  23. Ograniczone użycie semantic typing (silne typowanie)   -    Stany lotu (IDLE, BOOST, ...) są zdefiniowane jako zwykły enum, a nie enum class. Przez to mogą być niejawnie konwertowane na int, co może prowadzić do błędów (np. przypisanie wartości spoza zakresu). enum class zapewnia bezpieczeństwo typów.
  8. Brak pełnej centralizacji maszyny stanów (FSM)   -    System ma dwa poziomy stanów: SystemMode (DEBUG, FLIGHT, DUMP, SLEEP) oraz FlightState (IDLE, BOOST, COAST, ...). Logika przejść jest rozproszona: w loop() jest switch dla trybów systemowych, w updateFlightState() switch dla stanów lotu, a dodatkowo osobne funkcje detectLaunch(), detectBurnout() itp. Brak jednej, spójnej maszyny stanów z jawnymi przejściami. W systemie krytycznym wszystkie stany i przejścia powinny być widoczne w jednym miejscu.
  13. Brak architektury watchdog   -    W kodzie zdefiniowano funkcje initWatchdog() i watchdog(), ale nigdzie nie są one wywołane – brak inicjalizacji watchdog w init() i brak okresowego odświeżania w pętli głównej. Watchdog jest podstawowym mechanizmem wykrywania zawieszenia programu w systemach krytycznych. Bez niego, jeśli program wejdzie w nieskończoną pętlę lub zablokuje się na operacji I/O, system nigdy się nie zresetuje.
  19. Brak jednolitego modelu obsługi błędów   -    Błędy są obsługiwane lokalnie poprzez ustawienie bitów w errorFlags, ale nie ma centralnego mechanizmu, który agreguje błędy. Różne funkcje wypisują komunikaty o błędach na Serial, ale nie ma spójnego logowania ani reakcji.
  11. Ograniczone programowanie defensywne   -    Mimo że kod sprawdza, czy komunikacja z sensorami się powiodła (np. if(lsm.getEvent(...))), brakuje walidacji zakresów odczytanych wartości. Przykład: w handleBmp() ciśnienie może wynieść 0 hPa lub ujemną wysokość – nie jest to sprawdzane. W handleLsm() przyspieszenie może przekroczyć zakres ±16g z powodu błędu. System powinien traktować każdą wartość z sensora jako potencjalnie błędną i wykonywać sanity check (np. if (abs(ax) > 20.0f) ax = 0).
  3. Magic numbers   -    W kodzie rozsiane są liczby bez nazwanych stałych: 1013.25 (w handleBmp i setOffsets), 4095.0f (w handleBattery), 50, 25, 10 (w setOffsets), 30000 (timeout), 0.5f (filtr), 1.0f (w detectApogee), 20 i 60000 (w handleFlightMode), 200, 100 itp. Utrudnia to strojenie i zrozumienie logiki. Każda taka liczba powinna być zdefiniowana jako constexpr z opisową nazwą.
  22. Brak pełnej izolacji konfiguracji   -    Mimo że istnieje config.h, niektóre parametry są wciąż rozproszone: 1013.25 (ciśnienie referencyjne), 4095.0f (zakres ADC), 9600 (baudrate GPS), 0x76 (adres I2C – brak go w config.h?). W Rakieta.cpp pojawiają się też liczby jak 0.5f dla filtru, 1.0f dla detekcji apogeum. Wszystkie parametry powinny być zebrane w jednym miejscu (config.h lub struktura konfiguracyjna).
*/

/*
  NA TERAZ (ŁATWE):

  6. Zbyt szeroki scope funkcji/danych   -    Wiele funkcji pomocniczych jest zadeklarowanych jako prywatne metody klasy Rakieta, choć mogłyby być funkcjami wolnymi static lub w anonimowej przestrzeni nazw w pliku Rakieta.cpp. Przykłady: calcChecksum (choć nie istnieje), floatToString (brak), ale też flashRecoverAfterReset(), flashDumpFileList() – nie potrzebują dostępu do prywatnych pól? Część z nich faktycznie korzysta z pól, ale wiele mogłoby być static. Zwiększa to widoczność symboli i utrudnia analizę przepływu.
  7. Niejawne konwersje typów   -    W kodzie występują niejawne konwersje, np. w handleBattery: rawValue (int) jest mnożone przez 4.2f – to konwersja na float, ale brak static_cast. W detectApogee porównanie float z float bez upewnienia się, że wartości nie są NaN. W setOffsets dodawanie do siebie wartości różnych typów (int, float). Standard JSF wymaga jawnych rzutowań (static_cast) dla wszystkich konwersji, aby uniknąć niezdefiniowanego zachowania i utraty precyzji.
  9. Funkcja loop() ma wiele odpowiedzialności   -    Rakieta::loop() nie jest bardzo długa, ale pośrednio przez wywołania handleDebugMode(), handleFlightMode() itp. zarządza całym systemem. handleFlightMode() natomiast wykonuje odczyt sensorów, aktualizację stanu lotu, przygotowanie pakietu, wysyłkę LoRa, zapis na flash – to zbyt wiele odpowiedzialności. Zgodnie z zasadą "jedna funkcja – jedna odpowiedzialność", każda z tych operacji powinna być wydzielona do osobnych funkcji.
  12. Brak pełnej architektury odzyskiwania po awarii (fault recovery)   -    Kod ustawia flagi błędów (errorFlags), ale nie definiuje jawnego bezpiecznego stanu ani trybu ograniczonej funkcjonalności. Na przykład, gdy GPS przestanie działać w locie, system kontynuuje lot według ostatnich danych, zamiast przejść w tryb awaryjny (np. nawigacja bezwładnościowa). Wymagane jest zdefiniowanie dla każdego błędu konkretnej reakcji: reset komponentu, przejście do trybu degraded, awaryjne lądowanie.
  14. Ograniczone sprawdzanie zakresów danych   -    Dane z sensorów są używane w obliczeniach bez weryfikacji, czy są fizycznie możliwe. Na przykład w detectApogee() porównuje się data.bmp1.lastVerticalSpeed <= APOGEE_VELOCITY_THRESHOLD – ale jeśli lastVerticalSpeed jest NaN lub nieskończonością, wynik będzie nieprawidłowy. Należy przed każdym użyciem sprawdzać, czy wartość jest skończona i mieści się w oczekiwanym przedziale (np. isfinite()).
  15. Brak jawnych timeoutów dla części operacji   -    Operacje odczytu z I2C, SPI czy GPS nie mają zdefiniowanych timeoutów. Na przykład gps.encode() czyta z bufora UART, ale jeśli dane przestaną napływać, program nie ma informacji o tym po czasie. W transmit() jest timeout dla LoRa, ale to wyjątek. Każda operacja, która może blokować, powinna mieć limit czasu, po którym zostaje przerwana, a komponent oznaczony jako uszkodzony.
  16. Brak separacji warstw logicznych (HAL vs. logika lotu)   -    Klasa Rakieta bezpośrednio miesza wywołania bibliotek sensorów (Adafruit_LSM6DS, TinyGPS++, SX1262) z logiką decyzyjną lotu (detekcja startu, apogeum). Utrudnia to testowanie (nie można zasymulować sensorów) i zmiany sprzętu. Wzorzec Hardware Abstraction Layer (HAL) nakazuje oddzielenie sterowania od konfiguracji i odczytu sprzętu.
  25. Ograniczone użycie compile-time validation   -    Brak static_assert dla parametrów konfiguracyjnych. Na przykład SF (spreading factor) powinien być w zakresie 6–12, ale nie jest to sprawdzone w czasie kompilacji. static_assert(SF >= 6 && SF <= 12, "SF out of range") wykryłoby błąd natychmiast, zamiast czekać na nieprawidłowe działanie LoRa w locie.
  26. Brak pełnej analizy stanów awaryjnych (FMEA)   -    Kod nie definiuje zachowania dla wielu potencjalnych awarii: co się dzieje, gdy BMP1 działa, ale BMP2 nie? Gdy GPS wysyła błędne dane (np. nagła zmiana pozycji o kilometr)? Gdy LoRa nie może nadać pakietu przez dłuższy czas? Brakuje analizy trybów uszkodzeń i efektów (FMEA) oraz zdefiniowanych reakcji dla każdego scenariusza.
  27. Brak formalnego odzyskiwania stanu (state recovery)   -    Po wykryciu błędu (np. errorFlags |= LSM_ERROR) system nie podejmuje próby przywrócenia sprawności – nie resetuje czujnika, nie ponawia inicjalizacji, nie przełącza na redundantny sensor. W lotnictwie wymaga się, aby system miał zdolność do powrotu do znanego bezpiecznego stanu po błędzie przejściowym (np. restart magistrali I2C).
  28. Ograniczone logowanie diagnostyczne   -    W trybie FLIGHT logowane są tylko dane telemetryczne (flashWriteString), ale nie loguje się błędów, zmian stanów, timeoutów ani innych zdarzeń systemowych. W przypadku katastrofy brak logów uniemożliwi analizę przyczyny. System powinien zapisywać pełną diagnostykę (z timestampem) do pamięci nieulotnej, a w trybie DEBUG wysyłać przez Serial.
  29. Zbyt duża złożoność cyklomatyczna (cyclomatic complexity)   -    Funkcje takie jak updateFlightState() mają zagnieżdżone switch i if, co prowadzi do złożoności > 15. handleFlightMode() również zawiera wiele ścieżek. Wysoka złożoność utrudnia testowanie wszystkich możliwych przejść i zwiększa ryzyko niewykrytych błędów. Należy podzielić te funkcje na mniejsze, proste jednostki.
  30. Brak formalnej separacji faz systemu   -    Fazy: startup, kalibracja, standby, lot, odzyskiwanie, awaria – nie są odseparowane architektonicznie. W kodzie warunki typu if (currentMode == FLIGHT) są rozsiane, a przejścia między fazami są ukryte w różnych funkcjach. Wzorzec "State Machine" z oddzielnymi klasami dla każdego stanu (np. class FlightState, class DebugState) ułatwia dodawanie nowych faz i zapewnia, że logika każdej fazy jest zamknięta w jednym miejscu.
  
  NA PÓŹNIEJ:
  10. Użycie delay()   -    W Rakieta.cpp wielokrotnie używane jest delay(): w setOffsets() delay(25) i delay(200), w initGPS() delay(500), w transmit() delay(1), w systemSleep() delay(time). delay() blokuje wykonanie całego programu na zadany czas, co w systemie czasu rzeczywistego jest niedopuszczalne – prowadzi do utraty danych z sensorów i braku reakcji na krytyczne zdarzenia. Należy zastąpić delay() nieblokującymi timerami opartymi na millis().
  20. Ograniczona przewidywalność czasowa   -    Użycie delay(), String, Serial.print oraz blokujących operacji I2C/SPI powoduje, że czas wykonania funkcji jest nieokreślony. W systemie czasu rzeczywistego wymagana jest analiza najgorszego czasu wykonania (WCET) i dowiedzenie, że wszystkie terminy są dotrzymane. Obecny kod nie daje żadnych gwarancji – np. handleBmp() może wykonać się szybko lub wolno w zależności od stanu czujnika.
  21. Możliwe użycie funkcji niedeterministycznych   -    millis() jest zwykle deterministyczne, ale String alokuje pamięć, co może powodować nieprzewidywalne opóźnienia. Ponadto biblioteki Adafruit mogą wewnętrznie używać delay() lub pętli oczekujących, które nie mają gwarantowanego czasu. Standard JSF wymaga eliminacji wszystkich niedeterministycznych konstrukcji, a jeśli to niemożliwe – ścisłej kontroli.
  24. Brak pełnej kontroli overflow/underflow   -    W obliczeniach całkujących (np. data.lsm.lastTotalSpeed += data.lsm.lastTotalAccel * dt) nie ma zabezpieczeń przed przekroczeniem zakresu float ani przed wartościami NaN. W systemie krytycznym po każdej operacji arytmetycznej należy sprawdzić, czy wynik jest skończony i mieści się w oczekiwanym przedziale, a w razie potrzeby nasycić wartość lub przejść w stan awaryjny.
*/

/*
  TO DO:
  -sprawdzić  "AIR VEHICLE C++ CODING STANDARDS FOR THE SYSTEM DEVELOPMENT AND DEMONSTRATION PROGRAM"
    5. Ograniczyć widoczność symboli   -   Funkcje lokalne oznaczać jako static lub umieszczać w anonymous namespace. Minimalizuje to coupling.
    10. Dodać sanity-checki   -   Walidować zakresy danych sensorów i timeouty komunikacji. To zwiększy odporność systemu.
    12. Wprowadzić fault manager   -   Stworzyć centralny system zarządzania błędami. Wszystkie moduły powinny raportować błędy w jednolity sposób.
    13. Rozdzielić warstwy systemu   -   Oddzielić logikę lotu od bezpośredniej obsługi hardware. Ułatwi to rozwój i testowanie.
    14. Dodać recovery modes   -   Zdefiniować zachowanie systemu po awarii sensora lub komunikacji. System powinien mieć jawne fail-safe behavior.
    15. Ujednolicić konfigurację   -   Wszystkie parametry systemu trzymać w jednym module konfiguracyjnym. Ułatwi to strojenie rakiety.
    18. Rozszerzyć diagnostykę   -   Logować więcej informacji o błędach i stanach systemu. Ułatwi to debugowanie lotów.
    19. Ograniczyć złożoność funkcji   -   Długie funkcje dzielić na mniejsze kroki logiczne. Poprawia to maintainability.
    20. Dodać compile-time checks   -   Używać static_assert dla krytycznych parametrów konfiguracyjnych. Pozwala to wykrywać błędy podczas kompilacji.
  
  -zrobić graficzne przedstawienie blokowe jak ma działać cały system


  -dodać funkcję sprawdzającą ile jest dostępnego miejsca na flash - jeśli jest mało powiadom użytkownika
  -dodać led_2 jako oznacznie errorów
  -w funkcjach filter dodać pozostałe czujniki ze sprawdzaniem errorów
  -zrobić funkcję handleErrors()
  -checkRadio() zapisuje dostarczoną komendę na flash

  Dokumentacja i komentarze:
      Dodaj szczegółowe komentarze do kodu, zwłaszcza w sekcjach związanych z obsługą błędów i watchdogiem.
      Zaktualizuj sekcję TO DO w kodzie na podstawie Twoich odpowiedzi.

  Obsługa błędów:
      Rozbuduj funkcję handleErrors(), aby automatycznie restartowała uszkodzone komponenty.
      Dodaj logikę do trybu FLIGHT, która sprawdza stan wszystkich komponentów na początku lotu.

  Zapis danych:
      Upewnij się, że zapis do flash jest odporny na przerwania (np. poprzez blokady).
      Rozważ kompresję danych przed zapisem, jeśli pamięć flash jest ograniczona.

  Optymalizacja energii:
      Wprowadź bardziej zaawansowane strategie oszczędzania energii (np. głębszy sleep dla nieużywanych komponentów).
      Monitoruj zużycie energii w różnych trybach.

  Testowanie:
      Stwórz scenariusze testowe w trybie DEBUG, aby przetestować różne ścieżki kodu.
      Symuluj awarie komponentów i sprawdź, czy mechanizmy naprawcze działają poprawnie.

  namespace
  {
    // Funkcje pomocnicze, które nie potrzebują dostępu do pól klasy
    float clamp(float val, float minVal, float maxVal)
    {
        return (val < minVal) ? minVal : (val > maxVal) ? maxVal : val;
    }
    
    bool isValueValid(float val, float minVal, float maxVal)
    {
        return !isnan(val) && !isinf(val) && val >= minVal && val <= maxVal;
    }
  }
*/

// === Konstruktor ===
Rakieta::Rakieta():
  // === Zmienne stanu ===
  currentMode(SystemMode::DEBUG),
  currentFlightState(FlightState::IDLE),

  // === Offsets ===
  offsets{},

  // === Data ===
  data{},
  
  // === Operacyjne ===
  offsetsSet(false),
  drogueDeployed(false),
  mainDeployed(false),
  errorFlags(LORA_ERROR | LSM_ERROR | BMP1_ERROR | BMP2_ERROR | ADXL_ERROR | GPS_ERROR | FLASH_ERROR),

  // === Urządzenia On/Off ===
  led1IsOn(false),
  led1OffTime(0),
  led2IsOn(false),
  led2OffTime(0),
  buzzerIsOn(false),
  buzzerOffTime(0),
  solenoid1IsOn(false),
  solenoid1OffTime(0),
  solenoid2IsOn(false),
  solenoid2OffTime(0),
  
  // === Dane przefiltrowane ===
  filteredAccelX(0.0f),
  filteredAccelY(0.0f),
  filteredAccelZ(0.0f),
  filteredGyroX(0.0f),
  filteredGyroY(0.0f),
  filteredGyroZ(0.0f),
  fusedAltitude(0.0f),
  prevFusedAltitude(0.0f),
  
  // === Czasy dla detekcji ===
  inFlight(false),
  launchDetectTime(0),
  burnoutDetectTime(0),
  apogeeDetectTime(0),
  descentDetectTime(0),
  landedDetectTime(0),

  // === Obiekty SPI ===
  spiFast(SPI4_SCK, SPI4_MISO, SPI4_MOSI),
  spiFlash(SPI1_SCK, SPI1_MISO, SPI1_MOSI),
  spiLora(SPI_LORA_SCK, SPI_LORA_MISO, SPI_LORA_MOSI),

  // === Czujniki i peryferia ===
  lsm(),
  bmp1(),
  bmp2(),
  adxl(CS_ADXL, &spiFast),
  gps(),
  gpsSerial(RX_MAX, TX_MAX),

  // === LoRa ===
  packet(0),
  loraMsgStartTime(0),
  operationDone(false),
  loraSettings(2000000, MSBFIRST, SPI_MODE0),
  lora(new Module(CS_LORA, DIO1_LORA, RST_LORA, BUSY_LORA, spiLora)),
  
  // === Flash ===
  flashWriteCount(0),
  dumpAddress(0),
  flashDataFile(),
  dumpFile(),
  flashTransport(CS_FLASH, &spiFlash),
  flash(&flashTransport),
  fatfs(),
  
  // === Zarządzanie czasem ===
  lastErrorCheckTime(0),
  lastFlightLoop(0),
  lastDebugPrint(0),
  lastDumpProgress(0),
  lastSleepCheck(0),
  lastFlightModeLoop(0)
{
}

Rakieta::~Rakieta()
{
  lora.clearDio1Action();

  if (flashDataFile) flashCloseFile();
}

void Rakieta::initGPIO()
{
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SOLENOID_1, OUTPUT);
  pinMode(SOLENOID_2, OUTPUT);
  pinMode(BATTERY, INPUT);
  
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(SOLENOID_1, LOW);
  digitalWrite(SOLENOID_2, LOW);
  
  Serial.println("[GPIO] OK");
  
  pinMode(MODE1, INPUT_PULLUP);
  pinMode(MODE2, INPUT_PULLUP);
  pinMode(MODE3, INPUT_PULLUP);
  pinMode(MODE4, INPUT_PULLUP);
  Serial.println("[DIP SWITCH] OK");
}

void Rakieta::initSPI()
{
  spiFlash.begin();
  Serial.println("[SPI_FLASH] OK");
  
  spiLora.begin();
  Serial.println("[SPI_LORA] OK");
  
  spiFast.begin();
  Serial.println("[SPI_CZUJNIKI] OK");
}

bool Rakieta::initLSM()
{
  if (lsm.begin_SPI(CS_LSM, &spiFast))
  {
    Serial.println("[LSM] OK");
    errorFlags &= ~LSM_ERROR;
    return true;
  }
  Serial.println("[LSM] BRAK ODPOWIEDZI");
  errorFlags |= LSM_ERROR;
  return false;
}

bool Rakieta::initADXL()
{
  if (adxl.begin())
  {
    adxl.setRange(ADXL343_RANGE_16_G);
    Serial.println("[ADXL375] OK, zakres 16G");
    errorFlags &= ~ADXL_ERROR;
    return true;
  }
  Serial.println("[ADXL375] BRAK ODPOWIEDZI");
  errorFlags |= ADXL_ERROR;
  return false;
}

bool Rakieta::initBMP1()
{
  bool ok = true;
  
  if (bmp1.begin_SPI(CS_BMP1, &spiFast))
  {
    bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp1.setOutputDataRate(BMP3_ODR_100_HZ);
    Serial.println("[BMP388 #1] OK");
    errorFlags &= ~BMP1_ERROR;
    return true;
  }
  Serial.println("[BMP388 #1] BRAK ODPOWIEDZI");
  errorFlags |= BMP1_ERROR;
  return false;
}

bool Rakieta::initBMP2()
{
  if (bmp2.begin_SPI(CS_BMP2, &spiFast))
  {
    bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp2.setOutputDataRate(BMP3_ODR_100_HZ);
    Serial.println("[BMP388 #2] OK");
    errorFlags &= ~BMP2_ERROR;
    return true;
  }
  Serial.println("[BMP388 #2] BRAK ODPOWIEDZI");
  errorFlags |= BMP2_ERROR;
  return false;
  
  return ok;
}

bool Rakieta::initGPS()
{
  gpsSerial.begin(GPS_BAUNDRATE, SERIAL_8N1);
  Serial.println("[GPS] UART2 OK");
  
  delay(500);
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }
  
  if (gps.charsProcessed() > 0)
  {
    Serial.println("[GPS] Dane odbierane");
    errorFlags &= ~GPS_ERROR;
    return true;
  }
  Serial.println("[GPS] Brak danych (sprawdź połączenie)");
  errorFlags |= GPS_ERROR;
  return false;
}

void Rakieta::updateLeds(const uint32_t now)
{
  if (led1IsOn && led1OffTime < now)
  {
    digitalWrite(LED_1, LOW);
    Serial.printf("[LED1] Updated");
  }
  if (led2IsOn && led2OffTime < now)
  {
    digitalWrite(LED_2, LOW);
    Serial.printf("[LED2] Updated");
  }
}

void Rakieta::updateBuzzer(const uint32_t now)
{
  if (buzzerIsOn && buzzerOffTime < now)
  {
    digitalWrite(BUZZER, LOW);
    Serial.printf("[BUZZER] Updated");
  }
}

void Rakieta::updateSolenoid(const uint32_t now)
{
  if (solenoid1IsOn && solenoid1OffTime < now)
  {
    digitalWrite(SOLENOID_1, LOW);
    Serial.println("[SOLENOID1] Updated");
  }
  
  if (solenoid2IsOn && solenoid2OffTime < now)
  {
    digitalWrite(SOLENOID_2, LOW);
    Serial.println("[SOLENOID2] Updated");
  }
}

void Rakieta::systemReset()
{
  lora.clearDio1Action();
  if (flashDataFile) flashCloseFile();

  NVIC_SystemReset();
}

void Rakieta::systemSleep(const uint32_t time)
{
  delay(time);
  // HAL_SuspendTick();
  // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  // HAL_Delay(time);
  // HAL_ResumeTick();
}

void Rakieta::setSystemMode()
{
  uint8_t m1 = digitalRead(MODE1) == LOW ? 0 : 1;
  uint8_t m2 = digitalRead(MODE2) == LOW ? 0 : 1;
  uint8_t m3 = digitalRead(MODE3) == LOW ? 0 : 1;
  uint8_t m4 = digitalRead(MODE4) == LOW ? 0 : 1;
  
  uint8_t modeCode = (m4 << 3) | (m3 << 2) | (m2 << 1) | m1;
  
  switch (modeCode)
  {
    case 0b0000: currentMode = SystemMode::DEBUG;  break;
    case 0b0001: currentMode = SystemMode::FLIGHT; break;
    case 0b0010: currentMode = SystemMode::DUMP;   break;
    case 0b0011: currentMode = SystemMode::SLEEP;  break;
    default:     currentMode = SystemMode::DEBUG;  break;
  }
}

void Rakieta::printSystemMode()
{
  Serial.print("[SYSTEM MODE] ");
  switch (currentMode)
  {
    case SystemMode::DEBUG:  Serial.println("DEBUG");  break;
    case SystemMode::FLIGHT: Serial.println("FLIGHT"); break;
    case SystemMode::DUMP:   Serial.println("DUMP");   break;
    case SystemMode::SLEEP:  Serial.println("SLEEP");  break;
  }
}

void Rakieta::printFlightMode()  /// prawdopodobnie do usunięcia bo nigdzie nie będzie używane
{
  Serial.print("[FLIGHT MODE] ");
  switch (currentFlightState)
  {
    case FlightState::IDLE:    Serial.println("IDLE");     break;
    case FlightState::BOOST:   Serial.println("BOOST");    break;
    case FlightState::COAST:   Serial.println("COAST");    break;
    case FlightState::APOGEE:  Serial.println("APOGEE");   break;
    case FlightState::DESCENT: Serial.println("DESCENT");  break;
    case FlightState::LANDED:  Serial.println("LANDED");   break;
  }
}

void Rakieta::handleBattery()  // tzreba sprawdzić czy odpowiednie są przeliczniki
{
  float rawValue = analogRead(BATTERY);
  data.battery.voltage = (rawValue * BATTERY_FULL_VOLTAGE / BATTERY_MAX_READ);
}

void Rakieta::handleLsm()
{
  sensors_event_t accel, gyro, temp;
  if(lsm.getEvent(&accel, &gyro, &temp))
  {
    float ax = accel.acceleration.x - offsets.lsm.ax;
    float ay = accel.acceleration.y - offsets.lsm.ay;
    float az = accel.acceleration.z - offsets.lsm.az;
    float gx = gyro.gyro.x - offsets.lsm.gx;
    float gy = gyro.gyro.y - offsets.lsm.gy;
    float gz = gyro.gyro.z - offsets.lsm.gz;

    // Sprawdź NaN i Inf
    if (isnan(ax) || isinf(ax) || isnan(ay) || isinf(ay) || isnan(az) || isinf(az) ||
        isnan(gx) || isinf(gx) || isnan(gy) || isinf(gy) || isnan(gz) || isinf(gz) ||
        fabs(ax) > MAX_ACCEL_LSM || fabs(ay) > MAX_ACCEL_LSM || fabs(az) > MAX_ACCEL_LSM ||
        fabs(gx) > MAX_GYRO || fabs(gy) > MAX_GYRO || fabs(gz) > MAX_GYRO)
    {
      errorFlags |= LSM_ERROR;
      return;
    }

    data.lsm.ax = ax;
    data.lsm.ay = ay;
    data.lsm.az = az;
    data.lsm.gx = gx;
    data.lsm.gy = gy;
    data.lsm.gz = gz;
    data.lsm.temp = temp.temperature;

    uint32_t now = millis();
    float dt = (now - data.lsm.lastTime) / 1000.0f;
    if (dt < 0.0f) dt = 0.001f;

    data.lsm.lastTotalAccel = sqrt(ax * ax + ay * ay + az * az);
    data.lsm.lastTotalSpeed += data.lsm.lastTotalAccel * dt;
    data.lsm.lastTotalAlti += data.lsm.lastTotalSpeed * dt;
    data.lsm.lastTotalRotation = data.lsm.gx + data.lsm.gy + data.lsm.gz;
    data.lsm.lastTime = now;

    errorFlags &= ~LSM_ERROR;
  }
  else
    errorFlags |= LSM_ERROR;
}

void Rakieta::handleAdxl()
{
  sensors_event_t event;
  if (adxl.getEvent(&event))
  {
    float ax = event.acceleration.x - offsets.adxl.ax;
    float ay = event.acceleration.y - offsets.adxl.ay;
    float az = event.acceleration.z - offsets.adxl.az;
    
    if (isnan(ax) || isinf(ax) || isnan(ay) || isinf(ay) || isnan(az) || isinf(az) ||
        fabs(ax) > MAX_ACCEL_ADXL || fabs(ay) > MAX_ACCEL_ADXL || fabs(az) > MAX_ACCEL_ADXL)
    {
        errorFlags |= ADXL_ERROR;
        return;
    }
    
    data.adxl.ax = ax;
    data.adxl.ay = ay;
    data.adxl.az = az;
    
    uint32_t now = millis();
    float dt = (now - data.adxl.lastTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    
    data.adxl.lastTotalAccel = sqrt(ax * ax + ay * ay + az * az);
    data.adxl.lastTotalSpeed += data.adxl.lastTotalAccel * dt;
    data.adxl.lastTime = now;

    errorFlags &= ~ADXL_ERROR;
  }
  else
    errorFlags |= ADXL_ERROR;
}

void Rakieta::handleBmp()
{
  if (bmp1.performReading())
  {
    float pressure = bmp1.pressure;
    float altitude = bmp1.readAltitude(SEA_LEVEL_PRESSURE_HPA) - offsets.bmp1.alti;
    float temp = bmp1.temperature;
    
    if (isnan(pressure) || isinf(pressure) || pressure < MIN_PRESSURE || pressure > MAX_PRESSURE ||
        isnan(altitude) || isinf(altitude) || altitude < MIN_ALTITUDE || altitude > MAX_ALTITUDE ||
        isnan(temp) || isinf(temp) || temp < MIN_TEMP || temp > MAX_TEMP)
    {
      errorFlags |= BMP1_ERROR;
    }
    else
    {
      data.bmp1.pressure = pressure;
      data.bmp1.altitude = altitude;
      data.bmp1.temp = temp;
      
      if (data.bmp1.maxAltitude < altitude) data.bmp1.maxAltitude = altitude;
      
      uint32_t now = millis();
      float dt = (now - data.bmp1.lastTime) / 1000.0f;
      if (dt > 0.001f)
      {
          data.bmp1.lastVerticalSpeed = (altitude - data.bmp1.lastAltitude) / dt;
          data.bmp1.lastAltitude = altitude;
          data.bmp1.lastTime = now;
      }
      
      errorFlags &= ~BMP1_ERROR;
    }
  }
  else
    errorFlags |= BMP1_ERROR;

  if (bmp2.performReading())
  {
    float pressure = bmp2.pressure;
    float altitude = bmp2.readAltitude(SEA_LEVEL_PRESSURE_HPA) - offsets.bmp2.alti;
    float temp = bmp2.temperature;
    
    if (isnan(pressure) || isinf(pressure) || pressure < MIN_PRESSURE || pressure > MAX_PRESSURE ||
        isnan(altitude) || isinf(altitude) || altitude < MIN_ALTITUDE || altitude > MAX_ALTITUDE ||
        isnan(temp) || isinf(temp) || temp < MIN_TEMP || temp > MAX_TEMP)
    {
      errorFlags |= BMP2_ERROR;
    }
    else
    {
      data.bmp2.pressure = pressure;
      data.bmp2.altitude = altitude;
      data.bmp2.temp = temp;
      
      if (data.bmp2.maxAltitude < altitude) data.bmp2.maxAltitude = altitude;
      
      uint32_t now = millis();
      float dt = (now - data.bmp2.lastTime) / 1000.0f;
      if (dt > 0.001f)
      {
          data.bmp2.lastVerticalSpeed = (altitude - data.bmp2.lastAltitude) / dt;
          data.bmp2.lastAltitude = altitude;
          data.bmp2.lastTime = now;
      }
      
      errorFlags &= ~BMP2_ERROR;
    }
  }
  else
    errorFlags |= BMP2_ERROR;
}

void Rakieta::handleGPS()
{
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());
    
  if (gps.location.isValid() && gps.location.isUpdated())
  {
    float lat = gps.location.lat() - offsets.gps.lat;
    float lng = gps.location.lng() - offsets.gps.lng;
      
    if (!isnan(lat) && !isinf(lat) && !isnan(lng) && !isinf(lng) && fabs(lat) <= 90.0f && fabs(lng) <= 180.0f)
    {
      data.gps.lat = lat;
      data.gps.lng = lng;
    }
    else
      errorFlags |= GPS_ERROR;
  }
    
  if (gps.altitude.isValid() && gps.altitude.isUpdated())
  {
    float alt = gps.altitude.meters();
    
    if (!isnan(alt) && !isinf(alt) && alt >= MIN_ALTITUDE && alt <= MAX_ALTITUDE)
    {
      data.gps.alti = alt;
      if (data.gps.alti > data.gps.maxAlti) data.gps.maxAlti = data.gps.alti - offsets.gps.alti;
    }
    else
      errorFlags |= GPS_ERROR;
  }
    
  if (gps.time.isValid() && gps.time.isUpdated())
  {
    data.gps.h = gps.time.hour();
    data.gps.m = gps.time.minute();
    data.gps.s = gps.time.second();
    data.gps.centi = gps.time.centisecond();
  }
  
  if (gps.speed.isValid() && gps.speed.isUpdated())
  {
    float speed = gps.speed.mps();

    if (!isnan(speed) && !isinf(speed) && speed >= MIN_SPEED && speed <= MAX_SPEED) data.gps.speed = speed;
    else errorFlags |= GPS_ERROR;
  }
  
  if (gps.course.isValid() && gps.course.isUpdated())
  {
    float course = gps.course.deg();
    if (!isnan(course) && !isinf(course) && course >= 0.0f && course <= 360.0f) data.gps.course = course;
  }
    
  if (gps.satellites.isValid() && gps.satellites.isUpdated())
  {
    uint8_t satNum = gps.satellites.value();
    if (satNum <= 20) data.gps.satNum = satNum;
  }
  
  if (gps.hdop.isValid() && gps.hdop.isUpdated())
  {
    uint8_t hdop = gps.hdop.hdop();
    if (hdop <= 99) data.gps.hdop = hdop;
  }
}

void Rakieta::readSensorsData()
{
  handleBattery();
  handleLsm();
  handleAdxl();
  handleBmp();
  handleGPS();
}

void Rakieta::printData()
{
  Serial.printf("IMU: Accel: %.2f %.2f %.2f G | Gyro: %.2f %.2f %.2f dps | Temp: %.2f *C\n",
                data.lsm.ax, data.lsm.ay, data.lsm.az, data.lsm.gx, data.lsm.gy, data.lsm.gz, data.lsm.temp);
  Serial.printf("IMU: Speed: %.2f m/s | Accel: %.3f m/s^2 | Gyro: %.3f d\n",
                data.lsm.lastTotalSpeed, data.lsm.lastTotalAccel, data.lsm.lastTotalRotation);

  Serial.printf("ADXL: X=%.2f Y=%.2f Z=%.2f G\n", data.adxl.ax, data.adxl.ay, data.adxl.az);
  Serial.printf("ADXL: Speed: %.2f m/s | Accel: %.3f m/s^2\n", data.adxl.lastTotalSpeed, data.adxl.lastTotalAccel);

  Serial.printf("BMP1: P=%.2f hPa, Alt=%.1f m, T=%.2f C\n", data.bmp1.pressure, data.bmp1.altitude, data.bmp1.temp);
  Serial.printf("BMP1: Speed: %.2f\n", data.bmp1.lastVerticalSpeed);
  
  Serial.printf("BMP2: P=%.2f hPa, Alt=%.1f m, T=%.2f C\n", data.bmp2.pressure, data.bmp2.altitude, data.bmp2.temp);
  Serial.printf("BMP2: Speed: %.2f\n", data.bmp2.lastVerticalSpeed);
  
  Serial.printf("GPS: Lat=%.6f Lon=%.6f\n", data.gps.lat, data.gps.lng);
  Serial.printf("GPS: Alt=%.1f m\n",data.gps.alti);
  Serial.printf("GPS: Time=%2d:%2d:%2d:%d\n", data.gps.h, data.gps.m, data.gps.s, data.gps.centi);
  Serial.printf("GPS: Speed:%.2f\n", data.gps.speed);
  Serial.printf("GPS: Course:%.3f\n", data.gps.course);
  Serial.printf("GPS: Sat:%2d\n", data.gps.satNum);
  Serial.printf("GPS: hdop:%2d\n", data.gps.hdop);
}

void Rakieta::setOffsets()
{
  Serial.println("\n=== Wyznaczanie offsetów ===\n");

  uint8_t num = OFFSETS_SENSORS_READ;
  for (uint8_t i = 0; i < num; i++)
  {
    sensors_event_t accel, gyro, temp;
    if (lsm.getEvent(&accel, &gyro, &temp))
    {
      offsets.lsm.ax += accel.acceleration.x;
      offsets.lsm.ay += accel.acceleration.y;
      offsets.lsm.az += accel.acceleration.z;
      offsets.lsm.gx += gyro.gyro.x;
      offsets.lsm.gy += gyro.gyro.y;
      offsets.lsm.gz += gyro.gyro.z;
      offsets.lsm.valid++;
    }

    sensors_event_t event;
    if (adxl.getEvent(&event))
    {
      offsets.adxl.ax += event.acceleration.x * ADXL375_MG2G_MULTIPLIER;
      offsets.adxl.ay += event.acceleration.y * ADXL375_MG2G_MULTIPLIER;
      offsets.adxl.az += event.acceleration.z * ADXL375_MG2G_MULTIPLIER;
      offsets.adxl.valid++;
    }

    if (bmp1.performReading())
    {
      offsets.bmp1.alti += bmp1.readAltitude(SEA_LEVEL_PRESSURE_HPA);
      offsets.bmp1.valid++;
    }

    if (bmp2.performReading())
    {
      offsets.bmp2.alti += bmp2.readAltitude(SEA_LEVEL_PRESSURE_HPA);
      offsets.bmp2.valid++;
    }

    delay(25);
  }

  offsets.lsm.ax = (offsets.lsm.valid ? ((float)offsets.lsm.ax / (float)offsets.lsm.valid) : 0);
  offsets.lsm.ay = (offsets.lsm.valid ? ((float)offsets.lsm.ay / (float)offsets.lsm.valid) : 0);
  offsets.lsm.az = (offsets.lsm.valid ? ((float)offsets.lsm.az / (float)offsets.lsm.valid) : 0);
  offsets.lsm.gx = (offsets.lsm.valid ? ((float)offsets.lsm.gx / (float)offsets.lsm.valid) : 0);
  offsets.lsm.gy = (offsets.lsm.valid ? ((float)offsets.lsm.gy / (float)offsets.lsm.valid) : 0);
  offsets.lsm.gz = (offsets.lsm.valid ? ((float)offsets.lsm.gz / (float)offsets.lsm.valid) : 0);
  offsets.adxl.ax = (offsets.adxl.valid ? ((float)offsets.adxl.ax / (float)offsets.adxl.valid) : 0);
  offsets.adxl.ay = (offsets.adxl.valid ? ((float)offsets.adxl.ay / (float)offsets.adxl.valid) : 0);
  offsets.adxl.az = (offsets.adxl.valid ? ((float)offsets.adxl.az / (float)offsets.adxl.valid) : 0);
  offsets.bmp1.alti = (offsets.bmp1.valid ? ((float)offsets.bmp1.alti / (float)offsets.bmp1.valid) : 0);
  offsets.bmp2.alti = (offsets.bmp2.valid ? ((float)offsets.bmp2.alti / (float)offsets.bmp2.valid) : 0);

  num = OFFSETS_GPS_READ;
  uint32_t timeout = millis() + 30000;

  while (offsets.gps.valid < num || millis() < timeout)
  {
    if (gps.location.isValid() && gps.altitude.isValid())
    {
      offsets.gps.lat += gps.location.lat();
      offsets.gps.lng += gps.location.lng();
      offsets.gps.alti += gps.altitude.meters();
      offsets.gps.valid++;
    }
    delay(200);
  }

  offsets.gps.lat = int32_t(offsets.gps.valid ? ((float)offsets.gps.lat / (float)offsets.gps.valid) : 0);
  offsets.gps.lng = int32_t(offsets.gps.valid ? ((float)offsets.gps.lng / (float)offsets.gps.valid) : 0);
  offsets.gps.alti = (offsets.gps.valid ? ((float)offsets.gps.alti / (float)offsets.gps.valid) : 0);

  offsetsSet = true;
}

void Rakieta::prepareOffsetsMsg(char* buffer, size_t bufferSize)
{
  if (!offsetsSet) return;
  if (buffer == nullptr || bufferSize <= 0) return;

  memset(buffer, 0, bufferSize);

  int written = snprintf(buffer, bufferSize,
    "ValidNum:Lsm:%d,Adxl%d,Bmp1%d,Bmp2%d,GPS%d,"
    "Offsets:Lsm:{ax:%f,ay:%f,az:%f,gx:%f,gy:%f,gz:%f}"
    "Adxl:{ax:%f,ay:%f,az:%f}"
    "GPS:{lat:%f,lng:%f,altiM:%f}",
    offsets.lsm.valid,
    offsets.adxl.valid,
    offsets.bmp1.valid,
    offsets.bmp2.valid,
    offsets.gps.valid,
    offsets.lsm.ax,
    offsets.lsm.ay,
    offsets.lsm.az,
    offsets.lsm.gx,
    offsets.lsm.gy,
    offsets.lsm.gz,
    offsets.adxl.ax,
    offsets.adxl.ay,
    offsets.adxl.az,
    offsets.gps.lat,
    offsets.gps.lng,
    offsets.gps.alti
  );

  if (written < 0 || static_cast<size_t>(written) >= bufferSize)
  {
    errorFlags |= MSG_TOO_LONG_ERROR;
    
    snprintf(buffer, bufferSize, "ERROR:MSG_TOO_LONG,%lu", millis());
  }
}

void Rakieta::prepareDataLineMsg(char* buffer, size_t bufferSize)
{
  if (buffer == nullptr || bufferSize <= 0) return;

  memset(buffer, 0, bufferSize);

  int written = snprintf(buffer, bufferSize,
    "RocketData:%lu,%u,%d,%04X,"
    "%.6f,%.6f,%.2f,%02u,%02u,%02u,%02u,%.2f,%.0f,%u,%u,"
    "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.4f,"
    "%.4f,%.4f,%.4f,%.4f,"
    "%.2f,%.2f,%.2f,%.4f,"
    "%.2f,%.2f,%.2f,%.4f,"
    "%.1f",

    // Timestamp i podstawowe
    millis(),
    packet,
    static_cast<int>(currentFlightState),
    errorFlags,

    // GPS
    data.gps.lat,
    data.gps.lng,
    data.gps.alti,
    data.gps.h,
    data.gps.m,
    data.gps.s,
    data.gps.centi,
    data.gps.speed,
    data.gps.course,
    data.gps.satNum,
    data.gps.hdop,
    
    // LSM6
    data.lsm.ax,
    data.lsm.ay,
    data.lsm.az,
    data.lsm.gx,
    data.lsm.gy,
    data.lsm.gz,
    data.lsm.temp,
    data.lsm.lastTotalSpeed,

    // ADXL
    data.adxl.ax,
    data.adxl.ay,
    data.adxl.az,
    data.adxl.lastTotalSpeed,
    
    // BMP
    data.bmp1.temp,
    data.bmp1.pressure,  /// sprawdzić czy jest w Pa czy w hPa
    data.bmp1.altitude,
    data.bmp1.lastVerticalSpeed,
    data.bmp2.temp,
    data.bmp2.pressure,  /// sprawdzić czy jest w Pa czy w hPa
    data.bmp2.altitude,
    data.bmp2.lastVerticalSpeed,

    // BATTERY
    data.battery.voltage
  );

  if (written < 0 || static_cast<size_t>(written) >= bufferSize)
  {
    errorFlags |= MSG_TOO_LONG_ERROR;
    
    snprintf(buffer, bufferSize, "ERROR:MSG_TOO_LONG,%lu", millis());
  }
}

void Rakieta::filterAcceleration()  /// nigdzie nie używane
{
  filteredAccelX = FUSION_ALPHA * data.lsm.ax + (1 - FUSION_ALPHA) * filteredAccelX;
  filteredAccelY = FUSION_ALPHA * data.lsm.ay + (1 - FUSION_ALPHA) * filteredAccelY;
  filteredAccelZ = FUSION_ALPHA * data.lsm.az + (1 - FUSION_ALPHA) * filteredAccelZ;
}

void Rakieta::filterGyro()  /// nigdzie nie używane
{
  filteredGyroX = FUSION_ALPHA * data.lsm.gx + (1 - FUSION_ALPHA) * filteredGyroX;
  filteredGyroY = FUSION_ALPHA * data.lsm.gy + (1 - FUSION_ALPHA) * filteredGyroY;
  filteredGyroZ = FUSION_ALPHA * data.lsm.gz + (1 - FUSION_ALPHA) * filteredGyroZ;
}

void Rakieta::calculateOrientation()  /// nigdzie nie używane
{
  data.orientation.pitch = atan2(-filteredAccelX, sqrt(filteredAccelY*filteredAccelY + filteredAccelZ*filteredAccelZ));
  data.orientation.roll = atan2(filteredAccelY, filteredAccelZ);
}

void Rakieta::fuseBMPAndIMU()  /// nigdzie nie używane
{
  uint32_t now = millis();
  float dt = (now - data.lsm.lastTime) / 1000.0f;
  if (dt > 0.001)
  {
    float altitudeFromIMU = prevFusedAltitude + data.lsm.lastTotalAlti;
    
    fusedAltitude = FUSION_ALPHA * data.bmp1.altitude + (1 - FUSION_ALPHA) * altitudeFromIMU;
    prevFusedAltitude = fusedAltitude;
  }
}

bool Rakieta::initLora()
{
  int state = lora.begin(FREQUENCY, BANDWIDTH, SF, CODING_RATE, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, POWER, PREAMBLE_LENGTH);

  if (state == RADIOLIB_ERR_NONE)
  {
    lora.setDio1Action(Rakieta::setOperationFlag);
    Serial.println(prepareLoraStatusMsg());
    Serial.println("[LoRa] OK");
    errorFlags &= ~LORA_ERROR;
    startListening();
    return true;
  }
  Serial.print("[LoRa] Błąd: ");
  Serial.println(state);
  errorFlags |= LORA_ERROR;
  return false;
}

void Rakieta::setOperationFlag()
{
  operationDone = true;
}

void Rakieta::prepareLoraStatusMsg(char* buffer, size_t bufferSize)
{
  if (buffer == nullptr || bufferSize <= 0) return;

  memset(buffer, 0, bufferSize);

  int written = snprintf(buffer, bufferSize,
    "=== RADIO STATUS ===\nFrequency:%luMHz,Moc:%ludBm,SF:%luBW:%lukHz,CR:4/%lu",
    FREQUENCY,
    POWER,
    SF,
    BANDWIDTH,
    CODING_RATE
  );

  if (written < 0 || static_cast<size_t>(written) >= bufferSize)
  {
    errorFlags |= MSG_TOO_LONG_ERROR;
    
    snprintf(buffer, bufferSize, "ERROR:MSG_TOO_LONG,%lu", millis());
  }
}

void Rakieta::preparePacket()
{
  uint32_t now = millis();
  message.add(uint32_t(now / 10), timePos, timeLen);
  message.add(uint16_t(packet), packetPos, packetLen);
  message.add(uint16_t(errorFlags), errorPos, errorLen);
  message.add(uint8_t(currentFlightState), statusPos, statusLen);

  message.add(int16_t(data.gps.lat * 100000), gpsLatPos, gpsLatLen, true);
  message.add(int16_t(data.gps.lng * 100000), gpsLngPos, gpsLngLen, true);
  message.add(uint16_t(data.gps.alti * 10), gpsAltiPos, gpsAltiLen);
  message.add(uint8_t(data.gps.h), gpsHourPos, gpsHourLen);
  message.add(uint8_t(data.gps.m), gpsMinPos, gpsMinLen);
  message.add(uint8_t(data.gps.s), gpsSecPos, gpsSecLen);
  message.add(uint8_t(data.gps.centi), gpsCentisecPos, gpsCentisecLen);
  message.add(int16_t(data.gps.speed * 10), gpsSpeedPos, gpsSpeedLen, true);
  message.add(uint16_t(data.gps.course), gpsCoursePos, gpsCourseLen);
  message.add(uint8_t(data.gps.satNum), gpsSatNumPos, gpsSatNumLen);
  message.add(uint8_t(data.gps.hdop), gpsHdopPos, gpsHdopLen);
  
  message.add(int16_t(data.lsm.ax * 100), lsmAccelXPos, lsmAccelXLen, true);
  message.add(int16_t(data.lsm.ay * 100), lsmAccelYPos, lsmAccelYLen, true);
  message.add(int16_t(data.lsm.az * 100), lsmAccelZPos, lsmAccelZLen, true);
  message.add(int16_t(data.lsm.gx * 10), lsmGyroXPos, lsmGyroXLen, true);
  message.add(int16_t(data.lsm.gy * 10), lsmGyroYPos, lsmGyroYLen, true);
  message.add(int16_t(data.lsm.gz * 10), lsmGyroZPos, lsmGyroZLen, true);
  message.add(int8_t(data.lsm.temp), lsmTempPos, lsmTempLen, true);
  message.add(int16_t(data.lsm.lastTotalSpeed * 10), lsmSpeedPos, lsmSpeedLen, true);
  
  message.add(int16_t(data.adxl.ax * 10), adxlAccelXPos, adxlAccelXLen, true);
  message.add(int16_t(data.adxl.ay * 10), adxlAccelYPos, adxlAccelYLen, true);
  message.add(int16_t(data.adxl.az * 10), adxlAccelZPos, adxlAccelZLen, true);
  message.add(int16_t(data.adxl.lastTotalSpeed * 10), adxlSpeedPos, adxlSpeedLen, true);

  message.add(int8_t(data.bmp1.temp), bmp1TempPos, bmp1TempLen, true);
  message.add(uint16_t(data.bmp1.pressure * 10), bmp1PressPos, bmp1PressLen);
  message.add(uint16_t(data.bmp1.altitude * 10), bmp1AltiPos, bmp1AltiLen);
  message.add(int16_t(data.bmp1.lastVerticalSpeed * 10), bmp1SpeedPos, bmp1SpeedLen, true);

  message.add(int8_t(data.bmp2.temp), bmp2TempPos, bmp2TempLen, true);
  message.add(uint16_t(data.bmp2.pressure * 10), bmp2PressPos, bmp2PressLen);
  message.add(uint16_t(data.bmp2.altitude * 10), bmp2AltiPos, bmp2AltiLen);
  message.add(int16_t(data.bmp2.lastVerticalSpeed * 10), bmp2SpeedPos, bmp2SpeedLen, true);

  message.add(uint8_t(data.battery.voltage * 10), batteryPos, batteryLen);
}

void Rakieta::sendPacket()
{
  packet++;
  preparePacket();
  uint8_t *txPacket = message.data();

  Serial.println("\n\nSending: ");

  for (int i=0; i<ARRAY_SIZE; i++)
    Serial.print(txPacket[i], HEX);
  Serial.println("");

  for (int i=0; i<ARRAY_SIZE; i++)
    for (int j=7; j>=0; --j)
      Serial.print((txPacket[i] >> j) & 1);

  Serial.println("\nEnd of the message.\n\n");

  transmit(txPacket, ARRAY_SIZE);

  led1IsOn = true;
  led1OffTime = millis() + LED_DELAY;
  digitalWrite(LED_1, HIGH);

  Serial.println("Sending DONE");
}

void Rakieta::transmit(const uint8_t* msg, const size_t len)
{
  if (msg == nullptr || len == 0 || len > 255)
  {
    return;
  }

  if (!operationDone)
  {
    if (millis() - loraMsgStartTime >= TX_TIMEOUT)
    {
      Serial.println("ERROR: LoRa timeout, forcing radio reset");
      lora.standby();
      delay(1);
      startListening();
    }
    else
    {
      Serial.println("LoRa busy");
      return;
    }
  }
  
  Serial.print(F("Sending: "));
  Serial.println(msg);

  operationDone = false;
  loraMsgStartTime = millis();
  int state = lora.startTransmit(reinterpret_cast<const uint8_t*>(msg), len);

  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Transmit error: "));
    Serial.println(state);
    errorFlags |= LORA_ERROR;
    operationDone = true;
    startListening();
  }
}

void Rakieta::transmit(const char* msg, size_t len)
{
  if (msg == nullptr) return;
  transmit(reinterpret_cast<const uint8_t*>(msg), len);
}

void Rakieta::startListening()
{
  int state = lora.startReceive();
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Error starting listening: "));
    Serial.println(state);
  }
}

/// trzeba pododawać komendy
void Rakieta::handleCommand(const char* command)
{
  if (command == nullptr) return;
  
  // Użycie strcmp do porównywania napisów
  if (strcmp(command, "RESET") == 0) { systemReset(); }
  else if (strcmp(command, "SET_OFFSETS") == 0) { setOffsets(); }
  else if (strcmp(command, "GET_GPS_OFFSET") == 0) { sendGpsOffset(); }
  else if (strcmp(command, "GET_RAW_DATA") == 0) { readSensorsData(); sendPacket(); }
  else if (strcmp(command, "GET_OFFSETS") == 0)
  {
    char buffer[256];
    prepareOffsetsMsg(buffer, sizeof(buffer));
    transmit(buffer);
  }
  else if (strcmp(command, "GET_DATA") == 0)
  {
    char buffer[256];
    prepareDataLineMsg(buffer, sizeof(buffer));
    transmit(buffer);
  }
  else { transmit("ERROR:UNKNOWN_COMMAND"); }
}

void Rakieta::checkRadio()
{
  uint8_t buffer[256];
  size_t len = 0;

  int state = lora.readData(buffer, sizeof(buffer) - 1, len);

  if (state == RADIOLIB_ERR_NONE)
  {
    buffer[len] = '\0';
    /*
        Serial.println(F("== MESSAGE RECEIVED =="));
        Serial.print(F("Message: "));
        Serial.println(msg);
        Serial.print(F("RSSI: "));
        Serial.print(lora.getRSSI());
        Serial.print(F(" dBm, "));
        Serial.print(F("SNR: "));
        Serial.print(lora.getSNR());
        Serial.println(F(" dB"));
    */
    handleCommand(reinterpret_cast<char*>(buffer));
    errorFlags &= ~LORA_ERROR;
  }
  else if (state != RADIOLIB_ERR_RX_TIMEOUT)
  {
    errorFlags |= LORA_ERROR;
  }
  else
  {
    Serial.print(F("Data reading error: "));
    Serial.println(state);
    errorFlags |= LORA_ERROR;
  }
}

void Rakieta::sendGpsOffset()
{
  if (!offsetsSet) return;

  char buffer[128];
  const size_t bufferSize = sizeof(buffer);
  memset(buffer, 0, bufferSize);

  int written = snprintf(buffer, bufferSize,
    "GSP offset:Now:%lu,lat:%f,lng:%f,alti:%f",
    millis(),
    offsets.gps.lat,
    offsets.gps.lng,
    offsets.gps.alti
  );
  
  if (written < 0 || static_cast<size_t>(written) >= bufferSize)
  {
    errorFlags |= MSG_TOO_LONG_ERROR;
    
    snprintf(buffer, bufferSize, "ERROR:MSG_TOO_LONG,%lu", millis());
  }
  
  transmit(buffer);
}

bool Rakieta::initFlash()
{
  if (flash.begin())
  {
    uint32_t jedec = flash.getJEDECID();
    Serial.print("[FLASH] OK, JEDEC: 0x");
    Serial.println(jedec, HEX);
    errorFlags &= ~FLASH_ERROR;
    return true;
  }
  Serial.println("[FLASH] BRAK ODPOWIEDZI");
  errorFlags |= FLASH_ERROR;
  return false;
}

bool Rakieta::flashFindNextFileNumber(char* fileName, size_t bufferSize)
{
  if (fileName == nullptr || bufferSize < 14)  // minimum "data_0000.csv" + null
  {
    return false;
  }
  
  uint16_t fileNumber = 0;

  while (fileNumber < MAX_FILE_NUMBER)
  {
    snprintf(fileName, bufferSize, "data_%04d.csv", fileNumber);
    
    if (!fatfs.exists(fileName))
    {
      errorFlags &= ~FLASH_FILE_ERROR;
      return true;
    }
      
    fileNumber++;
  }
  
  Serial.println("Too many data files!");
  errorFlags |= FLASH_FILE_ERROR;
  return false;
}

bool Rakieta::flashOpenNewFile()
{
  if (flashDataFile)
  {
    flashCloseFile();
  }

  if (!flashFindNextFileNumber())
  {
    Serial.println("Cannot open a file");
    return false;
  }
  
  flashDataFile = fatfs.open(currentFileName.c_str(), FILE_WRITE);
  if (!flashDataFile)
  {
    Serial.println("Failed to open file for writing");
    errorFlags |= FLASH_FILE_ERROR;
    return false;
  }
  
  /// trzeba pozmieniać te nagłówki
  flashWriteString("timestamp,packet,status,error,"
                 "gps_lat,gps_lng,gps_alt,gps_h,gps_m,gps_s,gps_c,"
                 "gps_speed,gps_course,gps_satNum,gps_hdop,"
                 "lsm_ax,lsm_ay,lsm_az,lsm_gx,lsm_gy,lsm_gz,lsm_temp,lsm_speed,"
                 "adxl_ax,adxl_ay,adxl_az,adxl_speed,"
                 "bmp1_temp,bmp1_press,bmp1_alt,bmp1_speed,"
                 "bmp2_temp,bmp2_press,bmp2_alt,bmp2_speed,"
                 "max_temp,battery");
  flashDataFile.flush();
  
  Serial.print("File opened: "); 
  Serial.println(currentFileName.c_str());
  errorFlags &= ~FLASH_FILE_ERROR;
  return true;
}

void Rakieta::flashWriteString(const char* msg)
{
  if (msg == nullptr)
  {
    errorFlags |= FLASH_NULL_MSG_ERROR;
    return;
  }

  if (flash.isBusy())
  {
    Serial.println("Flash not ready!");
    return;
  }
  
  if (!flashDataFile)
  {
    Serial.println("File not open!");
    errorFlags |= FLASH_FILE_ERROR;
    return;
  }
  
  int written = flashDataFile.println(msg);

  if (written <= 0)
  {
    Serial.println("Write failed!");
    errorFlags |= FLASH_FILE_ERROR;
    return;
  }
  
  flashWriteCount++;

  if (flashWriteCount >= FLUSH_AFTER_WRITES)
  {
    flashFlushBuffer();
  }
}

void Rakieta::flashFlushBuffer()
{
  if (flashDataFile)
  {
    flashDataFile.flush();
    flashWriteCount = 0;
    Serial.println("[FLASH] Buffer flushed.");
  }
}

void Rakieta::flashCloseFile()
{
  if (flashDataFile)
  {
    flashFlushBuffer();

    flashDataFile.close();
    Serial.println("[FLASH] File closed.");
  }
}

bool Rakieta::flashRecoverAfterReset()
{
  if (!fatfs.begin(&flash))
  {
    Serial.println("[FLASH] Mount failed. Attempting format...");
    if (!fatfs.begin(&flash, true))
    {
      Serial.println("[FLASH] Format failed!");
      errorFlags |= FLASH_ERROR;
      return false;
    }
    Serial.println("[FLASH] Format successful.");
  }

  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    Serial.println("[FLASH] Cannot open root directory.");
    return false;
  }

  uint16_t maxNumber = 0;
  char fileName[20];
  File32 file;

  while ((file = root.openNextFile()))
  {
    if (!file.isDirectory())
    {
      file.getName(fileName, sizeof(fileName));

      if (strncmp(fileName, "data_", 5) == 0)
      {
        char* dotPos = strrchr(fileName, '.');
        if (dotPos != nullptr && strcmp(dotPos, ".csv") == 0)
        {
          // Wyciągnij numer z nazwy pliku
          int num = atoi(fileName + 5);
          if (num > maxNumber) maxNumber = num;
        }
      }
    }
    file.close();
  }
  root.close();

  char filename[32];
  snprintf(filename, sizeof(filename), "data_%04d.csv", maxNumber);

  flashDataFile = fatfs.open(filename, FILE_WRITE);
  if (!flashDataFile)
  {
    flashDataFile = fatfs.open(filename, FILE_WRITE | O_CREAT | O_APPEND);
    if (!flashDataFile)
    {
      Serial.println("[FLASH] Cannot open/create file for recovery!");
      return false;
    }
  }

  flashDataFile.seekEnd();
  flashDataFile.println("\n--- RECOVERY AFTER RESET ---");
  flashDataFile.flush();
  Serial.printf("[FLASH] Recovery OK. File: %s\n", filename);
  return true;
}

void Rakieta::flashDumpFileList()
{
  Serial.println("\n=== FLASH FILE LIST ===");

  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    Serial.println("[FLASH] Cannot open root directory.");
    return;
  }

  char fileName[20];
  File32 file;

  while ((file = root.openNextFile()))
  {
    if (!file.isDirectory())
    {
      file.getName(fileName, sizeof(fileName));

      size_t len = strlen(fileName);
      if (len > 4 && strcmp(fileName + len - 4, ".csv") == 0)
      {
        Serial.print(" - ");
        Serial.print(fileName);
        Serial.print(" (size: ");
        Serial.print(file.size());
        Serial.println(" bytes)");
      }
    }
    file.close();
  }
  root.close();
  Serial.println("========================\n");
}

void Rakieta::flashDumpFileData(const uint16_t fileNumber)
{
  char filename[32];
  snprintf(filename, sizeof(filename), "data_%04d.csv", fileNumber);

  File32 dumpFile = fatfs.open(filename, FILE_READ);
  if (!dumpFile)
  {
    Serial.printf("[FLASH] Cannot open file %s\n", filename);
    return;
  }

  Serial.printf("=== DUMP OF %s ===\n", filename);
  char line[256];

  while (dumpFile.available())
  {
    int bytesRead = 0;
    while (dumpFile.available() && bytesRead < (int)sizeof(line) - 1)
    {
      char c = dumpFile.read();
      line[bytesRead++] = c;
      if (c == '\n') break;
    }

    line[bytesRead] = '\0';
    Serial.print(line);
  }

  dumpFile.close();
  Serial.println("=== END OF FILE ===\n");
}

void Rakieta::flashDumpLastFile()
{
  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    Serial.println("[FLASH] Cannot open root directory.");
    return;
  }

  uint16_t lastNumber = 0;
  char fileName[20];
  File32 dumpFile;

  while ((dumpFile = root.openNextFile()))
  {
    if (!dumpFile.isDirectory())
    {
      dumpFile.getName(dumpFile, sizeof(dumpFile));

      if (strncmp(fileName, "data_", 5) == 0)
      {
        char* dotPos = strrchr(fileName, '.');
        if (dotPos != nullptr && strcmp(dotPos, ".csv") == 0)
        {
          int num = atoi(fileName + 5);
          if (num > lastNumber) lastNumber = num;
        }
      }
    }
    dumpFile.close();
  }
  root.close();

  if (lastNumber == 0)
  {
    Serial.println("[FLASH] No data files found.");
    return;
  }

  flashDumpFileData(lastNumber);
}

// do zastanowienia się
bool Rakieta::detectLaunch()
{
  if (data.lsm.lastTotalAccel > LAUNCH_ACCEL_THRESHOLD)
  {
    uint32_t now = millis();
    if (launchDetectTime == 0) launchDetectTime = now;
    if (now - launchDetectTime >= LAUNCH_DEBOUNCE_MS) return true;
  }
  else launchDetectTime = 0;

  return false;
}

// do zastanowienia się
bool Rakieta::detectBurnout()
{
  if (data.lsm.lastTotalAccel < BURNOUT_ACCEL_THRESHOLD)
  {
    uint32_t now = millis();
    if (burnoutDetectTime == 0) burnoutDetectTime = now;
    if (now - burnoutDetectTime >= BURNOUT_DEBOUNCE_MS) return true;
  }
  else burnoutDetectTime = 0;

  return false;
}

// do zastanowienia się
bool Rakieta::detectApogee()
{
  // Apogeum gdy prędkość spada poniżej progu i osiągnięto maksimum wysokości
  if (data.bmp1.lastVerticalSpeed <= APOGEE_VELOCITY_THRESHOLD && data.bmp1.altitude <= data.bmp1.maxAltitude - 1)
  {
    uint32_t now = millis();
    if (apogeeDetectTime == 0) apogeeDetectTime = now;
    if (now - apogeeDetectTime >= APOGEE_DEBOUNCE_MS) return true;
  }
  else apogeeDetectTime = 0;

  return false;
}

// do zastanowienia się
bool Rakieta::detectLanding()
{
  if (abs(data.bmp1.lastVerticalSpeed) < LANDING_VELOCITY_THRESHOLD)
  {
    uint32_t now = millis();
    if (landedDetectTime == 0) landedDetectTime = now;
    if (now - landedDetectTime >= LANDING_DEBOUNCE_MS) return true;
  }
  else landedDetectTime = 0;
  
  return false;
}

// do zastanowienia się
bool Rakieta::checkDeploymentConditions(const ParachuteType type)
{
  float currentSpeed = abs(data.bmp1.lastVerticalSpeed);
  float currentAltitude = data.bmp1.altitude;

  if (type == DROGUE)
  {
    if (currentSpeed > DEPLOY_DROGUE_MAX_SPEED || currentAltitude < DEPLOY_DROGUE_MIN_ALTITUDE) return false;
    return true;
  }
  else if (type == MAIN)
  {
    if (currentSpeed > DEPLOY_MAIN_MAX_SPEED || currentAltitude < DEPLOY_MAIN_MIN_ALTITUDE) return false;
    return drogueDeployed;
  }
  return false;
}

void Rakieta::sendFlightSummary()
{
  uint32_t now = millis();
  
  char buffer[256];
  
  snprintf(buffer, sizeof(buffer),
    "FlightSummary:launch:%lu,burnout:%lu,apogee:%lu,descent:%lu,landed:%lu",
    (launchDetectTime == 0 ? 0 : now - launchDetectTime),
    (burnoutDetectTime == 0 ? 0 : now - burnoutDetectTime),
    (apogeeDetectTime == 0 ? 0 : now - apogeeDetectTime),
    (descentDetectTime == 0 ? 0 : now - descentDetectTime),
    (landedDetectTime == 0 ? 0 : now - landedDetectTime)
  );

  flashWriteString(msg);
  transmit(msg);
  Serial.println(msg);
}

void Rakieta::drogueParashuteOpen()
{
  solenoid1IsOn = true;
  solenoid1OffTime = millis() + SOLENOID_DELAY;
  digitalWrite(SOLENOID_1, HIGH);
}

void Rakieta::mainParashuteOpen()
{
  solenoid2IsOn = true;
  solenoid2OffTime = millis() + SOLENOID_DELAY;
  digitalWrite(SOLENOID_2, HIGH);
}

void Rakieta::updateFlightState()
{
  uint32_t now = millis();
  
  switch (currentFlightState)
  {
    case FlightState::IDLE:
      if (detectLaunch())
      {
        currentFlightState = BOOST;
        launchDetectTime = now;
        inFlight = true;
        Serial.println("[FLIGHT] -> BOOST");
      }
      break;
    case FlightState::BOOST:
    {
      if (detectBurnout())
      {
        currentFlightState = COAST;
        burnoutDetectTime = now;
        Serial.println("[FLIGHT] -> COAST");
      }
      break;
    }
    case FlightState::COAST:
    {
      if (detectApogee())
      {
        currentFlightState = APOGEE;
        apogeeDetectTime = now;
        Serial.println("[FLIGHT] -> APOGEE");
      }
      break;
    }
    case FlightState::APOGEE:
    {
      if ((!drogueDeployed && checkDeploymentConditions(DROGUE)) || (millis() - launchDetectTime > DROGUE_PARASHUTE_TIMEOUT))
      {
        drogueParashuteOpen();
        drogueDeployed = true;
        currentFlightState = DESCENT;
        descentDetectTime = now;
        Serial.println("[FLIGHT] Drogue parachute deployed.");
        Serial.println("[FLIGHT] -> DESCENT");
      }
      break;
    }
    case FlightState::DESCENT:
    {
      if ((!mainDeployed && checkDeploymentConditions(MAIN)) || (millis() - launchDetectTime > MAIN_PARASHUTE_TIMEOUT))
      {
        mainParashuteOpen();
        mainDeployed = true;
        Serial.println("[FLIGHT] Main parachute deployed.");
      }

      if (detectLanding())
      {
        currentFlightState = LANDED;
        landedDetectTime = now;
        Serial.println("[FLIGHT] -> LANDED");
      }
      break;
    }
    case FlightState::LANDED:
      sendFlightSummary();
      inFlight = false;
      flashCloseFile();
      break;

    default:
      break;
  }
}

void Rakieta::initWatchdog()
{
  // Włącz watchdog z timeoutem ~8 sekund (LSI ~32 kHz, preskaler 64, reload 4000)
  IWDG1->KR = 0x5555;   // odblokowanie dostępu do rejestrów
  IWDG1->PR = 4;        // preskaler 64 (2^4 = 64)
  IWDG1->RLR = 4000;    // reload value
  IWDG1->KR = 0xAAAA;   // odświeżenie (wymagane przed uruchomieniem)
  IWDG1->KR = 0xCCCC;   // uruchomienie
}

void Rakieta::watchdog()
{
  IWDG1->KR = 0xAAAA;
}

void Rakieta::reinitComponent(bool (*initFunc)())
{
  Serial.print("[ERROR] Reinit component");
  
  bool success = (this->*initFunc)();
  
  if (success)
  {
      Serial.println("[ERROR] Component recovered");
  }
  else
  {
      Serial.println("[ERROR] Component recovery FAILED");
  }
}

void Rakieta::handleErrors()
{
  if (currentFlightState == FlightState::BOOST || currentFlightState == FlightState::COAST) return;

  // 1. LoRa (komunikacja)
  if (errorFlags & LORA_ERROR)   reinitComponent(&Rakieta::initLora);
  
  // 2. Flash (zapis danych)
  if (errorFlags & FLASH_ERROR)  reinitComponent(&Rakieta::initFlash);
  
  // 3. Sensory – w kolejności ważności dla lotu
  if (errorFlags & BMP1_ERROR)   reinitComponent(&Rakieta::initBMP1);
  if (errorFlags & BMP2_ERROR)   reinitComponent(&Rakieta::initBMP2);
  if (errorFlags & LSM_ERROR)    reinitComponent(&Rakieta::initLSM);
  if (errorFlags & ADXL_ERROR)   reinitComponent(&Rakieta::initADXL);
  if (errorFlags & GPS_ERROR)    reinitComponent(&Rakieta::initGPS);
  
  // 4. Dodatkowe flagi plików (tylko loguj, nie reinicjalizuj)
  if (errorFlags & FLASH_FILE_ERROR)
  {
    Serial.println("[ERROR] Flash file error – try to reopen file");
    if (flashDataFile) flashCloseFile();
    flashOpenNewFile();
    if (flashDataFile) errorFlags &= ~FLASH_FILE_ERROR;
  }
}





// === Obsługa trybów ===
void Rakieta::handleDebugMode()  /// do zmieniania w locie
{
  Serial.println("\n=== TRYB DEBUG: wypisywanie czujnikow ===");
  Serial.println("----------------------------------------");

  checkRadio();

  /// trzeba usuwać i sprawdzać po kolei
  handleLsm();
  handleBmp();
  handleAdxl();
  handleGPS();
  handleBattery();
  printData();
}

void Rakieta::handleFlightMode()
{
  checkRadio();
  
  uint32_t interval = inFlight ? INTERVAL_FLIGHT : INTERVAL_IDLE;
  uint32_t now = millis();

  if (now - lastRun >= interval)
  {
    lastFlightLoop = now;

    readSensorsData();
    updateFlightState();

    char msg[256];
    prepareDataLineMsg(msg, sizeof(msg));
    sendPacket();
    flashWriteString(msg);
  }
}

void Rakieta::handleDumpMode()  /// nie wiem co tu się dzieje obecnie
{
  const uint32_t startAddr = 0;
  const uint32_t totalLength = 512;
  uint8_t buffer[16];
  char ascii[17];
  
  for (uint32_t i = 0; i < 16 && dumpAddress < totalLength; i++)
  {
    flash.readBuffer(dumpAddress, buffer, 16);
    
    Serial.printf("%08X: ", dumpAddress);
    for (int j = 0; j < 16; j++)
    {
      Serial.printf("%02X ", buffer[j]);
      ascii[j] = (buffer[j] >= 32 && buffer[j] <= 126) ? buffer[j] : '.';
    }
    ascii[16] = '\0';
    Serial.printf(" | %s\n", ascii);
    
    dumpAddress += 16;
  }
  
  if (dumpAddress >= totalLength)
  {
    Serial.println("\n--- Koniec dump. Przechodzę do SLEEP ---");
    currentMode = SystemMode::SLEEP;
  }
}

void Rakieta::handleSleepMode()  /// chyba będzie ok
{
  SystemMode oldMode = currentMode;
  setSystemMode();
  
  if (currentMode != oldMode)
  {
    Serial.println("Zmiana trybu – resetowanie systemu...");
    systemSleep(100);
    systemReset();
  }
  else
  {
    currentMode = SystemMode::SLEEP;
  }
  
  // TODO: tu można dodać faktyczne uśpienie (__WFI())
  systemSleep(5000);
}

void Rakieta::updateState(const uint32_t now)
{
  switch (currentMode)  /// zamienić to na osobną funkcję
  {
    case SystemMode::DEBUG:
      if (now - lastDebugPrint >= INTERVAL_DEBUG)
      {
        lastDebugPrint = now;
        handleDebugMode();
      }
      break;
    case SystemMode::FLIGHT:
      if (!flashDataFile) flashOpenNewFile();

      if (now - lastFlightLoop >= INTERVAL_FLIGHT)
      {
        lastFlightLoop = now;
        handleFlightMode();
      }
      break;
    case SystemMode::DUMP:
      if (now - lastDumpProgress >= INTERVAL_DUMP)
      {
        lastDumpProgress = now;
        handleDumpMode();
      }
      break;
    case SystemMode::SLEEP:
      if (now - lastSleepCheck >= INTERVAL_SLEEP)
      {
        lastSleepCheck = now;
        handleSleepMode();
      }
      break;
  }
}





void Rakieta::init()
{
  Serial.println("\n=== ROCKET ON-BOARD COMPUTER INIT ===\n");

  initGPIO();
  initSPI();

  initLSM();
  initADXL();
  initBMP1();
  initBMP2();
  initGPS();
  initLora();
  initFlash();

  setSystemMode();
  printSystemMode();

  uint8_t ile = 5;
  if (errorFlags)
  {
    ile = 10;
    Serial.println(errorFlags);
  }

  for (int i=0; i<ile; i++)
  {
    digitalWrite(LED_1, HIGH);
    systemSleep(200);
    digitalWrite(LED_1, LOW);
    systemSleep(200);
  }
  
  if (currentMode != SystemMode::SLEEP) setOffsets();

  initWatchdog();

  Serial.println("\n=== INICJALIZACJA ZAKOŃCZONA ===\n");
}

void Rakieta::loop()
{
  uint32_t now = millis();
  
  if (now - lastErrorCheckTime >= INTERVAL_ERROR_CHECK)
  {
    lastErrorCheckTime = now;
    handleErrors();
  } 

  updateState(now);
  updateLeds(now);
  updateBuzzer(now);
  updateSolenoid(now);

  watchdog();
  systemSleep(5);
}











