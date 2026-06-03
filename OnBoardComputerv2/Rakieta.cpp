#include "Rakieta.h"


/*
  -sprawdzić  "AIR VEHICLE C++ CODING STANDARDS FOR THE SYSTEM DEVELOPMENT AND DEMONSTRATION PROGRAM"

  NA TERAZ (ŁATWE):

  OGÓLNIE DO ZROBIENIA:
    7 – Wielokrotne return w funkcjach – zmienisz na wzór z blokiem do { ... } while(0) i break (lub flagę).
    10 – Atomowy dostęp do volatile operationDone – dodasz wyłączanie przerwań (__disable_irq() / __enable_irq()).
    13 – delay() w setOffsets() – zamienisz na n blokującą pętlę z millis().  // częściowo
    29 – delay() w systemSleep() – zmienisz na prawdziwe uśpienie (__WFI()) lub docelową implementację.
    34 – Długość dumpa (512 bajtów) – zwiększysz na cały flash (flash.size()).
    37 – checkDeploymentConditions() – zmienisz na fuzję sensorów.
    53 – handleDumpMode() – przerobisz na dump całego flasha z resetem adresu.
  
  NA PÓŹNIEJ:
    1. Użycie delay() w funkcjach inicjalizujących (AV Rule 131) występuje w: initGPS() – delay(500), setOffsets() – delay(25) i delay(200), systemSleep() – delay(time), transmit() – delay(1), handleSleepMode() – systemSleep(100) i systemSleep(5000)
    10. Użycie delay()   -    W Rakieta.cpp wielokrotnie używane jest delay(): w setOffsets() delay(25) i delay(200), w initGPS() delay(500), w transmit() delay(1), w systemSleep() delay(time). delay() blokuje wykonanie całego programu na zadany czas, co w systemie czasu rzeczywistego jest niedopuszczalne – prowadzi do utraty danych z sensorów i braku reakcji na krytyczne zdarzenia. Należy zastąpić delay() nieblokującymi timerami opartymi na millis().
    14. Ograniczone sprawdzanie zakresów danych   -    Dane z sensorów są używane w obliczeniach bez weryfikacji, czy są fizycznie możliwe. Na przykład w detectApogee() porównuje się data.bmp1.lastVerticalSpeed <= APOGEE_VELOCITY_THRESHOLD – ale jeśli lastVerticalSpeed jest NaN lub nieskończonością, wynik będzie nieprawidłowy. Należy przed każdym użyciem sprawdzać, czy wartość jest skończona i mieści się w oczekiwanym przedziale (np. isfinite()).
    15. Brak jawnych timeoutów dla części operacji   -    Operacje odczytu z I2C, SPI czy GPS nie mają zdefiniowanych timeoutów. Na przykład gps.encode() czyta z bufora UART, ale jeśli dane przestaną napływać, program nie ma informacji o tym po czasie. W transmit() jest timeout dla LoRa, ale to wyjątek. Każda operacja, która może blokować, powinna mieć limit czasu, po którym zostaje przerwana, a komponent oznaczony jako uszkodzony.
    20. Ograniczona przewidywalność czasowa   -    Użycie delay(), String, debug oraz blokujących operacji I2C/SPI powoduje, że czas wykonania funkcji jest nieokreślony. W systemie czasu rzeczywistego wymagana jest analiza najgorszego czasu wykonania (WCET) i dowiedzenie, że wszystkie terminy są dotrzymane. Obecny kod nie daje żadnych gwarancji – np. handleBmp() może wykonać się szybko lub wolno w zależności od stanu czujnika.
    21. Możliwe użycie funkcji niedeterministycznych   -    millis() jest zwykle deterministyczne, ale String alokuje pamięć, co może powodować nieprzewidywalne opóźnienia. Ponadto biblioteki Adafruit mogą wewnętrznie używać delay() lub pętli oczekujących, które nie mają gwarantowanego czasu. Standard JSF wymaga eliminacji wszystkich niedeterministycznych konstrukcji, a jeśli to niemożliwe – ścisłej kontroli.
    24. Brak pełnej kontroli overflow/underflow   -    W obliczeniach całkujących (np. data.lsm.lastTotalSpeed += data.lsm.lastTotalAccel * dt) nie ma zabezpieczeń przed przekroczeniem zakresu float ani przed wartościami NaN. W systemie krytycznym po każdej operacji arytmetycznej należy sprawdzić, czy wynik jest skończony i mieści się w oczekiwanym przedziale, a w razie potrzeby nasycić wartość lub przejść w stan awaryjny.
    28. Ograniczone logowanie diagnostyczne   -    W trybie FLIGHT logowane są tylko dane telemetryczne (flashWriteString), ale nie loguje się błędów, zmian stanów, timeoutów ani innych zdarzeń systemowych. W przypadku katastrofy brak logów uniemożliwi analizę przyczyny. System powinien zapisywać pełną diagnostykę (z timestampem) do pamięci nieulotnej, a w trybie DEBUG wysyłać przez Serial.
    29. Zbyt duża złożoność cyklomatyczna (cyclomatic complexity)   -    Funkcje takie jak updateFlightState() mają zagnieżdżone switch i if, co prowadzi do złożoności > 15. handleFlightMode() również zawiera wiele ścieżek. Wysoka złożoność utrudnia testowanie wszystkich możliwych przejść i zwiększa ryzyko niewykrytych błędów. Należy podzielić te funkcje na mniejsze, proste jednostki.

  WŁASNE:
  -zrobić graficzne przedstawienie blokowe jak ma działać cały system

  -dodać funkcję sprawdzającą ile jest dostępnego miejsca na flash - jeśli jest mało powiadom użytkownika
  -w funkcji checkRadio() dodać zapis parametrów sygnału (RSSI, SNR)
*/

volatile bool Rakieta::operationDone = false;

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
  
  // === Czasy dla detekcji ===
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
  loraSettings(2000000, MSBFIRST, SPI_MODE0),
  loraModule(CS_LORA, DIO1_LORA, RST_LORA, BUSY_LORA, spiLora),
  lora(&loraModule),
  
  // === Flash ===
  flashWriteCount(0),
  dumpAddress(0),
  flashDataFile(),
  dumpFile(),
  flashTransport(CS_FLASH, &spiFlash),
  flash(&flashTransport),
  fatfs(),

  // === Zarządzanie czasem ===
  lastFlightModeLoop(0)
{
}

Rakieta::~Rakieta()
{
  lora.clearDio1Action();

  if (flashDataFile.isOpen()) flashCloseFile();
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
  
  debugln("[GPIO] OK");
  
  pinMode(MODE1, INPUT_PULLUP);
  pinMode(MODE2, INPUT_PULLUP);
  pinMode(MODE3, INPUT_PULLUP);
  pinMode(MODE4, INPUT_PULLUP);
  debugln("[DIP SWITCH] OK");
}

void Rakieta::initSPI()
{
  spiFlash.begin();
  debugln("[SPI_FLASH] OK");
  
  spiLora.begin();
  debugln("[SPI_LORA] OK");
  
  spiFast.begin();
  debugln("[SPI_CZUJNIKI] OK");
}

bool Rakieta::initLSM()
{
  if (lsm.begin_SPI(CS_LSM, &spiFast))
  {
    debugln("[LSM] OK");
    errorFlags &= ~LSM_ERROR;
    return true;
  }
  debugln("[LSM] BRAK ODPOWIEDZI");
  errorFlags |= LSM_ERROR;
  return false;
}

bool Rakieta::initADXL()
{
  if (adxl.begin())
  {
    adxl.setRange(ADXL343_RANGE_16_G);
    debugln("[ADXL375] OK, zakres 16G");
    errorFlags &= ~ADXL_ERROR;
    return true;
  }
  debugln("[ADXL375] BRAK ODPOWIEDZI");
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
    debugln("[BMP388 #1] OK");
    errorFlags &= ~BMP1_ERROR;
    return true;
  }
  debugln("[BMP388 #1] BRAK ODPOWIEDZI");
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
    debugln("[BMP388 #2] OK");
    errorFlags &= ~BMP2_ERROR;
    return true;
  }
  debugln("[BMP388 #2] BRAK ODPOWIEDZI");
  errorFlags |= BMP2_ERROR;
  return false;
}

bool Rakieta::initGPS()
{
  gpsSerial.begin(GPS_BAUNDRATE, SERIAL_8N1);
  debugln("[GPS] UART2 OK");
  
  systemSleep(500);
  watchdog();

  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }
  
  if (gps.charsProcessed() > 0)
  {
    debugln("[GPS] Dane odbierane");
    errorFlags &= ~GPS_ERROR;
    return true;
  }
  debugln("[GPS] Brak danych (sprawdź połączenie)");
  errorFlags |= GPS_ERROR;
  return false;
}

void Rakieta::updateLeds(const uint32_t now)
{
  if (led1IsOn && led1OffTime < now)
  {
    digitalWrite(LED_1, LOW);
    debugf("[LED1] Updated");
  }
  if (led2IsOn && led2OffTime < now)
  {
    digitalWrite(LED_2, LOW);
    debugf("[LED2] Updated");
  }
}

void Rakieta::updateBuzzer(const uint32_t now)
{
  if (buzzerIsOn && buzzerOffTime < now)
  {
    digitalWrite(BUZZER, LOW);
    debugf("[BUZZER] Updated");
  }
}

void Rakieta::updateSolenoid(const uint32_t now)
{
  if (solenoid1IsOn && solenoid1OffTime < now)
  {
    digitalWrite(SOLENOID_1, LOW);
    debugln("[SOLENOID1] Updated");
  }
  
  if (solenoid2IsOn && solenoid2OffTime < now)
  {
    digitalWrite(SOLENOID_2, LOW);
    debugln("[SOLENOID2] Updated");
  }
}

void Rakieta::systemReset()
{
  lora.clearDio1Action();
  if (flashDataFile.isOpen()) flashCloseFile();

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

void Rakieta::printSystemMode() const
{
  debug("[SYSTEM MODE] ");
  switch (currentMode)
  {
    case SystemMode::DEBUG:  debugln("DEBUG");  break;
    case SystemMode::FLIGHT: debugln("FLIGHT"); break;
    case SystemMode::DUMP:   debugln("DUMP");   break;
    case SystemMode::SLEEP:  debugln("SLEEP");  break;
  }
}

void Rakieta::printFlightMode() const
{
  debug("[FLIGHT MODE] ");
  switch (currentFlightState)
  {
    case FlightState::IDLE:    debugln("IDLE");     break;
    case FlightState::BOOST:   debugln("BOOST");    break;
    case FlightState::COAST:   debugln("COAST");    break;
    case FlightState::APOGEE:  debugln("APOGEE");   break;
    case FlightState::DESCENT: debugln("DESCENT");  break;
    case FlightState::LANDED:  debugln("LANDED");   break;
  }
}

void Rakieta::handleBattery()
{
  float rawValue = analogRead(BATTERY);
  data.battery.voltage = (static_cast<float>(rawValue) * BATTERY_FULL_VOLTAGE / BATTERY_MAX_READ);
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

    uint32_t now = millis();

    data.lsm.ax = ax;
    data.lsm.ay = ay;
    data.lsm.az = az;
    data.lsm.gx = gx;
    data.lsm.gy = gy;
    data.lsm.gz = gz;
    data.lsm.temp = temp.temperature;
    data.lsm.lastTime = now;

    float dt = (now - data.lsm.lastTime) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    else if (dt <= 0.0f) return;
    else if (dt > 0.001f)
    {
      data.lsm.lastTotalAccel = sqrt(ax * ax + ay * ay + az * az);
      data.lsm.lastTotalSpeed += data.lsm.lastTotalAccel * dt;
      data.lsm.lastTotalAlti += data.lsm.lastTotalSpeed * dt;
      data.lsm.lastTotalRotation = data.lsm.gx + data.lsm.gy + data.lsm.gz;
    }

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

    uint32_t now = millis();
    
    data.adxl.ax = ax;
    data.adxl.ay = ay;
    data.adxl.az = az;
    data.adxl.lastTime = now;
    
    float dt = (now - data.adxl.lastTime) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    else if (dt <= 0.0f) return;
    else if (dt > 0.001f)
    {
      data.adxl.lastTotalAccel = sqrt(ax * ax + ay * ay + az * az);
      data.adxl.lastTotalSpeed += data.adxl.lastTotalAccel * dt;
      data.adxl.lastTotalAlti += data.adxl.lastTotalSpeed * dt;
    }

    errorFlags &= ~ADXL_ERROR;
  }
  else
    errorFlags |= ADXL_ERROR;
}

void Rakieta::handleBmp1()
{
  if (bmp1.performReading())
  {
    float pressure = bmp1.pressure;
    float altitude = bmp1.readAltitude(REFERENCE_PRESSURE_HPA) - offsets.bmp1.alti;
    float temp = bmp1.temperature;
    
    if (isnan(pressure) || isinf(pressure) || pressure < MIN_PRESSURE || pressure > MAX_PRESSURE ||
        isnan(altitude) || isinf(altitude) || altitude < MIN_ALTITUDE || altitude > MAX_ALTITUDE ||
        isnan(temp) || isinf(temp) || temp < MIN_TEMP || temp > MAX_TEMP)
    {
      errorFlags |= BMP1_ERROR;
    }
    else
    {
      uint32_t now = millis();
      
      data.bmp1.pressure = pressure;
      data.bmp1.altitude = altitude;
      data.bmp1.temp = temp;
      data.bmp1.lastTime = now;
      
      if (data.bmp1.maxAltitude < altitude) data.bmp1.maxAltitude = altitude;
      
      float dt = (now - data.bmp1.lastTime) / 1000.0f;
      if (dt > 0.1f) dt = 0.1f;
      else if (dt <= 0.0f) return;
      else if (dt > 0.001f)
      {
          data.bmp1.lastVerticalSpeed = (altitude - data.bmp1.lastAltitude) / dt;
          data.bmp1.lastAltitude = altitude;
      }
      
      errorFlags &= ~BMP1_ERROR;
    }
  }
  else
    errorFlags |= BMP1_ERROR;
}

void Rakieta::handleBmp2()
{
  if (bmp2.performReading())
  {
    float pressure = bmp2.pressure;
    float altitude = bmp2.readAltitude(REFERENCE_PRESSURE_HPA) - offsets.bmp2.alti;
    float temp = bmp2.temperature;
    
    if (isnan(pressure) || isinf(pressure) || pressure < MIN_PRESSURE || pressure > MAX_PRESSURE ||
        isnan(altitude) || isinf(altitude) || altitude < MIN_ALTITUDE || altitude > MAX_ALTITUDE ||
        isnan(temp) || isinf(temp) || temp < MIN_TEMP || temp > MAX_TEMP)
    {
      errorFlags |= BMP2_ERROR;
    }
    else
    {
      uint32_t now = millis();
      
      data.bmp2.pressure = pressure;
      data.bmp2.altitude = altitude;
      data.bmp2.temp = temp;
      data.bmp2.lastTime = now;
      
      if (data.bmp2.maxAltitude < altitude) data.bmp2.maxAltitude = altitude;
      
      float dt = (now - data.bmp2.lastTime) / 1000.0f;
      if (dt > 0.1f) dt = 0.1f;
      else if (dt <= 0.0f) return;
      else if (dt > 0.001f)
      {
        data.bmp2.lastVerticalSpeed = (altitude - data.bmp2.lastAltitude) / dt;
        data.bmp2.lastAltitude = altitude;
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
  handleBmp1();
  handleBmp2();
  handleGPS();
}

void Rakieta::printData() const
{
  debugf("IMU: Accel: %.2f %.2f %.2f G | Gyro: %.2f %.2f %.2f dps | Temp: %.2f *C\n",
                data.lsm.ax, data.lsm.ay, data.lsm.az, data.lsm.gx, data.lsm.gy, data.lsm.gz, data.lsm.temp);
  debugf("IMU: Speed: %.2f m/s | Accel: %.3f m/s^2 | Gyro: %.3f d\n",
                data.lsm.lastTotalSpeed, data.lsm.lastTotalAccel, data.lsm.lastTotalRotation);

  debugf("ADXL: X=%.2f Y=%.2f Z=%.2f G\n", data.adxl.ax, data.adxl.ay, data.adxl.az);
  debugf("ADXL: Speed: %.2f m/s | Accel: %.3f m/s^2\n", data.adxl.lastTotalSpeed, data.adxl.lastTotalAccel);

  debugf("BMP1: P=%.2f hPa, Alt=%.1f m, T=%.2f C\n", data.bmp1.pressure, data.bmp1.altitude, data.bmp1.temp);
  debugf("BMP1: Speed: %.2f\n", data.bmp1.lastVerticalSpeed);
  
  debugf("BMP2: P=%.2f hPa, Alt=%.1f m, T=%.2f C\n", data.bmp2.pressure, data.bmp2.altitude, data.bmp2.temp);
  debugf("BMP2: Speed: %.2f\n", data.bmp2.lastVerticalSpeed);
  
  debugf("GPS: Lat=%.6f Lon=%.6f\n", data.gps.lat, data.gps.lng);
  debugf("GPS: Alt=%.1f m\n",data.gps.alti);
  debugf("GPS: Time=%2d:%2d:%2d:%d\n", data.gps.h, data.gps.m, data.gps.s, data.gps.centi);
  debugf("GPS: Speed:%.2f\n", data.gps.speed);
  debugf("GPS: Course:%.3f\n", data.gps.course);
  debugf("GPS: Sat:%2d\n", data.gps.satNum);
  debugf("GPS: hdop:%2d\n", data.gps.hdop);
}

void Rakieta::setOffsets()
{
  static uint32_t lastOffestSensorsTime = 0;
  static uint32_t lastOffestGpsTime = 0;
  debugln("\n=== Wyznaczanie offsetów ===\n");

  uint8_t num = OFFSETS_SENSORS_READ;
  uint8_t i = 0;
  uint32_t now = 0;

  while (i < num)
  {
    now = millis();
    if (now - lastOffestSensorsTime >= OFFSET_SENSORS_INTERVAL)
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
        offsets.bmp1.alti += bmp1.readAltitude(REFERENCE_PRESSURE_HPA);
        offsets.bmp1.valid++;
      }

      if (bmp2.performReading())
      {
        offsets.bmp2.alti += bmp2.readAltitude(REFERENCE_PRESSURE_HPA);
        offsets.bmp2.valid++;
      }

      watchdog();
      i++;
    }
  }

  offsets.lsm.ax = (offsets.lsm.valid ? (offsets.lsm.ax / static_cast<float>(offsets.lsm.valid)) : 0);
  offsets.lsm.ay = (offsets.lsm.valid ? (offsets.lsm.ay / static_cast<float>(offsets.lsm.valid)) : 0);
  offsets.lsm.az = (offsets.lsm.valid ? (offsets.lsm.az / static_cast<float>(offsets.lsm.valid)) : 0);
  offsets.lsm.gx = (offsets.lsm.valid ? (offsets.lsm.gx / static_cast<float>(offsets.lsm.valid)) : 0);
  offsets.lsm.gy = (offsets.lsm.valid ? (offsets.lsm.gy / static_cast<float>(offsets.lsm.valid)) : 0);
  offsets.lsm.gz = (offsets.lsm.valid ? (offsets.lsm.gz / static_cast<float>(offsets.lsm.valid)) : 0);
  offsets.adxl.ax = (offsets.adxl.valid ? (offsets.adxl.ax / static_cast<float>(offsets.adxl.valid)) : 0);
  offsets.adxl.ay = (offsets.adxl.valid ? (offsets.adxl.ay / static_cast<float>(offsets.adxl.valid)) : 0);
  offsets.adxl.az = (offsets.adxl.valid ? (offsets.adxl.az / static_cast<float>(offsets.adxl.valid)) : 0);
  offsets.bmp1.alti = (offsets.bmp1.valid ? (offsets.bmp1.alti / static_cast<float>(offsets.bmp1.valid)) : 0);
  offsets.bmp2.alti = (offsets.bmp2.valid ? (offsets.bmp2.alti / static_cast<float>(offsets.bmp2.valid)) : 0);

  num = OFFSETS_GPS_READ;
  uint32_t timeout = millis() + 30000;

  while (offsets.gps.valid < num || now < timeout)
  {
    now = millis();
    if (now - lastOffestGpsTime >= OFFSET_GPS_INTERVAL)
    {
      if (gps.location.isValid() && gps.altitude.isValid())
      {
        offsets.gps.lat += gps.location.lat();
        offsets.gps.lng += gps.location.lng();
        offsets.gps.alti += gps.altitude.meters();
        offsets.gps.valid++;
      }
      watchdog();
    }
  }

  offsets.gps.lat = (offsets.gps.valid ? (offsets.gps.lat / static_cast<float>(offsets.gps.valid)) : 0);
  offsets.gps.lng = (offsets.gps.valid ? (offsets.gps.lng / static_cast<float>(offsets.gps.valid)) : 0);
  offsets.gps.alti = (offsets.gps.valid ? (offsets.gps.alti / static_cast<float>(offsets.gps.valid)) : 0);

  offsetsSet = true;
}

void Rakieta::prepareOffsetsMsg(char* buffer, size_t bufferSize)
{
  if (!offsetsSet) return;
  if (buffer == nullptr || bufferSize <= 0) return;

  memset(buffer, 0, bufferSize);

  int16_t written = snprintf(buffer, bufferSize,
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
  int16_t written = snprintf(buffer, bufferSize,
    "RocketData:%lu,%lu,%d,%04X,"
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
    data.gps.lat, data.gps.lng, data.gps.alti,
    data.gps.h, data.gps.m, data.gps.s, data.gps.centi,
    data.gps.speed, data.gps.course, data.gps.satNum, data.gps.hdop,
    
    // LSM6
    data.lsm.ax, data.lsm.ay, data.lsm.az,
    data.lsm.gx, data.lsm.gy, data.lsm.gz,
    data.lsm.temp, data.lsm.lastTotalSpeed,

    // ADXL
    data.adxl.ax, data.adxl.ay, data.adxl.az, data.adxl.lastTotalSpeed,
    
    // BMP
    data.bmp1.temp, data.bmp1.pressure, data.bmp1.altitude, data.bmp1.lastVerticalSpeed,
    data.bmp2.temp, data.bmp2.pressure, data.bmp2.altitude, data.bmp2.lastVerticalSpeed,

    // BATTERY
    data.battery.voltage
  );

  if (written < 0 || static_cast<size_t>(written) >= bufferSize)
  {
    errorFlags |= MSG_TOO_LONG_ERROR;
    snprintf(buffer, bufferSize, "ERROR:MSG_TOO_LONG,%lu", millis());
  }
}

void Rakieta::filterGyro()
{
  if (errorFlags & LSM_ERROR) return;
  data.filtered.gx = FUSION_ALPHA * data.lsm.gx + (1.0f - FUSION_ALPHA) * data.filtered.gx;
  data.filtered.gy = FUSION_ALPHA * data.lsm.gy + (1.0f - FUSION_ALPHA) * data.filtered.gy;
  data.filtered.gz = FUSION_ALPHA * data.lsm.gz + (1.0f - FUSION_ALPHA) * data.filtered.gz;
}

void Rakieta::calculateOrientation()
{
  data.filtered.pitch = atan2(-data.filtered.ax, sqrt(data.filtered.ay*data.filtered.ay + data.filtered.az*data.filtered.az));
  data.filtered.roll = atan2(data.filtered.ay, data.filtered.az);
}

void Rakieta::filterAcceleration()
{
  float weightLSM = 0.6f;
  float weightADXL = 0.4f;

  if (isnan(data.lsm.lastTotalAccel) || isinf(data.lsm.lastTotalAccel || data.lsm.lastTotalAccel > LSM_MAX_G)) weightLSM = 0.0f;
  if (isnan(data.adxl.lastTotalAccel) || isinf(data.adxl.lastTotalAccel || data.adxl.lastTotalAccel > ADXL_MAX_G)) weightADXL = 0.0f;
  
  float ax_fused = 0.0f, ay_fused = 0.0f, az_fused = 0.0f;
  float totalWeight = 0.0f;
  
  if (!(errorFlags & LSM_ERROR) && weightLSM > 0.0f)
  {
    ax_fused += data.lsm.ax * weightLSM;
    ay_fused += data.lsm.ay * weightLSM;
    az_fused += data.lsm.az * weightLSM;
    totalWeight += weightLSM;
  }
  if (!(errorFlags & ADXL_ERROR) && weightADXL > 0.0f)
  {
    ax_fused += data.adxl.ax * weightADXL;
    ay_fused += data.adxl.ay * weightADXL;
    az_fused += data.adxl.az * weightADXL;
    totalWeight += weightADXL;
  }
  if (totalWeight > 0.0f)
  {
    ax_fused /= totalWeight;
    ay_fused /= totalWeight;
    az_fused /= totalWeight;
    
    data.filtered.ax = FUSION_ALPHA * ax_fused + (1.0f - FUSION_ALPHA) * data.filtered.ax;
    data.filtered.ay = FUSION_ALPHA * ay_fused + (1.0f - FUSION_ALPHA) * data.filtered.ay;
    data.filtered.az = FUSION_ALPHA * az_fused + (1.0f - FUSION_ALPHA) * data.filtered.az;
  }
}

void Rakieta::filterSpeed()
{
  float currentSpeed = data.filtered.speed;
  float mach = currentSpeed / SPEED_OF_SOUND;

  float fusedSpeed = 0.0f;
  float totalWeight = 0.0f;

  // Barometry (tylko jeśli mach < 0.9)
  if (mach < MACH_IGNORE_BARO)
  {
    if (!(errorFlags & BMP1_ERROR) && !isnan(data.bmp1.lastVerticalSpeed))
    {
      fusedSpeed += fabs(data.bmp1.lastVerticalSpeed) * WEIGHT_BMP_SPEED;
      totalWeight += WEIGHT_BMP_SPEED;
    }
    if (!(errorFlags & BMP2_ERROR) && !isnan(data.bmp2.lastVerticalSpeed))
    {
      fusedSpeed += fabs(data.bmp2.lastVerticalSpeed) * WEIGHT_BMP_SPEED;
      totalWeight += WEIGHT_BMP_SPEED;
    }
  }
  
  // LSM (zawsze, chyba że błąd)
  if (!(errorFlags & LSM_ERROR) && !isnan(data.lsm.lastTotalSpeed))
  {
    fusedSpeed += data.lsm.lastTotalSpeed * WEIGHT_LSM_SPEED;
    totalWeight += WEIGHT_LSM_SPEED;
  }

  // ADXL (zawsze, chyba że błąd)
  if (!(errorFlags & ADXL_ERROR) && !isnan(data.adxl.lastTotalSpeed))
  {
    fusedSpeed += data.adxl.lastTotalSpeed * WEIGHT_ADXL_SPEED;
    totalWeight += WEIGHT_ADXL_SPEED;
  }

  // GPS (tylko jeśli mach < 0.5 i dane poprawne)
  if (mach < MACH_IGNORE_GPS && data.gps.speed > 0.0f && !isnan(data.gps.speed))
  {
    fusedSpeed += data.gps.speed * WEIGHT_GPS_SPEED;
    totalWeight += WEIGHT_GPS_SPEED;
  }

  if (totalWeight > 0.0f)
  {
    fusedSpeed /= totalWeight;
    data.filtered.speed = FUSION_ALPHA * fusedSpeed + (1.0f - FUSION_ALPHA) * data.filtered.speed;
  }
}

void Rakieta::filterAlti()
{
  float fusedAlti = 0.0f;
  float totalWeight = 0.0f;
  float mach = data.filtered.speed / SPEED_OF_SOUND;

  // BMP
  if (mach < MACH_IGNORE_BARO)
  {
    if (!(errorFlags & BMP1_ERROR) && !isnan(data.bmp1.altitude))
    {
      fusedAlti += data.bmp1.altitude * WEIGHT_BMP_ALTI;
      totalWeight += WEIGHT_BMP_ALTI;
    }
    
    if (!(errorFlags & BMP2_ERROR) && !isnan(data.bmp2.altitude))
    {
      fusedAlti += data.bmp2.altitude * WEIGHT_BMP_ALTI;
      totalWeight += WEIGHT_BMP_ALTI;
    }
  }

  // LSM (wysokość całkowana, jeśli dostępna)
  if (!(errorFlags & LSM_ERROR) && !isnan(data.lsm.lastTotalAlti))
  {
    fusedAlti += data.lsm.lastTotalAlti * WEIGHT_LSM_ALTI;
    totalWeight += WEIGHT_LSM_ALTI;
  }

  // ADXL (wysokość całkowana, jeśli dostępna)
  if (!(errorFlags & ADXL_ERROR) && !isnan(data.adxl.lastTotalAlti))
  {
    fusedAlti += data.adxl.lastTotalAlti * WEIGHT_ADXL_ALTI;
    totalWeight += WEIGHT_ADXL_ALTI;
  }

  // GPS
  if (mach < MACH_IGNORE_GPS && !(errorFlags & GPS_ERROR) && !isnan(data.gps.alti))
  {
    fusedAlti += data.gps.alti * WEIGHT_GPS_ALTI;
    totalWeight += WEIGHT_GPS_ALTI;
  }

  if (totalWeight > 0.0f)
  {
    fusedAlti /= totalWeight;
    data.filtered.alti = FUSION_ALPHA * fusedAlti + (1.0f - FUSION_ALPHA) * data.filtered.alti;
  }
}

bool Rakieta::initLora()
{
  int16_t state = lora.begin(FREQUENCY, BANDWIDTH, SF, CODING_RATE, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, POWER, PREAMBLE_LENGTH);

  if (state == RADIOLIB_ERR_NONE)
  {
    lora.setDio1Action(Rakieta::setOperationFlag);
    char buffer[256];
    prepareOffsetsMsg(buffer, sizeof(buffer));
    debugln(buffer);
    debugln("[LoRa] OK");
    errorFlags &= ~LORA_ERROR;
    startListening();
    return true;
  }
  debug("[LoRa] Błąd: ");
  debugln(state);
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

  int16_t written = snprintf(buffer, bufferSize,
    "=== RADIO STATUS ===\nFrequency:%fMHz,Moc:%fdBm,SF:%luBW:%lukHz,CR:4/%lu",
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

  debugln("\n\nSending: ");

  for (uint8_t i=0; i<ARRAY_SIZE; i++)
    debugHex(txPacket[i]);
  debugln("");

  for (uint8_t i=0; i<ARRAY_SIZE; i++)
    for (uint8_t j=7; j>=0; --j)
      debug((txPacket[i] >> j) & 1);

  debugln("\nEnd of the message.\n\n");

  transmit(txPacket, ARRAY_SIZE);

  led1IsOn = true;
  led1OffTime = millis() + LED_DELAY;
  digitalWrite(LED_1, HIGH);

  debugln("Sending DONE");
}

void Rakieta::transmit(const uint8_t* msg, const size_t len)
{
  static uint32_t loraMsgStartTime= 0;

  if (msg == nullptr || len == 0 || len > 255) return;

  __disable_irq();
  operationDone = false;

  if (!operationDone)
  {
    if (millis() - loraMsgStartTime >= TX_TIMEOUT)
    {
      debugln("ERROR: LoRa timeout, forcing radio reset");
      lora.standby();
      systemSleep(1);
      startListening();
      operationDone = true;
    }
    else
    {
      debugln("LoRa busy");
      return;
    }
  }
  __enable_irq();
  
  debug(F("Sending: "));
  Serial.write(msg, len);
  debugln();

  loraMsgStartTime = millis();
  int16_t state = lora.startTransmit(reinterpret_cast<const uint8_t*>(msg), len);

  if (state != RADIOLIB_ERR_NONE)
  {
    debug(F("Transmit error: "));
    debugln(state);
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
  int16_t state = lora.startReceive();
  if (state != RADIOLIB_ERR_NONE)
  {
    debug(F("Error starting listening: "));
    debugln(state);
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
    char offBuf[256];
    prepareOffsetsMsg(offBuf, sizeof(offBuf));
    transmit(offBuf, sizeof(offBuf));
  }
  else if (strcmp(command, "GET_DATA") == 0)
  {
    char dataBuf[256];
    prepareDataLineMsg(dataBuf, sizeof(dataBuf));
    transmit(dataBuf, sizeof(dataBuf));
  }
  else
  {
    char errBuf[25];
    snprintf(errBuf, sizeof(errBuf), "ERROR:UNKNOWN_COMMAND");
    transmit(errBuf, sizeof(errBuf));
  }
}

void Rakieta::checkRadio()
{
  uint8_t buffer[256];

  int16_t state = lora.readData(buffer, sizeof(buffer) - 1);

  if (state == RADIOLIB_ERR_NONE)
  {
    buffer[state] = '\0';
    /*
      debugln(F("== MESSAGE RECEIVED =="));
      debug(F("Message: "));
      debugln(msg);
      debug(F("RSSI: "));
      debug(lora.getRSSI());
      debug(F(" dBm, SNR: "));
      debug(lora.getSNR());
      debugln(F(" dB"));
    */
    handleCommand(reinterpret_cast<char*>(buffer));
    flashWriteString(reinterpret_cast<char*>(buffer));
    errorFlags &= ~LORA_ERROR;
  }
  else if (state != RADIOLIB_ERR_RX_TIMEOUT)
  {
    errorFlags |= LORA_ERROR;
  }
  else
  {
    debug(F("Data reading error: "));
    debugln(state);
    errorFlags |= LORA_ERROR;
  }
}

void Rakieta::sendGpsOffset()
{
  if (!offsetsSet) return;

  char buffer[128];
  const size_t bufferSize = sizeof(buffer);
  memset(buffer, 0, bufferSize);

  int16_t written = snprintf(buffer, bufferSize,
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
  
  transmit(buffer, bufferSize);
}

bool Rakieta::initFlash()
{
  if (flash.begin())
  {
    uint32_t jedec = flash.getJEDECID();
    debug("[FLASH] OK, JEDEC: 0x");
    debugHex(jedec);
    errorFlags &= ~FLASH_ERROR;
    return true;
  }
  debugln("[FLASH] BRAK ODPOWIEDZI");
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
  
  debugln("Too many data files!");
  errorFlags |= FLASH_FILE_ERROR;
  return false;
}

bool Rakieta::flashOpenNewFile()
{
  if (flashDataFile.isOpen()) flashCloseFile();

  char fileName[15];
  if (!flashFindNextFileNumber(fileName, sizeof(fileName)))
  {
    debugln("Cannot open a file");
    return false;
  }
  watchdog();
  
  flashDataFile = fatfs.open(fileName, FILE_WRITE);
  if (!flashDataFile)
  {
    debugln("Failed to open file for writing");
    errorFlags |= FLASH_FILE_ERROR;
    return false;
  }
  watchdog();
  
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
  watchdog();
  
  debug("File opened: "); 
  debugln(fileName);
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
    debugln("Flash not ready!");
    return;
  }
  
  if (!flashDataFile)
  {
    debugln("File not open!");
    errorFlags |= FLASH_FILE_ERROR;
    return;
  }
  
  int16_t written = flashDataFile.println(msg);

  if (written <= 0)
  {
    debugln("Write failed!");
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
  if (flashDataFile.isOpen())
  {
    flashDataFile.flush();
    flashWriteCount = 0;
    debugln("[FLASH] Buffer flushed.");
    watchdog();
  }
}

void Rakieta::flashCloseFile()
{
  if (flashDataFile.isOpen())
  {
    flashFlushBuffer();

    flashDataFile.close();
    debugln("[FLASH] File closed.");
    watchdog();
  }
}

bool Rakieta::flashRecoverAfterReset()
{
  if (!fatfs.begin(&flash))
  {
    debugln("[FLASH] Mount failed. Attempting format...");
    if (!fatfs.begin(&flash, true))
    {
      debugln("[FLASH] Format failed!");
      errorFlags |= FLASH_ERROR;
      return false;
    }
    debugln("[FLASH] Format successful.");
    watchdog();
  }

  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    debugln("[FLASH] Cannot open root directory.");
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
          uint16_t num = atoi(fileName + 5);
          if (num > maxNumber) maxNumber = num;
        }
      }
    }
    watchdog();
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
      debugln("[FLASH] Cannot open/create file for recovery!");
      return false;
    }
  }
  watchdog();

  flashDataFile.seekEnd();
  flashDataFile.println("\n--- RECOVERY AFTER RESET ---");
  flashDataFile.flush();
  watchdog();
  debugf("[FLASH] Recovery OK. File: %s\n", filename);
  return true;
}

void Rakieta::flashDumpFileList()
{
  debugln("\n=== FLASH FILE LIST ===");

  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    debugln("[FLASH] Cannot open root directory.");
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
        debug(" - ");
        debug(fileName);
        debug(" (size: ");
        debug(file.size());
        debugln(" bytes)");
      }
    }
    watchdog();
    file.close();
  }
  root.close();
  debugln("========================\n");
}

void Rakieta::flashDumpFileData(const uint16_t fileNumber)
{
  char filename[32];
  snprintf(filename, sizeof(filename), "data_%04d.csv", fileNumber);

  File32 dumpFile = fatfs.open(filename, FILE_READ);
  if (!dumpFile)
  {
    debugf("[FLASH] Cannot open file %s\n", filename);
    return;
  }
  watchdog();

  debugf("=== DUMP OF %s ===\n", filename);
  char line[256];

  while (dumpFile.available())
  {
    int16_t bytesRead = 0;
    while (dumpFile.available() && bytesRead < (int)sizeof(line) - 1)
    {
      char c = dumpFile.read();
      line[bytesRead++] = c;
      if (c == '\n') break;
    }

    line[bytesRead] = '\0';
    debug(line);
  }
  watchdog();

  dumpFile.close();
  debugln("=== END OF FILE ===\n");
}

void Rakieta::flashDumpLastFile()
{
  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    debugln("[FLASH] Cannot open root directory.");
    return;
  }

  uint16_t lastNumber = 0;
  char fileName[20];
  File32 dumpFile;

  while ((dumpFile = root.openNextFile()))
  {
    if (!dumpFile.isDirectory())
    {
      dumpFile.getName(fileName, sizeof(fileName));

      if (strncmp(fileName, "data_", 5) == 0)
      {
        char* dotPos = strrchr(fileName, '.');
        if (dotPos != nullptr && strcmp(dotPos, ".csv") == 0)
        {
          uint16_t num = atoi(fileName + 5);
          if (num > lastNumber) lastNumber = num;
        }
      }
    }
    watchdog();
    dumpFile.close();
  }
  root.close();

  if (lastNumber == 0)
  {
    debugln("[FLASH] No data files found.");
    return;
  }

  flashDumpFileData(lastNumber);
}

// do zastanowienia się
bool Rakieta::detectLaunch()
{
  if (isnan(data.lsm.lastTotalAccel) || isinf(data.lsm.lastTotalAccel))
  {
    return false;  // Nie możemy podjąć decyzji – dane uszkodzone
  }

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
  if (isnan(data.lsm.lastTotalAccel) || isinf(data.lsm.lastTotalAccel))
  {
    return false;  // Nie możemy podjąć decyzji – dane uszkodzone
  }

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
  if (isnan(data.bmp1.lastVerticalSpeed) || isinf(data.bmp1.lastVerticalSpeed) ||
      isnan(data.bmp1.altitude) || isinf(data.bmp1.altitude))
  {
    return false;  // Nie możemy podjąć decyzji – dane uszkodzone
  }

  // Apogeum gdy prędkość spada poniżej progu i osiągnięto maksimum wysokości
  if (data.bmp1.lastVerticalSpeed <= APOGEE_VELOCITY_THRESHOLD && data.bmp1.altitude <= data.bmp1.maxAltitude - APOGEE_ALTITUDE_HYSTERESIS)
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
  if (isnan(data.bmp1.lastVerticalSpeed) || isinf(data.bmp1.lastVerticalSpeed))
  {
    return false;  // Nie możemy podjąć decyzji – dane uszkodzone
  }

  if (abs(data.bmp1.lastVerticalSpeed) < LANDING_VELOCITY_THRESHOLD)
  {
    uint32_t now = millis();
    if (landedDetectTime == 0) landedDetectTime = now;
    if (now - landedDetectTime >= LANDING_DEBOUNCE_MS) return true;
  }
  else landedDetectTime = 0;
  
  return false;
}

bool Rakieta::checkDeploymentConditions(const ParachuteType type)
{
  if (type == ParachuteType::DROGUE)
  {
    if (data.filtered.speed > DEPLOY_DROGUE_MAX_SPEED || data.filtered.alti < DEPLOY_DROGUE_MIN_ALTITUDE) return false;
    return true;
  }
  else if (type == ParachuteType::MAIN)
  {
    if (data.filtered.speed > DEPLOY_MAIN_MAX_SPEED || data.filtered.alti < DEPLOY_MAIN_MIN_ALTITUDE) return false;
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

  flashWriteString(buffer);
  transmit(buffer, sizeof(buffer));
  debugln(buffer);
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
  static bool inFlight = false;
  uint32_t now = millis();
  
  switch (currentFlightState)
  {
    case FlightState::IDLE:
      if (detectLaunch())
      {
        currentFlightState = FlightState::BOOST;
        launchDetectTime = now;
        inFlight = true;
        debugln("[FLIGHT] -> BOOST");
      }
      break;
    case FlightState::BOOST:
    {
      if (detectBurnout())
      {
        currentFlightState = FlightState::COAST;
        burnoutDetectTime = now;
        debugln("[FLIGHT] -> COAST");
      }
      break;
    }
    case FlightState::COAST:
    {
      if (detectApogee())
      {
        currentFlightState = FlightState::APOGEE;
        apogeeDetectTime = now;
        debugln("[FLIGHT] -> APOGEE");
      }
      break;
    }
    case FlightState::APOGEE:
    {
      if ((!drogueDeployed && checkDeploymentConditions(ParachuteType::DROGUE)) || (millis() - launchDetectTime > DROGUE_PARASHUTE_TIMEOUT))
      {
        drogueParashuteOpen();
        drogueDeployed = true;
        currentFlightState = FlightState::DESCENT;
        descentDetectTime = now;
        debugln("[FLIGHT] Drogue parachute deployed.");
        debugln("[FLIGHT] -> DESCENT");
      }
      break;
    }
    case FlightState::DESCENT:
    {
      if ((!mainDeployed && checkDeploymentConditions(ParachuteType::MAIN)) || (millis() - launchDetectTime > MAIN_PARASHUTE_TIMEOUT))
      {
        mainParashuteOpen();
        mainDeployed = true;
        debugln("[FLIGHT] Main parachute deployed.");
      }

      if (detectLanding())
      {
        currentFlightState = FlightState::LANDED;
        landedDetectTime = now;
        debugln("[FLIGHT] -> LANDED");
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
  static uint32_t lastWatchdogTime = 0;

  uint32_t now = millis();
  if (now - lastWatchdogTime >= WATCHDOG_INTERVAL)
  {
    IWDG1->KR = 0xAAAA;
    lastWatchdogTime = now;
  }
}

void Rakieta::reinitComponent(bool (Rakieta::*initFunc)())
{
  debugln("[ERROR] Reinit component");
  
  bool success = (this->*initFunc)();
  if (success) debugln("[ERROR] Component recovered");
  else debugln("[ERROR] Component recovery FAILED");
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

  watchdog();
  
  // 4. Dodatkowe flagi plików (tylko loguj, nie reinicjalizuj)
  if (errorFlags & FLASH_FILE_ERROR)
  {
    debugln("[ERROR] Flash file error – try to reopen file");
    if (flashDataFile.isOpen()) flashCloseFile();
    if (!flashDataFile.isOpen()) flashOpenNewFile();
    if (flashDataFile.isOpen()) errorFlags &= ~FLASH_FILE_ERROR;
    watchdog();
  }
}





// === Obsługa trybów ===
void Rakieta::handleDebugMode()  /// do zmieniania w locie
{
  debugln("\n=== TRYB DEBUG: wypisywanie czujnikow ===");
  debugln("----------------------------------------");

  checkRadio();

  /// trzeba usuwać i sprawdzać po kolei
  handleLsm();
  handleBmp1();
  handleBmp2();
  handleAdxl();
  handleGPS();
  handleBattery();
  printData();
}

void Rakieta::handleFlightMode()
{
  checkRadio();
  
  uint32_t interval = (currentFlightState == FlightState::LANDED) ? INTERVAL_IDLE : INTERVAL_FLIGHT;
  uint32_t now = millis();

  if (now - lastFlightModeLoop >= interval)
  {
    lastFlightModeLoop = now;

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
  /*
  const uint32_t startAddr = 0;
  const uint32_t totalLength = 512;
  uint8_t buffer[16];
  char ascii[17];
  
  for (uint32_t i = 0; i < 16 && dumpAddress < totalLength; i++)
  {
    flash.readBuffer(dumpAddress, buffer, 16);
    
    debugf("%08X: ", dumpAddress);
    for (uint8_t j = 0; j < 16; j++)
    {
      debugf("%02X ", buffer[j]);
      ascii[j] = (buffer[j] >= 32 && buffer[j] <= 126) ? buffer[j] : '.';
    }
    ascii[16] = '\0';
    debugf(" | %s\n", ascii);
    
    dumpAddress += 16;
  }
  
  if (dumpAddress >= totalLength)
  {
    debugln("\n--- Koniec dump. Przechodzę do SLEEP ---");
    currentMode = SystemMode::SLEEP;
  }
  */
}

void Rakieta::handleSleepMode()  /// chyba będzie ok
{
  SystemMode oldMode = currentMode;
  setSystemMode();
  
  if (currentMode != oldMode)
  {
    debugln("Zmiana trybu – resetowanie systemu...");
    systemSleep(100);
    systemReset();
  }
  else
  {
    currentMode = SystemMode::SLEEP;
  }
  
  // TODO: tu można dodać faktyczne uśpienie (__WFI())
  systemSleep(10000);
}

void Rakieta::handleMode(const uint32_t now)
{
  static uint32_t lastFlightLoop = 0;
  static uint32_t lastDebugPrint = 0;
  static uint32_t lastDumpProgress = 0;
  static uint32_t lastSleepCheck = 0;

  switch (currentMode)
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
  debugln("\n=== ROCKET ON-BOARD COMPUTER INIT ===\n");

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
    debugln(errorFlags);
  }

  for (uint8_t i=0; i<ile; i++)
  {
    digitalWrite(LED_1, HIGH);
    systemSleep(200);
    digitalWrite(LED_1, LOW);
    systemSleep(200);
  }
  
  if (currentMode != SystemMode::SLEEP) setOffsets();

  initWatchdog();

  debugln("\n=== INICJALIZACJA ZAKOŃCZONA ===\n");
}

void Rakieta::loop()
{
  static uint32_t lastErrorCheckTime = 0;

  uint32_t now = millis();
  
  if (now - lastErrorCheckTime >= INTERVAL_ERROR_CHECK)
  {
    lastErrorCheckTime = now;
    handleErrors();
  } 

  watchdog();
  handleMode(now);
  watchdog();
  updateLeds(now);
  watchdog();
  updateBuzzer(now);
  watchdog();
  updateSolenoid(now);
  watchdog();
  systemSleep(5);
}











