#include "Rakieta.h"


/*
  -"AIR VEHICLE C++ CODING STANDARDS FOR THE SYSTEM DEVELOPMENT AND DEMONSTRATION PROGRAM"
*/

volatile bool Rakieta::operationDone = false;

/**
 * @brief Default constructor. Initializes all member variables to default values.
 */
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
  summarySend(0),
  flashWriteCount(0),
  flashDataFile(),
  dumpFile(),
  flashTransport(CS_FLASH, &spiFlash),
  flash(&flashTransport),
  fatfs(),

  // === Zarządzanie czasem ===
  lastFlightModeLoop(0)
{
}

/**
 * @brief Destructor. Clears LoRa DIO1 action and closes flash file if open.
 */
Rakieta::~Rakieta()
{
  lora.clearDio1Action();

  if (flashDataFile.isOpen()) flashCloseFile();
}

/**
 * @brief Initializes GPIO pins for LEDs, buzzer, solenoids, battery read and DIP switches.
 * @return void
 */
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

/**
 * @brief Initializes SPI interfaces for flash, LoRa and fast sensors.
 * @return void
 */
void Rakieta::initSPI()
{
  spiFlash.begin();
  debugln("[SPI_FLASH] OK");
  
  spiLora.begin();
  debugln("[SPI_LORA] OK");
  
  spiFast.begin();
  debugln("[SPI_CZUJNIKI] OK");
}

/**
 * @brief Initializes LSM6DSO sensor via SPI.
 * @return true if initialization succeeded, false otherwise.
 */
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

/**
 * @brief Initializes ADXL375 sensor via SPI.
 * @return true if initialization succeeded, false otherwise.
 */
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

/**
 * @brief Initializes first BMP388 sensor via SPI.
 * @return true if initialization succeeded, false otherwise.
 */
bool Rakieta::initBMP1()
{
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

/**
 * @brief Initializes second BMP388 sensor via SPI.
 * @return true if initialization succeeded, false otherwise.
 */
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

/**
 * @brief Initializes GPS module over UART and waits for first valid data.
 * @return true if GPS data is being received, false otherwise.
 */
bool Rakieta::initGPS()
{
  gpsSerial.begin(GPS_BAUNDRATE, SERIAL_8N1);
  debugln("[GPS] UART2 OK");
  
  watchdog();

  uint32_t gpsTimeout = millis() + 2000;
  while (millis() < gpsTimeout)
  {
    while (gpsSerial.available())
    {
      gps.encode(gpsSerial.read());
    }
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

/**
 * @brief Updates LED1 and LED2 state based on timeouts.
 * @param now Current millis() value.
 * @return void
 */
void Rakieta::updateLeds(const uint32_t now)
{
  if (led1IsOn && led1OffTime < now)
  {
    digitalWrite(LED_1, LOW);
    led1IsOn = false;
    debugf("[LED1] Updated");
  }
  if (led2IsOn && led2OffTime < now)
  {
    digitalWrite(LED_2, LOW);
    led2IsOn = false;
    debugf("[LED2] Updated");
  }
}

/**
 * @brief Updates buzzer state based on timeout.
 * @param now Current millis() value.
 * @return void
 */
void Rakieta::updateBuzzer(const uint32_t now)
{
  if (buzzerIsOn && buzzerOffTime < now)
  {
    digitalWrite(BUZZER, LOW);
    buzzerIsOn = false;
    debugf("[BUZZER] Updated");
  }
}

/**
 * @brief Updates solenoid1 and solenoid2 state based on timeouts.
 * @param now Current millis() value.
 * @return void
 */
void Rakieta::updateSolenoid(const uint32_t now)
{
  if (solenoid1IsOn && solenoid1OffTime < now)
  {
    digitalWrite(SOLENOID_1, LOW);
    solenoid1IsOn = false;
    debugln("[SOLENOID1] Updated");
  }
  
  if (solenoid2IsOn && solenoid2OffTime < now)
  {
    digitalWrite(SOLENOID_2, LOW);
    solenoid2IsOn = false;
    debugln("[SOLENOID2] Updated");
  }
}

/**
 * @brief Performs a full system reset using NVIC reset.
 * @return void
 */
void Rakieta::systemReset()
{
  lora.clearDio1Action();
  if (flashDataFile.isOpen()) flashCloseFile();

  NVIC_SystemReset();
}

/**
 * @brief Puts the system into sleep mode for a given time using HAL.
 * @param time Sleep duration in milliseconds.
 * @return void
 */
void Rakieta::systemSleep(const uint32_t time)
{
  // delay(time);
  HAL_SuspendTick();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  HAL_Delay(time);
  HAL_ResumeTick();
}

/**
 * @brief Reads DIP switches and sets the current system mode (DEBUG, FLIGHT, DUMP, SLEEP).
 * @return void
 */
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

/**
 * @brief Prints the current system mode to debug serial.
 * @return void
 */
void Rakieta::printSystemMode() const
{
  debug("[SYSTEM MODE] ");
  switch (currentMode)
  {
    case SystemMode::DEBUG:  debugln("DEBUG");           break;
    case SystemMode::FLIGHT: debugln("FLIGHT");          break;
    case SystemMode::DUMP:   debugln("DUMP");            break;
    case SystemMode::SLEEP:  debugln("SLEEP");           break;
    default:                 debugln("!! INCORRECT !!"); break;
  }
}

/**
 * @brief Prints the current flight state (IDLE, BOOST, COAST, APOGEE, DESCENT, LANDED) to debug serial.
 * @return void
 */
void Rakieta::printFlightMode() const
{
  debug("[FLIGHT MODE] ");
  switch (currentFlightState)
  {
    case FlightState::IDLE:    debugln("IDLE");            break;
    case FlightState::BOOST:   debugln("BOOST");           break;
    case FlightState::COAST:   debugln("COAST");           break;
    case FlightState::APOGEE:  debugln("APOGEE");          break;
    case FlightState::DESCENT: debugln("DESCENT");         break;
    case FlightState::LANDED:  debugln("LANDED");          break;
    default:                   debugln("!! INCORRECT !!"); break;
  }
}

/**
 * @brief Reads battery voltage from analog pin and stores it in data structure.
 * @return void
 */
void Rakieta::handleBattery()
{
  float rawValue = analogRead(BATTERY);
  data.battery.voltage = (static_cast<float>(rawValue) * BATTERY_FULL_VOLTAGE / BATTERY_MAX_READ);
}

/**
 * @brief Reads LSM6DSO sensor data, applies offsets, integrates acceleration and updates data structure.
 * @return void
 */
void Rakieta::handleLsm()
{
  sensors_event_t accel, gyro, temp;
  if (lsm.getEvent(&accel, &gyro, &temp))
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
    data.lsm.lastTime = now;

    if (dt > 0.1f) dt = 0.1f;
    else if (dt <= 0.0f) return;
    else if (dt > 0.001f)
    {
      data.lsm.lastTotalAccel = sqrt(ax * ax + ay * ay + az * az);
      data.lsm.lastTotalSpeed += data.lsm.lastTotalAccel * dt;
      data.lsm.lastTotalAlti += data.lsm.lastTotalSpeed * dt;
    }

    errorFlags &= ~LSM_ERROR;
  }
  else
    errorFlags |= LSM_ERROR;
}

/**
 * @brief Reads ADXL375 sensor data, applies offsets, integrates acceleration and updates data structure.
 * @return void
 */
void Rakieta::handleAdxl()
{
  sensors_event_t event;
  if (adxl.getEvent(&event))
  {
    float ax = event.acceleration.x - offsets.adxl.ax;  /// dorobić chyba trzeba będzie mnożnik z 16G do 200G: ADXL375_MG2G_MULTIPLIER
    float ay = event.acceleration.y - offsets.adxl.ay;  /// dorobić chyba trzeba będzie mnożnik z 16G do 200G: ADXL375_MG2G_MULTIPLIER
    float az = event.acceleration.z - offsets.adxl.az;  /// dorobić chyba trzeba będzie mnożnik z 16G do 200G: ADXL375_MG2G_MULTIPLIER
    
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
    data.adxl.lastTime = now;
  
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

/**
 * @brief Reads first BMP388 sensor data (pressure, altitude, temperature) and updates vertical speed.
 * @return void
 */
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
      data.bmp1.pressure = pressure;
      data.bmp1.altitude = altitude;
      data.bmp1.temp = temp;
      
      if (data.bmp1.maxAltitude < altitude) data.bmp1.maxAltitude = altitude;
      
      uint32_t now = millis();
      float dt = (now - data.bmp1.lastTime) / 1000.0f;
      data.bmp1.lastTime = now;
      
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

/**
 * @brief Reads second BMP388 sensor data (pressure, altitude, temperature) and updates vertical speed.
 * @return void
 */
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
      data.bmp2.pressure = pressure;
      data.bmp2.altitude = altitude;
      data.bmp2.temp = temp;
      
      if (data.bmp2.maxAltitude < altitude) data.bmp2.maxAltitude = altitude;
      
      uint32_t now = millis();
      float dt = (now - data.bmp2.lastTime) / 1000.0f;
      data.bmp2.lastTime = now;
      
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

/**
 * @brief Reads and parses GPS data, updates position, altitude, time, speed, course, satellites and HDOP.
 * @return void
 */
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

/**
 * @brief Calls all sensor handlers (battery, LSM, ADXL, BMP1, BMP2, GPS) to update raw data.
 * @return void
 */
void Rakieta::readSensorsData()
{
  handleBattery();
  handleLsm();
  handleAdxl();
  handleBmp1();
  handleBmp2();
  handleGPS();
}

/**
 * @brief Prints all current sensor and filtered data to debug serial.
 * @return void
 */
void Rakieta::printData() const
{
  debugf("IMU: Accel: %.2f %.2f %.2f G | Gyro: %.2f %.2f %.2f dps | Temp: %.2f *C\n",
         data.lsm.ax, data.lsm.ay, data.lsm.az, data.lsm.gx, data.lsm.gy, data.lsm.gz, data.lsm.temp);
  debugf("IMU: Alti: %.2f m | Speed: %.2f m/s | Accel: %.3f m/s^2\n",
         data.lsm.lastTotalAlti, data.lsm.lastTotalSpeed, data.lsm.lastTotalAccel);

  debugf("ADXL: Accel: %.2f %.2f %.2f G\n", data.adxl.ax, data.adxl.ay, data.adxl.az);
  debugf("ADXL: Alti: %.2f m | Speed: %.2f m/s | Accel: %.3f m/s^2\n",
         data.adxl.lastTotalAlti, data.adxl.lastTotalSpeed, data.adxl.lastTotalAccel);

  debugf("BMP1: P=%.2f hPa, Alt=%.1f m, T=%.2f C\n", data.bmp1.pressure, data.bmp1.altitude, data.bmp1.temp);
  debugf("BMP1: Alti: %.2f m | Speed: %.2f m/s\n",
         data.bmp1.lastAltitude, data.bmp1.lastVerticalSpeed);
  
  debugf("BMP2: P=%.2f hPa, Alt=%.1f m, T=%.2f C\n", data.bmp2.pressure, data.bmp2.altitude, data.bmp2.temp);
  debugf("BMP2: Alti: %.2f m | Speed: %.2f m/s\n",
         data.bmp2.lastAltitude, data.bmp2.lastVerticalSpeed);
  
  debugf("GPS: Lat=%.6f Lon=%.6f\n Alt=%.1f m", data.gps.lat, data.gps.lng, data.gps.alti);
  debugf("GPS: Time=%2d:%2d:%2d:%04d\n", data.gps.h, data.gps.m, data.gps.s, data.gps.centi);
  debugf("GPS: Speed:%.2f Course:%.3f\n", data.gps.speed, data.gps.course);
  debugf("GPS: Sat:%2d hdop:%2d\n", data.gps.satNum, data.gps.hdop);

  debugf("BAT: %03.2f\n", data.battery.voltage);

  debugf("Filtered: Accel: %.2f %.2f %.2f G | Gyro: %.2f %.2f %.2f dps\n",
         data.filtered.ax, data.filtered.ay, data.filtered.az, data.filtered.gx, data.filtered.gy, data.filtered.gz);
  debugf("Filtered: Alti: %.2f m | Speed: %.2f m/s | Accel: %.3f m/s^2\n",
         data.filtered.alti, data.filtered.speed, data.filtered.accel);
  debugf("Filtered: roll:%.2f pitch:%.2f\n", data.filtered.roll, data.filtered.pitch);

}

/**
 * @brief Resets all sensor offsets to zero and sets offsetsSet flag to false.
 * @return void
 */
void Rakieta::resetOffsets()
{
  memset(&offsets, 0, sizeof(offsets));
  offsetsSet = false;
  debugln("[OFFSETS] Reset complete..");
}

/**
 * @brief Measures sensor offsets by averaging multiple readings (sensors and GPS).
 * @return void
 */
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
      lastOffestSensorsTime = now;
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
        offsets.adxl.ax += event.acceleration.x;  /// dorobić chyba trzeba będzie mnożnik z 16G do 200G: ADXL375_MG2G_MULTIPLIER
        offsets.adxl.ay += event.acceleration.y;  /// dorobić chyba trzeba będzie mnożnik z 16G do 200G: ADXL375_MG2G_MULTIPLIER
        offsets.adxl.az += event.acceleration.z;  /// dorobić chyba trzeba będzie mnożnik z 16G do 200G: ADXL375_MG2G_MULTIPLIER
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

  while (offsets.gps.valid < num && now < timeout)
  {
    now = millis();
    if (now - lastOffestGpsTime >= OFFSET_GPS_INTERVAL)
    {
      lastOffestGpsTime = now;
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

/**
 * @brief Prepares a GPS offset message string into the provided buffer.
 * @param buffer Destination buffer.
 * @param bufferSize Size of the buffer.
 * @return void
 */
void Rakieta::prepareGpsOffset(char* buffer, const size_t bufferSize)
{
  if (!offsetsSet) return;
  if (buffer == nullptr || bufferSize <= 0) return;

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
}

/**
 * @brief Prepares a detailed offsets message string into the provided buffer.
 * @param buffer Destination buffer.
 * @param bufferSize Size of the buffer.
 * @return void
 */
void Rakieta::prepareOffsetsMsg(char* buffer, const size_t bufferSize)
{
  if (!offsetsSet) return;
  if (buffer == nullptr || bufferSize <= 0) return;

  memset(buffer, 0, bufferSize);

  int16_t written = snprintf(buffer, bufferSize,
    "ValidNum:Lsm:%d,Adxl:%d,Bmp1:%d,Bmp2:%d,GPS:%d,"
    "Offsets:Lsm:{ax:%.3f,ay:%.3f,az:%.3f,gx:%.3f,gy:%.3f,gz:%.3f}"
    "Adxl:{ax:%.3f,ay:%.3f,az:%.3f}"
    "BMP:{1:%.2f,2:%.2f}"
    "GPS:{lat:%.6f,lng:%.6f,altiM:%.2f}",
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
    offsets.bmp1.alti,
    offsets.bmp2.alti,
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

/**
 * @brief Prepares a data line message (CSV-like) with all current telemetry values into the buffer.
 * @param buffer Destination buffer.
 * @param bufferSize Size of the buffer.
 * @return void
 */
void Rakieta::prepareDataLineMsg(char* buffer, const size_t bufferSize)
{
  if (buffer == nullptr || bufferSize <= 0) return;

  memset(buffer, 0, bufferSize);
  int16_t written = snprintf(buffer, bufferSize,
    "RocketData:%lu,%lu,%04X,%d,"
    "%.6f,%.6f,%.2f,%02u,%02u,%02u,%02u,%.2f,%.0f,%u,%u,"
    "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,"
    "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,%.4f,"
    "%.2f,%.2f,%.2f,%.4f,"
    "%.1f,"
    "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f",

    // Timestamp i podstawowe
    millis(),
    packet,
    errorFlags,
    static_cast<uint16_t>(currentFlightState),

    // GPS
    data.gps.lat, data.gps.lng, data.gps.alti,
    data.gps.h, data.gps.m, data.gps.s, data.gps.centi,
    data.gps.speed, data.gps.course, data.gps.satNum, data.gps.hdop,
    
    // LSM6
    data.lsm.ax, data.lsm.ay, data.lsm.az,
    data.lsm.gx, data.lsm.gy, data.lsm.gz,
    data.lsm.temp, data.lsm.lastTotalAlti, data.lsm.lastTotalSpeed,
    data.lsm.lastTotalAccel,

    // ADXL
    data.adxl.ax, data.adxl.ay, data.adxl.az,
    data.adxl.lastTotalAlti, data.adxl.lastTotalSpeed, data.adxl.lastTotalAccel,
    
    // BMP1
    data.bmp1.temp, data.bmp1.pressure, data.bmp1.altitude, data.bmp1.lastVerticalSpeed,

    // BMP2
    data.bmp2.temp, data.bmp2.pressure, data.bmp2.altitude, data.bmp2.lastVerticalSpeed,

    // BATTERY
    data.battery.voltage,

    // FILTERED
    data.filtered.ax, data.filtered.ay, data.filtered.az,
    data.filtered.gx, data.filtered.gy, data.filtered.gz,
    data.filtered.alti, data.filtered.speed, data.filtered.accel,
    data.filtered.roll, data.filtered.pitch
  );

  if (written < 0 || static_cast<size_t>(written) >= bufferSize)
  {
    errorFlags |= MSG_TOO_LONG_ERROR;
    snprintf(buffer, bufferSize, "ERROR:MSG_TOO_LONG,%lu", millis());
  }
}

/**
 * @brief Applies a low-pass filter to gyroscope data (from LSM).
 * @return void
 */
void Rakieta::filterGyro()
{
  if (errorFlags & LSM_ERROR) return;
  data.filtered.gx = FUSION_ALPHA * data.lsm.gx + (1.0f - FUSION_ALPHA) * data.filtered.gx;
  data.filtered.gy = FUSION_ALPHA * data.lsm.gy + (1.0f - FUSION_ALPHA) * data.filtered.gy;
  data.filtered.gz = FUSION_ALPHA * data.lsm.gz + (1.0f - FUSION_ALPHA) * data.filtered.gz;
}

/**
 * @brief Calculates roll and pitch angles from filtered accelerometer data.
 * @return void
 */
void Rakieta::calculateOrientation()
{
  data.filtered.pitch = atan2(-data.filtered.ax, sqrt(data.filtered.ay*data.filtered.ay + data.filtered.az*data.filtered.az)) * 180 / 3.1415f;
  data.filtered.roll = atan2(data.filtered.ay, data.filtered.az) * 180 / 3.1415f;
}

/**
 * @brief Fuses accelerometer data from LSM and ADXL with adaptive weights, then applies low-pass filter.
 * @return void
 */
void Rakieta::filterAccel()
{
  float fusedAx = 0.0f, fusedAy = 0.0f, fusedAz = 0.0f;
  float totalWeight = 0.0f;
  float mach = data.filtered.speed / SPEED_OF_SOUND;

  // LSM
  if (!(errorFlags & LSM_ERROR))
  {
    float weight = WEIGHT_LSM_ACCEL;
    if (data.lsm.lastTotalAccel > LSM_REDUCE_WAGE) weight *= LSM_REDUCE_FACTOR;
    if (fabs(data.lsm.lastTotalAccel) <= LSM_MAX_G)
    {
      fusedAx += data.lsm.ax * weight;
      fusedAy += data.lsm.ay * weight;
      fusedAz += data.lsm.az * weight;
      totalWeight += weight;
    }
  }

  // ADXL
  if (!(errorFlags & ADXL_ERROR) )
  {
    float weight = WEIGHT_ADXL_ACCEL;
    if (fabs(data.adxl.ax) <= ADXL_MAX_G && fabs(data.adxl.ay) <= ADXL_MAX_G && fabs(data.adxl.az) <= ADXL_MAX_G)
    {
      fusedAx += data.adxl.ax * weight;
      fusedAy += data.adxl.ay * weight;
      fusedAz += data.adxl.az * weight;
      totalWeight += weight;
    }
  }

  if (totalWeight > 0.0f)
  {
    data.filtered.ax = FUSION_ALPHA * (fusedAx / totalWeight) + (1.0f - FUSION_ALPHA) * data.filtered.ax;
    data.filtered.ay = FUSION_ALPHA * (fusedAy / totalWeight) + (1.0f - FUSION_ALPHA) * data.filtered.ay;
    data.filtered.az = FUSION_ALPHA * (fusedAz / totalWeight) + (1.0f - FUSION_ALPHA) * data.filtered.az;
    data.filtered.accel = sqrt(data.filtered.ax * data.filtered.ax +
                               data.filtered.ay * data.filtered.ay +
                               data.filtered.az * data.filtered.az);
  }
}

/**
 * @brief Fuses speed data from barometers, LSM, ADXL and GPS with adaptive weights, then filters.
 * @return void
 */
void Rakieta::filterSpeed()
{
  float currentSpeed = data.filtered.speed;
  float mach = currentSpeed / SPEED_OF_SOUND;

  float fusedSpeed = 0.0f;
  float totalWeight = 0.0f;

  // Barometry (tylko jeśli mach < 0.9)
  if (mach < MACH_IGNORE_BARO)
  {
    if (!(errorFlags & BMP1_ERROR) && !isnan(data.bmp1.lastVerticalSpeed) && (millis() - data.bmp1.lastTime) < 500U)
    {
      fusedSpeed += fabs(data.bmp1.lastVerticalSpeed) * WEIGHT_BMP_SPEED;
      totalWeight += WEIGHT_BMP_SPEED;
    }
    if (!(errorFlags & BMP2_ERROR) && !isnan(data.bmp2.lastVerticalSpeed) && (millis() - data.bmp2.lastTime) < 500U)
    {
      fusedSpeed += fabs(data.bmp2.lastVerticalSpeed) * WEIGHT_BMP_SPEED;
      totalWeight += WEIGHT_BMP_SPEED;
    }
  }
  
  // LSM (zawsze, chyba że błąd)
  if (!(errorFlags & LSM_ERROR) && !isnan(data.lsm.lastTotalSpeed) && (millis() - data.lsm.lastTime) < 500U)
  {
    fusedSpeed += data.lsm.lastTotalSpeed * WEIGHT_LSM_SPEED;
    totalWeight += WEIGHT_LSM_SPEED;
  }

  // ADXL (zawsze, chyba że błąd)
  if (!(errorFlags & ADXL_ERROR) && !isnan(data.adxl.lastTotalSpeed) && (millis() - data.adxl.lastTime) < 500U)
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

/**
 * @brief Fuses altitude data from barometers, LSM, ADXL and GPS with adaptive weights, then filters.
 * @return void
 */
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
  if (!(errorFlags & LSM_ERROR) && !isnan(data.lsm.lastTotalAlti) && (millis() - data.lsm.lastTime) < 500U)
  {
    fusedAlti += data.lsm.lastTotalAlti * WEIGHT_LSM_ALTI;
    totalWeight += WEIGHT_LSM_ALTI;
  }

  // ADXL (wysokość całkowana, jeśli dostępna)
  if (!(errorFlags & ADXL_ERROR) && !isnan(data.adxl.lastTotalAlti) && (millis() - data.adxl.lastTime) < 500U)
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

    if (data.filtered.alti > data.filtered.maxAlti) data.filtered.maxAlti = data.filtered.alti;
  }
}

/**
 * @brief Initializes LoRa module with predefined frequency, bandwidth, spreading factor etc.
 * @return true if initialization succeeded, false otherwise.
 */
bool Rakieta::initLora()
{
  int16_t state = lora.begin(FREQUENCY, BANDWIDTH, SF, CODING_RATE, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, POWER, PREAMBLE_LENGTH);

  if (state == RADIOLIB_ERR_NONE)
  {
    lora.setDio1Action(Rakieta::setOperationFlag);
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

/**
 * @brief Static function called by LoRa DIO1 interrupt to set operationDone flag.
 * @return void
 */
void Rakieta::setOperationFlag()
{
  operationDone = true;
}

/**
 * @brief Prepares a LoRa status message string into the provided buffer.
 * @param buffer Destination buffer.
 * @param bufferSize Size of the buffer.
 * @return void
 */
void Rakieta::prepareLoraStatusMsg(char* buffer, const size_t bufferSize)
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

/**
 * @brief Builds the binary telemetry packet by adding all fields to BitStorage.
 * @return void
 */
void Rakieta::preparePacket()
{
  message.clean();

  uint32_t now = millis();
  // HEADER jest dodawany automatycznie
  message.add(uint32_t(now / 10), timePos, timeLen);
  message.add(uint16_t(packet), packetPos, packetLen);
  message.add(uint16_t(errorFlags), errorPos, errorLen);
  message.add(uint8_t(currentMode), statusPos, statusLen);
  message.add(uint8_t(currentFlightState), flightstatusPos, flightstatusLen);
  message.add(uint8_t(drogueDeployed ? 1 : 0), drogueParachutePos, drogueParachuteLen);
  message.add(uint8_t(mainDeployed ? 1 : 0), mainParachutePos, mainParachuteLen);

  message.add(int32_t(data.gps.lat * 100000), gpsLatPos, gpsLatLen, true);
  message.add(int32_t(data.gps.lng * 100000), gpsLngPos, gpsLngLen, true);
  message.add(uint16_t(data.gps.alti * 10), gpsAltiPos, gpsAltiLen, true);
  message.add(uint8_t(data.gps.h), gpsHourPos, gpsHourLen);
  message.add(uint8_t(data.gps.m), gpsMinPos, gpsMinLen);
  message.add(uint8_t(data.gps.s), gpsSecPos, gpsSecLen);
  message.add(uint8_t(data.gps.centi), gpsCentisecPos, gpsCentisecLen);
  message.add(int16_t(data.gps.speed * 10), gpsSpeedPos, gpsSpeedLen, true);
  message.add(uint16_t(data.gps.course), gpsCoursePos, gpsCourseLen);
  message.add(uint8_t(data.gps.satNum), gpsSatNumPos, gpsSatNumLen);
  message.add(uint8_t(data.gps.hdop), gpsHdopPos, gpsHdopLen);
  
  message.add(int16_t(data.lsm.ax * 10), lsmAccelXPos, lsmAccelXLen, true);
  message.add(int16_t(data.lsm.ay * 10), lsmAccelYPos, lsmAccelYLen, true);
  message.add(int16_t(data.lsm.az * 10), lsmAccelZPos, lsmAccelZLen, true);
  message.add(int16_t(data.lsm.gx * 10), lsmGyroXPos, lsmGyroXLen, true);
  message.add(int16_t(data.lsm.gy * 10), lsmGyroYPos, lsmGyroYLen, true);
  message.add(int16_t(data.lsm.gz * 10), lsmGyroZPos, lsmGyroZLen, true);
  message.add(int8_t(data.lsm.temp), lsmTempPos, lsmTempLen, true);
  message.add(int16_t(data.lsm.lastTotalAlti * 10), lsmAltiPos, lsmAltiLen, true);
  message.add(int16_t(data.lsm.lastTotalSpeed * 10), lsmSpeedPos, lsmSpeedLen, true);
  message.add(int16_t(data.lsm.lastTotalAccel * 10), lsmAccelPos, lsmAccelLen, true);
  
  message.add(int16_t(data.adxl.ax * 10), adxlAccelXPos, adxlAccelXLen, true);
  message.add(int16_t(data.adxl.ay * 10), adxlAccelYPos, adxlAccelYLen, true);
  message.add(int16_t(data.adxl.az * 10), adxlAccelZPos, adxlAccelZLen, true);
  message.add(int16_t(data.adxl.lastTotalAlti * 10), adxlAltiPos, adxlAltiLen, true);
  message.add(int16_t(data.adxl.lastTotalSpeed * 10), adxlSpeedPos, adxlSpeedLen, true);
  message.add(int16_t(data.adxl.lastTotalAccel * 10), adxlAccelPos, adxlAccelLen, true);

  message.add(int8_t(data.bmp1.temp), bmp1TempPos, bmp1TempLen, true);
  message.add(uint16_t(data.bmp1.pressure / 100), bmp1PressPos, bmp1PressLen);
  message.add(uint16_t(data.bmp1.altitude * 10), bmp1AltiPos, bmp1AltiLen, true);
  message.add(int16_t(data.bmp1.lastVerticalSpeed * 10), bmp1SpeedPos, bmp1SpeedLen, true);

  message.add(int8_t(data.bmp2.temp), bmp2TempPos, bmp2TempLen, true);
  message.add(uint16_t(data.bmp2.pressure / 100), bmp2PressPos, bmp2PressLen);
  message.add(uint16_t(data.bmp2.altitude * 10), bmp2AltiPos, bmp2AltiLen, true);
  message.add(int16_t(data.bmp2.lastVerticalSpeed * 10), bmp2SpeedPos, bmp2SpeedLen, true);

  message.add(uint8_t(data.battery.voltage * 10), batteryPos, batteryLen);

  message.add(int16_t(data.filtered.ax * 10), filteredAccelXPos, filteredAccelXLen, true);
  message.add(int16_t(data.filtered.ay * 10), filteredAccelYPos, filteredAccelYLen, true);
  message.add(int16_t(data.filtered.az * 10), filteredAccelZPos, filteredAccelZLen, true);
  message.add(int16_t(data.filtered.gx * 10), filteredGyroXPos, filteredGyroXLen, true);
  message.add(int16_t(data.filtered.gy * 10), filteredGyroYPos, filteredGyroYLen, true);
  message.add(int16_t(data.filtered.gz * 10), filteredGyroZPos, filteredGyroZLen, true);
  message.add(int16_t(data.filtered.alti * 10), filteredAltiPos, filteredAltiLen, true);
  message.add(int16_t(data.filtered.speed * 10), filteredSpeedPos, filteredSpeedLen, true);
  message.add(int16_t(data.filtered.accel * 10), filteredAccelPos, filteredAccelLen, true);
  message.add(int16_t(data.filtered.roll), filteredRollPos, filteredRollLen, true);
  message.add(int16_t(data.filtered.pitch), filteredPitchPos, filteredPitchLen, true);
}

/**
 * @brief Increments packet counter, prepares packet, sends it via LoRa and turns on LED1.
 * @return void
 */
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

/**
 * @brief Transmits a binary message via LoRa (non-blocking, with timeout and busy check).
 * @param msg Pointer to data bytes.
 * @param len Length of data in bytes.
 * @return void
 */
void Rakieta::transmit(const uint8_t* msg, const size_t len)
{
  static uint32_t loraMsgStartTime= 0;

  if (msg == nullptr || len == 0 || len > 255) return;

  __disable_irq();

  if (!operationDone)
  {
    if (millis() - loraMsgStartTime >= LORA_TX_TIMEOUT)
    {
      debugln("ERROR: LoRa timeout, forcing radio reset");
      lora.standby();
      startListening();
    }
    else
    {
      debugln("LoRa busy");
      __enable_irq();
      return;
    }
  }
  
  operationDone = false;

  debug("Sending: ");
  Serial.write(msg, len);
  debugln();

  loraMsgStartTime = millis();
  int16_t state = lora.startTransmit(msg, len);

  if (state != RADIOLIB_ERR_NONE)
  {
    debug("Transmit error: ");
    debugln(state);
    errorFlags |= LORA_ERROR;
    operationDone = true;
    startListening();
  }
  __enable_irq();
}

/**
 * @brief Overload to transmit a null-terminated string.
 * @param msg Pointer to string.
 * @param len Length of string in bytes.
 * @return void
 */
void Rakieta::transmit(const char* msg, const size_t len)
{
  if (msg == nullptr) return;
  uint8_t buf[256];
  memcpy(buf, msg, len);
  transmit(buf, len);
}

/**
 * @brief Puts LoRa into continuous receive mode.
 * @return void
 */
void Rakieta::startListening()
{
  int16_t state = lora.startReceive();
  if (state != RADIOLIB_ERR_NONE)
  {
    debug("Error starting listening: ");
    debugln(state);
  }
}

/**
 * @brief Parses and executes a command received via LoRa or UART.
 * @param command Null-terminated string containing the command and optional argument.
 * @return void
 */
void Rakieta::handleCommand(const char* command)
{
  if (command == nullptr) return;

  // Kopia komendy do modyfikacji (dla strtok)
  char cmdBuffer[64];
  strncpy(cmdBuffer, command, sizeof(cmdBuffer) - 1);
  cmdBuffer[sizeof(cmdBuffer) - 1] = '\0';

  char* cmd = strtok(cmdBuffer, " ");
  if (cmd == nullptr) return;

  // Pobranie ewentualnego argumentu
  char* arg = strtok(nullptr, " ");
  
  // Użycie strcmp do porównywania napisów
  if (strcmp(cmd, "RESET") == 0) { systemReset(); }
  else if (strcmp(cmd, "RESET_OFFSETS") == 0) { resetOffsets(); }
  else if (strcmp(cmd, "SET_OFFSETS") == 0) { setOffsets(); }
  else if (strcmp(cmd, "GET_RAW_DATA") == 0) { readSensorsData(); sendPacket(); }
  else if (strcmp(cmd, "INIT_WATCHDOG") == 0) { initWatchdog(); }
  else if (strcmp(cmd, "DEPLOY_DROGUE") == 0) { drogueParashuteOpen(); }
  else if (strcmp(cmd, "DEPLOY_MAIN") == 0) { mainParashuteOpen(); }
  else if (strcmp(cmd, "VERSION") == 0) { debugln("Rocket v1.0"); transmit("Rocket v1.0", 11); }
  else if (strcmp(cmd, "GET_GPS_OFFSET") == 0)
  {
    char offBuf[128];
    prepareGpsOffset(offBuf, sizeof(offBuf));
    debugln(offBuf);
    transmit(offBuf, sizeof(offBuf));
  }
  else if (strcmp(cmd, "GET_OFFSETS") == 0)
  {
    char offBuf[256];
    prepareOffsetsMsg(offBuf, sizeof(offBuf));
    debugln(offBuf);
    transmit(offBuf, sizeof(offBuf));
  }
  else if (strcmp(cmd, "GET_LORA_STATUS") == 0)
  {
    char offBuf[128];
    prepareLoraStatusMsg(offBuf, sizeof(offBuf));
    debugln(offBuf);
    transmit(offBuf, sizeof(offBuf));
  }
  else if (strcmp(cmd, "GET_DATA") == 0)
  {
    char dataBuf[1024];
    prepareDataLineMsg(dataBuf, sizeof(dataBuf));
    debugln(dataBuf);
    transmit(dataBuf, sizeof(dataBuf));
  }
  else if (strcmp(cmd, "HELP") == 0)
  {
    const char* helpMsg =
        "Commands: HELP, VERSION, STATUS, LED1 ON/OFF, LED2 ON/OFF, BUZZER ON/OFF, "
        "DEPLOY_DROGUE, DEPLOY_MAIN, SET_MODE <mode>, SET_FLIGHT_STATE <state>, "
        "INIT_WATCHDOG, GET_ERRORS, CLEAR_ERRORS, RESET, "
        "SET_OFFSETS, GET_GPS_OFFSET, GET_OFFSETS, GET_RAW_DATA";
    debugln(helpMsg);
    transmit(helpMsg, strlen(helpMsg));
  }
  else if (strcmp(cmd, "GET_STATUS") == 0)
  {
    char statusBuf[128];
    snprintf(statusBuf, sizeof(statusBuf),
      "Mode:%d, Flight:%d, Errors:0x%04X, Uptime:%lu, Batt:%.2fV",
      static_cast<int>(currentMode), static_cast<int>(currentFlightState),
      errorFlags, millis() / 1000, data.battery.voltage);
    debugln(statusBuf);
    transmit(statusBuf, strlen(statusBuf));
  }
  else if (strcmp(cmd, "LED1") == 0 && arg != nullptr)
  {
    if (strcmp(arg, "ON") == 0) { digitalWrite(LED_1, HIGH); led1IsOn = true; led1OffTime = millis() + 10000; }
    else if (strcmp(arg, "OFF") == 0) { digitalWrite(LED_1, LOW); led1OffTime = 0; }
  }
  else if (strcmp(cmd, "LED2") == 0 && arg != nullptr)
  {
    if (strcmp(arg, "ON") == 0) { digitalWrite(LED_2, HIGH); led2IsOn = true; led2OffTime = millis() + 10000; }
    else if (strcmp(arg, "OFF") == 0) { digitalWrite(LED_2, LOW); led2OffTime = 0; }
  }
  else if (strcmp(cmd, "BUZZER") == 0 && arg != nullptr)
  {
    if (strcmp(arg, "ON") == 0) { digitalWrite(BUZZER, HIGH); buzzerIsOn = true; buzzerOffTime = millis() + 5000; }
    else if (strcmp(arg, "OFF") == 0) { digitalWrite(BUZZER, LOW); buzzerOffTime = 0; }
  }
  else if (strcmp(cmd, "SET_MODE") == 0 && arg != nullptr)
  {
    if (strcmp(arg, "DEBUG") == 0) currentMode = SystemMode::DEBUG;
    else if (strcmp(arg, "FLIGHT") == 0) currentMode = SystemMode::FLIGHT;
    else if (strcmp(arg, "DUMP") == 0) currentMode = SystemMode::DUMP;
    else if (strcmp(arg, "SLEEP") == 0) currentMode = SystemMode::SLEEP;
    else { debugln("Invalid mode"); transmit("Invalid mode", 12); return; }
    debugln("Mode changed");
    transmit("Mode changed", 12);
  }
  else if (strcmp(cmd, "SET_FLIGHT_STATE") == 0 && arg != nullptr)
  {
    if (strcmp(arg, "IDLE") == 0) currentFlightState = FlightState::IDLE;
    else if (strcmp(arg, "BOOST") == 0) currentFlightState = FlightState::BOOST;
    else if (strcmp(arg, "COAST") == 0) currentFlightState = FlightState::COAST;
    else if (strcmp(arg, "APOGEE") == 0) currentFlightState = FlightState::APOGEE;
    else if (strcmp(arg, "DESCENT") == 0) currentFlightState = FlightState::DESCENT;
    else if (strcmp(arg, "LANDED") == 0) currentFlightState = FlightState::LANDED;
    else { debugln("Invalid flight mode"); transmit("Invalid flight mode", 19); return; }
    debugln("Flight mode changed");
    transmit("Flight mode changed", 19);
  }
  else if (strcmp(cmd, "GET_ERRORS") == 0)
  {
    char errBuf[128];
    snprintf(errBuf, sizeof(errBuf),
             "Errors: LORA=%d LSM=%d BMP1=%d BMP2=%d ADXL=%d GPS=%d FLASH=%d FILE=%d LONG=%d",
             !!(errorFlags & LORA_ERROR), !!(errorFlags & LSM_ERROR),
             !!(errorFlags & BMP1_ERROR), !!(errorFlags & BMP2_ERROR),
             !!(errorFlags & ADXL_ERROR), !!(errorFlags & GPS_ERROR),
             !!(errorFlags & FLASH_ERROR), !!(errorFlags & FLASH_FILE_ERROR),
              !!(errorFlags & MSG_TOO_LONG_ERROR));
    debugln(errBuf);
    transmit(errBuf, strlen(errBuf));
  }
  else if (strcmp(cmd, "CLEAR_ERRORS") == 0)
  {
    errorFlags = 0;
    debugln("Errors cleared");
    transmit("Errors cleared", 14);
  }
  else
  {
    char errBuf[25];
    snprintf(errBuf, sizeof(errBuf), "ERROR:UNKNOWN_COMMAND");
    debugln(errBuf);
    transmit(errBuf, sizeof(errBuf));
  }
}

/**
 * @brief Handles received LoRa packet, checks header (0xFF66), logs and dispatches command.
 * @return void
 */
void Rakieta::checkRadio()
{
  uint8_t buffer[50];

  int16_t state = lora.readData(buffer, sizeof(buffer) - 1);

  if (state == RADIOLIB_ERR_NONE)
  {
    size_t len = lora.getPacketLength();
    if (len <= 2) return;
    if (len < sizeof(buffer)) buffer[len] = '\0';
    else buffer[sizeof(buffer) - 1] = '\0';

    if (buffer[0] == 0x66 && buffer[1] == 0xFF)
    {
      // Pomijamy nagłówek, reszta to komenda
      char* cmd = reinterpret_cast<char*>(buffer + 2);
      // Zabezpieczenie przed brakiem null termination
      if (len - 2 < sizeof(buffer) - 2) cmd[len - 2] = '\0';

      char logBuf[128];
      snprintf(logBuf, sizeof(logBuf), "RX: %s | RSSI=%.1f SNR=%.1f", cmd, lora.getRSSI(), lora.getSNR());
      debugln(logBuf);
      flashWriteString(logBuf);

      logEvent("Radio command received");
      handleCommand(cmd);
      errorFlags &= ~LORA_ERROR;
    }
  }
  else if (state != RADIOLIB_ERR_RX_TIMEOUT)
  {
    errorFlags |= LORA_ERROR;
  }
  else
  {
    debug("Data reading error: ");
    debugln(state);
    errorFlags |= LORA_ERROR;
  }
  startListening();
}

/**
 * @brief Initializes flash memory and mounts FAT filesystem (formats if needed).
 * @return true if successful, false otherwise.
 */
bool Rakieta::initFlash()
{
  if (!flash.begin())
  {
    debugln("[FLASH] BRAK ODPOWIEDZI");
    errorFlags |= FLASH_ERROR;
    return false;
  }

  uint32_t jedec = flash.getJEDECID();
  debug("[FLASH] OK, JEDEC: 0x");
  debugHex(jedec);
  debugln();

  if (!fatfs.begin(&flash))
  {
    debugln("[FLASH] FAT mount failed. Attempting format...");
    watchdog();
    if (!fatfs.begin(&flash, true))
    {
      debugln("[FLASH] Format failed!");
      errorFlags |= FLASH_ERROR;
      return false;
    }
    debugln("[FLASH] Format successful.");
    watchdog();
  }

  debugln("[FLASH] FAT mounted OK");

  errorFlags &= ~FLASH_ERROR;
  return true;
}

/**
 * @brief Finds the next available file number for a new data file (e.g., data_xxxx.csv).
 * @param fileName Buffer to receive the generated file name.
 * @param bufferSize Size of the buffer.
 * @return true if a free file number was found, false if limit reached.
 */
bool Rakieta::flashFindNextFileNumber(char* fileName, const size_t bufferSize)
{
  if (fileName == nullptr || bufferSize < 14) return false;
  
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

/**
 * @brief Finds the highest existing file number among data_xxxx.csv files.
 * @return Maximum file number found, or 0 if none.
 */
uint16_t Rakieta::findMaxFileNumber()
{
  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    debugln("[FLASH] Cannot open root directory.");
    return 0;
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
          uint16_t num = atoi(fileName + 5);
          if (num > maxNumber) maxNumber = num;
        }
      }
    }
    watchdog();
    file.close();
  }
  root.close();
  return maxNumber;
}

/**
 * @brief Opens a new data file for writing (with next free number) and writes CSV header.
 * @return true if file opened successfully, false otherwise.
 */
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
  flashWriteString("timestamp,packet,error,status,flightStatus,"
                 "gps_lat,gps_lng,gps_alt,gps_h,gps_m,gps_s,gps_c,"
                 "gps_speed,gps_course,gps_satNum,gps_hdop,"
                 "lsm_ax,lsm_ay,lsm_az,lsm_gx,lsm_gy,lsm_gz,lsm_temp,lsm_alti,lsm_speed,lsm_accel,"
                 "adxl_ax,adxl_ay,adxl_az,adxl_alti,adxl_speed,adxl_accel,"
                 "bmp1_temp,bmp1_press,bmp1_alt,bmp1_speed,"
                 "bmp2_temp,bmp2_press,bmp2_alt,bmp2_speed,"
                 "battery,"
                 "filtered_ax,filtered_ay,filtered_az,filtered_gx,filtered_gy,filtered_gz,"
                 "filtered_alti,filtered_speed,filtered_accel,filtered_roll,filtered_pitch");
  flashDataFile.flush();
  watchdog();
  
  debug("File opened: "); 
  debugln(fileName);
  errorFlags &= ~FLASH_FILE_ERROR;
  return true;
}

/**
 * @brief Writes a string to the currently open flash file, flushes after every FLUSH_AFTER_WRITES.
 * @param msg Null-terminated string to write.
 * @return void
 */
void Rakieta::flashWriteString(const char* msg)
{
  if (!flashDataFile)
  {
    debugln("File not open!");
    errorFlags |= FLASH_FILE_ERROR;
    return;
  }
  
  if (flash.isBusy())
  {
    debugln("Flash not ready!");
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

/**
 * @brief Writes a log message prefixed with "#LOG:" to the flash file.
 * @param msg Log message string.
 * @return void
 */
void Rakieta::writeLogToFlash(const char* msg)
{
  if (!flashDataFile)
  {
    errorFlags |= FLASH_FILE_ERROR;
    return;
  }
  if (flash.isBusy()) return;
  
  char logLine[256];
  snprintf(logLine, sizeof(logLine), "#LOG:%lu,%s", millis(), msg);
  flashWriteString(logLine);
}

/**
 * @brief Forces a flush of the flash file buffer.
 * @return void
 */
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

/**
 * @brief Closes the current flash file (flushes buffer first).
 * @return void
 */
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

/**
 * @brief Lists all CSV files on flash to debug serial.
 * @return void
 */
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

/**
 * @brief Prints the content of a specified data file to debug serial.
 * @param fileNumber Four-digit file number (e.g., 12 -> 0012).
 * @return void
 */
void Rakieta::flashDumpFileData(const uint16_t fileNumber)
{
  char filename[32];
  snprintf(filename, sizeof(filename), "data_%04d.csv", fileNumber);

  dumpFile = fatfs.open(filename, FILE_READ);
  if (!dumpFile)
  {
    debugf("[FLASH] Cannot open file %s\n", filename);
    return;
  }
  watchdog();

  debugf("=== DUMP OF %s ===\n", filename);
  char line[512];

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

/**
 * @brief Dumps the last (highest numbered) data file to debug serial.
 * @return void
 */
void Rakieta::flashDumpLastFile()
{
  uint16_t lastNumber = findMaxFileNumber();
  if (lastNumber == 0)
  {
    debugln("[FLASH] No data files found.");
    return;
  }
  flashDumpFileData(lastNumber);
}

/**
 * @brief Detects launch based on filtered acceleration threshold with debounce.
 * @return true if launch is detected, false otherwise.
 */
bool Rakieta::detectLaunch()
{
  if (isnan(data.filtered.accel) || isinf(data.filtered.accel) || !isfinite(data.filtered.accel)) return false;

  if (data.filtered.accel > LAUNCH_ACCEL_THRESHOLD)
  {
    uint32_t now = millis();
    if (launchDetectTime == 0) launchDetectTime = now;
    if (now - launchDetectTime >= LAUNCH_DEBOUNCE_MS) return true;
  }
  else launchDetectTime = 0;

  return false;
}

/**
 * @brief Detects motor burnout when filtered acceleration drops below threshold.
 * @return true if burnout is detected, false otherwise.
 */
bool Rakieta::detectBurnout()
{
  if (isnan(data.filtered.accel) || isinf(data.filtered.accel) || !isfinite(data.filtered.accel)) return false;

  if (data.filtered.accel < BURNOUT_ACCEL_THRESHOLD)
  {
    uint32_t now = millis();
    if (burnoutDetectTime == 0) burnoutDetectTime = now;
    if (now - burnoutDetectTime >= BURNOUT_DEBOUNCE_MS) return true;
  }
  else burnoutDetectTime = 0;

  return false;
}

/**
 * @brief Detects apogee when filtered speed is below threshold and altitude has reached maximum.
 * @return true if apogee detected, false otherwise.
 */
bool Rakieta::detectApogee()
{
  if (isnan(data.filtered.speed) || isinf(data.filtered.speed) || !isfinite(data.filtered.speed) ||
      isnan(data.filtered.alti) || isinf(data.filtered.alti) || !isfinite(data.filtered.alti))
    return false;

  // Apogeum gdy prędkość spada poniżej progu i osiągnięto maksimum wysokości
  if (data.filtered.speed <= APOGEE_VELOCITY_THRESHOLD && data.filtered.alti <= data.filtered.maxAlti - APOGEE_ALTITUDE_HYSTERESIS)
  {
    uint32_t now = millis();
    if (apogeeDetectTime == 0) apogeeDetectTime = now;
    if (now - apogeeDetectTime >= APOGEE_DEBOUNCE_MS) return true;
  }
  else apogeeDetectTime = 0;

  return false;
}

/**
 * @brief Detects landing when filtered speed is near zero for a debounce period.
 * @return true if landing detected, false otherwise.
 */
bool Rakieta::detectLanding()
{
  if (isnan(data.filtered.speed) || isinf(data.filtered.speed) || !isfinite(data.filtered.speed)) return false;

  if (fabs(data.filtered.speed) < LANDING_VELOCITY_THRESHOLD)
  {
    uint32_t now = millis();
    if (landedDetectTime == 0) landedDetectTime = now;
    if (now - landedDetectTime >= LANDING_DEBOUNCE_MS) return true;
  }
  else landedDetectTime = 0;
  
  return false;
}

/**
 * @brief Checks if conditions for deploying a parachute are met (speed and altitude limits, drogue prerequisite for main).
 * @param type Type of parachute (DROGUE or MAIN).
 * @return true if deployment allowed, false otherwise.
 */
bool Rakieta::checkDeploymentConditions(const ParachuteType type) const
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

/**
 * @brief Sends flight summary (timestamps of key events) via LoRa and writes to flash.
 * @return void
 */
void Rakieta::sendFlightSummary()
{
  uint32_t now = millis();
  
  char buffer[512];
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

/**
 * @brief Activates solenoid1 to deploy drogue parachute (sets timer).
 * @return void
 */
void Rakieta::drogueParashuteOpen()
{
  solenoid1IsOn = true;
  solenoid1OffTime = millis() + SOLENOID_DELAY;
  digitalWrite(SOLENOID_1, HIGH);
}

/**
 * @brief Activates solenoid2 to deploy main parachute (sets timer).
 * @return void
 */
void Rakieta::mainParashuteOpen()
{
  solenoid2IsOn = true;
  solenoid2OffTime = millis() + SOLENOID_DELAY;
  digitalWrite(SOLENOID_2, HIGH);
}

/**
 * @brief State machine for flight phase transitions (IDLE -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED).
 * @return void
 */
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
      if (!drogueDeployed && (checkDeploymentConditions(ParachuteType::DROGUE) || (millis() - launchDetectTime > DROGUE_PARASHUTE_TIMEOUT)))
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
      if ((!mainDeployed && (checkDeploymentConditions(ParachuteType::MAIN)) || (millis() - launchDetectTime > MAIN_PARASHUTE_TIMEOUT)))
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
      if (summarySend < 5)
      {
        sendFlightSummary();
        inFlight = false;
        summarySend++;
      }
      flashCloseFile();
      break;

    default:
      break;
  }
}

/**
 * @brief Initializes the independent watchdog (IWDG) with 8-second timeout.
 * @return void
 */
void Rakieta::initWatchdog()
{
  static bool watchdogInitialized = false;
  if (watchdogInitialized) return;

  // Włącz watchdog z timeoutem ~8 sekund (LSI ~32 kHz, preskaler 64, reload 4000)
  IWDG1->KR = IWDG_KEY_UNLOCK;    // odblokowanie dostępu do rejestrów
  IWDG1->PR = IWDG_PRESCALER;     // preskaler 64 (2^4 = 64)
  IWDG1->RLR = IWDG_RELOAD;       // reload value
  IWDG1->KR = IWDG_KEY_REFRESH;   // odświeżenie (wymagane przed uruchomieniem)
  IWDG1->KR = IWDG_KEY_START;     // uruchomienie
  watchdogInitialized = true;
}

/**
 * @brief Refreshes the watchdog counter if enough time has passed.
 * @return void
 */
void Rakieta::watchdog()
{
  static uint32_t lastWatchdogTime = 0;

  uint32_t now = millis();
  if (now - lastWatchdogTime >= WATCHDOG_INTERVAL)
  {
    IWDG1->KR = IWDG_KEY_REFRESH;
    lastWatchdogTime = now;
  }
}

/**
 * @brief Attempts to reinitialize a component by calling its init function.
 * @param initFunc Pointer to member function that initializes the component.
 * @return void
 */
void Rakieta::reinitComponent(bool (Rakieta::*initFunc)())
{
  debugln("[ERROR] Reinit component");
  
  bool success = (this->*initFunc)();
  if (success) debugln("[ERROR] Component recovered");
  else debugln("[ERROR] Component recovery FAILED");
}

/**
 * @brief Checks error flags and tries to recover failed components (LoRa, flash, sensors) in flight mode.
 * @return void
 */
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
    debugln("[ERROR] Flash file error – try to reopen file");
    if (flashDataFile.isOpen()) flashCloseFile();
    if (!flashDataFile.isOpen()) flashOpenNewFile();
    if (flashDataFile.isOpen()) errorFlags &= ~FLASH_FILE_ERROR;
  }

  watchdog();
}

/**
 * @brief Reads serial commands from USB and executes them.
 * @return void
 */
void Rakieta::handleSerialCommands()
{
  if (Serial.available())
  {
    char cmd[32];
    int len = Serial.readBytesUntil('\n', cmd, sizeof(cmd) - 1);
    if (len > 0)
    {
      cmd[len] = '\0';
      handleCommand(cmd);
    }
  }
}

/**
 * @brief Main handler for DEBUG mode: reads sensors, prints data, sends packet and responds to commands.
 * @return void
 */
void Rakieta::handleDebugMode()
{
  static bool wypisanie = false;
  if (!wypisanie)
  {
    debugln("\n=== TRYB DEBUG: wypisywanie czujnikow ===");
    debugln("----------------------------------------");
    wypisanie = true;
  }

  handleSerialCommands();
  checkRadio();

  /// trzeba usuwać i sprawdzać po kolei
  handleLsm();
  handleBmp1();
  handleBmp2();
  handleAdxl();
  handleGPS();
  handleBattery();
  /// gdzy będzie ok zamienić na readSensorsData()

  printData();
  sendPacket();  /// opcjonalnie
}

/**
 * @brief Main handler for FLIGHT mode: reads sensors, filters, updates flight state, logs data and sends packet.
 * @return void
 */
void Rakieta::handleFlightMode()
{
  checkRadio();
  
  uint32_t interval = (currentFlightState == FlightState::LANDED) ? INTERVAL_TOUCHDOWN : INTERVAL_BURN;
  uint32_t now = millis();

  if (now - lastFlightModeLoop >= interval)
  {
    lastFlightModeLoop = now;

    readSensorsData();
    filterGyro();
    filterAccel();
    filterSpeed();
    filterAlti();
    calculateOrientation();

    updateFlightState();

    char msg[1024];
    prepareDataLineMsg(msg, sizeof(msg));
    sendPacket();
    flashWriteString(msg);
  }
}

/**
 * @brief Main handler for DUMP mode: prints menu, reads user input and dumps flash files.
 * @param now Current millis() value.
 * @return void
 */
void Rakieta::handleDumpMode(const uint32_t now)
{
  static enum DumpState {
    WAITING_FOR_COMMAND,
    WAITING_FOR_FILE_NUMBER
  } state = WAITING_FOR_COMMAND;

  static uint32_t lastMenuPrint = 0;

  // Wypisanie menu co 10 sekund lub po wejściu w tryb
  if (state == WAITING_FOR_COMMAND && (now - lastMenuPrint >= 10000 || lastMenuPrint == 0))
  {
    lastMenuPrint = now;
    debugln("\n=== TRYB DUMP ===");
    debugln("1. Wyświetl listę plików");
    debugln("2. Wybierz numer pliku do odczytu (4-cyfry)");
    debugln("3. Wypisz zawartość ostatniego pliku");
    debug("Wybierz opcję (1-3): ");
  }
  watchdog();

  // Odczyt komendy z Serial (tylko gdy oczekujemy)
  if (state == WAITING_FOR_COMMAND && Serial.available())
  {
    char cmd = Serial.read();
    // opróżnienie bufora do końca linii
    while (Serial.available() && Serial.peek() != '\n') Serial.read();
    if (Serial.peek() == '\n') Serial.read();

    switch (cmd)
    {
      case '1':
        debugln("\n--- LISTA PLIKÓW ---");
        flashDumpFileList();
        debugln("---------------------");
        break;
      case '2':
        debug("\nPodaj numer pliku (4 cyfry, np. 0012): ");
        state = WAITING_FOR_FILE_NUMBER;
        break;
      case '3':
        debugln("\n--- OSTATNI PLIK ---");
        flashDumpLastFile();
        debugln("--------------------");
        break;
      default:
        debugln("Nieprawidłowa opcja. Wybierz 1, 2 lub 3.");
        break;
    }
  }
  else if (state == WAITING_FOR_FILE_NUMBER && Serial.available())
  {
    char buf[8] = {0};
    int idx = 0;
    uint32_t startTime = millis();
    // czytaj do napotkania '\n' lub timeout 10s
    while (millis() - startTime < 10000)
    {
      if (Serial.available())
      {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
          break;
        if (idx < 7 && isdigit(c))
          buf[idx++] = c;
      }
      delay(5);
      watchdog();
    }

    uint16_t fileNumber = atoi(buf);
    char filename[32];
    snprintf(filename, sizeof(filename), "data_%04d.csv", fileNumber);

    if (fileNumber > 0 && fileNumber <= MAX_FILE_NUMBER && fatfs.exists(filename))
    {
      debugf("Zrzucam plik %s\n", filename);
      flashDumpFileData(fileNumber);
    }
    else
    {
      debugf("Plik %s nie istnieje.\n", filename);
    }
    state = WAITING_FOR_COMMAND;
    lastMenuPrint = 0;
  }
  watchdog();
}

/**
 * @brief Main handler for SLEEP mode: enters sleep, wakes up to check DIP switch and resets if mode changed.
 * @param now Current millis() value.
 * @return void
 */
void Rakieta::handleSleepMode(const uint32_t now)
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

  systemSleep(5000);
}

/**
 * @brief Dispatches the appropriate mode handler based on current system mode.
 * @param now Current millis() value.
 * @return void
 */
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
        handleDumpMode(now);
      }
      break;
    case SystemMode::SLEEP:
      if (now - lastSleepCheck >= INTERVAL_SLEEP)
      {
        lastSleepCheck = now;
        handleSleepMode(now);
      }
      break;
    default:
      break;
  }
}

/**
 * @brief Performs full system initialisation: GPIO, SPI, sensors, LoRa, flash, sets system mode.
 * @return void
 */
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
    digitalWrite(LED_2, HIGH);
    systemSleep(200);
    digitalWrite(LED_2, LOW);
    systemSleep(200);
  }
  
  debugln("\n=== INICJALIZACJA ZAKOŃCZONA ===\n");
}

/**
 * @brief Main loop: error handling, mode handling, updates LEDs, buzzer, solenoids and sleeps briefly.
 * @return void
 */
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
  now = millis();
  updateLeds(now);
  updateBuzzer(now);
  updateSolenoid(now);
  systemSleep(5);
}
