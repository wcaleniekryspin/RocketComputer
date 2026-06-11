#include <BitStorage.h>
#include <math.h>

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

// ============================================
// DEFINICJE POZYCJI BITOWYCH (zgodne z C++)
// ============================================
// HEADER for 16 bits
constexpr uint16_t timePos                            = 16;
constexpr uint16_t timeLen                            = 22;
constexpr uint16_t packetPos                          = (timePos + timeLen);
constexpr uint16_t packetLen                          = 16;
constexpr uint16_t errorPos                           = (packetPos + packetLen);
constexpr uint16_t errorLen                           = 16;
constexpr uint16_t statusPos                          = (errorPos + errorLen);
constexpr uint16_t statusLen                          = 3;
constexpr uint16_t flightstatusPos                    = (statusPos + statusLen);
constexpr uint16_t flightstatusLen                    = 3;
constexpr uint16_t drogueParachutePos                 = (flightstatusPos + flightstatusLen);
constexpr uint16_t drogueParachuteLen                 = 1;
constexpr uint16_t mainParachutePos                   = (drogueParachutePos + drogueParachuteLen);
constexpr uint16_t mainParachuteLen                   = 1;

constexpr uint16_t gpsLatPos                          = (mainParachutePos + mainParachuteLen);
constexpr uint16_t gpsLatLen                          = (17+1);
constexpr uint16_t gpsLngPos                          = (gpsLatPos + gpsLatLen);
constexpr uint16_t gpsLngLen                          = (17+1);
constexpr uint16_t gpsAltiPos                         = (gpsLngPos + gpsLngLen);
constexpr uint16_t gpsAltiLen                         = (17+1);
constexpr uint16_t gpsHourPos                         = (gpsAltiPos + gpsAltiLen);
constexpr uint16_t gpsHourLen                         = 5;
constexpr uint16_t gpsMinPos                          = (gpsHourPos + gpsHourLen);
constexpr uint16_t gpsMinLen                          = 6;
constexpr uint16_t gpsSecPos                          = (gpsMinPos + gpsMinLen);
constexpr uint16_t gpsSecLen                          = 6;
constexpr uint16_t gpsCentisecPos                     = (gpsSecPos + gpsSecLen);
constexpr uint16_t gpsCentisecLen                     = 7;
constexpr uint16_t gpsSpeedPos                        = (gpsCentisecPos + gpsCentisecLen);
constexpr uint16_t gpsSpeedLen                        = (14+1);
constexpr uint16_t gpsCoursePos                       = (gpsSpeedPos + gpsSpeedLen);
constexpr uint16_t gpsCourseLen                       = 9;
constexpr uint16_t gpsSatNumPos                       = (gpsCoursePos + gpsCourseLen);
constexpr uint16_t gpsSatNumLen                       = 5;
constexpr uint16_t gpsHdopPos                         = (gpsSatNumPos + gpsSatNumLen);
constexpr uint16_t gpsHdopLen                         = 5;

constexpr uint16_t lsmAccelXPos                       = (gpsHdopPos + gpsHdopLen);
constexpr uint16_t lsmAccelXLen                       = (14+1);
constexpr uint16_t lsmAccelYPos                       = (lsmAccelXPos + lsmAccelXLen);
constexpr uint16_t lsmAccelYLen                       = (14+1);
constexpr uint16_t lsmAccelZPos                       = (lsmAccelYPos + lsmAccelYLen);
constexpr uint16_t lsmAccelZLen                       = (14+1);
constexpr uint16_t lsmGyroXPos                        = (lsmAccelZPos + lsmAccelZLen);
constexpr uint16_t lsmGyroXLen                        = (14+1);
constexpr uint16_t lsmGyroYPos                        = (lsmGyroXPos + lsmGyroXLen);
constexpr uint16_t lsmGyroYLen                        = (14+1);
constexpr uint16_t lsmGyroZPos                        = (lsmGyroYPos + lsmGyroYLen);
constexpr uint16_t lsmGyroZLen                        = (14+1);
constexpr uint16_t lsmTempPos                         = (lsmGyroZPos + lsmGyroZLen);
constexpr uint16_t lsmTempLen                         = (9+1);
constexpr uint16_t lsmAltiPos                         = (lsmTempPos + lsmTempLen);
constexpr uint16_t lsmAltiLen                         = (17+1);
constexpr uint16_t lsmSpeedPos                        = (lsmAltiPos + lsmAltiLen);
constexpr uint16_t lsmSpeedLen                        = (14+1);
constexpr uint16_t lsmAccelPos                        = (lsmSpeedPos + lsmSpeedLen);
constexpr uint16_t lsmAccelLen                        = (14+1);

constexpr uint16_t adxlAccelXPos                      = (lsmAccelPos + lsmAccelLen);
constexpr uint16_t adxlAccelXLen                      = (15+1);
constexpr uint16_t adxlAccelYPos                      = (adxlAccelXPos + adxlAccelXLen);
constexpr uint16_t adxlAccelYLen                      = (15+1);
constexpr uint16_t adxlAccelZPos                      = (adxlAccelYPos + adxlAccelYLen);
constexpr uint16_t adxlAccelZLen                      = (15+1);
constexpr uint16_t adxlAltiPos                        = (adxlAccelZPos + adxlAccelZLen);
constexpr uint16_t adxlAltiLen                        = (17+1);
constexpr uint16_t adxlSpeedPos                       = (adxlAltiPos + adxlAltiLen);
constexpr uint16_t adxlSpeedLen                       = (14+1);
constexpr uint16_t adxlAccelPos                       = (adxlSpeedPos + adxlSpeedLen);
constexpr uint16_t adxlAccelLen                       = (15+1);

constexpr uint16_t bmp1TempPos                        = (adxlAccelPos + adxlAccelLen);
constexpr uint16_t bmp1TempLen                        = (9+1);
constexpr uint16_t bmp1PressPos                       = (bmp1TempPos + bmp1TempLen);
constexpr uint16_t bmp1PressLen                       = 11;
constexpr uint16_t bmp1AltiPos                        = (bmp1PressPos + bmp1PressLen);
constexpr uint16_t bmp1AltiLen                        = (17+1);
constexpr uint16_t bmp1SpeedPos                       = (bmp1AltiPos + bmp1AltiLen);
constexpr uint16_t bmp1SpeedLen                       = (14+1);

constexpr uint16_t bmp2TempPos                        = (bmp1SpeedPos + bmp1SpeedLen);
constexpr uint16_t bmp2TempLen                        = (9+1);
constexpr uint16_t bmp2PressPos                       = (bmp2TempPos + bmp2TempLen);
constexpr uint16_t bmp2PressLen                       = 11;
constexpr uint16_t bmp2AltiPos                        = (bmp2PressPos + bmp2PressLen);
constexpr uint16_t bmp2AltiLen                        = (17+1);
constexpr uint16_t bmp2SpeedPos                       = (bmp2AltiPos + bmp2AltiLen);
constexpr uint16_t bmp2SpeedLen                       = (14+1);

constexpr uint16_t batteryPos                         = (bmp2SpeedPos + bmp2SpeedLen);
constexpr uint16_t batteryLen                         = 6;

constexpr uint16_t filteredAccelXPos                  = (batteryPos + batteryLen);
constexpr uint16_t filteredAccelXLen                  = (15+1);
constexpr uint16_t filteredAccelYPos                  = (filteredAccelXPos + filteredAccelXLen);
constexpr uint16_t filteredAccelYLen                  = (15+1);
constexpr uint16_t filteredAccelZPos                  = (filteredAccelYPos + filteredAccelYLen);
constexpr uint16_t filteredAccelZLen                  = (15+1);
constexpr uint16_t filteredGyroXPos                   = (filteredAccelZPos + filteredAccelZLen);
constexpr uint16_t filteredGyroXLen                   = (14+1);
constexpr uint16_t filteredGyroYPos                   = (filteredGyroXPos + filteredGyroXLen);
constexpr uint16_t filteredGyroYLen                   = (14+1);
constexpr uint16_t filteredGyroZPos                   = (filteredGyroYPos + filteredGyroYLen);
constexpr uint16_t filteredGyroZLen                   = (14+1);
constexpr uint16_t filteredAltiPos                    = (filteredGyroZPos + filteredGyroZLen);
constexpr uint16_t filteredAltiLen                    = (17+1);
constexpr uint16_t filteredSpeedPos                   = (filteredAltiPos + filteredAltiLen);
constexpr uint16_t filteredSpeedLen                   = (14+1);
constexpr uint16_t filteredAccelPos                   = (filteredSpeedPos + filteredSpeedLen);
constexpr uint16_t filteredAccelLen                   = (15+1);
constexpr uint16_t filteredRollPos                    = (filteredAccelPos + filteredAccelLen);
constexpr uint16_t filteredRollLen                    = 8;
constexpr uint16_t filteredPitchPos                   = (filteredRollPos + filteredRollLen);
constexpr uint16_t filteredPitchLen                   = 8;
// CHECKSUM for 8 bits

// ============================================
// PARAMETRY FIZYCZNE
// ============================================
const uint16_t DELAY_MS = 50;        // 20 Hz
const float MASS = 5.0;              // masa rakiety [kg]
const float THRUST = 300.0;          // siła ciągu [N]
const float BURN_TIME = 5.0;         // czas pracy silnika [s]
const float GRAVITY = 9.81;          // przyspieszenie grawitacyjne [m/s²]
const float DRAG_DROGUE = -3.0;      // przyśpieszenie podczas opadania z drogue [m/s²]
const float MAIN_SPEED = -5.0;       // prędkość opadania z main [m/s]
const float MAIN_DEPLOY_ALT = 200.0; // wysokość otwarcia main [m]
const float IDLE_TIME = 5.0;         // czas oczekiwania na wyrównaniu [s]

// ============================================
// ZMIENNE GLOBALNE
// ============================================
uint32_t packet = 0;
BitStorage message;
float time_sec = 0;
float altitude = 0;
float speed = 0;
float accel = 0;
float max_altitude = 0;
uint8_t flight_phase = 0; // 0-IDLE, 1-BOOST, 2-COAST, 3-APOGEE, 4-DESCENT_DROGUE, 5-DESCENT_MAIN, 6-LANDED
bool drogue_deployed = false;
bool main_deployed = false;
bool landed = false;

// ============================================
// FUNKCJA OBLICZAJĄCA PARAMETRY LOTU
// ============================================
void calculateFlightParameters(float dt)
{
  speed += accel * dt;
  altitude += speed * dt;
  
  // Zabezpieczenia
  if (altitude < 0) altitude = 0;
  if (landed)
  {
    speed = 0;
    accel = 0;
    altitude = 0;
    return;
  }
  
  // ===== FAZY LOTU =====
  if (flight_phase == 0) { // IDLE
    if (time_sec >= IDLE_TIME) {
      flight_phase = 1;
      Serial.println("INFO: BOOST phase started");
    }
  }
  else if (flight_phase == 1) { // BOOST
    if (time_sec - IDLE_TIME >= BURN_TIME) {
      flight_phase = 2;
      Serial.println("INFO: Engine burnout, COAST phase");
    }
  }
  else if (flight_phase == 2) { // COAST
    if (speed <= 0) {
      flight_phase = 3;
      max_altitude = altitude;
      Serial.printf("INFO: APOGEE reached at %.1f m\n", max_altitude);
      // Opóźnienie przed otwarciem drogue? Natychmiast:
      flight_phase = 4;
      drogue_deployed = true;
      Serial.println("INFO: Drogue parachute deployed");
    }
  }
  else if (flight_phase == 4) { // DESCENT with drogue
    if (altitude <= MAIN_DEPLOY_ALT) {
      flight_phase = 5;
      main_deployed = true;
      speed = MAIN_SPEED;   // natychmiastowa zmiana prędkości
      accel = 0;
      Serial.println("INFO: Main parachute deployed");
    }
  }
  else if (flight_phase == 5) { // DESCENT with main
    if (altitude <= 0) {
      flight_phase = 6;
      landed = true;
      speed = 0;
      accel = 0;
      altitude = 0;
      Serial.println("INFO: LANDED");
    }
  }
  
  // ===== OBLICZANIE PRZYŚPIESZENIA W DANEJ FAZIE =====
  if (!landed) {
    if (flight_phase == 1) { // BOOST
      accel = (THRUST / MASS) - GRAVITY;
    }
    else if (flight_phase == 2) { // COAST
      accel = -GRAVITY;
    }
    else if (flight_phase == 4) { // Drogue descent
      accel = DRAG_DROGUE;
    }
    else if (flight_phase == 5) { // Main descent
      accel = 0; // stała prędkość
      speed = MAIN_SPEED; // utrzymuj stałą
    }
    else {
      accel = 0;
    }
  }
}

// ============================================
// FUNKCJA OBLICZAJĄCA CIŚNIENIE Z WYSOKOŚCI
// ============================================
float calculatePressure(float alt) {
  // Wzór barometryczny (uproszczony)
  // P = P0 * exp(-alt / H) gdzie H ≈ 8500m
  return 1013.25 * exp(-alt / 8500.0);
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
  
  // Komunikaty startowe
  Serial.println("Rocket Telemetry Simulator Started");
  Serial.printf("Packet size: %d bytes\n", ARRAY_SIZE);
  delay(1000);
}

// ============================================
// LOOP
// ============================================
void loop()
{
  static uint32_t last_time = 0;
  uint32_t now = millis();
  
  if (now - last_time < DELAY_MS) return;

  float dt = (now - last_time) / 1000.0;
  last_time = now;
  time_sec += dt;
  packet++;

  calculateFlightParameters(dt);
  
  // ===== SYMULACJA SZUMÓW I WARTOŚCI =====
  // Błędy (okazjonalne)
  uint16_t e = 0;
  if ((int)time_sec % 47 == 0 && time_sec > 10)  e |= (1 << 0);
  if ((int)time_sec % 93 == 0 && time_sec > 20)  e |= (1 << 2);
  if ((int)time_sec % 131 == 0 && time_sec > 30) e |= (1 << 5);
  
  // Status i flight state
  uint8_t currentMode = 1;
  uint8_t currentFlightState = flight_phase; // 0..6
  
  // GPS (z szumem)
  int32_t gps_lat = (time_sec * 10);
  int32_t gps_lng = (time_sec * 15);
  float gps_alti = altitude + (random(100) - 50) / 100.0;
  float gps_speed = speed + (random(100) - 50) / 50.0;
  
  // LSM (przyspieszenia, żyroskopy, temp)
  float lsm_accel_x = (random(200) - 100) / 100.0;
  float lsm_accel_y = (random(200) - 100) / 100.0;
  float lsm_accel_z = accel + GRAVITY + (random(200) - 100) / 100.0;
  float lsm_alti = altitude + (random(200) - 100) / 100.0; // błąd ±1m
  float lsm_speed = speed + (random(200) - 100) / 50.0;
  float lsm_accel_mag = sqrt(lsm_accel_x*lsm_accel_x + lsm_accel_y*lsm_accel_y + lsm_accel_z*lsm_accel_z);
  
  // ADXL (podobne)
  float adxl_accel_x = (random(200) - 100) / 100.0;
  float adxl_accel_y = (random(200) - 100) / 100.0;
  float adxl_accel_z = accel + GRAVITY + (random(200) - 100) / 100.0;
  float adxl_alti = altitude + (random(200) - 100) / 80.0;
  float adxl_speed = speed + (random(200) - 100) / 50.0;
  float adxl_accel_mag = sqrt(adxl_accel_x*adxl_accel_x + adxl_accel_y*adxl_accel_y + adxl_accel_z*adxl_accel_z);
  
  // BMP1 i BMP2
  float pressure = calculatePressure(altitude);
  float pressure_noise = pressure * (1 + (random(100) - 50) / 5000.0);
  int8_t bmp1_temp = 25 + (random(20) - 10) / 10;
  int8_t bmp2_temp = 25 + (random(20) - 10) / 10;
  float bmp1_alti = altitude + (random(200) - 100) / 100.0;
  float bmp2_alti = altitude + (random(200) - 100) / 100.0;
  float bmp1_speed = speed + (random(200) - 100) / 50.0;
  float bmp2_speed = speed + (random(200) - 100) / 50.0;
  
  // Bateria
  float battery_voltage = 4.2 - (time_sec / 1800.0);
  battery_voltage = max(2.7, battery_voltage);
  
  // Wartości filtrowane (średnie z sensorów)
  float filtered_accel_x = (lsm_accel_x + adxl_accel_x) / 2.0;
  float filtered_accel_y = (lsm_accel_y + adxl_accel_y) / 2.0;
  float filtered_accel_z = (lsm_accel_z + adxl_accel_z) / 2.0;
  float filtered_alti = (lsm_alti + adxl_alti + bmp1_alti + bmp2_alti + gps_alti) / 5.0;
  float filtered_speed = (lsm_speed + adxl_speed + bmp1_speed + bmp2_speed + gps_speed) / 5.0;
  float filtered_accel = (filtered_accel_x + filtered_accel_y + filtered_accel_z) / 3.0; // lub magnituda
  
  // Kąty (oscylacje)
  float roll = 5 * sin(time_sec * 0.3) + (random(100) - 50) / 20.0;
  float pitch = 3 * sin(time_sec * 0.5) + (random(100) - 50) / 20.0;

  message.clean();
  
  // HEADER jest dodawany automatycznie przez BitStorage
  message.add(uint32_t(now / 10), timePos, timeLen);
  message.add(uint32_t(packet), packetPos, packetLen);
  message.add(uint32_t(e), errorPos, errorLen);
  message.add(uint32_t(currentMode), statusPos, statusLen);
  message.add(uint32_t(currentFlightState), flightstatusPos, flightstatusLen);
  message.add(uint32_t(drogue_deployed ? 1 : 0), drogueParachutePos, drogueParachuteLen);
  message.add(uint32_t(main_deployed ? 1 : 0), mainParachutePos, mainParachuteLen);

  message.add(int32_t(gps_lat), gpsLatPos, gpsLatLen, true);
  message.add(int32_t(gps_lng), gpsLngPos, gpsLngLen, true);
  message.add(int32_t(gps_alti * 10), gpsAltiPos, gpsAltiLen, true);
  message.add(uint32_t(12), gpsHourPos, gpsHourLen);
  message.add(uint32_t(0), gpsMinPos, gpsMinLen);
  message.add(uint32_t(0), gpsSecPos, gpsSecLen);
  message.add(uint32_t(0), gpsCentisecPos, gpsCentisecLen);
  message.add(int32_t(gps_speed * 10), gpsSpeedPos, gpsSpeedLen, true);
  message.add(uint32_t(90), gpsCoursePos, gpsCourseLen);
  message.add(uint32_t(8), gpsSatNumPos, gpsSatNumLen);
  message.add(uint32_t(1), gpsHdopPos, gpsHdopLen);

  message.add(int32_t(lsm_accel_x * 10), lsmAccelXPos, lsmAccelXLen, true);
  message.add(int32_t(lsm_accel_y * 10), lsmAccelYPos, lsmAccelYLen, true);
  message.add(int32_t(lsm_accel_z * 10), lsmAccelZPos, lsmAccelZLen, true);
  message.add(int32_t(0), lsmGyroXPos, lsmGyroXLen, true);
  message.add(int32_t(0), lsmGyroYPos, lsmGyroYLen, true);
  message.add(int32_t(0), lsmGyroZPos, lsmGyroZLen, true);
  message.add(int32_t(25), lsmTempPos, lsmTempLen, true);
  message.add(int32_t(lsm_alti * 10), lsmAltiPos, lsmAltiLen, true);
  message.add(int32_t(lsm_speed * 10), lsmSpeedPos, lsmSpeedLen, true);
  message.add(int32_t(lsm_accel_mag * 10), lsmAccelPos, lsmAccelLen, true);

  message.add(int32_t(adxl_accel_x * 10), adxlAccelXPos, adxlAccelXLen, true);
  message.add(int32_t(adxl_accel_y * 10), adxlAccelYPos, adxlAccelYLen, true);
  message.add(int32_t(adxl_accel_z * 10), adxlAccelZPos, adxlAccelZLen, true);
  message.add(int32_t(adxl_alti * 10), adxlAltiPos, adxlAltiLen, true);
  message.add(int32_t(adxl_speed * 10), adxlSpeedPos, adxlSpeedLen, true);
  message.add(int32_t(adxl_accel_mag * 10), adxlAccelPos, adxlAccelLen, true);

  message.add(int32_t(bmp1_temp), bmp1TempPos, bmp1TempLen, true);
  message.add(uint32_t(pressure_noise), bmp1PressPos, bmp1PressLen);
  message.add(int32_t(bmp1_alti * 10), bmp1AltiPos, bmp1AltiLen, true);
  message.add(int32_t(bmp1_speed * 10), bmp1SpeedPos, bmp1SpeedLen, true);

  message.add(int32_t(bmp2_temp), bmp2TempPos, bmp2TempLen, true);
  message.add(uint32_t(pressure_noise), bmp2PressPos, bmp2PressLen);
  message.add(int32_t(bmp2_alti * 10), bmp2AltiPos, bmp2AltiLen, true);
  message.add(int32_t(bmp2_speed * 10), bmp2SpeedPos, bmp2SpeedLen, true);

  message.add(uint32_t(battery_voltage * 10), batteryPos, batteryLen);

  message.add(int32_t(filtered_accel_x * 10), filteredAccelXPos, filteredAccelXLen, true);
  message.add(int32_t(filtered_accel_y * 10), filteredAccelYPos, filteredAccelYLen, true);
  message.add(int32_t(filtered_accel_z * 10), filteredAccelZPos, filteredAccelZLen, true);
  message.add(int32_t(0), filteredGyroXPos, filteredGyroXLen, true);
  message.add(int32_t(0), filteredGyroYPos, filteredGyroYLen, true);
  message.add(int32_t(0), filteredGyroZPos, filteredGyroZLen, true);
  message.add(int32_t(filtered_alti * 10), filteredAltiPos, filteredAltiLen, true);
  message.add(int32_t(filtered_speed * 10), filteredSpeedPos, filteredSpeedLen, true);
  message.add(int32_t(filtered_accel * 10), filteredAccelPos, filteredAccelLen, true);

  message.add(int32_t(roll), filteredRollPos, filteredRollLen, true);
  message.add(int32_t(pitch), filteredPitchPos, filteredPitchLen, true);
  
  uint8_t *txPacket = message.data();
  for (uint8_t i = 0; i < ARRAY_SIZE; i++) Serial.write(txPacket[i]);
}
