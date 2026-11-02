/*
 * Tempo Sensor — датчик темпа гольф-свинга
 * Плата: Seeed XIAO nRF52840 Sense (встроенный IMU LSM6DS3 + BLE)
 *
 * Что делает:
 *   1. Ждёт, пока клюшка неподвижна (адрес).
 *   2. Старт замаха — угловая скорость превысила START_DPS.
 *   3. Вершина — доминирующая ось гироскопа меняет знак.
 *   4. Удар — пик ускорения выше IMPACT_G.
 *   5. Отправляет по BLE строку "B,D,G" (backswing мс, downswing мс, пик g*10).
 *
 * Библиотеки (Arduino IDE → Library Manager):
 *   - ArduinoBLE
 *   - Seeed Arduino LSM6DS3
 * Плата: Boards Manager → "Seeed nRF52 mbed-enabled Boards" → XIAO nRF52840 Sense
 */

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

// ---------- Настройки (подбираются под крепление) ----------
const float START_DPS   = 40.0;   // порог начала замаха, °/с
const float STILL_DPS   = 15.0;   // "неподвижно", °/с
const uint16_t STILL_MS = 400;    // сколько держать неподвижно перед свингом
const float IMPACT_G    = 6.0;    // порог удара, g (на шафте ~6, на перчатке ~3.5)
const uint16_t MIN_BACK_MS = 250; // короче — не свинг, а дрожание
const uint16_t MAX_BACK_MS = 2500;
const uint16_t MIN_DOWN_MS = 80;
const uint16_t MAX_DOWN_MS = 900;
const uint16_t COOLDOWN_MS = 1500;
const bool DEBUG = true;          // печатать телеметрию в Serial Monitor
const uint32_t SLEEP_AFTER_MS = 10UL * 60UL * 1000UL;   // автосон через 10 мин без движения (пробуждение — взять в руку)
const float MOTION_DPS = 30.0;    // что считать "движением" для таймера сна

// ---------- BLE ----------
const char* SERVICE_UUID = "7a1e0000-5b8c-4c2a-9f6e-2d4b3c1a0001";
const char* TEMPO_UUID   = "7a1e0001-5b8c-4c2a-9f6e-2d4b3c1a0001";
BLEService tempoService(SERVICE_UUID);
BLECharacteristic tempoChar(TEMPO_UUID, BLERead | BLENotify, 20);

LSM6DS3 imu(I2C_MODE, 0x6A);

enum State { IDLE, ARMED, BACKSWING, DOWNSWING, COOLDOWN };
State state = IDLE;

uint32_t tStill = 0, tStart = 0, tTop = 0, tImpact = 0, tCool = 0;
int8_t axisSign = 0;   // знак доминирующей оси в замахе
uint8_t axis = 0;      // 0=x 1=y 2=z
float peakG = 0;

uint32_t tLastMotion = 0;

// Глубокий сон: IMU остаётся на 26 Гц и будит плату по движению через INT1 (P0.11).
// После пробуждения плата стартует заново (setup → 3 мигания → BLE).
void goToSleep() {
  if (DEBUG) Serial.println(">> SLEEP");
  BLE.stopAdvertise();
  BLE.end();
  digitalWrite(LED_BUILTIN, HIGH);
  // LSM6DS3: wake-up interrupt на INT1
  imu.writeRegister(0x11, 0x00);   // CTRL2_G: гироскоп выкл
  imu.writeRegister(0x10, 0x20);   // CTRL1_XL: акселерометр 26 Гц, ±2g
  imu.writeRegister(0x5C, 0x00);   // WAKE_UP_DUR: без задержки
  imu.writeRegister(0x5B, 0x06);   // WAKE_UP_THS: порог 6/64 g ≈ 0.19 g
  imu.writeRegister(0x5C, 0x20);   // WAKE_UP_DUR: 1 отсчёт подряд выше порога
  imu.writeRegister(0x58, 0x90);   // TAP_CFG: interrupts enable + slope filter
  delay(400);                      // даём фильтру наклона успокоиться
  imu.writeRegister(0x5E, 0x20);   // MD1_CFG: wake-up → INT1
  // P0.11 = INT1: вход, pull-down (пока без SENSE)
  NRF_P0->PIN_CNF[11] = (1UL << 2);
  // ждём, пока INT1 опустится (защёлка сброшена, плата неподвижна)
  uint8_t src;
  for (int i = 0; i < 50; i++) {
    imu.readRegister(&src, 0x1B);  // WAKE_UP_SRC — чтение сбрасывает
    if (!(NRF_P0->IN & (1UL << 11))) break;
    delay(20);
  }
  if (DEBUG) { Serial.print(">> INT1="); Serial.println((NRF_P0->IN >> 11) & 1); delay(20); }
  // теперь включаем SENSE на высокий уровень и выключаемся
  NRF_P0->PIN_CNF[11] = (2UL << 16) | (1UL << 2);
  delay(20);
  NRF_POWER->SYSTEMOFF = 1;
  while (1) {}
}

void blink(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, LOW);  delay(60);  // LOW = горит
    digitalWrite(LED_BUILTIN, HIGH); delay(80);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  imu.settings.gyroRange = 2000;           // °/с
  imu.settings.gyroSampleRate = 208;       // Гц
  imu.settings.accelRange = 16;            // g
  imu.settings.accelSampleRate = 208;
  if (imu.begin() != 0) {
    Serial.println("IMU error");
    while (1) blink(1);
  }

  if (!BLE.begin()) {
    Serial.println("BLE error");
    while (1) blink(2);
  }
  BLE.setLocalName("TempoSensor");
  BLE.setDeviceName("TempoSensor");
  BLE.setAdvertisedService(tempoService);
  tempoService.addCharacteristic(tempoChar);
  BLE.addService(tempoService);
  tempoChar.writeValue("0,0,0");
  BLE.advertise();
  Serial.println("TempoSensor ready");
  blink(3);
  tLastMotion = millis();
}

void sendResult(uint16_t back, uint16_t down, float g) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%u,%u,%u", back, down, (unsigned)(g * 10));
  tempoChar.writeValue(buf);
  Serial.println(buf);
}

void loop() {
  BLE.poll();

  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float omega = sqrtf(gx*gx + gy*gy + gz*gz);
  float amag  = sqrtf(ax*ax + ay*ay + az*az);
  uint32_t now = millis();

  // ---- автосон ----
  if (omega > MOTION_DPS) tLastMotion = now;
  if (now - tLastMotion > SLEEP_AFTER_MS) goToSleep();

  // ---- отладка: максимумы за последние 500 мс ----
  static float dbgOmega = 0, dbgG = 0; static uint32_t dbgT = 0;
  if (omega > dbgOmega) dbgOmega = omega;
  if (amag  > dbgG)     dbgG = amag;
  if (DEBUG && now - dbgT >= 500) {
    const char* names[] = {"IDLE","ARMED","BACK","DOWN","COOL"};
    Serial.print("["); Serial.print(names[state]); Serial.print("] omega=");
    Serial.print(dbgOmega, 0); Serial.print(" dps  g="); Serial.print(dbgG, 1);
    Serial.print("  idle="); Serial.print((now - tLastMotion) / 1000); Serial.println("s  fw5");
    dbgOmega = 0; dbgG = 0; dbgT = now;
  }

  switch (state) {

    case IDLE:
      if (omega < STILL_DPS) {
        if (tStill == 0) tStill = now;
        else if (now - tStill >= STILL_MS) { state = ARMED; if (DEBUG) Serial.println(">> ARMED"); }
      } else tStill = 0;
      break;

    case ARMED:
      if (omega > START_DPS) {
        if (DEBUG) Serial.println(">> BACKSWING");
        tStart = now;
        // доминирующая ось и её знак
        float a[3] = { fabsf(gx), fabsf(gy), fabsf(gz) };
        axis = (a[0] >= a[1] && a[0] >= a[2]) ? 0 : (a[1] >= a[2] ? 1 : 2);
        float v = axis == 0 ? gx : (axis == 1 ? gy : gz);
        axisSign = v >= 0 ? 1 : -1;
        peakG = 0;
        state = BACKSWING;
      }
      break;

    case BACKSWING: {
      float v = axis == 0 ? gx : (axis == 1 ? gy : gz);
      uint32_t dt = now - tStart;
      if (dt > MAX_BACK_MS) { if (DEBUG) Serial.println(">> timeout, reset"); state = IDLE; tStill = 0; break; }
      if (dt >= MIN_BACK_MS && v * axisSign < 0) {   // смена знака = вершина
        tTop = now;
        state = DOWNSWING;
        if (DEBUG) { Serial.print(">> TOP after "); Serial.print(dt); Serial.println(" ms"); }
      }
      break;
    }

    case DOWNSWING: {
      uint32_t dt = now - tTop;
      if (amag > peakG) peakG = amag;
      if (dt > MAX_DOWN_MS) { if (DEBUG) { Serial.print(">> no impact, peak g="); Serial.println(peakG, 1); } state = IDLE; tStill = 0; break; }
      if (dt >= MIN_DOWN_MS && amag > IMPACT_G) {
        tImpact = now;
        if (DEBUG) Serial.print(">> IMPACT  ");
        sendResult(tTop - tStart, tImpact - tTop, peakG);
        digitalWrite(LED_BUILTIN, LOW);
        tCool = now;
        state = COOLDOWN;
      }
      break;
    }

    case COOLDOWN:
      if (now - tCool > COOLDOWN_MS) {
        digitalWrite(LED_BUILTIN, HIGH);
        state = IDLE; tStill = 0;
      }
      break;
  }

  delay(4);  // ~200 Гц
}
