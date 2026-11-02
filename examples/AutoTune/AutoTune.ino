// AlashPID: автонастройка. Реле раскачивает объект около уставки, по колебаниям находятся Ku и Tu, потом коэффициенты.
// Здесь объект виртуальная печка, поэтому пример работает без железа. Для реального объекта замените чтение температуры
// и подачу мощности (см. комментарии). Во время настройки не трогайте объект, она занимает несколько колебаний.
#include <AlashPID.h>

float temperature = 20.0f;
void stepOven(float power, float dt) { temperature += dt * ((20.0f + 0.5f * power - temperature) / 20.0f); }

AlashPIDAutoTune tuner(50, 100, 0, 0.5f);   // уставка 50, «греть» = 100, «не греть» = 0, гистерезис 0,5 градуса
AlashPID pid;
bool applied = false;

void setup() {
  Serial.begin(115200);
  pid.setOutputLimits(0, 100);
  pid.setSetpoint(50);
  tuner.setTimeout(30UL * 60UL * 1000UL);   // не дольше 30 минут
  Serial.println(F("Автонастройка..."));
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 100) return;        // шаг 0,1 с (в модели; на реальном объекте подберите под его скорость)
  last = millis();

  if (tuner.running()) {
    float power = tuner.run(temperature);   // реальный объект: tuner.run(readTemperature()); затем analogWrite(HEATER, power)
    stepOven(power, 0.1f);
    return;
  }
  if (tuner.done() && !applied) {           // настройка закончена: один раз применяем результат
    applied = true;
    tuner.apply(pid, AlashPID::RULE_SOME_OVERSHOOT);
    Serial.print(F("Ku=")); Serial.print(tuner.ku()); Serial.print(F(" Tu=")); Serial.println(tuner.tu());
    Serial.print(F("Kp=")); Serial.print(pid.kp()); Serial.print(F(" Ki=")); Serial.print(pid.ki()); Serial.print(F(" Kd=")); Serial.println(pid.kd());
  }
  if (tuner.done()) {                       // дальше работаем обычным регулятором
    float power = pid.compute(temperature);
    stepOven(power, 0.1f);
    Serial.print(pid.setpoint()); Serial.print(' '); Serial.println(temperature);
  } else {
    Serial.println(F("Автонастройка не удалась: проверьте, что объект реагирует на выход (нагреватель включён)"));
    delay(5000);
  }
}
