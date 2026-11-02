// AlashPID без железа: виртуальная печка. Подойдёт любая плата, ничего подключать не нужно.
// Откройте Serial Plotter (115200): видно, как температура выходит на уставку. Уставка меняется каждые 40 секунд.
// Попробуйте изменить Kp, Ki, Kd и посмотреть, как меняется кривая: слишком большие значения дают раскачку.
#include <AlashPID.h>

AlashPID pid(2.0f, 0.4f, 0.0f);

float temperature = 20.0f;            // «температура» модели
const float ROOM = 20.0f;

// Модель печки: греется от мощности 0..100, остывает в комнату; инерция около 20 с
void stepOven(float power, float dt) {
  temperature += dt * ((ROOM + 0.5f * power - temperature) / 20.0f);
}

void setup() {
  Serial.begin(115200);
  pid.setOutputLimits(0, 100);        // мощность в процентах
  pid.setSetpoint(60);
}

void loop() {
  static uint32_t last = 0, started = millis();
  if (millis() - last >= 100) {       // шаг модели 0,1 с
    last = millis();
    float power = pid.compute(temperature);
    stepOven(power, 0.1f);
    pid.setSetpoint(((millis() - started) / 40000) % 2 ? 40 : 60);
    Serial.print(pid.setpoint()); Serial.print(' ');
    Serial.print(temperature); Serial.print(' ');
    Serial.println(power);
  }
}
