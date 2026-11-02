// [ru] AlashPID: держим постоянную освещённость. Фоторезистор читаем с A0, светодиод светит через ШИМ.
//      Регулятор сам подбирает яркость светодиода так, чтобы освещённость равнялась уставке, даже если в комнате светлеет или темнеет.
//
//      Подключение: фоторезистор + резистор 10 кОм делителем на A0; светодиод через 220 Ом к пину 9 (на ESP32 укажите свой пин ШИМ), оба к GND.
//      Светодиод должен светить на фоторезистор.
// [kk] AlashPID: тұрақты жарықтықты ұстау. Фоторезисторды A0-ден оқимыз, жарықдиод ШИМ арқылы жанады.
//      Реттегіш жарықдиод жарықтығын өзі таңдайды, сонда бөлме жарықтанса немесе қараңғыласа да жарықтық уставкаға тең болады.
//
//      Қосу: фоторезистор + 10 кОм резистор бөлгіш ретінде A0-ге; жарықдиод 220 Ом арқылы 9 пинге (ESP32-де өз ШИМ пиніңізді көрсетіңіз), екеуі де GND-ға.
//      Жарықдиод фоторезисторға қарай жануы керек.
// [en] AlashPID: hold a constant light level. We read a photoresistor on A0, an LED shines through PWM.
//      The controller adjusts the LED brightness so the light level equals the setpoint, even if the room gets brighter or darker.
//
//      Wiring: photoresistor + 10 kOhm resistor as a divider to A0; LED through 220 Ohm to pin 9 (on ESP32 give your own PWM pin), both to GND.
//      The LED must shine on the photoresistor.
#include <AlashPID.h>

const uint8_t SENSOR_PIN = A0;
const uint8_t LED_PIN = 9;

AlashPID pid(0.8f, 0.6f, 0.0f);       // Kp, Ki, Kd; DIRECT: больше яркость, больше освещённость

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pid.setSetpoint(600);               // желаемое значение analogRead (на Uno 0..1023)
  pid.setOutputLimits(0, 255);        // ШИМ 0..255
  pid.setSampleTime(50);              // считать раз в 50 мс
}

void loop() {
  float light = analogRead(SENSOR_PIN);
  float duty = pid.compute(light);    // новая яркость
  analogWrite(LED_PIN, (int)duty);

  if (pid.updated()) {                // печатаем для Serial Plotter: уставка, измерение, выход
    Serial.print(pid.setpoint()); Serial.print(' ');
    Serial.print(light); Serial.print(' ');
    Serial.println(duty);
  }
}
