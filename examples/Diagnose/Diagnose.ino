// [ru] AlashPID: самопроверка. Печатает коэффициенты, пределы, ошибку, слагаемые P, I, D и подсказки.
//      Язык подсказок: LANG_RU, LANG_KK или LANG_EN.
// [kk] AlashPID: өзін-өзі тексеру. Коэффициенттерді, шектерді, қатені, P, I, D құраушыларын және кеңестерді басып шығарады.
//      Кеңес тілі: LANG_RU, LANG_KK немесе LANG_EN.
// [en] AlashPID: self-check. Prints the gains, limits, error, the P, I, D terms and tips.
//      Tips language: LANG_RU, LANG_KK or LANG_EN.
#include <AlashPID.h>

AlashPID pid(2.0f, 0.0f, 0.0f);

void setup() {
  Serial.begin(115200);
  pid.setLanguage(AlashPID::LANG_RU);
  pid.setSetpoint(100);
  pid.setOutputLimits(0, 255);
  for (int i = 0; i < 5; i++) pid.computeDt(20.0f, 0.1f);   // несколько шагов с входом 20, чтобы было что показать
  pid.diagnose(Serial);
}

void loop() {}
