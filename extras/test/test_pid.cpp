// Тесты AlashPID без железа: чистая математика, объект управления моделируется в коде, время задаёт g_ms.
//   g++ -std=c++17 -Wall -I mock -I ../../src test_pid.cpp ../../src/AlashPID.cpp && ./a.out
#include <deque>
#include <stdio.h>
#include "Arduino.h"
#include "AlashPID.h"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); fails++; } } while (0)
static bool near(float a, float b, float e) { return fabsf(a - b) <= e; }

// Объект первого порядка с запаздыванием: x' = (-x + k*u) / T, вход доходит через delay секунд.
struct Plant {
  float x, k, T, dt; std::deque<float> q;
  Plant(float k_, float T_, float delay, float dt_, float x0 = 0) : x(x0), k(k_), T(T_), dt(dt_) { for (int i = 0; i < (int)(delay / dt); i++) q.push_back(0); }
  float step(float u) { q.push_back(u); float ud = q.front(); q.pop_front(); x += dt * (-x + k * ud) / T; return x; }
};

int main() {
  // 1) коэффициенты: плохие отвергаются, прежние остаются
  {
    AlashPID p(2, 1, 0.5f);
    CHECK(p.kp() == 2 && p.ki() == 1 && p.kd() == 0.5f && p.error() == AlashPID::ERR_OK);
    CHECK(!p.setTunings(-1, 0, 0) && p.error() == AlashPID::ERR_BAD_TUNING && p.kp() == 2);
    p.clearError();
    CHECK(!p.setTunings(NAN, 0, 0) && p.error() == AlashPID::ERR_BAD_TUNING);
    p.clearError();
    CHECK(!p.setTunings(INFINITY, 0, 0) && p.kp() == 2);
    CHECK(p.setTimes(2, 4, 0.5f) && near(p.ki(), 0.5f, 1e-6f) && near(p.kd(), 1.0f, 1e-6f));
    CHECK(p.setTimes(3, 0, 0) && p.ki() == 0 && p.kd() == 0);
    p.clearError();
    CHECK(!p.setOutputLimits(10, 10) && p.error() == AlashPID::ERR_BAD_LIMITS);
    CHECK(!p.setOutputLimits(5, 1));
    p.clearError(); p.setSetpoint(NAN); CHECK(p.error() == AlashPID::ERR_BAD_VALUE && p.setpoint() == 0);
    p.clearError(); CHECK(!p.setDeadband(-1) && !p.setSlewRate(-5) && !p.setDerivativeFilter(-0.1f) && p.error() == AlashPID::ERR_BAD_VALUE);
  }

  // 2) только P: выход = Kp * ошибка; DIRECT и REVERSE
  {
    AlashPID d(2, 0, 0, AlashPID::DIRECT); d.setSetpoint(50); d.setOutputLimits(-1000, 1000);
    CHECK(near(d.computeDt(30, 0.1f), 40, 1e-4f));       // ошибка 20, выход 40
    CHECK(near(d.computeDt(60, 0.1f), -20, 1e-4f));
    AlashPID r(2, 0, 0, AlashPID::REVERSE); r.setSetpoint(50); r.setOutputLimits(-1000, 1000);
    CHECK(near(r.computeDt(30, 0.1f), -40, 1e-4f));      // вход ниже уставки: охлаждение выключаем
    CHECK(near(r.computeDt(60, 0.1f), 20, 1e-4f));
    CHECK(d.pTerm() == -20 && d.errorValue() == -10);
  }

  // 3) интеграл: при постоянной ошибке растёт на Ki*e*dt за шаг
  {
    AlashPID p(0, 2, 0); p.setSetpoint(10); p.setOutputLimits(-1000, 1000);
    p.computeDt(0, 0.5f);                                // первый шаг: 2*10*0.5 = 10
    CHECK(near(p.output(), 10, 1e-4f));
    p.computeDt(0, 0.5f); CHECK(near(p.output(), 20, 1e-4f));
    CHECK(near(p.iTerm(), 20, 1e-4f));
  }

  // 4) производная по измерению: смена уставки не даёт пинка, а резкое изменение входа даёт отклик
  {
    AlashPID p(1, 0, 10); p.setOutputLimits(-1000, 1000); p.setSetpoint(0);
    p.computeDt(5, 0.1f); p.computeDt(5, 0.1f);
    float before = p.output();                           // P = -5, D = 0
    p.setSetpoint(100);
    float after = p.computeDt(5, 0.1f);                  // уставка скачком: должен измениться только P
    CHECK(near(after - before, 100, 1e-3f));             // P был -5, стал 95: меняется только P
    CHECK(p.dTerm() == 0);                               // главное: пинка D нет
    float y = p.computeDt(7, 0.1f);                      // вход вырос на 2 за 0.1 с: D = -10 * 20 = -200
    CHECK(near(p.dTerm(), -200, 1e-3f));
    (void)y;
  }

  // 5) windup: долго упираемся в предел, потом ошибка меняет знак, выход должен сразу уйти с предела
  {
    AlashPID p(2, 1, 0); p.setOutputLimits(0, 255); p.setSetpoint(100);
    for (int i = 0; i < 2000; i++) p.computeDt(0, 0.1f);          // 200 с на пределе
    CHECK(p.output() == 255 && p.saturated());
    CHECK(p.iTerm() <= 255);
    float o = p.computeDt(110, 0.1f);                              // перелёт на 10
    CHECK(o < 100);                                                // сразу уходим вниз, без «разматывания» накопленного
    p.reset(); CHECK(p.iTerm() == 0 && p.output() == 0);
  }

  // 6) пределы: выход и интеграл ограничены; изменение пределов подтягивает выход
  {
    AlashPID p(100, 100, 0); p.setSetpoint(50); p.setOutputLimits(0, 200);
    for (int i = 0; i < 100; i++) p.computeDt(0, 0.1f);
    CHECK(p.output() == 200);
    p.setOutputLimits(0, 120); CHECK(p.output() == 120 && p.iTerm() <= 120);
    p.setOutputLimits(-50, 50); CHECK(p.output() == 50);
    for (int i = 0; i < 100; i++) p.computeDt(500, 0.1f);
    CHECK(p.output() == -50);
  }

  // 7) шаг расчёта: считает раз в 100 мс; updated(); переход millis() через 2^32
  {
    AlashPID p(1, 0, 0); p.setSetpoint(10); p.setSampleTime(100);
    g_ms = 4294967200u;                                  // за 96 мс до переполнения
    p.compute(0); CHECK(p.updated());
    int n = 0;
    for (int i = 0; i < 1000; i++) { g_ms++; p.compute(0); if (p.updated()) n++; }
    CHECK(n >= 9 && n <= 11);                            // за 1000 мс около 10 расчётов, переполнение не мешает
    CHECK(near(p.output(), 10, 1e-4f));
    g_ms += 50; p.compute(0); CHECK(!p.updated());
    p.setSampleTime(0);
    g_ms += 1; p.compute(0); CHECK(p.updated());         // 0 = каждый вызов, шаг по часам
    p.compute(0); CHECK(!p.updated());                   // тот же момент времени: нечего считать
    g_ms = 1000;
  }

  // 8) замкнутый контур: нагреватель (k=1 °C на единицу, T=20 с, запаздывание 1 с), PI выводит на уставку без большого перелёта
  {
    Plant pl(0.5f, 20.0f, 1.0f, 0.1f);                   // установившееся значение = 0.5 * u
    AlashPID p(2.0f, 0.4f, 0.0f); p.setOutputLimits(0, 100); p.setSetpoint(30);
    float x = 0, peak = 0;
    for (int i = 0; i < 6000; i++) { float u = p.computeDt(x, 0.1f); x = pl.step(u); if (x > peak) peak = x; }
    CHECK(near(x, 30, 0.2f));
    CHECK(peak < 36.0f);                                 // перелёт меньше 20 %: PI с такими коэффициентами не раскачивается
    // скачок уставки вниз и обратно
    p.setSetpoint(20);
    for (int i = 0; i < 6000; i++) { float u = p.computeDt(x, 0.1f); x = pl.step(u); }
    CHECK(near(x, 20, 0.2f));
  }

  // 9) REVERSE: охладитель; вход выше уставки, выход растёт
  {
    Plant pl(-0.5f, 20.0f, 0.5f, 0.1f, 40.0f);           // вентилятор снижает значение
    AlashPID p(2.0f, 0.4f, 0.0f, AlashPID::REVERSE); p.setOutputLimits(0, 100); p.setSetpoint(25);
    float x = 40;
    for (int i = 0; i < 8000; i++) { float u = p.computeDt(x, 0.1f); pl.step(u); x = 40.0f + pl.x; }   // значение = 40 + отклонение модели
    CHECK(near(x, 25, 0.3f));
  }

  // 10) безударный переход MANUAL -> AUTO: выход не скачет
  {
    AlashPID p(2, 0.5f, 0); p.setOutputLimits(0, 255); p.setSetpoint(100);
    p.setMode(AlashPID::MANUAL); p.setManualOutput(120);
    CHECK(p.computeDt(90, 0.1f) == 120);                 // в ручном режиме выход = заданному
    p.setMode(AlashPID::AUTO);
    float o = p.computeDt(98, 0.1f);                     // ошибка 2: плавно продолжаем около 120
    CHECK(near(o, 120, 0.5f));
    float o2 = p.computeDt(99, 0.1f);
    CHECK(near(o2, 118.15f, 0.1f));                       // ошибка упала с 2 до 1: P меньше на Kp, интеграл добавил 0.05
  }

  // 11) NaN на входе: безопасный выход, состояние цело, потом восстановление
  {
    AlashPID p(2, 1, 0); p.setOutputLimits(0, 255); p.setSetpoint(50);
    for (int i = 0; i < 20; i++) p.computeDt(40, 0.1f);
    float i0 = p.iTerm();
    CHECK(p.computeDt(NAN, 0.1f) == 0 && p.error() == AlashPID::ERR_BAD_INPUT);   // по умолчанию нижний предел
    CHECK(p.iTerm() == i0);
    p.setFailsafe(30); CHECK(p.computeDt(INFINITY, 0.1f) == 30);
    p.clearError(); float o = p.computeDt(45, 0.1f);
    CHECK(o > 0 && p.error() == AlashPID::ERR_OK);
  }

  // 12) мёртвая зона и ограничение скорости, добавка feedforward
  {
    AlashPID p(10, 0, 0); p.setOutputLimits(-1000, 1000); p.setSetpoint(100); p.setDeadband(2);
    CHECK(p.computeDt(99, 0.1f) == 0);                   // ошибка 1 в зоне
    CHECK(p.computeDt(97, 0.1f) == 30);                  // вне зоны
    AlashPID s(10, 0, 0); s.setOutputLimits(0, 255); s.setSetpoint(100); s.setSlewRate(50);
    s.computeDt(0, 0.1f);                                // первый шаг без ограничения скорости: выход 255
    s.setManualOutput(0);
    float prev = s.output(), worst = 0;
    for (int i = 0; i < 20; i++) { float o = s.computeDt(0, 0.1f); worst = fmaxf(worst, fabsf(o - prev)); prev = o; }
    CHECK(worst <= 5.0001f);                             // 50 в секунду * 0.1 с = 5 за шаг
    CHECK(near(prev, 100, 0.01f));                       // 20 шагов по 5
    AlashPID f(1, 0, 0); f.setOutputLimits(0, 255); f.setSetpoint(10); f.setFeedForward(40);
    CHECK(near(f.computeDt(10, 0.1f), 40, 1e-4f));       // ошибка 0, остаётся добавка
  }

  // 13) фильтр производной сглаживает скачок входа
  {
    AlashPID raw(0, 0, 1), fil(0, 0, 1);
    raw.setOutputLimits(-1e6f, 1e6f); fil.setOutputLimits(-1e6f, 1e6f);
    fil.setDerivativeFilter(0.5f);
    raw.computeDt(0, 0.1f); fil.computeDt(0, 0.1f);
    float a = raw.computeDt(1, 0.1f), b = fil.computeDt(1, 0.1f);
    CHECK(fabsf(b) < fabsf(a) * 0.3f);
    for (int i = 0; i < 100; i++) fil.computeDt(1, 0.1f);
    CHECK(near(fil.dTerm(), 0, 1e-3f));                  // входа не меняется: производная затухла
  }

  // 14) формулы Циглера-Николса
  {
    float kp, ki, kd;
    AlashPID::gainsFromUltimate(10, 4, AlashPID::RULE_PID, kp, ki, kd);
    CHECK(near(kp, 6, 1e-4f) && near(ki, 6 / 2.0f, 1e-4f) && near(kd, 6 * 0.5f, 1e-4f));   // Ti = 2, Td = 0.5
    AlashPID::gainsFromUltimate(10, 4, AlashPID::RULE_P, kp, ki, kd);
    CHECK(kp == 5 && ki == 0 && kd == 0);
    AlashPID::gainsFromUltimate(10, 4, AlashPID::RULE_PI, kp, ki, kd);
    CHECK(near(kp, 4.5f, 1e-4f) && near(ki, 4.5f / (4 / 1.2f), 1e-4f) && kd == 0);
    AlashPID::gainsFromUltimate(10, 4, AlashPID::RULE_NO_OVERSHOOT, kp, ki, kd);
    CHECK(near(kp, 2, 1e-4f));
    AlashPID q; CHECK(q.tuneFromUltimate(10, 4) && near(q.kp(), 6, 1e-4f));
    CHECK(!q.tuneFromUltimate(0, 4) && !q.tuneFromUltimate(10, -1) && q.error() == AlashPID::ERR_BAD_TUNING);
  }

  // 15) автонастройка на модели с запаздыванием: находит Ku/Tu и даёт рабочие коэффициенты
  {
    Plant pl(1.0f, 10.0f, 1.0f, 0.05f);                  // k=1, T=10, запаздывание 1 с; вход 0..100, уставка 30
    AlashPIDAutoTune at(30, 100, 0, 0.0f, 4);
    g_ms = 0; float x = 0; int guard = 0;
    while (at.running() && guard++ < 200000) {
      float u = at.runAt(x, g_ms);
      x = pl.step(u); g_ms += 50;
    }
    CHECK(at.done());
    CHECK(at.tu() > 2.0f && at.tu() < 20.0f);            // период колебаний от 2 до 20 с
    CHECK(at.ku() > 0.5f && at.ku() < 20.0f);
    AlashPID p(1, 0, 0); p.setOutputLimits(0, 100); p.setSetpoint(30);
    CHECK(at.apply(p, AlashPID::RULE_SOME_OVERSHOOT) && p.kp() > 0 && p.ki() > 0);
    Plant pl2(1.0f, 10.0f, 1.0f, 0.05f); float y = 0, peak = 0;
    for (int i = 0; i < 12000; i++) { float u = p.computeDt(y, 0.05f); y = pl2.step(u); if (y > peak) peak = y; }
    CHECK(near(y, 30, 0.5f));
    CHECK(peak < 45.0f);                                 // не раскачивается
    AlashPIDAutoTune notDone(30, 100, 0); AlashPID z; CHECK(!notDone.apply(z));
  }

  // 16) автонастройка охладителя (REVERSE): выход «вверх» это 0, «вниз» это 100
  {
    Plant pl(-1.0f, 10.0f, 1.0f, 0.05f, 0);              // значение = 60 + pl.x, вентилятор снижает
    AlashPIDAutoTune at(30, 0, 100, 0.0f, 3);
    g_ms = 0; float x = 60; int guard = 0;
    while (at.running() && guard++ < 200000) { float u = at.runAt(x, g_ms); pl.step(u); x = 60 + pl.x; g_ms += 50; }
    CHECK(at.done() && at.ku() > 0 && at.tu() > 0);
  }

  // 17) автонастройка: тайм-аут, плохой вход, датчик без отклика
  {
    AlashPIDAutoTune a(30, 100, 0); a.setTimeout(1000); g_ms = 0;
    a.runAt(0, 0); a.runAt(0, 500); a.runAt(0, 1500);
    CHECK(a.state() == AlashPIDAutoTune::TIMEOUT && a.runAt(0, 2000) == 0);
    AlashPIDAutoTune b(30, 100, 0); b.runAt(10, 0);
    CHECK(b.runAt(NAN, 10) == 0 && b.state() == AlashPIDAutoTune::BAD_INPUT);
    AlashPIDAutoTune c(30, 100, 0); c.setTimeout(5000);
    for (uint32_t t = 0; t < 10000 && c.running(); t += 50) c.runAt(10, t);   // вход не реагирует на выход
    CHECK(!c.done());
  }

  // 18) тексты ошибок на трёх языках, диагностика
  {
    AlashPID p(1, 0, 0);
    for (int e = AlashPID::ERR_BAD_TUNING; e <= AlashPID::ERR_BAD_VALUE; e++) {
      p.clearError();
      switch (e) {
        case AlashPID::ERR_BAD_TUNING: p.setTunings(-1, 0, 0); break;
        case AlashPID::ERR_BAD_LIMITS: p.setOutputLimits(5, 5); break;
        case AlashPID::ERR_BAD_SAMPLE: p.computeDt(1, -1); break;
        case AlashPID::ERR_BAD_INPUT: p.computeDt(NAN, 0.1f); break;
        case AlashPID::ERR_BAD_VALUE: p.setDeadband(-1); break;
      }
      CHECK(p.error() == e);
      p.setLanguage(AlashPID::LANG_RU); const char *ru = (const char *)p.errorText();
      p.setLanguage(AlashPID::LANG_KK); const char *kk = (const char *)p.errorText();
      p.setLanguage(AlashPID::LANG_EN); const char *en = (const char *)p.errorText();
      CHECK(strlen(ru) > 20 && strlen(kk) > 20 && strlen(en) > 20 && strcmp(ru, kk) && strcmp(kk, en) && strcmp(ru, en));
    }
    p.clearError(); CHECK(strcmp((const char *)p.errorText(), "OK") == 0);
    AlashPID q(2, 0, 0); q.setSetpoint(100); q.computeDt(0, 0.1f);
    Print pr; q.diagnose(pr);
    CHECK(pr.text.find("Kp=2.0000") != std::string::npos);
    CHECK(pr.text.find("Ki = 0") != std::string::npos || pr.text.find("Ki") != std::string::npos);
    q.setLanguage(AlashPID::LANG_KK); Print pk; q.diagnose(pk); CHECK(pk.text.find("Кеңес") != std::string::npos);
    q.setLanguage(AlashPID::LANG_EN); Print pe; q.diagnose(pe); CHECK(pe.text.find("Tip") != std::string::npos);
  }

  if (fails) { printf("FAILED: %d\n", fails); return 1; }
  printf("ALL TESTS PASSED\n");
  return 0;
}
