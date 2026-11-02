// AlashPID: ПИД-регулятор без глобальных переменных и указателей: out = pid.compute(input).
// Защита от windup, производная по измерению (без пинка при смене уставки), фильтр производной,
// мёртвая зона, ограничение скорости выхода, безударное включение, автонастройка (реле). Без зависимостей.
// MIT License. Alash Electronics, https://github.com/Alash-electronics/AlashPID
#ifndef ALASH_PID_H
#define ALASH_PID_H

#include <Arduino.h>

class AlashPID {
 public:
  // DIRECT: больше выход, больше вход (нагреватель, мотор). REVERSE: больше выход, меньше вход (вентилятор, охладитель).
  enum Direction : uint8_t { DIRECT = 0, REVERSE = 1 };
  enum Mode : uint8_t { MANUAL = 0, AUTO = 1 };
  // Правила подбора коэффициентов по предельному усилению Ku и периоду Tu
  enum Rule : uint8_t { RULE_P = 0, RULE_PI, RULE_PID, RULE_NO_OVERSHOOT, RULE_SOME_OVERSHOOT, RULE_TYREUS_LUYBEN };
  enum Error : uint8_t { ERR_OK = 0, ERR_BAD_TUNING, ERR_BAD_LIMITS, ERR_BAD_SAMPLE, ERR_BAD_INPUT, ERR_BAD_VALUE };
  enum Language : uint8_t { LANG_RU = 0, LANG_KK = 1, LANG_EN = 2 };

  AlashPID(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f, Direction direction = DIRECT);

  // ---- Настройка -------------------------------------------------------------------
  bool setTunings(float kp, float ki, float kd);                   // ki в 1/с, kd в с; не отрицательные
  bool setTimes(float kp, float integralTimeS, float derivativeTimeS);   // то же через времена: ki = kp / Ti, kd = kp * Td (Ti = 0 выключает интеграл)
  void setSetpoint(float setpoint);                                // уставка
  bool setOutputLimits(float minOutput, float maxOutput);          // по умолчанию 0..255 (ШИМ)
  bool setSampleTime(uint32_t ms);                                 // как часто считать; 0 = при каждом вызове (шаг берётся по часам)
  void setDirection(Direction d) { _dir = d; }
  bool setDerivativeFilter(float seconds);                         // постоянная времени фильтра производной; 0 = без фильтра
  bool setDeadband(float band);                                    // ошибка меньше band считается нулём
  bool setSlewRate(float unitsPerSecond);                          // не быстрее, чем на столько в секунду; 0 = без ограничения
  void setFeedForward(float value) { _ff = value; }                // добавка к выходу (например, известная нагрузка)
  void setFailsafe(float value) { _failsafe = value; _hasFailsafe = true; }   // выход при неисправном датчике; по умолчанию нижний предел

  // ---- Работа ------------------------------------------------------------------------------
  float compute(float input);                    // новый выход; между периодами возвращает прежний. Время берёт из millis()
  float computeDt(float input, float dtSeconds); // то же с явным шагом (таймер, прерывание, симуляция)
  bool updated() const { return _updated; }      // true, если последний вызов посчитал новое значение
  void setMode(Mode m);                          // MANUAL / AUTO; переход в AUTO безударный
  void setManualOutput(float value);             // выход в ручном режиме
  void reset();                                  // сбросить интеграл и память

  // ---- Состояние ---------------------------------------------------------------------
  float output() const { return _out; }
  float setpoint() const { return _sp; }
  float errorValue() const { return _e; }
  float pTerm() const { return _p; }
  float iTerm() const { return _i; }
  float dTerm() const { return _d; }
  float kp() const { return _kp; }
  float ki() const { return _ki; }
  float kd() const { return _kd; }
  Mode mode() const { return _mode; }
  Direction direction() const { return _dir; }
  bool saturated() const { return _sat; }        // выход упёрся в предел

  // ---- Подбор коэффициентов ---------------------------------------------------------------
  static void gainsFromUltimate(float ku, float tu, Rule rule, float &kp, float &ki, float &kd);
  bool tuneFromUltimate(float ku, float tu, Rule rule = RULE_PID);

  // ---- Диагностика -------------------------------------------------------------------
  Error error() const { return _err; }
  void clearError() { _err = ERR_OK; }
  void setLanguage(Language lang) { _lang = lang; }
  const __FlashStringHelper *errorText();
  void diagnose(Print &out);

 private:
  float clampOut(float v) const { return v < _min ? _min : (v > _max ? _max : v); }
  float _kp, _ki, _kd, _sp = 0, _min = 0, _max = 255, _tau = 0, _dead = 0, _slew = 0, _ff = 0, _failsafe = 0;
  float _p = 0, _i = 0, _d = 0, _dF = 0, _e = 0, _out = 0, _lastIn = 0;
  uint32_t _sample = 0, _last = 0;
  Direction _dir;
  Mode _mode = AUTO;
  Error _err = ERR_OK;
  Language _lang = LANG_RU;
  bool _updated = false, _started = false, _hasLast = false, _hasOut = false, _initPending = false, _sat = false, _hasFailsafe = false;
  uint32_t _nCalc = 0, _nBad = 0;
};

// Автонастройка методом реле (Åström-Hägglund): выход переключается между двумя значениями, система раскачивается около уставки,
// по размаху и периоду колебаний находятся Ku и Tu, а по ним коэффициенты. Вызывайте run() в loop() вместо pid.compute().
class AlashPIDAutoTune {
 public:
  enum State : uint8_t { IDLE = 0, RUNNING, DONE, TIMEOUT, BAD_INPUT, FAILED };
  // outPushUp: выход, который толкает величину вверх (нагреватель включён); outPushDown: вниз (выключен). Для охладителя наоборот.
  AlashPIDAutoTune(float setpoint, float outPushUp, float outPushDown, float hysteresis = 0, uint8_t cycles = 4);

  void setTimeout(uint32_t ms) { _timeout = ms; }          // 0 = без ограничения
  float run(float input);                                   // шаг автонастройки; возвращает выход, который надо подать на объект
  float runAt(float input, uint32_t nowMs);                 // то же с явным временем
  State state() const { return _state; }
  bool done() const { return _state == DONE; }
  bool running() const { return _state == RUNNING || _state == IDLE; }
  float ku() const { return _ku; }
  float tu() const { return _tu; }                          // период колебаний, секунды
  float amplitude() const { return _amp; }
  bool apply(AlashPID &pid, AlashPID::Rule rule = AlashPID::RULE_PID);   // записать найденные коэффициенты в регулятор

 private:
  float _sp, _up, _down, _h, _out, _ku = 0, _tu = 0, _amp = 0, _min, _max, _sumTu = 0, _sumAmp = 0;
  uint32_t _timeout = 0, _t0 = 0, _tSwitch = 0;
  uint8_t _cycles, _skip = 2, _measured = 0;
  bool _goingUp = true, _haveSwitch = false, _haveMin = false, _haveMax = false;
  float _lastMin = 0, _lastMax = 0;
  State _state = IDLE;
};

#endif
