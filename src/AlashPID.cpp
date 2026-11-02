// MIT License. Alash Electronics.
#include "AlashPID.h"

#define T(ru, kk, en) (_lang == LANG_KK ? F(kk) : _lang == LANG_EN ? F(en) : F(ru))

static inline bool finiteF(float v) { return v == v && (v - v) == 0.0f; }     // не NaN и не бесконечность

AlashPID::AlashPID(float kp, float ki, float kd, Direction direction) : _kp(0), _ki(0), _kd(0), _dir(direction) {
  setTunings(kp, ki, kd);
}

// ---- Настройка -----------------------------------------------------------------------------
bool AlashPID::setTunings(float kp, float ki, float kd) {
  if (!finiteF(kp) || !finiteF(ki) || !finiteF(kd) || kp < 0 || ki < 0 || kd < 0) { _err = ERR_BAD_TUNING; return false; }
  _kp = kp; _ki = ki; _kd = kd;
  return true;
}

bool AlashPID::setTimes(float kp, float integralTimeS, float derivativeTimeS) {
  if (!finiteF(kp) || !finiteF(integralTimeS) || !finiteF(derivativeTimeS) || kp < 0 || integralTimeS < 0 || derivativeTimeS < 0) {
    _err = ERR_BAD_TUNING; return false;
  }
  return setTunings(kp, integralTimeS > 0 ? kp / integralTimeS : 0.0f, kp * derivativeTimeS);
}

void AlashPID::setSetpoint(float setpoint) {
  if (!finiteF(setpoint)) { _err = ERR_BAD_VALUE; return; }
  _sp = setpoint;
}

bool AlashPID::setOutputLimits(float minOutput, float maxOutput) {
  if (!finiteF(minOutput) || !finiteF(maxOutput) || !(minOutput < maxOutput)) { _err = ERR_BAD_LIMITS; return false; }
  _min = minOutput; _max = maxOutput;
  _i = clampOut(_i);
  _out = clampOut(_out);
  if (!_hasFailsafe) _failsafe = _min;
  return true;
}

bool AlashPID::setSampleTime(uint32_t ms) { _sample = ms; return true; }

bool AlashPID::setDerivativeFilter(float seconds) {
  if (!finiteF(seconds) || seconds < 0) { _err = ERR_BAD_VALUE; return false; }
  _tau = seconds; return true;
}

bool AlashPID::setDeadband(float band) {
  if (!finiteF(band) || band < 0) { _err = ERR_BAD_VALUE; return false; }
  _dead = band; return true;
}

bool AlashPID::setSlewRate(float unitsPerSecond) {
  if (!finiteF(unitsPerSecond) || unitsPerSecond < 0) { _err = ERR_BAD_VALUE; return false; }
  _slew = unitsPerSecond; return true;
}

// ---- Работа ---------------------------------------------------------------------------------
void AlashPID::reset() {
  _i = 0; _d = 0; _dF = 0; _p = 0; _e = 0;
  _started = false; _hasLast = false; _hasOut = false; _initPending = false; _sat = false;
  _out = clampOut(0.0f);
}

void AlashPID::setMode(Mode m) {
  if (m == _mode) return;
  _mode = m;
  if (m == AUTO) _initPending = true;           // безударный переход: интеграл подбирается при ближайшем расчёте
}

void AlashPID::setManualOutput(float value) {
  if (!finiteF(value)) { _err = ERR_BAD_VALUE; return; }
  _out = clampOut(value);
  _hasOut = true;
}

float AlashPID::compute(float input) {
  uint32_t now = millis();
  if (!_started) {                              // первый вызов: шаг ещё неизвестен
    _started = true; _last = now;
    return computeDt(input, 0.0f);
  }
  uint32_t elapsed = now - _last;               // безопасно при переполнении millis()
  if (elapsed == 0 || (_sample > 0 && elapsed < _sample)) { _updated = false; return _out; }
  _last = now;
  return computeDt(input, elapsed / 1000.0f);
}

float AlashPID::computeDt(float input, float dt) {
  _updated = false;
  if (!finiteF(input)) {                        // датчик неисправен: безопасный выход, состояние не портим
    _err = ERR_BAD_INPUT; _nBad++;
    _out = clampOut(_hasFailsafe ? _failsafe : _min);
    _updated = true;
    return _out;
  }
  if (!finiteF(dt) || dt < 0) { _err = ERR_BAD_SAMPLE; return _out; }

  float e = _dir == DIRECT ? _sp - input : input - _sp;
  if (_dead > 0 && e < _dead && e > -_dead) e = 0;
  _e = e;

  if (_mode == MANUAL) {                        // следим за входом, чтобы переход в AUTO был плавным
    _lastIn = input; _hasLast = true; _hasOut = true;
    _updated = true;
    return _out;
  }

  float dIn = (_hasLast && dt > 0) ? (input - _lastIn) : 0.0f;
  float rawD = dt > 0 ? (_dir == DIRECT ? -dIn : dIn) / dt : 0.0f;       // производная по измерению, а не по ошибке: нет пинка при смене уставки
  if (_initPending) {                           // безударное включение: подобрать интеграл так, чтобы выход не скакнул
    _i = clampOut(_out - _kp * e - _ff);
    _dF = 0; rawD = 0; _initPending = false;
  }
  _lastIn = input; _hasLast = true;
  if (_tau > 0 && dt > 0) _dF += (dt / (_tau + dt)) * (rawD - _dF);
  else _dF = rawD;

  _p = _kp * e;
  _d = _kd * _dF;

  // Условное интегрирование: пока выход упёрся (в предел или в ограничение скорости) в сторону ошибки, интеграл не растёт
  float u = _p + _i + _d + _ff;
  float target = clampOut(u);
  if (_slew > 0 && _hasOut && dt > 0) {
    float step = _slew * dt;
    if (target > _out + step) target = _out + step;
    else if (target < _out - step) target = _out - step;
  }
  bool windup = (target < u && e > 0) || (target > u && e < 0);
  if (!windup) _i = clampOut(_i + _ki * e * dt);

  u = _p + _i + _d + _ff;
  float o = clampOut(u);
  _sat = (o != u);
  if (_slew > 0 && _hasOut && dt > 0) {
    float step = _slew * dt;
    if (o > _out + step) o = _out + step;
    else if (o < _out - step) o = _out - step;
  }
  _out = o; _hasOut = true;
  _updated = true; _nCalc++;
  return _out;
}

// ---- Подбор коэффициентов ------------------------------------------------------------------------
void AlashPID::gainsFromUltimate(float ku, float tu, Rule rule, float &kp, float &ki, float &kd) {
  float ti = 0, td = 0;
  switch (rule) {
    case RULE_P:               kp = 0.5f * ku; break;
    case RULE_PI:              kp = 0.45f * ku; ti = tu / 1.2f; break;
    case RULE_NO_OVERSHOOT:    kp = 0.2f * ku;  ti = tu / 2.0f; td = tu / 3.0f; break;
    case RULE_SOME_OVERSHOOT:  kp = 0.33f * ku; ti = tu / 2.0f; td = tu / 3.0f; break;
    case RULE_TYREUS_LUYBEN:   kp = ku / 2.2f;  ti = 2.2f * tu; td = tu / 6.3f; break;
    case RULE_PID: default:    kp = 0.6f * ku;  ti = tu / 2.0f; td = tu / 8.0f; break;
  }
  ki = ti > 0 ? kp / ti : 0.0f;
  kd = kp * td;
}

bool AlashPID::tuneFromUltimate(float ku, float tu, Rule rule) {
  if (!finiteF(ku) || !finiteF(tu) || ku <= 0 || tu <= 0) { _err = ERR_BAD_TUNING; return false; }
  float kp, ki, kd;
  gainsFromUltimate(ku, tu, rule, kp, ki, kd);
  return setTunings(kp, ki, kd);
}

// ---- Диагностика ------------------------------------------------------------------------------
const __FlashStringHelper *AlashPID::errorText() {
  switch (_err) {
    case ERR_OK: return F("OK");
    case ERR_BAD_TUNING:
      return T("Коэффициенты Kp, Ki, Kd должны быть числами не меньше нуля. Прежние значения сохранены. Направление (нагрев или охлаждение) задаётся setDirection(), а не знаком.",
               "Kp, Ki, Kd коэффициенттері нөлден кем емес сан болуы керек. Бұрынғы мәндер сақталды. Бағыт setDirection() арқылы беріледі, таңба арқылы емес.",
               "Kp, Ki, Kd must be numbers not below zero. The previous values were kept. Use setDirection() for heating or cooling, not a negative sign.");
    case ERR_BAD_LIMITS:
      return T("Пределы выхода заданы неверно: минимум должен быть меньше максимума. Для ШИМ на Arduino это 0 и 255.",
               "Шығыс шектері қате: минимум максимумнан кіші болуы керек. Arduino ШИМ үшін бұл 0 және 255.",
               "Output limits are wrong: the minimum must be below the maximum. For Arduino PWM that is 0 and 255.");
    case ERR_BAD_SAMPLE:
      return T("Шаг времени отрицательный или не число. Используйте compute(вход) или положительный шаг в секундах.",
               "Уақыт қадамы теріс немесе сан емес. compute(кіріс) немесе оң қадам (секунд) қолданыңыз.",
               "The time step is negative or not a number. Use compute(input) or a positive step in seconds.");
    case ERR_BAD_INPUT:
      return T("Вход не число (NaN или бесконечность): датчик не отвечает или вычисление дало ошибку. Выход переведён в безопасное значение (setFailsafe).",
               "Кіріс сан емес (NaN немесе шексіздік): датчик жауап бермейді. Шығыс қауіпсіз мәнге ауыстырылды (setFailsafe).",
               "The input is not a number (NaN or infinity): the sensor does not answer. The output went to the safe value (setFailsafe).");
    case ERR_BAD_VALUE:
      return T("Параметр вне допустимого: уставка не число, либо фильтр, мёртвая зона или скорость отрицательные.",
               "Параметр жарамсыз: уставка сан емес, не сүзгі, өлі аймақ немесе жылдамдық теріс.",
               "A parameter is out of range: the setpoint is not a number, or the filter, deadband or slew rate is negative.");
  }
  return F("");
}

void AlashPID::diagnose(Print &out) {
  out.println(T("AlashPID: диагностика", "AlashPID: диагностика", "AlashPID: diagnostics"));
  out.print(T("Kp=", "Kp=", "Kp=")); out.print(_kp, 4);
  out.print(T("  Ki=", "  Ki=", "  Ki=")); out.print(_ki, 4);
  out.print(T("  Kd=", "  Kd=", "  Kd=")); out.println(_kd, 4);
  out.print(T("Уставка: ", "Уставка: ", "Setpoint: ")); out.print(_sp, 3);
  out.print(T("   пределы выхода: ", "   шығыс шектері: ", "   output limits: ")); out.print(_min, 2); out.print(F("..")); out.println(_max, 2);
  out.println(_mode == AUTO ? T("Режим: автоматический", "Режим: автоматты", "Mode: automatic") : T("Режим: ручной (регулятор не считает)", "Режим: қолмен (реттегіш есептемейді)", "Mode: manual (the controller does not compute)"));
  out.println(_dir == DIRECT ? T("Направление: DIRECT (больше выход, больше вход)", "Бағыты: DIRECT (шығыс көп, кіріс көп)", "Direction: DIRECT (more output, more input)")
                             : T("Направление: REVERSE (больше выход, меньше вход)", "Бағыты: REVERSE (шығыс көп, кіріс аз)", "Direction: REVERSE (more output, less input)"));
  out.print(T("Шаг расчёта, мс: ", "Есептеу қадамы, мс: ", "Sample time, ms: ")); out.println((long)_sample);
  out.print(T("Ошибка: ", "Қате: ", "Error: ")); out.print(_e, 3);
  out.print(F("  P=")); out.print(_p, 3); out.print(F("  I=")); out.print(_i, 3); out.print(F("  D=")); out.print(_d, 3);
  out.print(T("  выход: ", "  шығыс: ", "  output: ")); out.println(_out, 3);
  out.print(T("Расчётов: ", "Есептеулер: ", "Computations: ")); out.print((long)_nCalc);
  out.print(T("   плохих входов: ", "   жарамсыз кірістер: ", "   bad inputs: ")); out.println((long)_nBad);
  // подсказки
  if (_kp == 0 && _ki == 0 && _kd == 0)
    out.println(T("Подсказка: все коэффициенты равны нулю, выход не будет меняться. Начните с Kp.", "Кеңес: барлық коэффициент нөл, шығыс өзгермейді. Kp-дан бастаңыз.", "Tip: all gains are zero, the output will not change. Start with Kp."));
  else if (_ki == 0 && _mode == AUTO)
    out.println(T("Подсказка: Ki = 0, поэтому останется постоянная ошибка. Добавьте небольшой Ki.", "Кеңес: Ki = 0, сондықтан тұрақты қате қалады. Аздап Ki қосыңыз.", "Tip: Ki = 0, so a steady error will remain. Add a small Ki."));
  if (_sat)
    out.println(T("Подсказка: выход упёрся в предел. Проверьте setOutputLimits(), мощность нагрузки и направление (DIRECT/REVERSE).", "Кеңес: шығыс шекке тіреліп тұр. setOutputLimits(), жүктеме қуаты мен бағытты (DIRECT/REVERSE) тексеріңіз.", "Tip: the output is stuck at a limit. Check setOutputLimits(), the load power and the direction (DIRECT/REVERSE)."));
  out.println(T("Если выход растёт, а вход уходит от уставки, поменяйте DIRECT на REVERSE. Если раскачивается, уменьшите Kp и Ki.", "Шығыс өсіп, кіріс уставкадан алыстаса, DIRECT-ті REVERSE-ке ауыстырыңыз. Тербелсе, Kp мен Ki азайтыңыз.", "If the output grows while the input runs away from the setpoint, switch DIRECT to REVERSE. If it oscillates, lower Kp and Ki."));
  if (_err != ERR_OK) { out.print(T("Последняя ошибка: ", "Соңғы қате: ", "Last error: ")); out.println(errorText()); }
}

// =================================================================================================
// Автонастройка
// =================================================================================================
AlashPIDAutoTune::AlashPIDAutoTune(float setpoint, float outPushUp, float outPushDown, float hysteresis, uint8_t cycles)
    : _sp(setpoint), _up(outPushUp), _down(outPushDown), _h(hysteresis < 0 ? 0 : hysteresis), _out(outPushDown), _cycles(cycles < 2 ? 2 : cycles) {}

float AlashPIDAutoTune::run(float input) { return runAt(input, millis()); }

float AlashPIDAutoTune::runAt(float input, uint32_t now) {
  if (_state == DONE || _state == TIMEOUT || _state == BAD_INPUT || _state == FAILED) return _down;
  if (!finiteF(input)) { _state = BAD_INPUT; return _down; }
  if (_state == IDLE) {
    _state = RUNNING; _t0 = now;
    _goingUp = input < _sp;
    _min = _max = input;
    _out = _goingUp ? _up : _down;
    return _out;
  }
  if (_timeout > 0 && now - _t0 > _timeout) { _state = TIMEOUT; return _down; }

  if (_goingUp) {                               // выход «вверх»: вход сначала ещё падает (минимум), потом растёт
    if (input < _min) _min = input;
    if (input > _sp + _h) {                     // пересекли верхнюю границу: переключаем на «вниз»
      _lastMin = _min; _haveMin = true;
      if (_haveSwitch) {                        // один период между двумя такими переключениями
        float period = (now - _tSwitch) / 1000.0f;
        if (_haveMax && _haveMin) {
          if (_skip > 0) _skip--;
          else { _sumTu += period; _sumAmp += (_lastMax - _lastMin) * 0.5f; _measured++; }
        }
      }
      _tSwitch = now; _haveSwitch = true;
      _goingUp = false; _max = input; _out = _down;
    }
  } else {
    if (input > _max) _max = input;
    if (input < _sp - _h) {
      _lastMax = _max; _haveMax = true;
      _goingUp = true; _min = input; _out = _up;
    }
  }

  if (_measured >= _cycles) {
    _tu = _sumTu / _measured;
    _amp = _sumAmp / _measured;
    float d = (_up > _down ? _up - _down : _down - _up) * 0.5f;
    float a2 = _amp * _amp - _h * _h;
    if (_amp <= 0 || a2 <= 0 || d <= 0 || _tu <= 0) { _state = FAILED; return _down; }
    float root = sqrtf(a2);
    _ku = 4.0f * d / (3.14159265f * root);
    _state = DONE;
    return _down;
  }
  return _out;
}

bool AlashPIDAutoTune::apply(AlashPID &pid, AlashPID::Rule rule) {
  if (_state != DONE) return false;
  return pid.tuneFromUltimate(_ku, _tu, rule);
}
