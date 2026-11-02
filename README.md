# AlashPID

[Русский](#русский) | [Қазақша](#қазақша) | [English](#english)

---

## Русский

Регулятор, который **держит значение на уставке**: температуру печки, скорость мотора, освещённость. Пишете `analogWrite(pin, pid.compute(датчик))`, и всё. Без указателей на глобальные переменные, с защитой от типичных проблем (раскрутка интеграла, пинок при смене уставки, сбой датчика) и **автонастройкой коэффициентов**. Без зависимостей.

```cpp
#include <AlashPID.h>
AlashPID pid(2.0, 0.5, 0.0);          // Kp, Ki, Kd

void setup() {
  pid.setSetpoint(60);                // желаемое значение
  pid.setOutputLimits(0, 255);        // диапазон ШИМ
}
void loop() {
  float out = pid.compute(readSensor());   // читаем датчик, получаем выход
  analogWrite(9, (int)out);
}
```

### Что умеет
- **`compute(вход)` возвращает выход:** без указателей и глобальных переменных, вход и выход обычные `float`.
- **Защита от windup:** интеграл не растёт, пока выход упёрся в предел или в ограничение скорости; после перелёта выход сразу уходит с предела.
- **Производная по измерению:** при смене уставки нет скачка выхода; `setDerivativeFilter(сек)` сглаживает шум датчика.
- **Безударный `setMode(MANUAL/AUTO)`:** выход при переключении не прыгает.
- **Сбой датчика:** если вход NaN, выход переходит в безопасное значение (`setFailsafe`), внутреннее состояние не портится.
- **Мёртвая зона, ограничение скорости, feedforward:** `setDeadband`, `setSlewRate`, `setFeedForward`.
- **Шаг расчёта:** `setSampleTime(мс)` или `computeDt(вход, секунды)` для таймера; безопасно при переполнении `millis()`.
- **Автонастройка:** `AlashPIDAutoTune` методом реле находит Ku и Tu, правила Циглера-Николса и Тайреуса-Люйбена.
- **Видно, что происходит:** `pTerm()`, `iTerm()`, `dTerm()`, `saturated()` и `diagnose(Serial)` с подсказками на трёх языках.
- Работает на **Arduino AVR, ESP32, ESP8266**.

### Чем отличается от других библиотек
Мы изучили PID (Brett Beauregard, он же PID_v1), QuickPID, ArduPID, GyverPID, FastPID (все MIT, форк возможен, но заново удобнее):
- PID_v1 и QuickPID требуют указатели на глобальные переменные `double Input, Output, Setpoint`; у нас `out = pid.compute(in)`.
- У PID_v1 только ограничение интеграла и нет фильтра производной; у нас условное интегрирование (учитывает и предел скорости) и фильтр.
- При NaN от датчика в PID_v1 состояние портится навсегда; у нас безопасный выход и восстановление.
- Автонастройка есть в QuickPID, но тексты и разбор проблем только на английском; у нас `diagnose()` на ru/kk/en и примеры без железа.
- ServoEasing и подобные не пересекаются: это про другое.

### Примеры
`Basic`, `Simulation`, `AutoTune`, `Diagnose`

### Тесты
Проверено без железа на моделях объекта: пропорциональное и интегральное действие, производная без пинка, windup, пределы, шаг и переполнение `millis()`, замкнутый контур нагревателя и охладителя, безударный переход, NaN, мёртвая зона, ограничение скорости, автонастройка с последующим запуском регулятора: `extras/test`.

Подробное руководство: [docs/WIKI.md](docs/WIKI.md).

---

## Қазақша

Мәнді **уставкада ұстап тұратын** реттегіш: пеш температурасы, мотор жылдамдығы, жарықтық. `analogWrite(pin, pid.compute(датчик))` жазасыз, болды. Глобал айнымалыларға көрсеткіштерсіз, типтік мәселелерден қорғаумен (интегралдың өсіп кетуі, уставка өзгергендегі соққы, датчик ақауы) және **коэффициенттерді автоматты баптаумен**. Тәуелділіксіз.

```cpp
#include <AlashPID.h>
AlashPID pid(2.0, 0.5, 0.0);          // Kp, Ki, Kd

void setup() {
  pid.setSetpoint(60);                // желаемое значение
  pid.setOutputLimits(0, 255);        // диапазон ШИМ
}
void loop() {
  float out = pid.compute(readSensor());   // читаем датчик, получаем выход
  analogWrite(9, (int)out);
}
```

### Мүмкіндіктері
- **`compute(кіріс)` шығысты қайтарады:** көрсеткіштерсіз және глобал айнымалысыз, кіріс пен шығыс қарапайым `float`.
- **Windup-тан қорғау:** шығыс шекке немесе жылдамдық шектеуіне тірелгенде интеграл өспейді; асып кеткеннен кейін шығыс шектен бірден кетеді.
- **Өлшем бойынша туынды:** уставка өзгергенде шығыс секірмейді; `setDerivativeFilter(сек)` датчик шуын тегістейді.
- **Соққысыз `setMode(MANUAL/AUTO)`:** ауыстырғанда шығыс секірмейді.
- **Датчик ақауы:** кіріс NaN болса, шығыс қауіпсіз мәнге ауысады (`setFailsafe`), ішкі күй бүлінбейді.
- **Өлі аймақ, жылдамдық шектеуі, feedforward:** `setDeadband`, `setSlewRate`, `setFeedForward`.
- **Есептеу қадамы:** `setSampleTime(мс)` немесе таймер үшін `computeDt(кіріс, секунд)`; `millis()` толғанда да қауіпсіз.
- **Автоматты баптау:** `AlashPIDAutoTune` реле әдісімен Ku мен Tu табады, Циглер-Николс және Тайреус-Люйбен ережелері.
- **Не болып жатқаны көрінеді:** `pTerm()`, `iTerm()`, `dTerm()`, `saturated()` және үш тілде кеңес беретін `diagnose(Serial)`.
- **Arduino AVR, ESP32, ESP8266** тақталарында жұмыс істейді.

### Басқа кітапханалардан айырмашылығы
PID (Brett Beauregard, PID_v1), QuickPID, ArduPID, GyverPID, FastPID қарадық (барлығы MIT):
- PID_v1 және QuickPID глобал `double Input, Output, Setpoint` айнымалыларына көрсеткіш сұрайды; бізде `out = pid.compute(in)`.
- PID_v1-де туынды сүзгісі жоқ; бізде шартты интегралдау мен сүзгі бар.
- Датчиктен NaN келсе, PID_v1 күйі мәңгі бүлінеді; бізде қауіпсіз шығыс пен қалпына келу.
- QuickPID-те автобаптау бар, бірақ мәтіндер тек ағылшынша; бізде `diagnose()` ru/kk/en және құрылғысыз мысалдар.

### Мысалдар
`Basic`, `Simulation`, `AutoTune`, `Diagnose`

### Тесттер
Құрылғысыз нысан модельдерінде тексерілген: пропорционалды және интегралдық әсер, соққысыз туынды, windup, шектер, қадам және `millis()` толуы, қыздырғыш пен салқындатқыштың тұйық контуры, соққысыз ауысу, NaN, өлі аймақ, жылдамдық шектеуі, автобаптау: `extras/test`.

Толық нұсқаулық: [docs/WIKI.md](docs/WIKI.md).

---

## English

A controller that **holds a value at a setpoint**: oven temperature, motor speed, light level. Write `analogWrite(pin, pid.compute(sensor))` and you are done. No pointers to global variables, protection from the usual problems (integral windup, kick on setpoint change, sensor failure) and **automatic gain tuning**. No dependencies.

```cpp
#include <AlashPID.h>
AlashPID pid(2.0, 0.5, 0.0);          // Kp, Ki, Kd

void setup() {
  pid.setSetpoint(60);                // желаемое значение
  pid.setOutputLimits(0, 255);        // диапазон ШИМ
}
void loop() {
  float out = pid.compute(readSensor());   // читаем датчик, получаем выход
  analogWrite(9, (int)out);
}
```

### Features
- **`compute(input)` returns the output:** no pointers and no globals, input and output are plain `float`.
- **Anti-windup:** the integral stops growing while the output is stuck at a limit or slew limit; after an overshoot the output leaves the limit at once.
- **Derivative on measurement:** no output jump when the setpoint changes; `setDerivativeFilter(sec)` smooths sensor noise.
- **Bumpless `setMode(MANUAL/AUTO)`:** the output does not jump on switching.
- **Sensor failure:** a NaN input sends the output to a safe value (`setFailsafe`) and leaves the internal state intact.
- **Deadband, slew rate, feed-forward:** `setDeadband`, `setSlewRate`, `setFeedForward`.
- **Sample time:** `setSampleTime(ms)` or `computeDt(input, seconds)` for a timer; safe over `millis()` overflow.
- **Auto-tuning:** `AlashPIDAutoTune` finds Ku and Tu by the relay method; Ziegler-Nichols and Tyreus-Luyben rules.
- **See what happens:** `pTerm()`, `iTerm()`, `dTerm()`, `saturated()` and `diagnose(Serial)` with tips in three languages.
- Works on **Arduino AVR, ESP32, ESP8266**.

### How it differs from other libraries
We studied PID (Brett Beauregard's PID_v1), QuickPID, ArduPID, GyverPID and FastPID (all MIT):
- PID_v1 and QuickPID need pointers to global `double Input, Output, Setpoint`; ours is `out = pid.compute(in)`.
- PID_v1 only clamps the integral and has no derivative filter; ours uses conditional integration (also aware of the slew limit) and a filter.
- A NaN from the sensor corrupts PID_v1 state for good; ours goes to a safe output and recovers.
- QuickPID has auto-tune, but all text and troubleshooting are English only; ours has `diagnose()` in ru/kk/en and hardware-free examples.

### Examples
`Basic`, `Simulation`, `AutoTune`, `Diagnose`

### Tests
Checked without hardware on plant models: proportional and integral action, derivative without kick, windup, limits, sample time and `millis()` overflow, closed loop for a heater and a cooler, bumpless transfer, NaN, deadband, slew limit, auto-tune followed by a controller run: `extras/test`.

Full guide: [docs/WIKI.md](docs/WIKI.md).

---

MIT License, Alash Electronics.
