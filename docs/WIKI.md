# AlashPID: руководство / нұсқаулық / guide

[Русский](#русский) | [Қазақша](#қазақша) | [English](#english)

---

## Русский

Регулятор, который **держит значение на уставке**: температуру печки, скорость мотора, освещённость. Пишете `analogWrite(pin, pid.compute(датчик))`, и всё. Без указателей на глобальные переменные, с защитой от типичных проблем (раскрутка интеграла, пинок при смене уставки, сбой датчика) и **автонастройкой коэффициентов**. Без зависимостей.

### Установка
Arduino IDE: Скетч, Подключить библиотеку, Управлять библиотеками, найти **AlashPID** и нажать Установить. Или скачайте ZIP с GitHub: Скетч, Подключить библиотеку, Добавить .ZIP библиотеку.

### Платы
Arduino Uno / Nano / Mega (AVR), ESP32, ESP8266 и другие, где работает Arduino-ядро.

### Быстрый старт
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

### Справочник по методам
| Метод | Что делает |
|---|---|
| `AlashPID(kp, ki, kd, direction)` | Создать регулятор; DIRECT (нагрев) или REVERSE (охлаждение) |
| `compute(input)` | Новый выход; между периодами возвращает прежний |
| `computeDt(input, dt)` | То же с явным шагом в секундах |
| `updated()` | Был ли посчитан новый выход |
| `setSetpoint(v)` | Уставка |
| `setTunings(kp, ki, kd) / setTimes(kp, Ti, Td)` | Коэффициенты или времена |
| `setOutputLimits(min, max)` | Пределы выхода (по умолчанию 0..255) |
| `setSampleTime(ms)` | Период расчёта; 0 = при каждом вызове |
| `setDerivativeFilter(sec)` | Фильтр производной (шум датчика) |
| `setDeadband(x) / setSlewRate(x) / setFeedForward(x)` | Мёртвая зона / скорость выхода / добавка |
| `setFailsafe(x)` | Выход при неисправном датчике |
| `setMode(MANUAL/AUTO) / setManualOutput(x)` | Ручной и автоматический режим, переход безударный |
| `pTerm() / iTerm() / dTerm() / saturated()` | Слагаемые выхода и признак упора в предел |
| `tuneFromUltimate(Ku, Tu, rule)` | Коэффициенты по Ku и Tu (Циглер-Николс и др.) |
| `AlashPIDAutoTune(sp, up, down, hyst, cycles)` | Автонастройка реле: выход «вверх», «вниз», гистерезис |
| `run(input) / done() / apply(pid, rule)` | Шаг настройки / закончена ли / записать коэффициенты |
| `diagnose(Serial) / errorText() / setLanguage()` | Самопроверка и сообщения на ru/kk/en |

### Если что-то не работает
**Выход упирается в максимум или минимум и не отпускает**

Проверьте направление: для нагревателя DIRECT, для охладителя REVERSE (`setDirection`). Убедитесь, что пределы подходят (`setOutputLimits`) и мощности нагрузки хватает: `diagnose(Serial)` скажет, упёрся ли выход.

**Значение раскачивается вокруг уставки**

Уменьшите Kp и Ki. Если датчик шумит, добавьте `setDerivativeFilter(0.5)` или уменьшите Kd. Для автоматического подбора используйте `AlashPIDAutoTune` (пример AutoTune).

**Значение не доходит до уставки немного**

Нужен интеграл: задайте Ki больше нуля (например 0.1 от Kp). Если Ki есть, а ошибка остаётся, выход упёрся в предел.

**Регулятор считает слишком редко или слишком часто**

`setSampleTime(мс)` задаёт период. Он должен быть заметно короче постоянной времени объекта: для печки 100..1000 мс, для мотора 10..50 мс. Если вызываете из таймера, используйте `computeDt(вход, секунды)`.

**Автонастройка не заканчивается**

Объект должен реагировать на выход: нагреватель должен реально греть, а датчик показывать это. Задайте `setTimeout`, и после неудачи `state()` скажет причину. Если датчик шумит, увеличьте гистерезис.

### Самопроверка
В библиотеке есть `diagnose(Serial)`: она печатает, что видит плата, и подсказывает, что проверить. Язык подсказок: `setLanguage(LANG_RU / LANG_KK / LANG_EN)`.

### Примеры
`Basic`, `Simulation`, `AutoTune`, `Diagnose`

---

## Қазақша

Мәнді **уставкада ұстап тұратын** реттегіш: пеш температурасы, мотор жылдамдығы, жарықтық. `analogWrite(pin, pid.compute(датчик))` жазасыз, болды. Глобал айнымалыларға көрсеткіштерсіз, типтік мәселелерден қорғаумен (интегралдың өсіп кетуі, уставка өзгергендегі соққы, датчик ақауы) және **коэффициенттерді автоматты баптаумен**. Тәуелділіксіз.

### Орнату
Arduino IDE: Sketch, Include Library, Manage Libraries, **AlashPID** табып, Install басыңыз. Немесе GitHub-тан ZIP жүктеп, Sketch, Include Library, Add .ZIP Library.

### Тақталар
Arduino Uno / Nano / Mega (AVR), ESP32, ESP8266 және Arduino ядросы бар басқалар.

### Жылдам бастау
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

### Әдістер анықтамасы
| Әдіс | Не істейді |
|---|---|
| `AlashPID(kp, ki, kd, direction)` | Реттегіш жасау; DIRECT немесе REVERSE |
| `compute(input)` | Жаңа шығыс |
| `computeDt(input, dt)` | Қадаммен (секунд) |
| `updated()` | Жаңа шығыс есептелді ме |
| `setSetpoint(v)` | Уставка |
| `setTunings(kp, ki, kd) / setTimes(kp, Ti, Td)` | Коэффициенттер немесе уақыттар |
| `setOutputLimits(min, max)` | Шығыс шектері |
| `setSampleTime(ms)` | Есептеу кезеңі |
| `setDerivativeFilter(sec)` | Туынды сүзгісі |
| `setDeadband(x) / setSlewRate(x) / setFeedForward(x)` | Өлі аймақ / жылдамдық / қосымша |
| `setFailsafe(x)` | Датчик ақауындағы шығыс |
| `setMode(MANUAL/AUTO) / setManualOutput(x)` | Қолмен және автоматты режим |
| `pTerm() / iTerm() / dTerm() / saturated()` | Шығыс құраушылары |
| `tuneFromUltimate(Ku, Tu, rule)` | Ku және Tu бойынша коэффициенттер |
| `AlashPIDAutoTune(sp, up, down, hyst, cycles)` | Реле автобаптауы |
| `run(input) / done() / apply(pid, rule)` | Баптау қадамы |
| `diagnose(Serial) / errorText() / setLanguage()` | Өзін-өзі тексеру |

### Бірдеңе жұмыс істемесе
**Шығыс максимумға немесе минимумға тірелді**

Бағытты тексеріңіз: қыздырғышқа DIRECT, салқындатқышқа REVERSE. Шектер мен жүктеме қуатын тексеріңіз; `diagnose(Serial)` айтады.

**Мән уставка төңірегінде тербеледі**

Kp мен Ki азайтыңыз. Датчик шулы болса, `setDerivativeFilter(0.5)` қосыңыз. Автоматты таңдау үшін `AlashPIDAutoTune` қолданыңыз.

**Мән уставкаға сәл жетпейді**

Интеграл керек: Ki нөлден үлкен болсын. Ki бар болса да қате қалса, шығыс шекке тіреліп тұр.

**Реттегіш тым сирек немесе тым жиі есептейді**

`setSampleTime(мс)` кезеңді береді. Ол нысанның уақыт тұрақтысынан айтарлықтай қысқа болуы керек. Таймерден шақырсаңыз, `computeDt(кіріс, секунд)`.

**Автобаптау аяқталмайды**

Нысан шығысқа жауап беруі керек. `setTimeout` беріп, сәтсіз болса `state()` себебін айтады. Датчик шулы болса, гистерезисті үлкейтіңіз.

### Өзін-өзі тексеру
Кітапханада `diagnose(Serial)` бар: тақта не көргенін басып шығарады және нені тексеру керегін айтады. Кеңес тілі: `setLanguage(LANG_RU / LANG_KK / LANG_EN)`.

### Мысалдар
`Basic`, `Simulation`, `AutoTune`, `Diagnose`

---

## English

A controller that **holds a value at a setpoint**: oven temperature, motor speed, light level. Write `analogWrite(pin, pid.compute(sensor))` and you are done. No pointers to global variables, protection from the usual problems (integral windup, kick on setpoint change, sensor failure) and **automatic gain tuning**. No dependencies.

### Installation
Arduino IDE: Sketch, Include Library, Manage Libraries, search **AlashPID** and click Install. Or download the ZIP from GitHub: Sketch, Include Library, Add .ZIP Library.

### Boards
Arduino Uno / Nano / Mega (AVR), ESP32, ESP8266 and others with an Arduino core.

### Quick start
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

### Method reference
| Method | What it does |
|---|---|
| `AlashPID(kp, ki, kd, direction)` | Create a controller; DIRECT (heating) or REVERSE (cooling) |
| `compute(input)` | New output; returns the previous one between periods |
| `computeDt(input, dt)` | Same with an explicit step in seconds |
| `updated()` | Whether a new output was computed |
| `setSetpoint(v)` | Setpoint |
| `setTunings(kp, ki, kd) / setTimes(kp, Ti, Td)` | Gains or times |
| `setOutputLimits(min, max)` | Output limits (default 0..255) |
| `setSampleTime(ms)` | Compute period; 0 = every call |
| `setDerivativeFilter(sec)` | Derivative filter (sensor noise) |
| `setDeadband(x) / setSlewRate(x) / setFeedForward(x)` | Deadband / output rate / feed-forward |
| `setFailsafe(x)` | Output on a failed sensor |
| `setMode(MANUAL/AUTO) / setManualOutput(x)` | Manual and automatic mode, bumpless |
| `pTerm() / iTerm() / dTerm() / saturated()` | Output terms and the saturation flag |
| `tuneFromUltimate(Ku, Tu, rule)` | Gains from Ku and Tu (Ziegler-Nichols etc.) |
| `AlashPIDAutoTune(sp, up, down, hyst, cycles)` | Relay auto-tune: output up, down, hysteresis |
| `run(input) / done() / apply(pid, rule)` | Tuning step / is it done / apply the gains |
| `diagnose(Serial) / errorText() / setLanguage()` | Self-check and messages in ru/kk/en |

### If something does not work
**The output sticks at the maximum or minimum**

Check the direction: DIRECT for a heater, REVERSE for a cooler (`setDirection`). Check the limits (`setOutputLimits`) and that the load is strong enough: `diagnose(Serial)` tells whether the output is stuck.

**The value swings around the setpoint**

Lower Kp and Ki. If the sensor is noisy, add `setDerivativeFilter(0.5)` or lower Kd. Use `AlashPIDAutoTune` for automatic tuning (AutoTune example).

**The value stays slightly below the setpoint**

You need the integral: set Ki above zero (for example 0.1 of Kp). If Ki is set and the error stays, the output is at its limit.

**The controller computes too rarely or too often**

`setSampleTime(ms)` sets the period. It must be well below the plant time constant: 100..1000 ms for an oven, 10..50 ms for a motor. From a timer use `computeDt(input, seconds)`.

**Auto-tune never finishes**

The plant must react to the output: the heater must really heat and the sensor must show it. Set `setTimeout`; after a failure `state()` tells the reason. If the sensor is noisy, increase the hysteresis.

### Self-check
The library has `diagnose(Serial)`: it prints what the board sees and tells what to check. Tips language: `setLanguage(LANG_RU / LANG_KK / LANG_EN)`.

### Examples
`Basic`, `Simulation`, `AutoTune`, `Diagnose`

---

MIT License, Alash Electronics. https://github.com/Alash-electronics/AlashPID
