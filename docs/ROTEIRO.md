# Roteiro Sugerido — Construindo o `main.cpp`

Este é um caminho sugerido, não obrigatório. Cada etapa produz um resultado
testável antes de avançar para a próxima — não pule etapas, mesmo que pareça
óbvio, porque cada uma valida uma parte diferente do sistema.

---

## Etapa 1 — O robô se move

**Objetivo:** confirmar que os motores, a pinagem e o trim estão corretos
antes de adicionar qualquer lógica de sensor.

```cpp
#include <Arduino.h>
#include "config.h"
#include "MotorController.h"

MotorController motor;

void setup() {
    motor.initialize();
}

void loop() {
    motor.move(MotorController::FORWARD, 150);
    delay(2000);
    motor.stop();
    delay(1000);
    motor.move(MotorController::TURN_LEFT, 150);
    delay(700);
    motor.stop();
    while (1);   // trava aqui — só queremos testar uma vez
}
```

**Checkpoint:** o robô andou reto (sem puxar para um lado) e depois girou?
Se desviou, ajuste `MOTOR_TRIM_ESQ` / `MOTOR_TRIM_DIR` em `config.h`.

---

## Etapa 2 — O robô segue a linha

**Objetivo:** ler o sensor de linha e usar o PID para corrigir a trajetória.

```cpp
#include <Arduino.h>
#include "config.h"
#include "LineSensor.h"
#include "MotorController.h"

LineSensor      lineSensor;
MotorController motor;

void setup() {
    lineSensor.initialize();
    motor.initialize();
    delay(2000);   // tempo para posicionar o robô na linha
}

void loop() {
    lineSensor.readSensors();
    float pos = lineSensor.getLinePosition();

    if (lineSensor.getLinePattern() == LineSensor::LINE_LOST) {
        motor.stop();
        return;
    }

    motor.followLine(pos, 150);   // 150 = velocidade base
    delay(PD_SAMPLE_MS);
}
```

**Checkpoint:** o robô segue a linha sem oscilar (zigue-zague) e sem perder
a linha em curvas? Se oscilar, revise `PD_KP`/`PD_KD` em `config.h` — o
`docs/GUIA_BIBLIOTECA.md` explica o que cada ganho faz.

---

## Etapa 3 — O robô detecta objeto

**Objetivo:** adicionar o sensor ultrassônico e fazer o robô reagir à distância.

```cpp
#include "UltrasonicSensor.h"

UltrasonicSensor ultrasonic;

// no setup():
ultrasonic.initialize();

// no loop(), antes do followLine():
int dist = ultrasonic.readDistance();

if (ultrasonic.isReadingStable() && dist > 0 && dist <= ULTRASONIC_DISTANCE_CONTACT) {
    motor.stop();
    Serial.println("Objeto detectado!");
    delay(2000);   // por enquanto só paramos — a coleta vem na próxima etapa
    return;
}
```

**Checkpoint:** o robô para quando um objeto está na zona de contato e
continua seguindo a linha quando não há objeto?

---

## Etapa 4 — O robô coleta

**Objetivo:** adicionar a garra e a sequência completa de manobra.

```cpp
#include "GripperServo.h"

GripperServo gripper;

// no setup():
gripper.initialize();

// substituindo o "para e espera" da Etapa 3:
motor.stop();
gripper.close();
delay(COLLECT_STOP_DELAY);

// gira, avança, solta, recua, gira de volta — você decide os valores
// de delay() e direção. Comece simples (só fechar e abrir) e vá
// complicando a manobra conforme testar.

gripper.open();
```

**Checkpoint:** a garra fecha sem o servo "vibrar" indefinidamente (se isso
acontecer, confira se está usando a versão atual de `GripperServo.cpp` —
veja a nota sobre o bug de underflow no `GUIA_BIBLIOTECA.md`).

---

## Etapa 5 — Integração completa

**Objetivo:** juntar tudo em uma máquina de estados, sem misturar a lógica
de seguir linha com a lógica de coletar no mesmo bloco de código.

Sugestão de estrutura (não copie literalmente — adapte ao que você já escreveu):

```cpp
enum Estado { SEGUINDO, COLETANDO, RECUPERANDO };
Estado estado = SEGUINDO;

void loop() {
    switch (estado) {
        case SEGUINDO:    /* sua lógica da Etapa 2 + 3 */ break;
        case COLETANDO:   /* sua lógica da Etapa 4 */     break;
        case RECUPERANDO: /* o que fazer se perder a linha */ break;
    }
}
```

**Por que separar em estados?** Sem isso, o código de seguir linha e o
código de coletar competem pelo mesmo `loop()` e ficam confusos sobre
"o que fazer agora". Com estados, cada bloco de código só se preocupa
com uma responsabilidade.

**Checkpoint final:** o robô completa o percurso, detecta e coleta um
objeto, e volta a seguir a linha depois — sozinho, sem intervenção manual.

---

## E se travar?

Cada etapa tem um checkpoint específico. Se algo não funcionar, volte para
o checkpoint anterior e confirme que ele ainda funciona antes de avançar.
A maior parte dos problemas em projetos de robótica vem de uma etapa
anterior que parecia funcionar mas tinha um problema sutil.

Chame um monitor se travar por mais de 5 minutos em qualquer checkpoint.
