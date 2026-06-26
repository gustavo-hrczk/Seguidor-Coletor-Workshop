# Guia da Biblioteca Técnica

Este documento explica o que cada módulo faz, como usar, e principalmente
**por que** foi implementado dessa forma. Entender o "por quê" é o que
permite adaptar a biblioteca para situações que não estão aqui.

---

## Visão geral da arquitetura

```
main.cpp  (você escreve)
    │
    ├── LineSensor         → lê a linha
    ├── MotorController    → controla os motores
    ├── UltrasonicSensor   → lê a distância
    └── GripperServo       → controla a garra
            │
            └── config.h   → todas as constantes de calibração
```

Cada módulo é independente — você pode usar um sem os outros.
Todos dependem apenas do `config.h`.

---

## LineSensor

### O que faz

Lê 6 sensores QTR analógicos e calcula a posição da linha em relação ao
centro do robô. Retorna um valor float de -1.0 (extrema esquerda) a +1.0
(extrema direita).

### Decisão técnica: por que centro de massa ponderado, não contagem simples?

A abordagem mais simples seria: "se o sensor da esquerda está ativo, vira
esquerda". Isso é frágil — qualquer ruído de leitura ou variação de
iluminação muda drasticamente o comportamento, e o robô fica indeciso
nas bordas da linha.

A solução implementada usa **centro de massa ponderado por intensidade**:

```cpp
intensity = THRESHOLD - raw         // quanto mais "branco", maior a intensidade
peso[6]   = {-5, -3, -1, 1, 3, 5}   // simétrico em relação ao centro

posicao = soma(peso[i] × intensidade[i]) / soma(intensidade) / 5.0
```

Isso produz uma leitura **contínua**, não binária. O robô sabe não só
*que lado* a linha está, mas *quão longe* — permitindo correção
proporcional ao desvio, que é mais suave.

### Decisão técnica: por que sem filtro de leitura (debounce)?

Filtrar a leitura (média de várias amostras) reduziria ruído, mas introduz
atraso. Em um seguidor de linha, atraso de poucos milissegundos já é
suficiente para o robô sair da pista numa curva rápida. A prioridade aqui
é reatividade máxima — o controle de ruído fica a cargo do PID em
`MotorController`, que tem seu próprio amortecimento (termo derivativo).

### Métodos principais

```cpp
lineSensor.initialize();                                  // chamar uma vez no setup()
lineSensor.readSensors();                                 // chamar uma vez por ciclo do loop
float pos = lineSensor.getLinePosition();                 // -1.0 a +1.0
LineSensor::LinePattern p = lineSensor.getLinePattern();  // classificação
```

`LinePattern` classifica a leitura em: `STRAIGHT`, `CURVE_LIGHT`,
`CURVE_MEDIUM`, `CURVE_SHARP`, `INTERSECTION` (cruzamento — 5+ sensores
ativos) ou `LINE_LOST` (nenhum sensor ativo).

---

## MotorController

### O que faz

Controla dois motores DC via driver L298N. Implementa controlador PID de
seguimento de linha e três camadas de proteção física invisíveis para
quem usa a classe.

### Decisão técnica: trim individual por motor

Motores DC de baixo custo têm variação de fabricação — o mesmo PWM produz
velocidades levemente diferentes em motores "idênticos". Sem correção, o
robô puxa para um lado mesmo andando "reto".

```cpp
MOTOR_TRIM_ESQ  // fator multiplicativo do motor esquerdo (0.80–1.20)
MOTOR_TRIM_DIR  // fator multiplicativo do motor direito  (0.80–1.20)
```

Esses fatores são aplicados *depois* de qualquer cálculo de velocidade —
o resto do código nunca precisa saber que o trim existe.

### Decisão técnica: rampas de proteção do motor

O motor usado (Micro Motor Robocore 6V) tem corrente de stall (eixo
travado) de **1.6A**. Inverter o sentido do motor instantaneamente
(de +180 PWM para -180 PWM em um único comando) empurra a corrente
instantaneamente em direção a esse valor de stall — o pico mecânico
resultante já causou dano real de engrenagem durante o desenvolvimento.

A solução, implementada em `_applyRamp()`:

- **Arranque** (motor parado → em movimento): sobe em passos pequenos
- **Inversão de sentido**: desce até zero antes de inverter — nunca
  inverte a polaridade diretamente
- **Ajuste na mesma direção** (ex: PID corrigindo uma curva): rampa
  *proporcional* ao tamanho do ajuste — microajustes do PID chegam quase
  instantaneamente, ajustes grandes são suavizados

Essa terceira regra é importante: sem ela, o robô responderia "truncado"
em curvas, porque toda correção do PID levaria o mesmo tempo de rampa
mesmo sendo pequena.

### Decisão técnica: controlador PID, não apenas PD

PID = Proporcional + Integral + Derivativo.

- **P** (`PD_KP`): corrige o erro atual — quanto mais desviado da linha, mais corrige
- **D** (`PD_KD`): amortece oscilação — reage à *velocidade* de mudança do erro
- **I** (`PD_KI`): corrige deriva acumulada — sem isso, o robô pode ficar
  consistentemente um pouco torto em retas longas sem nunca convergir ao centro

O termo integral tem proteção anti-windup: dentro de uma zona morta
(`PID_INTEGRAL_DEADZONE`), o acumulador é zerado — ruído de leitura em
reta não deve virar correção acumulada.

### Métodos principais

```cpp
motor.initialize();
motor.move(MotorController::FORWARD, velocidade);   // 0–255
motor.stop();
motor.followLine(posicao, velocidadeBase);          // chamar a cada ciclo do loop
motor.resetPD();                                    // chamar sempre após parar ou fazer uma manobra
```

`Direction` disponíveis: `FORWARD`, `BACKWARD`, `TURN_LEFT`, `TURN_RIGHT`
(giro no eixo — dois motores em sentidos opostos), `CURVE_LEFT`,
`CURVE_RIGHT` (curva com apenas um motor ativo, raio maior).

---

## UltrasonicSensor

### O que faz

Lê o sensor HC-SR04 e valida a leitura antes de considerá-la confiável.

### Decisão técnica: por que validar antes de confiar na leitura?

Sensores ultrassônicos sofrem com reflexos em ângulos oblíquos e ruído
elétrico — uma leitura isolada pode ser completamente espúria. Confiar
direto na primeira leitura poderia disparar a coleta no momento errado.

A validação implementada usa **janela deslizante com degradação gradual**:

- Cada leitura dentro da tolerância da anterior incrementa um contador
- Após `SENSOR_FILTER_CYCLES` leituras consistentes, a leitura é
  considerada estável
- Uma leitura fora da tolerância reduz o contador *gradualmente*
  (não zera de uma vez) — isso evita "flicker" quando o objeto está
  bem na borda da tolerância

A tolerância também é adaptativa: ±2cm fixo até 20cm de distância,
percentual com teto de ±4cm acima disso — o ruído do sensor cresce
proporcionalmente com a distância.

### Métodos principais

```cpp
ultrasonic.initialize();
int dist = ultrasonic.readDistance();         // retorna -1 se não confiável
bool ok  = ultrasonic.isReadingStable();      // true após validação completa
ultrasonic.resetValidation();                 // chamar após cada coleta concluída
```

---

## GripperServo

### O que faz

Controla a garra via servo SG90 — abre e fecha com movimento suave.

### Decisão técnica: movimento grau a grau, não direto

Mover o servo diretamente para o ângulo final causa tranco mecânico —
pode soltar o objeto da garra ou estressar a engrenagem do servo. O
código move um grau por vez (`SERVO_STEP_DELAY_MS` entre cada passo),
distribuindo a carga ao longo do tempo.

### Decisão técnica: desligar o sinal PWM após posicionar (`detach()`)

Servos SG90 recebem PWM contínuo para manter posição — mas isso causa
aquecimento e vibração quando não há carga real puxando o servo para
fora da posição. Após o movimento terminar e aguardar estabilização
(`SERVO_STABILIZATION_TIME`), o sinal é desligado via `detach()`. A
posição mecânica é mantida pela própria engrenagem do servo.

### Bug histórico corrigido: underflow de `uint8_t`

A primeira versão usava `uint8_t` para a variável de iteração do
movimento grau a grau. Ao decrementar de 0, o valor "voltava" para 255
(comportamento de overflow do tipo), causando um loop infinito que
parecia o servo "vibrando sem parar". A correção usa uma variável `int`
local para a iteração — `uint8_t` só armazena o ângulo final.

### Métodos principais

```cpp
gripper.initialize();    // sempre força abertura física, independente do estado anterior
gripper.open();
gripper.close();
bool fechada = gripper.isClosed();
```

---

## config.h — única fonte de configuração

Todas as constantes de calibração (pinagem, threshold, PID, trim, ângulos
do servo, zonas de distância, tempos de manobra) ficam neste arquivo.
Nenhum módulo tem valores "hardcoded" — alterar `config.h` recalibra o
sistema sem tocar em nenhum `.cpp`.
