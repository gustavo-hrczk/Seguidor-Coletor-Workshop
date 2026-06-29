#define CONFIG_H

// ============================================================================
// config.h — Configuração central do Robô Seguidor Coletor
//
// Arquivo único de configuração. Alterar um valor aqui afeta todos os
// módulos que o consomem. Consulte CONFIG_INDEX.md para referência completa.
//
// Seções:
//   1. Pinos de hardware
//   2. Calibração dos motores
//   3. Velocidade base
//   4. Sensor de linha
//   5. Controlador PID
//   6. Recuperação de linha
//   7. Detecção de cruzamentos
//   8. Sensor ultrassônico
//   9. Garra (servo)
//  10. Coleta autônoma
//  11. Sistema
// ============================================================================


// ============================================================================
// 1. PINOS DE HARDWARE
// ============================================================================

// --- Driver de motor L298N ---
// PIN_IN1/IN2: controlam sentido do motor ligado ao canal ENB
// PIN_IN3/IN4: controlam sentido do motor ligado ao canal ENA
#define PIN_IN1 5
#define PIN_IN2 7
#define PIN_IN3 2
#define PIN_IN4 4
#define PIN_ENA 6    // PWM canal A — Timer 0
#define PIN_ENB 3    // PWM canal B — Timer 2

// --- Sensores de linha QTR-1 analógico × 6 ---
// Disposição: S1 = extrema esquerda … S6 = extrema direita
// Pesos do centro de massa ponderado: S1=-5 S2=-3 S3=-1 S4=+1 S5=+3 S6=+5
#define PIN_S1 A0
#define PIN_S2 A1
#define PIN_S3 A2
#define PIN_S4 A3
#define PIN_S5 A4
#define PIN_S6 A5

// --- Sensor ultrassônico HC-SR04 ---
#define PIN_TRIGGER 12
#define PIN_ECHO    13

// --- Servo SG90 ---
#define PIN_SERVO 9


// ============================================================================
// 2. CALIBRAÇÃO DOS MOTORES
//
// Motores TT apresentam assimetria física: mesmo recebendo o mesmo PWM,
// giram em velocidades ligeiramente diferentes, causando desvio em linha reta.
//
// Solução: fator de trim multiplicativo individual por motor.
//   1.00 = neutro (sem correção)
//   > 1.0 = aumenta a velocidade efetiva deste motor
//   < 1.0 = reduz a velocidade efetiva deste motor
//
// Perspectiva de referência: olhando o robô de cima com a frente
// apontando para longe. ESQ = lado esquerdo, DIR = lado direito.
//
// Como calibrar:
//   1. Superfície plana, sem linha, ~1 metro de distância
//   2. Ajuste um fator de cada vez em passos de 0.02
//   3. Robô desvia para a direita → motor DIR está mais rápido
//      → reduza MOTOR_TRIM_DIR ou aumente MOTOR_TRIM_ESQ
//   4. Registre os valores validados abaixo
// ============================================================================

#define MOTOR_TRIM_DIR  1.00f   // fator de correção motor DIREITO  (0.80–1.20)
#define MOTOR_TRIM_ESQ  1.00f   // fator de correção motor ESQUERDO (0.80–1.20)


// ============================================================================
// 2b. PROTEÇÃO DOS MOTORES — RAMPAS DE ACELERAÇÃO
//
// Motor: Micro Motor com Caixa de Redução 6V — Robocore
//   Corrente nominal: ~200 mA com carga
//   Corrente de stall (eixo travado): 1.6 A
//
// Problema: inversão brusca de sentido (ex: +180 -> -180 PWM em um ciclo)
// empurra a corrente instantaneamente em direção ao stall, gerando pico
// mecânico e elétrico que danifica as engrenagens da caixa de redução.
// Esse pico já causou falha real de engrenagem durante testes de bancada.
//
// Solução — duas rampas implementadas internamente em setMotorSpeed():
//
//   Rampa de ARRANQUE (0 -> alvo):
//     Sobe em MOTOR_RAMP_STEPS degraus ao longo de MOTOR_RAMP_UP_MS ms.
//     Protege arranque a frio e retomada após qualquer parada.
//
//   Rampa de INVERSÃO (positivo -> negativo ou vice-versa):
//     Desce até zero em MOTOR_RAMP_STEPS degraus (MOTOR_RAMP_DOWN_MS ms),
//     aguarda um ciclo em zero, depois sobe para o novo sentido.
//     Elimina o pico de corrente reversa que atinge o stall.
//
// Calibração para caixa de redução metálica 6V Robocore:
//   60 ms subida | 40 ms descida | 8 degraus por rampa
//   Degrau: ~7.5 ms na subida | ~5 ms na descida
//   Rápido o suficiente para o PID responder (PD_SAMPLE_MS = 10 ms),
//   seguro o suficiente para não atingir stall nas inversões.
// ============================================================================

#define MOTOR_RAMP_UP_MS    60   // ms da rampa de arranque (0 -> velocidade alvo)
#define MOTOR_RAMP_DOWN_MS  40   // ms da rampa antes de inversão de sentido
#define MOTOR_RAMP_STEPS     8   // número de degraus por rampa


// ============================================================================
// 3. VELOCIDADE BASE
//
// BASE_SPEED é a única constante de velocidade a editar diretamente.
// Todas as demais são derivadas automaticamente via offsets relativos,
// garantindo que o sistema inteiro se recalibre ao mudar um único valor.
//
// Offsets: positivo = mais rápido | negativo = mais lento
// ============================================================================

#define BASE_SPEED  200   // PWM base — fonte única da verdade de velocidade

// Derivadas diretas — NÃO editar
#define VELOCITY_GLOBAL  BASE_SPEED

// Offsets situacionais relativos ao BASE_SPEED
#define SPEED_OFFSET_RECOVERY     -10   // giro de recuperação de linha
#define SPEED_OFFSET_INTERSECTION -20   // passagem pelo cruzamento do 8
#define SPEED_OFFSET_APPROACH_MED -40   // objeto detectado entre LONG e SHORT
#define SPEED_OFFSET_APPROACH_SLW -50   // objeto entre SHORT e CONTACT

// Derivadas com offset — NÃO editar
#define PWM_SLOW               (BASE_SPEED + SPEED_OFFSET_RECOVERY)
#define SPEED_INTERSECTION     (BASE_SPEED + SPEED_OFFSET_INTERSECTION)
#define APPROACH_SPEED_MEDIUM  (BASE_SPEED + SPEED_OFFSET_APPROACH_MED)
#define APPROACH_SPEED_SLOW    (BASE_SPEED + SPEED_OFFSET_APPROACH_SLW)
#define APPROACH_SPEED_FAST    BASE_SPEED

// PWM mínimo para vencer atrito estático dos motores TT.
// Abaixo deste valor o motor recebe sinal mas não gira com carga.
// Independente do BASE_SPEED — calibrado fisicamente.
#define PWM_MIN_DEADZONE  140


// ============================================================================
// 4. SENSOR DE LINHA
//
// Convenção (QTR analógico, linha BRANCA sobre fundo PRETO):
//   Linha branca → analogRead BAIXO (~50–300)
//   Fundo preto  → analogRead ALTO  (~700–1023)
//   Sensor ATIVO quando raw <= THRESHOLD_LINE_SENSOR
//
// Calibrar com o Teste de Sensor (test_components) na pista real.
// Valor recomendado = ponto médio entre max_linha e min_fundo.
// ============================================================================

#define THRESHOLD_LINE_SENSOR  828


// ============================================================================
// 5. CONTROLADOR PID
//
// Fórmula: correction = Kp*erro + Kd*(erro - erro_anterior) + Ki*∫erro·dt
//
// Motor externo à curva: BASE_SPEED constante (preserva velocidade de avanço)
// Motor interno à curva: BASE_SPEED × (1 - |correction|), piso PD_MIN_INNER_SPEED
//
// Ajuste dos ganhos:
//   Kp: reatividade à posição — aumentar = reage mais rápido
//   Kd: amortecimento — aumentar = menos serpenteia em reta
//   Ki: corrige deriva acumulada — começar com 0.02, aumentar até 0.08
//       muito alto = oscila após curvas (windup)
//
// Anti-windup do integral:
//   PID_INTEGRAL_DEADZONE: dentro de ±valor, integral é zerado (ruído ignorado)
//   PID_INTEGRAL_MAX:      teto absoluto do acumulador
// ============================================================================

#define PD_KP  0.8f
#define PD_KD  0.4f
#define PD_KI  0.04f
#define PD_SAMPLE_MS          10     // intervalo entre cálculos (ms) — 100 Hz

#define PD_MIN_INNER_SPEED    80     // piso do motor interno em curvas
#define PID_INTEGRAL_DEADZONE 0.10f  // zona morta do integral (±10% da escala)
#define PID_INTEGRAL_MAX      1.0f   // teto anti-windup do acumulador


// ============================================================================
// 6. RECUPERAÇÃO DE LINHA PERDIDA
//
// Estratégia em dois estágios:
//   Estágio 1 (0 → RECOVERY_SPIN_MS):   gira na última direção conhecida
//   Estágio 2 (→ RECOVERY_TIMEOUT_MS):  gira na direção oposta
//   Timeout:                             STATE_STOPPED
// ============================================================================

#define RECOVERY_SPIN_MS     300
#define RECOVERY_TIMEOUT_MS 1500


// ============================================================================
// 7. DETECÇÃO DE CRUZAMENTOS
//
// Percurso em formato de 8 tem um cruzamento central.
// Quando >= CROSS_MIN_SENSORS_X sensores estão ativos simultaneamente,
// o robô está sobre o cruzamento e deve passar reto.
// ============================================================================

#define CROSS_MIN_SENSORS_X  5


// ============================================================================
// 8. SENSOR ULTRASSÔNICO HC-SR04
//
// Fases de aproximação (4 níveis — ver UltrasonicSensor::ApproachPhase):
//   OBJECT_NOT_DETECTED  : sem leitura estável
//   PHASE_1_DISTANT      : dist >= ULTRASONIC_DISTANCE_LONG     → velocidade normal
//   PHASE_2_APPROACHING  : dist >= ULTRASONIC_DISTANCE_SHORT    → desacelera
//   PHASE_3_CONTACT      : dist >= ULTRASONIC_DISTANCE_CONTACT  → quase parado
//   PHASE_4_CONTACT_ZONE : dist <  ULTRASONIC_DISTANCE_CONTACT  → gatilho de coleta
//
// O gatilho de STATE_COLLECTING usa ULTRASONIC_DISTANCE_CONTACT diretamente
// (equivalente a estar em PHASE_4_CONTACT_ZONE).
//
// ULTRASONIC_TIMEOUT_US: timeout do pulseIn em µs.
//   10000µs cobre até ~170cm — suficiente para o projeto.
//   Reduzir melhora responsividade a custo de alcance máximo.
// ============================================================================

#define ULTRASONIC_DISTANCE_LONG     25   // cm — início da fase 2
#define ULTRASONIC_DISTANCE_SHORT    10   // cm — início da fase 3
#define ULTRASONIC_DISTANCE_CONTACT   4   // cm — gatilho de coleta
#define ULTRASONIC_NOISE_TOLERANCE   20   // % de variação tolerada entre leituras
#define ULTRASONIC_TIMEOUT_US     10000   // µs — timeout do pulseIn
#define SENSOR_FILTER_CYCLES          2   // leituras consecutivas para validar


// ============================================================================
// 9. GARRA (SG90)
//
// O servo move grau a grau (SERVO_STEP_DELAY_MS entre cada passo) para
// evitar tranco mecânico. Após atingir o ângulo alvo, aguarda
// SERVO_STABILIZATION_TIME e executa detach() — elimina aquecimento
// causado por PWM contínuo sem carga, problema recorrente no SG90.
//
// Ângulos: evitar 0° — é o limite mecânico do SG90 e pode travar.
// ============================================================================

#define SERVO_ANGLE_OPEN          20   // graus — posição aberta
#define SERVO_ANGLE_CLOSED       170   // graus — posição fechada
#define SERVO_STEP_DELAY_MS       10   // ms entre cada grau (menor = mais rápido)
#define SERVO_STABILIZATION_TIME 400   // ms aguardando antes do detach()


// ============================================================================
// 10. COLETA AUTÔNOMA
//
// Parâmetros do ciclo STATE_COLLECTING.
// Tempos de manobra: calibrar na pista com o peso real do robô.
// ============================================================================

#define MANEUVER_SPEED_LOADED    180   // PWM com objeto na garra
#define MANEUVER_SPEED_UNLOADED  160   // PWM no retorno sem objeto

#define STRAFE_LOADED_MS         800   // ms de avanço lateral com objeto
#define STRAFE_UNLOADED_MS       650   // ms de recuo lateral sem objeto
#define TURN_90_LOADED_MS        900   // ms de giro 90° com carga
#define TURN_90_UNLOADED_MS      600   // ms de giro 90° sem carga
#define COLLECT_STOP_DELAY       300   // ms de pausa entre etapas da manobra


// ============================================================================
// 11. SISTEMA
// ============================================================================

#define BAUD_RATE   9600

// DEBUG_MODE true  → todos os módulos imprimem no Serial
// DEBUG_MODE false → código de log removido em compilação (libera memória Flash)
#define DEBUG_MODE  true
