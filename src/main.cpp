// ============================================================================
// main.cpp — Seguidor de Linha + Coletor Autônomo
//
// Máquina de quatro estados:
//   STATE_FOLLOWING  → segue linha com PID, detecta objeto
//   STATE_COLLECTING → coleta e manobra (bloqueante, lado fixo)
//   STATE_RECOVERING → busca linha perdida em dois estágios
//   STATE_STOPPED    → aguarda intervenção manual
//
// Velocidades: derivadas de BASE_SPEED (config.h §3)
// Trim:        MOTOR_TRIM_ESQ / MOTOR_TRIM_DIR (config.h §2)
// PID:         PD_KP / PD_KD / PD_KI (config.h §5)
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "LineSensor.h"
#include "MotorController.h"
#include "UltrasonicSensor.h"
#include "GripperServo.h"

// ── Estados ───────────────────────────────────────────────────────────────────
enum RobotState {
    STATE_FOLLOWING,
    STATE_COLLECTING,
    STATE_RECOVERING,
    STATE_STOPPED
};

// ── Instâncias globais ────────────────────────────────────────────────────────
LineSensor       lineSensor;
MotorController  motor;
UltrasonicSensor ultrasonic;
GripperServo     gripper;

// ── Estado da máquina ─────────────────────────────────────────────────────────
RobotState    robotState    = STATE_FOLLOWING;
unsigned long recoveryStart = 0;
unsigned long programStart  = 0;
int           collectCount  = 0;

// ── Protótipos ────────────────────────────────────────────────────────────────
void handleFollowing();
void handleCollecting();
void handleRecovering();
void logTransition(const __FlashStringHelper* from, const __FlashStringHelper* to);
void printFollowStatus(float pos, uint8_t spd, LineSensor::LinePattern pat);

// ============================================================================
// setup()
// ============================================================================
void setup() {
    Serial.begin(BAUD_RATE);
    delay(500);

    lineSensor.initialize();
    motor.initialize();
    ultrasonic.initialize();
    gripper.initialize();   // força abertura física independente do estado anterior

    motor.stop();

    Serial.println(F("\n╔══════════════════════════════════════════╗"));
    Serial.println(F("║     SEGUIDOR COLETOR — PROGRAMA FINAL    ║"));
    Serial.println(F("╠══════════════════════════════════════════╣"));
    Serial.print  (F("║  BASE="));      Serial.print(BASE_SPEED);
    Serial.print  (F("  Kp="));         Serial.print(PD_KP, 2);
    Serial.print  (F("  Kd="));         Serial.print(PD_KD, 2);
    Serial.print  (F("  Ki="));         Serial.println(PD_KI, 3);
    Serial.print  (F("║  TrimESQ="));   Serial.print(MOTOR_TRIM_ESQ, 2);
    Serial.print  (F("  TrimDIR="));    Serial.println(MOTOR_TRIM_DIR, 2);
    Serial.print  (F("║  Threshold=")); Serial.print(THRESHOLD_LINE_SENSOR);
    Serial.print  (F("  Contato="));    Serial.print(ULTRASONIC_DISTANCE_CONTACT);
    Serial.println(F("cm"));
    Serial.println(F("╠══════════════════════════════════════════╣"));
    Serial.println(F("║  Posicione o robo sobre a linha.         ║"));
    Serial.println(F("║  Iniciando em 3 segundos...              ║"));
    Serial.println(F("╚══════════════════════════════════════════╝"));

    delay(3000);

    programStart = millis();
    motor.resetPD();
    lineSensor.resetLastDirection();
    robotState = STATE_FOLLOWING;

    Serial.println(F("[Main] INICIADO"));
}

// ============================================================================
// loop()
// ============================================================================
void loop() {
    switch (robotState) {
        case STATE_FOLLOWING:  handleFollowing();  break;
        case STATE_COLLECTING: handleCollecting(); break;
        case STATE_RECOVERING: handleRecovering(); break;
        case STATE_STOPPED:    motor.stop();       break;
    }
}

// ============================================================================
// handleFollowing()
//
// Prioridade por ciclo:
//   1. Objeto na zona de contato → STATE_COLLECTING
//   2. Linha perdida             → STATE_RECOVERING
//   3. Cruzamento central do 8   → passa reto em SPEED_INTERSECTION
//   4. Normal                    → followLine() com velocidade por fase
// ============================================================================
void handleFollowing() {
    lineSensor.readSensors();
    float                   pos     = lineSensor.getLinePosition();
    LineSensor::LinePattern pattern = lineSensor.getLinePattern();

    int dist = ultrasonic.readDistance();
    UltrasonicSensor::ApproachPhase fase = ultrasonic.getApproachPhase();

    // 1. Objeto na zona de contato
    if (ultrasonic.isReadingStable() && dist > 0 && dist <= ULTRASONIC_DISTANCE_CONTACT) {
        motor.stop();
        logTransition(F("FOLLOWING"), F("COLLECTING"));
        robotState = STATE_COLLECTING;
        return;
    }

    // 2. Linha perdida
    if (pattern == LineSensor::LINE_LOST) {
        motor.stop();
        recoveryStart = millis();
        logTransition(F("FOLLOWING"), F("RECOVERING"));
        robotState = STATE_RECOVERING;
        return;
    }

    // 3. Cruzamento central do 8
    if (pattern == LineSensor::INTERSECTION) {
        motor.move(MotorController::FORWARD, SPEED_INTERSECTION);
        delay(PD_SAMPLE_MS);
        return;
    }

    // 4. Velocidade por fase do ultrassônico
    // PHASE_4_CONTACT_ZONE normalmente não chega aqui — o gatilho de coleta
    // (item 1 acima) já desvia o fluxo antes. Tratada explicitamente por
    // segurança, usando a mesma velocidade mínima de PHASE_3_CONTACT.
    uint8_t baseSpeed;
    switch (fase) {
        case UltrasonicSensor::PHASE_2_APPROACHING:  baseSpeed = APPROACH_SPEED_MEDIUM; break;
        case UltrasonicSensor::PHASE_3_CONTACT:      baseSpeed = APPROACH_SPEED_SLOW;   break;
        case UltrasonicSensor::PHASE_4_CONTACT_ZONE: baseSpeed = APPROACH_SPEED_SLOW;   break;
        default:                                       baseSpeed = BASE_SPEED;            break;
    }

    motor.followLine(pos, baseSpeed);
    printFollowStatus(pos, baseSpeed, pattern);
    delay(PD_SAMPLE_MS);
}

// ============================================================================
// handleCollecting()
//
// Sequência bloqueante — sensores ignorados durante toda a manobra.
// Lado fixo: sempre manobra para ESQUERDA.
// Ajuste TURN_90_LOADED_MS / STRAFE_LOADED_MS / STRAFE_UNLOADED_MS
// e TURN_90_UNLOADED_MS no config.h para calibrar o retorno ao eixo.
// ============================================================================
void handleCollecting() {
    collectCount++;
    Serial.print(F("\n[Coleta #")); Serial.println(collectCount);

    // Fecha garra e aguarda estabilização mecânica
    gripper.close();
    delay(COLLECT_STOP_DELAY);

    // COM CARGA — gira esquerda e avança lateralmente
    motor.move(MotorController::TURN_LEFT, MANEUVER_SPEED_LOADED);
    delay(TURN_90_LOADED_MS);
    motor.stop(); delay(50);

    motor.move(MotorController::FORWARD, MANEUVER_SPEED_LOADED);
    delay(STRAFE_LOADED_MS);
    motor.stop(); delay(100);

    // Solta o objeto
    gripper.open(); delay(200);

    // SEM CARGA — recua e gira de volta ao eixo original
    motor.move(MotorController::BACKWARD, MANEUVER_SPEED_UNLOADED);
    delay(STRAFE_UNLOADED_MS);
    motor.stop(); delay(50);

    motor.move(MotorController::TURN_RIGHT, MANEUVER_SPEED_UNLOADED);
    delay(TURN_90_UNLOADED_MS);
    motor.stop();

    // Prepara próximo ciclo
    ultrasonic.resetValidation();
    motor.resetPD();
    lineSensor.resetLastDirection();
    delay(300);

    logTransition(F("COLLECTING"), F("FOLLOWING"));
    robotState = STATE_FOLLOWING;
}

// ============================================================================
// handleRecovering()
//
// Estágio 1 (0 → RECOVERY_SPIN_MS):   gira na última direção conhecida
// Estágio 2 (→ RECOVERY_TIMEOUT_MS):  gira na direção oposta
// Timeout: STATE_STOPPED
// ============================================================================
void handleRecovering() {
    unsigned long elapsed = millis() - recoveryStart;

    lineSensor.readSensors();
    if (lineSensor.getLinePattern() != LineSensor::LINE_LOST) {
        motor.stop();
        motor.resetPD();
        lineSensor.resetLastDirection();
        logTransition(F("RECOVERING"), F("FOLLOWING"));
        robotState = STATE_FOLLOWING;
        return;
    }

    bool lastWasRight = (lineSensor.getLastDirection() == LineSensor::DIR_RIGHT);

    if (elapsed < RECOVERY_SPIN_MS) {
        motor.move(lastWasRight ? MotorController::TURN_RIGHT
                                : MotorController::TURN_LEFT, PWM_SLOW);
        return;
    }

    if (elapsed < RECOVERY_TIMEOUT_MS) {
        motor.move(lastWasRight ? MotorController::TURN_LEFT
                                : MotorController::TURN_RIGHT, PWM_SLOW);
        return;
    }

    motor.stop();
    logTransition(F("RECOVERING"), F("STOPPED"));
    Serial.println(F("[Main] TIMEOUT — reposicione e reinicie"));
    robotState = STATE_STOPPED;
}

// ============================================================================
// logTransition()
// ============================================================================
void logTransition(const __FlashStringHelper* from, const __FlashStringHelper* to) {
    if (!DEBUG_MODE) return;
    Serial.print(F("["));
    Serial.print((millis() - programStart) / 1000);
    Serial.print(F("s] "));
    Serial.print(from); Serial.print(F(" -> ")); Serial.println(to);
}

// ============================================================================
// printFollowStatus() — 3 Hz
// ============================================================================
void printFollowStatus(float pos, uint8_t spd, LineSensor::LinePattern pat) {
    if (!DEBUG_MODE) return;
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 300) return;
    lastPrint = millis();

    const __FlashStringHelper* label;
    switch (pat) {
        case LineSensor::STRAIGHT:     label = F("RETA");    break;
        case LineSensor::CURVE_LIGHT:  label = F("C-SUAVE"); break;
        case LineSensor::CURVE_MEDIUM: label = F("C-MEDIA"); break;
        case LineSensor::CURVE_SHARP:  label = F("C-AGUDA"); break;
        default:                       label = F("?");       break;
    }

    char bar[22];
    int idx = constrain((int)((pos + 1.0f) * 10.0f), 0, 20);
    for (int i = 0; i < 21; i++) bar[i] = (i == 10) ? '|' : '-';
    bar[idx] = '#'; bar[21] = '\0';

    Serial.print(F("["));      Serial.print(bar);  Serial.print(F("] "));
    Serial.print(pos, 2);
    Serial.print(F(" | "));    Serial.print(label);
    Serial.print(F(" | spd=")); Serial.print(spd);
    Serial.print(F(" | dist=")); Serial.print(ultrasonic.getLastValidDistance());
    Serial.println(F("cm"));
}
