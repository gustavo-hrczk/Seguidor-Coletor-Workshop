#include "GripperServo.h"

static const uint16_t DETACH_DELAY_MS = SERVO_STABILIZATION_TIME;

// ============================================================================
// Construtor
// ============================================================================
GripperServo::GripperServo()
    : _state(OPEN), _currentAngle(SERVO_ANGLE_OPEN) {}

// ============================================================================
// initialize()
// Força o movimento físico para ABERTO independente do estado anterior.
//
// Problema resolvido: se o servo estava fisicamente fechado antes do upload
// (posição retida mecanicamente), attach() + write() sem movimento real
// deixava a garra fechada. A solução é ignorar o estado lógico e sempre
// executar o movimento completo grau a grau até SERVO_ANGLE_OPEN.
//
// _state é forçado para CLOSED antes de chamar open() para garantir que
// a guard "if (_state == OPEN) return" não bloqueie o movimento.
// ============================================================================
void GripperServo::initialize() {
    _servo.attach(PIN_SERVO);

    // Força estado CLOSED para que open() não seja bloqueado pela guard
    _state        = CLOSED;
    _currentAngle = SERVO_ANGLE_CLOSED;   // assume pior caso: garra fechada

    // Informa posição atual ao driver antes de mover
    _servo.write(_currentAngle);
    delay(100);

    // Abre fisicamente — percorre o curso completo grau a grau
    open();

    if (DEBUG_MODE) Serial.println(F("[Gripper] Inicializado - ABERTO"));
}

// ============================================================================
// open() / close()
// ============================================================================
void GripperServo::open() {
    if (_state == OPEN) return;
    moveToAngle(SERVO_ANGLE_OPEN);
    _state = OPEN;
    if (DEBUG_MODE) Serial.println(F("[Gripper] ABERTO"));
}

void GripperServo::close() {
    if (_state == CLOSED) return;
    moveToAngle(SERVO_ANGLE_CLOSED);
    _state = CLOSED;
    if (DEBUG_MODE) Serial.println(F("[Gripper] FECHADO"));
}

// ============================================================================
// emergencyStop()
// ============================================================================
void GripperServo::emergencyStop() {
    if (_servo.attached()) _servo.detach();
    _state = ERROR;
    if (DEBUG_MODE) Serial.println(F("[Gripper] EMERGENCY STOP"));
}

// ============================================================================
// moveToAngle()
// Move grau a grau. Correção de underflow via variável local int.
// ============================================================================
void GripperServo::moveToAngle(uint8_t target) {
    target = constrain(target, SERVO_ANGLE_OPEN, SERVO_ANGLE_CLOSED);

    if (!_servo.attached()) {
        _servo.attach(PIN_SERVO);
        _servo.write(_currentAngle);
        delay(50);
    }

    int current = (int)_currentAngle;
    int tgt     = (int)target;
    int step    = (current < tgt) ? 1 : -1;

    while (current != tgt) {
        current += step;
        _servo.write(current);
        delay(SERVO_STEP_DELAY_MS);
    }

    _currentAngle = (uint8_t)current;
    delay(DETACH_DELAY_MS);
    _servo.detach();
}
