#include "MotorController.h"

// ============================================================================
// Construtor
// ============================================================================
MotorController::MotorController()
    : _currentLeft(0), _currentRight(0),
      _appliedLeft(0),  _appliedRight(0),
      _prevError(0.0f), _integral(0.0f), _lastPDTime(0) {}

// ============================================================================
// initialize()
// ============================================================================
void MotorController::initialize() {
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
    stop();
}

// ============================================================================
// applyTrimLeft() / applyTrimRight()
// ============================================================================
int MotorController::applyTrimLeft(int speed) const {
    if (speed == 0) return 0;
    int v = (int)(speed * MOTOR_TRIM_ESQ);
    v = constrain(v, -255, 255);
    if      (v > 0 && v <  PWM_MIN_DEADZONE) v =  PWM_MIN_DEADZONE;
    else if (v < 0 && v > -PWM_MIN_DEADZONE) v = -PWM_MIN_DEADZONE;
    return v;
}

int MotorController::applyTrimRight(int speed) const {
    if (speed == 0) return 0;
    int v = (int)(speed * MOTOR_TRIM_DIR);
    v = constrain(v, -255, 255);
    if      (v > 0 && v <  PWM_MIN_DEADZONE) v =  PWM_MIN_DEADZONE;
    else if (v < 0 && v > -PWM_MIN_DEADZONE) v = -PWM_MIN_DEADZONE;
    return v;
}

// ============================================================================
// _writePins()
// Envia velocidade diretamente ao hardware — sem rampa, sem trim.
// ============================================================================
void MotorController::_writePins(int in1, int in2, int pwmPin, int speed) {
    int pwm = constrain(abs(speed), 0, 255);
    if (speed == 0) {
        digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  analogWrite(pwmPin, 0);
    } else if (speed > 0) {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(pwmPin, pwm);
    } else {
        digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); analogWrite(pwmPin, pwm);
    }
}

// ============================================================================
// _applyRamp()
// Rampa de proteção para o Micro Motor Robocore 6V (stall 1.6A).
//
// Três cenários protegidos:
//
//   ARRANQUE (applied == 0, target != 0):
//     Sobe em MOTOR_RAMP_STEPS passos ao longo de MOTOR_RAMP_UP_MS.
//     Protege contra pico de corrente no arranque a frio.
//
//   INVERSÃO (applied e target com sinais opostos):
//     Desce até zero em MOTOR_RAMP_DOWN_MS, pausa um ciclo, sobe para target.
//     O motor nunca inverte polaridade sem passar por zero.
//
//   AJUSTE NA MESMA DIREÇÃO (sem inversão, sem arranque):
//     Rampa PROPORCIONAL ao delta entre applied e target.
//     Quanto menor a diferença, menor o delay — microajustes do PID
//     em curvas chegam ao hardware quase instantaneamente.
//     Isso resolve o truncamento em curvas mantendo a proteção de inversão.
//
//   PARADA (target == 0):
//     Direto — não há inversão de polaridade.
// ============================================================================
void MotorController::_applyRamp(int pin1, int pin2, int pwmPin,
                                  int target, int& applied) {
    // Parada — direto, sem rampa
    if (target == 0) {
        _writePins(pin1, pin2, pwmPin, 0);
        applied = 0;
        return;
    }

    bool arranque = (applied == 0);
    bool inversao = (applied != 0) &&
                    ((applied > 0 && target < 0) || (applied < 0 && target > 0));

    // ── Inversão: desce até zero primeiro ────────────────────────────────
    if (inversao) {
        int stepDelay = MOTOR_RAMP_DOWN_MS / MOTOR_RAMP_STEPS;
        int stepSize  = applied / MOTOR_RAMP_STEPS;

        for (int i = MOTOR_RAMP_STEPS - 1; i >= 0; i--) {
            int mid = stepSize * i;
            if (mid != 0 && abs(mid) < PWM_MIN_DEADZONE) mid = 0;
            _writePins(pin1, pin2, pwmPin, mid);
            delay(stepDelay);
        }
        _writePins(pin1, pin2, pwmPin, 0);
        applied = 0;
        delay(2);
    }

    // ── Arranque ou pós-inversão: sobe progressivamente ──────────────────
    if (arranque || inversao) {
        int stepDelay = MOTOR_RAMP_UP_MS / MOTOR_RAMP_STEPS;
        int stepSize  = target / MOTOR_RAMP_STEPS;

        for (int i = 1; i <= MOTOR_RAMP_STEPS; i++) {
            int mid = stepSize * i;
            if (abs(mid) < PWM_MIN_DEADZONE)
                mid = (target > 0) ? PWM_MIN_DEADZONE : -PWM_MIN_DEADZONE;
            _writePins(pin1, pin2, pwmPin, mid);
            delay(stepDelay);
        }
        _writePins(pin1, pin2, pwmPin, target);
        applied = target;
        return;
    }

    // ── Ajuste na mesma direção: rampa proporcional ao delta ─────────────
    // delta / 255 = fração da escala = fração do tempo máximo de rampa.
    // Microajuste de 5 PWM → ~1ms de delay. Ajuste grande de 100 PWM → ~20ms.
    // O PID em curvas faz ajustes pequenos frequentes — chegam rápido.
    // Ajustes grandes (ex: retomada após INTERSECTION) ainda têm suavização.
    int delta     = abs(target - applied);
    int stepDelay = (int)((float)delta / 255.0f * (float)MOTOR_RAMP_UP_MS);
    stepDelay     = constrain(stepDelay, 0, MOTOR_RAMP_UP_MS);

    if (stepDelay > 2) {
        // Aplica em dois passos intermediários para suavizar sem bloquear
        int mid = applied + (target - applied) / 2;
        if (mid != 0 && abs(mid) < PWM_MIN_DEADZONE)
            mid = (mid > 0) ? PWM_MIN_DEADZONE : -PWM_MIN_DEADZONE;
        _writePins(pin1, pin2, pwmPin, mid);
        delay(stepDelay / 2);
    }

    _writePins(pin1, pin2, pwmPin, target);
    applied = target;
}

// ============================================================================
// setMotorSpeed()
// ============================================================================
void MotorController::setMotorSpeed(int speedLeft, int speedRight) {
    _currentLeft  = speedLeft;
    _currentRight = speedRight;

    int trimLeft  = applyTrimLeft(speedLeft);
    int trimRight = applyTrimRight(speedRight);

    _applyRamp(PIN_IN3, PIN_IN4, PIN_ENA, trimLeft,  _appliedLeft);
    _applyRamp(PIN_IN1, PIN_IN2, PIN_ENB, trimRight, _appliedRight);
}

// ============================================================================
// move()
// ============================================================================
void MotorController::move(Direction direction, uint8_t speed) {
    speed = constrain(speed, 0, 255);
    switch (direction) {
        case FORWARD:     setMotorSpeed( speed,  speed); break;
        case BACKWARD:    setMotorSpeed(-speed, -speed); break;
        case TURN_LEFT:   setMotorSpeed( speed, -speed); break;
        case TURN_RIGHT:  setMotorSpeed(-speed,  speed); break;
        case CURVE_LEFT:  setMotorSpeed(0,        speed); break;
        case CURVE_RIGHT: setMotorSpeed(speed,        0); break;
        default:          stop();                         break;
    }
}

// ============================================================================
// stop()
// Parada direta — sem rampa, não inverte polaridade.
// ============================================================================
void MotorController::stop() {
    _writePins(PIN_IN3, PIN_IN4, PIN_ENA, 0);
    _writePins(PIN_IN1, PIN_IN2, PIN_ENB, 0);
    _currentLeft = _currentRight = 0;
    _appliedLeft = _appliedRight = 0;
}

// ============================================================================
// followLine() — Controlador PID
// ============================================================================
void MotorController::followLine(float linePosition, uint8_t baseSpeed) {
    unsigned long now = millis();
    if (now - _lastPDTime < PD_SAMPLE_MS) return;

    float dt    = (now - _lastPDTime) / 1000.0f;
    _lastPDTime = now;

    float error      = linePosition;
    float derivative = error - _prevError;
    _prevError       = error;

    if (fabs(error) > PID_INTEGRAL_DEADZONE) {
        _integral += error * dt;
        _integral  = constrain(_integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    } else {
        _integral = 0.0f;
    }

    float correction = (PD_KP * error)
                     + (PD_KD * derivative)
                     + (PD_KI * _integral);

    float corrNorm = constrain(fabs(correction), 0.0f, 1.0f);

    int outerSpeed = (int)baseSpeed;
    int innerSpeed = constrain(
        (int)(baseSpeed * (1.0f - corrNorm)),
        PD_MIN_INNER_SPEED,
        (int)baseSpeed
    );

    int leftSpeed, rightSpeed;
    if (correction >= 0.0f) {
        leftSpeed  = outerSpeed;
        rightSpeed = innerSpeed;
    } else {
        leftSpeed  = innerSpeed;
        rightSpeed = outerSpeed;
    }

    setMotorSpeed(leftSpeed, rightSpeed);

    if (DEBUG_MODE) {
        static unsigned long lastDbg = 0;
        if (now - lastDbg >= 200) {
            lastDbg = now;
            Serial.print(F("[PID] err="));  Serial.print(error, 3);
            Serial.print(F(" int="));       Serial.print(_integral, 3);
            Serial.print(F(" cor="));       Serial.print(correction, 3);
            Serial.print(F(" L="));         Serial.print(leftSpeed);
            Serial.print(F(" R="));         Serial.println(rightSpeed);
        }
    }
}

// ============================================================================
// resetPD()
// ============================================================================
void MotorController::resetPD() {
    _prevError   = 0.0f;
    _integral    = 0.0f;
    _lastPDTime  = 0;
    _appliedLeft = _appliedRight = 0;
}
