#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// MotorController — controle dos motores DC via driver L298N
//
// Trim de assimetria:
//   MOTOR_TRIM_DIR e MOTOR_TRIM_ESQ aplicam fator multiplicativo individual
//   dentro de setMotorSpeed(), garantindo que BASE_SPEED produza velocidades
//   físicas iguais nos dois lados.
//
// Rampas de proteção (seção 2b do config.h):
//   Toda mudança de velocidade passa por _applyRamp() antes de chegar ao
//   hardware. Dois cenários protegidos:
//     Arranque (0 -> v): sobe em MOTOR_RAMP_STEPS degraus / MOTOR_RAMP_UP_MS
//     Inversão (+ -> -): desce até 0 em MOTOR_RAMP_DOWN_MS antes de inverter
//   _appliedLeft/_appliedRight rastreiam a velocidade REAL no hardware,
//   separada da velocidade LÓGICA solicitada (_currentLeft/_currentRight).
//
// Controlador PID:
//   followLine() implementa PID completo com termo integral.
//   resetPD() zera _prevError, _integral e _lastPDTime — obrigatório
//   ao retomar seguimento após parada ou coleta.
// ============================================================================

class MotorController {
public:

    enum Direction {
        STOP        = 0,
        FORWARD     = 1,
        BACKWARD    = 2,
        TURN_LEFT   = 3,
        TURN_RIGHT  = 4,
        CURVE_LEFT  = 5,
        CURVE_RIGHT = 6
    };

    MotorController();
    void initialize();

    // Controle direto por motor (-255..+255).
    // Aplica trim, rampa e deadzone antes de enviar ao hardware.
    void setMotorSpeed(int speedLeft, int speedRight);

    void move(Direction direction, uint8_t speed = VELOCITY_GLOBAL);
    void stop();

    // Controlador PID — chamar uma vez por ciclo em STATE_FOLLOWING
    void followLine(float linePosition, uint8_t baseSpeed = VELOCITY_GLOBAL);

    // Zera _prevError, _integral, _lastPDTime e velocidades aplicadas
    void resetPD();

    int  getLeftSpeed()  const { return _currentLeft;  }
    int  getRightSpeed() const { return _currentRight; }
    bool isMoving()      const { return (_appliedLeft != 0 || _appliedRight != 0); }

private:
    int   _currentLeft;    // velocidade lógica solicitada
    int   _currentRight;
    int   _appliedLeft;    // velocidade real no hardware (após rampa)
    int   _appliedRight;

    float         _prevError;
    float         _integral;
    unsigned long _lastPDTime;

    int  applyTrimLeft(int speed)  const;
    int  applyTrimRight(int speed) const;

    // Aplica rampa de arranque ou inversão e envia ao hardware.
    // @param pin1, pin2, pwmPin  pinos do canal L298N
    // @param target              velocidade alvo com trim já aplicado
    // @param applied             referência à velocidade atualmente no hardware
    void _applyRamp(int pin1, int pin2, int pwmPin, int target, int& applied);

    // Envia velocidade diretamente ao hardware (sem rampa)
    void _writePins(int in1, int in2, int pwmPin, int speed);
};

#endif
