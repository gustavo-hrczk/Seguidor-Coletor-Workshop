#ifndef GRIPPER_SERVO_H
#define GRIPPER_SERVO_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

// ============================================================================
// GripperServo — controle da garra via servo SG90
//
// Responsabilidade: abrir e fechar a garra com movimento suave (grau a grau),
// prevenindo tranco mecânico e aquecimento do motor entre comandos.
//
// Design: após atingir o ângulo alvo, o PWM é desligado via detach().
// O SG90 aquece e vibra quando recebe sinal contínuo sem carga — o detach()
// elimina esse problema sem comprometer a posição mantida mecanicamente.
//
// Uso:
//   GripperServo gripper;
//   gripper.initialize();   // posiciona em ABERTO, desliga PWM
//   gripper.close();        // move grau a grau até SERVO_ANGLE_CLOSED
//   gripper.open();         // move grau a grau até SERVO_ANGLE_OPEN
// ============================================================================

class GripperServo {
public:

    enum State {
        OPEN,    // garra aberta  (ângulo = SERVO_ANGLE_OPEN)
        CLOSED,  // garra fechada (ângulo = SERVO_ANGLE_CLOSED)
        ERROR    // servo parado por emergência
    };

    GripperServo();

    // Posiciona em ABERTO, aguarda estabilização e desliga PWM.
    // Chamar uma vez no setup(), antes de qualquer outro método.
    void initialize();

    // Move para SERVO_ANGLE_OPEN — ignorado se já estiver aberta.
    void open();

    // Move para SERVO_ANGLE_CLOSED — ignorado se já estiver fechada.
    void close();

    // Desliga PWM imediatamente sem aguardar posição final.
    // Define estado ERROR — open() e close() não funcionam após isso.
    // Usar apenas em falhas críticas.
    void emergencyStop();

    State getState() const { return _state;          }
    bool  isOpen()   const { return _state == OPEN;  }
    bool  isClosed() const { return _state == CLOSED;}

private:
    Servo   _servo;
    State   _state;
    uint8_t _currentAngle;   // ângulo físico atual — rastreia posição real

    // Move grau a grau até o ângulo alvo.
    // Privado — chamado apenas por open() e close().
    void moveToAngle(uint8_t target);
};

#endif
