// ============================================================================
// main.cpp — Construa aqui o programa do seu robô
//
// A biblioteca técnica já está pronta em lib/ — consulte:
//   docs/GUIA_BIBLIOTECA.md     → explicação de cada módulo e decisão técnica
//   docs/referencia/            → exemplo de referência (não copie, use para entender)
//   programas/debug_mode.cpp    → testes isolados para calibrar seu robô
//
// Módulos disponíveis (já incluídos no projeto):
//   LineSensor         → leitura do sensor de linha QTR-6
//   MotorController    → controle dos motores com PID, trim e proteção
//   UltrasonicSensor   → leitura e validação do HC-SR04
//   GripperServo       → controle da garra
//
// Sugestão de progressão (veja docs/ROTEIRO.md para o passo a passo completo):
//   1. Faça o robô se mover (motores)
//   2. Faça o robô seguir a linha
//   3. Adicione detecção de objeto (ultrassônico)
//   4. Adicione a coleta (garra)
//   5. Integre tudo em uma máquina de estados
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "LineSensor.h"
#include "MotorController.h"
#include "UltrasonicSensor.h"
#include "GripperServo.h"

// ── Instâncias dos módulos ───────────────────────────────────────────────────
LineSensor       lineSensor;
MotorController  motor;
UltrasonicSensor ultrasonic;
GripperServo     gripper;

void setup() {
    Serial.begin(BAUD_RATE);

    // Inicialize os módulos que for usar:
    // lineSensor.initialize();
    // motor.initialize();
    // ultrasonic.initialize();
    // gripper.initialize();

    Serial.println(F("Robo pronto. Comece a programar em main.cpp"));
}

void loop() {
    // Seu código aqui
}
