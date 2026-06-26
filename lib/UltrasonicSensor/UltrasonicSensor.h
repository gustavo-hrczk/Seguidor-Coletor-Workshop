#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// UltrasonicSensor — leitura e validação do HC-SR04
//
// Responsabilidade: medir distância e garantir que apenas leituras estáveis
// sejam reportadas, filtrando reflexos espúrios e ruído.
//
// Algoritmo de validação (janela deslizante):
//   - Aceita leitura se estiver dentro da tolerância da última válida
//   - Incrementa contador a cada leitura consistente
//   - Marca readingStable = true após SENSOR_FILTER_CYCLES consecutivas
//   - Degrada o contador gradualmente em leituras inconsistentes (não zera)
//     → evita flicker quando o objeto se move na borda da tolerância
//
// Tolerância adaptativa:
//   ≤ 20 cm → ±2 cm fixo (ruído absoluto do sensor)
//   > 20 cm → percentual com teto de ±4 cm
//
// Fases de aproximação (4 níveis — granularidade corrigida):
//   OBJECT_NOT_DETECTED : sem leitura estável
//   PHASE_1_DISTANT     : dist >= ULTRASONIC_DISTANCE_LONG
//   PHASE_2_APPROACHING : dist >= ULTRASONIC_DISTANCE_SHORT
//   PHASE_3_CONTACT      : dist >= ULTRASONIC_DISTANCE_CONTACT
//                          (zona de transição — perto, mas ainda não é o gatilho)
//   PHASE_4_CONTACT_ZONE : dist <  ULTRASONIC_DISTANCE_CONTACT
//                          (gatilho real de coleta)
//
// Correção aplicada: antes desta versão, qualquer distância abaixo de
// ULTRASONIC_DISTANCE_SHORT (ex: 20cm) já era classificada como "contato",
// mesmo estando a 15-19cm do objeto — bem longe do gatilho real de coleta
// (ULTRASONIC_DISTANCE_CONTACT, ex: 4cm). A fase 3 agora cobre o intervalo
// intermediário, e uma nova fase 4 representa o gatilho real.
// ============================================================================

class UltrasonicSensor {
public:

    enum ApproachPhase {
        OBJECT_NOT_DETECTED  = 0,
        PHASE_1_DISTANT      = 1,
        PHASE_2_APPROACHING  = 2,
        PHASE_3_CONTACT      = 3,   // zona intermediária — perto, ainda não é o gatilho
        PHASE_4_CONTACT_ZONE = 4    // gatilho real de coleta (dist < CONTACT)
    };

    UltrasonicSensor();

    // Configura pinos TRIGGER (OUTPUT) e ECHO (INPUT)
    void initialize();

    // Realiza medição + validação em uma chamada. Retorna lastValidDistance.
    // Retorna -1 se ainda sem leitura estável. Chamar a cada ciclo do loop.
    int readDistance();

    // Fase de aproximação baseada na última distância válida
    ApproachPhase getApproachPhase() const;

    // true se leitura estável dentro do alcance útil
    bool isObjectDetected() const;

    // Executa apenas a etapa de validação (sem nova medição)
    bool validateReading();

    // Limpa completamente o estado — chamar após cada coleta concluída
    void resetValidation();

    int     getLastValidDistance() const { return lastValidDistance; }
    int     getCurrentDistance()   const { return currentDistance;   }
    bool    isReadingStable()      const { return readingStable;     }

    // Diagnóstico serial
    void printDistance() const;

private:
    int           currentDistance;
    int           lastValidDistance;
    int           previousDistance;
    uint8_t       validationCounter;
    bool          recentChange;
    bool          readingStable;
    unsigned long lastChangeTime;

    int  measureDistance();
    bool isWithinTolerance(int dist1, int dist2) const;
};

#endif
