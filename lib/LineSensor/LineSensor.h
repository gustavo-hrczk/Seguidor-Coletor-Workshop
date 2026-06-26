#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// LineSensor — leitura analógica + posição ponderada contínua (QTR-6)
//
// Convenção (linha BRANCA sobre fundo PRETO):
//   Linha branca → analogRead BAIXO  (≤ THRESHOLD_LINE_SENSOR) → sensor ATIVO
//   Fundo preto  → analogRead ALTO   (> THRESHOLD_LINE_SENSOR) → sensor inativo
//
// Posição retornada por getLinePosition():
//   -1.0 = linha na extrema esquerda (S1 ativo)
//    0.0 = linha centralizada (S3+S4 com igual intensidade)
//   +1.0 = linha na extrema direita  (S6 ativo)
//
// Algoritmo: centro de massa ponderado por intensidade.
//   intensity = THRESHOLD - raw (quanto mais sobre a linha, maior)
//   Pesos: S1=-5  S2=-3  S3=-1  S4=+1  S5=+3  S6=+5
//
// Sem filtro de debounce — reatividade máxima necessária para seguimento.
// Filtragem por janela deslizante é responsabilidade do UltrasonicSensor.
//
// Padrões para percurso em formato de 8:
//   LINE_LOST    : nenhum sensor ativo
//   STRAIGHT     : |pos| < 0.25
//   CURVE_LIGHT  : |pos| 0.25–0.50
//   CURVE_MEDIUM : |pos| 0.50–0.75
//   CURVE_SHARP  : |pos| > 0.75
//   INTERSECTION : >= CROSS_MIN_SENSORS_X ativos — cruzamento central do 8
// ============================================================================

class LineSensor {
public:

    // Estado completo de uma leitura — retornado por referência (sem cópia)
    struct SensorState {
        int     raw[6];        // leituras brutas 0–1023
        bool    active[6];     // true = sensor sobre linha branca
        uint8_t activeCount;   // total de sensores ativos
        float   position;      // posição ponderada -1.0..+1.0
    };

    enum LinePattern {
        LINE_LOST    = 0,
        STRAIGHT     = 1,
        CURVE_LIGHT  = 2,
        CURVE_MEDIUM = 3,
        CURVE_SHARP  = 4,
        INTERSECTION = 5
    };

    // Última direção conhecida antes de perder a linha — usada na recuperação
    enum LastDirection {
        DIR_LEFT   = -1,
        DIR_CENTER =  0,
        DIR_RIGHT  =  1
    };

    LineSensor();

    // Configura A0–A5 como INPUT
    void initialize();

    // Lê todos os sensores, calcula posição, atualiza estado interno.
    // Retornar referência constante evita cópia desnecessária.
    // Chamar uma vez por ciclo do loop.
    const SensorState& readSensors();

    float              getLinePosition()  const { return _state.position;        }
    LinePattern        getLinePattern()   const;
    LastDirection      getLastDirection() const { return _lastDir;               }
    uint8_t            getActiveCount()   const { return _state.activeCount;     }
    bool               isLineDetected()   const { return _state.activeCount > 0; }
    const SensorState& getState()         const { return _state;                 }

    // Diagnóstico serial — binário, barra visual e valores raw
    void printSensorValues() const;

    // Reseta para DIR_CENTER — chamar ao retomar seguimento
    void resetLastDirection() { _lastDir = DIR_CENTER; }

private:
    SensorState   _state;
    LastDirection _lastDir;

    // Pesos simétricos: S1=-5 … S6=+5
    static const int WEIGHTS[6];

    float computePosition() const;
};

#endif
