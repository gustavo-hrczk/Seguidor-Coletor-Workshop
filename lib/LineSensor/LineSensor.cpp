#include "LineSensor.h"

// Pesos simétricos: esquerda negativa, direita positiva.
// pos = 0 quando S3 e S4 leem a linha com mesma intensidade.
const int LineSensor::WEIGHTS[6] = { -5, -3, -1, 1, 3, 5 };

// ============================================================================
// Construtor
// ============================================================================
LineSensor::LineSensor() : _lastDir(DIR_CENTER) {
    memset(&_state, 0, sizeof(SensorState));
}

// ============================================================================
// initialize()
// A0–A5 são INPUT por padrão no Arduino — pinMode explícito documenta
// a intenção e protege contra reconfiguração acidental por outro módulo.
// ============================================================================
void LineSensor::initialize() {
    for (int i = 0; i < 6; i++) {
        pinMode(A0 + i, INPUT);
    }
}

// ============================================================================
// readSensors()
// Leitura direta, sem filtro — reatividade máxima para seguimento.
// Atualiza estado interno e retorna referência constante (sem cópia).
// ============================================================================
const LineSensor::SensorState& LineSensor::readSensors() {
    _state.activeCount = 0;

    for (int i = 0; i < 6; i++) {
        _state.raw[i]    = analogRead(A0 + i);
        // Linha branca reflete mais luz → tensão menor → raw baixo
        _state.active[i] = (_state.raw[i] <= THRESHOLD_LINE_SENSOR);
        if (_state.active[i]) _state.activeCount++;
    }

    _state.position = computePosition();

    // Atualiza última direção conhecida apenas com sensores ativos.
    // Em LINE_LOST, _lastDir é preservado para guiar a recuperação.
    if (_state.activeCount > 0) {
        if      (_state.position < -0.10f) _lastDir = DIR_LEFT;
        else if (_state.position >  0.10f) _lastDir = DIR_RIGHT;
        else                               _lastDir  = DIR_CENTER;
    }

    return _state;
}

// ============================================================================
// computePosition()
// Centro de massa ponderado por intensidade de leitura.
//
// intensity = THRESHOLD - raw
//   → sensor no centro da linha: raw mínimo → intensidade alta → mais peso
//   → sensor na borda da linha:  raw alto   → intensidade baixa → menos peso
//
// Resultado: weightedSum / totalWeight em escala -5..+5, dividido por 5
// para normalizar em -1.0..+1.0.
//
// LINE_LOST: retorna ±1.0 na última direção conhecida para manter o PID
// corrigindo na direção certa durante a manobra de recuperação.
// ============================================================================
float LineSensor::computePosition() const {
    if (_state.activeCount == 0) {
        if (_lastDir == DIR_RIGHT) return  1.0f;
        if (_lastDir == DIR_LEFT)  return -1.0f;
        return 0.0f;
    }

    long weightedSum = 0;
    int  totalWeight = 0;

    for (int i = 0; i < 6; i++) {
        if (_state.active[i]) {
            int intensity = THRESHOLD_LINE_SENSOR - _state.raw[i];
            if (intensity < 1) intensity = 1;   // peso mínimo 1 na borda
            weightedSum += (long)WEIGHTS[i] * intensity;
            totalWeight += intensity;
        }
    }

    if (totalWeight == 0) return 0.0f;

    float pos = (float)weightedSum / (float)totalWeight / 5.0f;
    if (pos < -1.0f) pos = -1.0f;
    if (pos >  1.0f) pos =  1.0f;
    return pos;
}

// ============================================================================
// getLinePattern()
// Prioridade: INTERSECTION > seguimento por magnitude.
//
// INTERSECTION (>= CROSS_MIN_SENSORS_X ativos):
//   Cruzamento central do percurso em 8. O robô passa reto em velocidade
//   reduzida (SPEED_INTERSECTION) sem alterar a lógica de seguimento.
//
// Faixas de seguimento (1–4 sensores ativos):
//   STRAIGHT     |pos| < 0.25  — absorve ruído em reta sem mascarar curvas
//   CURVE_LIGHT  |pos| 0.25–0.50
//   CURVE_MEDIUM |pos| 0.50–0.75
//   CURVE_SHARP  |pos| > 0.75  — curva fechada do 8
// ============================================================================
LineSensor::LinePattern LineSensor::getLinePattern() const {
    if (_state.activeCount == 0)
        return LINE_LOST;

    if (_state.activeCount >= CROSS_MIN_SENSORS_X)
        return INTERSECTION;

    float absPos = (_state.position < 0.0f) ? -_state.position : _state.position;

    if      (absPos < 0.25f) return STRAIGHT;
    else if (absPos < 0.50f) return CURVE_LIGHT;
    else if (absPos < 0.75f) return CURVE_MEDIUM;
    else                     return CURVE_SHARP;
}

// ============================================================================
// printSensorValues()
// Saída de diagnóstico em uma linha:
//   [binário 6 sensores] [barra visual -1..+1] posição cnt= raw: ...
//
// '#' na barra = posição atual | '|' no centro = zero
// Essencial para calibrar THRESHOLD_LINE_SENSOR na pista real.
// ============================================================================
void LineSensor::printSensorValues() const {
    if (!DEBUG_MODE) return;

    Serial.print(F("[Line] "));
    for (int i = 0; i < 6; i++) {
        Serial.print(_state.active[i] ? '1' : '0');
    }

    char bar[22];
    int  idx = 20 - (int)((_state.position + 1.0f) * 10.0f);
    if (idx < 0)  idx = 0;
    if (idx > 20) idx = 20;
    for (int i = 0; i < 21; i++) bar[i] = (i == 10) ? '|' : '-';
    bar[idx] = '#';
    bar[21]  = '\0';

    Serial.print(F(" ["));    Serial.print(bar);            Serial.print(F("] "));
    Serial.print(_state.position, 3);
    Serial.print(F(" cnt=")); Serial.print(_state.activeCount);
    Serial.print(F(" raw:"));
    for (int i = 0; i < 6; i++) {
        Serial.print(F(" ")); Serial.print(_state.raw[i]);
    }
    Serial.println();
}