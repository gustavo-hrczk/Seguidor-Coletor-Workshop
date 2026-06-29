# Robô Seguidor Coletor — Biblioteca Técnica

Biblioteca completa em C++ para Arduino Nano, com módulos de seguimento de linha,
detecção de objeto por ultrassom e coleta via garra servo.

Este repositório **não inclui o programa principal pronto**. A biblioteca técnica
está completa, testada e documentada — o `main.cpp` é construído por você.

---

## O que você recebe

| Pasta / arquivo             | Conteúdo                                                                         |
|-----------------------------|----------------------------------------------------------------------------------|
| `src/main.cpp`              | Ponto de partida — estrutura mínima, vazio para você preencher                   |
| `src/config.h`              | Configuração de hardware, PID, calibração — já calibrado para os robôs entregues |
| `lib/`                      | Biblioteca técnica completa: motores, sensor de linha, ultrassônico, garra       |
| `ferramentas_calibracao/`   | Programa de testes isolados — use para validar e calibrar seu robô               |
| `docs/GUIA_BIBLIOTECA.md`   | Explicação de cada módulo e as decisões técnicas por trás dele                   |
| `docs/ROTEIRO.md`           | Passo a passo sugerido para construir seu `main.cpp`                             |
| `docs/PRIMEIROS_PASSOS.md`  | Como instalar, compilar e gravar no Arduino                                      |

---

## Hardware esperado

| Componente            | Modelo                                         |
|-----------------------|------------------------------------------------|
| Microcontrolador      | Arduino Nano (ATmega328P)                      |
| Driver de motor       | L298N                                          |
| Motores               | Micro Motor com Caixa de Redução 6V — Robocore |
| Sensor de linha       | QTR-6 analógico                                |
| Sensor de distância   | HC-SR04 (ultrassônico)                         |
| Atuador da garra      | Servo SG90                                     |

> Os robôs entregues já vêm com este hardware montado. O `config.h` já está
> calibrado para eles — não é necessário recalibrar threshold ou trim a menos
> que perceba comportamento incorreto (veja `docs/PRIMEIROS_PASSOS.md`).

---

## Por onde começar

1. Leia `docs/PRIMEIROS_PASSOS.md` e prepare o ambiente (VS Code + PlatformIO)
2. Leia `docs/GUIA_BIBLIOTECA.md` para entender o que cada módulo faz
3. Siga `docs/ROTEIRO.md` para construir seu `main.cpp` em etapas
4. Use `ferramentas_calibracao/test_components.cpp` sempre que precisar validar
   um módulo isoladamente (ex: "os motores estão respondendo certo?")

---

## Suporte durante a apresentação

Os monitores da equipe estarão circulando para ajudar. Se travar por mais de
5 minutos em qualquer etapa, chame um monitor — é normal travar, é assim que
se aprende a depurar.
