# Primeiros Passos

## 1. Instalar o ambiente

1. Instale o [VS Code](https://code.visualstudio.com/)
2. Abra o VS Code → vá em Extensions (ícone de blocos na barra lateral)
3. Busque por **PlatformIO IDE** e instale
4. Aguarde o PlatformIO terminar a instalação inicial (pode levar alguns minutos)
5. Reinicie o VS Code

## 2. Abrir o projeto

1. No VS Code, clique no ícone do PlatformIO na barra lateral (formiga laranja)
2. Clique em **Open** → **Open Project**
3. Selecione a pasta deste repositório

## 3. Compilar (sem gravar ainda)

1. Na barra inferior do VS Code, clique no ícone de check (✓) — "Build"
2. Aguarde a compilação terminar
3. Se aparecer `[SUCCESS]` no final, está tudo certo

> Se aparecer erro de biblioteca não encontrada, aguarde — o PlatformIO baixa
> a biblioteca Servo automaticamente na primeira compilação.

## 4. Conectar o Arduino e gravar

1. Conecte o Arduino Nano via cabo USB
2. **Importante:** alguns cabos USB só carregam, não transmitem dados.
   Se a porta não aparecer, troque o cabo antes de qualquer outra coisa.
3. Clique no ícone de seta (→) na barra inferior — "Upload"
4. Aguarde a mensagem `[SUCCESS]`

## 5. Abrir o Monitor Serial

1. Clique no ícone de tomada/plug na barra inferior — "Monitor"
2. Você deve ver mensagens aparecendo se o código tiver `Serial.println()`

---

## Problemas comuns

### "Porta não encontrada" / upload falha

- Troque o cabo USB — muitos cabos micro-USB só fazem carga, não dados
- No Windows, pode ser necessário instalar o driver **CH340**
- Verifique se outro programa (outro monitor serial) não está usando a porta

### Compilação trava ou demora muito na primeira vez

- Normal — o PlatformIO está baixando o framework do Arduino e dependências
- Aguarde, não cancele

### Monitor Serial mostra caracteres estranhos

- Verifique se a velocidade do monitor está em **9600 baud**
  (já configurado no `platformio.ini`, mas confirme no canto da janela do monitor)

### Erro de compilação mencionando alguma função do `LineSensor`, `MotorController` etc.

- Confira se não há `main.cpp` duplicado — o projeto deve ter **apenas um**
  arquivo com `setup()` e `loop()` na pasta `src/`. Se você copiou o
  `test_components.cpp` para dentro de `src/`, remova o `main.cpp` de lá
  primeiro ou mova o teste para fora antes de compilar o programa principal.

---

## Quando usar o `ferramentas_calibracao/test_components.cpp`

Esse arquivo tem 6 testes isolados (motores, sensor de linha, ultrassônico,
garra, simulação de coleta e calibração de threshold). Para usá-lo:

1. Copie temporariamente o `main.cpp` da pasta `src/` para fora (ex: para a raiz do projeto, fora de `src/`)
2. Copie `test_components.cpp` para dentro de `src/`
3. Compile e grave normalmente
4. Abra o monitor serial — vai aparecer um menu numerado
5. Quando terminar de calibrar, faça o caminho inverso para voltar ao seu `main.cpp`

> Só pode haver um arquivo com `setup()`/`loop()` em `src/` por vez.
