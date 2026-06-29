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

> O `platformio.ini` já está configurado para usar `programas/` como pasta
> de origem — não é necessário copiar nada para `src/`.

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
2. Confirme que a velocidade está em **9600 baud**
3. Você verá o menu do `debug_mode.cpp` com os 6 testes disponíveis

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

- Confira se a pasta `programas/` tem **apenas um** arquivo com `setup()` e
  `loop()`. Se você criar seu próprio programa de robô (seguindo
  `docs/ROTEIRO.md`), remova ou renomeie `debug_mode.cpp` antes de
  adicionar o seu — não pode haver dois arquivos com `setup()`/`loop()`
  compilando juntos.

### Quero escrever meu próprio programa de robô, e não usar o `debug_mode.cpp`

- Crie um novo arquivo `.cpp` em `programas/` com seu próprio `setup()`/`loop()`
- Remova ou renomeie `debug_mode.cpp` antes de compilar
- Siga `docs/ROTEIRO.md` para a progressão sugerida
- Consulte `docs/referencia/` para ver dois exemplos completos já testados
