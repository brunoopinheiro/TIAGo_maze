# Programação robótica
## Residencia Softex
Leia o documento inteiro antes de começar a trabalhar.
Esboce sua solução antes de iniciar a programação.

## Descrição do trabalho
O robô TIAGo está preso em um mini-labirinto. Para sair, tem que seguir estas regras:
1. A base móvel pode se mover de duas maneiras:
    - Em linha reta até que a distância até a parede na frente do robô seja menor que um limiar definido pelo usuário
    - Em rotação pura, até ter realizado um deslocamento angular de mais ou menos 90 graus
2. O deslocamento linear é realizado por meio de um controlador baseado na distância fornecida pelo
laser.
3. A rotação é realizada por meio de um controlador baseado na odometria.
4. O robô começa se movendo em linha reta
5. Depois de completar uma ação, o robô tem que tomar uma decisão:
    - Se houver uma parede em ambos os lados do robô (menos de 1,5 metros), ele deve se mover em linha reta;
    - Se houver uma parede de um lado e outra na frente, ela deve virar em direção ao espaço livre;
    - Se não houver parede em ambos os lados (menos de 1,5 metros), deve virar a cabeça em + e - 90 graus para tirar uma foto.
6. Depois de tirar uma foto, o robô deve processá-la.
    - Se ele contém um pôster vermelho, o robô não pode virar naquela direção.
    - Se ele contiver um pôster verde, o robô deve virar nessa direção.
    - Se não houver pôster, o robô cumpriu a missão
7. Após completar sua missão, o robô deve fazer um sinal com o braço.

## Como organizar o código
- Você tem que implementar essas regras nos arquivos `main.py` e `camera.py`.
- Os dados de odometria e do laser são coletados por meio de dois subcriber no arquivo `main.py`.
- Os dados da câmera são coletados por meio de um subscriber no arquivo camera.py
- As mensagens de controle da base móvel e da cabeça são enviados por meio de dois publisher no arquivo main.py
- Os processamentos em relação aos pontos 2, 3 e 5 são executados em três funções definidas no arquivo main.py
- O processamento de imagem é executado por meio de um serviço definido no arquivo camera.py
-  A programação orientada ao objeto deve ser usada.

## Passos
Cada etapa deve ser validada pelo professor
- [X] Fornecer as distâncias calculadas pelo laser a -90, 0 e 90 graus.
- [X] Realizar o movimento em linha reta.
- [X] Fornecer a orientação do robô (o valor deve ser contı́nuo).
- [X] Realizar as rotações da base móvel.
- [X] Realizar as rotações da cabeça.
- [ ] Processar as imagens.
- [ ] Controlar o braço para fazer um sinal
- [ ] Implementar uma maquina de estado.

## Thresholding
- No processamento de imagem digital, o thresholding é o método mais simples de segmentar imagens.
- Os métodos mais simples substituem cada pixel em uma imagem por um pixel preto se a intensidade da imagem Ii,j é menor do que alguma constante fixa T (isto é , Ii,j < T ), ou um pixel branco se a intensidade da imagem for maior do que essa constante.
- Para uma imagem colorida, um limite diferente pode ser usado para cada camada ou pode ser aplicado em uma única camada.
- O resultado final é uma imagem com uma camada única.

## Instalação
- No workspace de TIAGo (nesse exemplo o nome do workspace é tiago ws)
    - Copiar os arquivos `maze.world`, `mazeTest1.world` e `mazeTest2.world` em: `tiago ws/src/tiago simulation/tiago gazebo/worlds/`
    - Copiar os diretórios `maze`, `wall`, `landmark green`, `landmark red`, `frontWall` e `cornerWall em: tiago ws/src/tiago simulation/tiago gazebo/models/`
- Para iniciar a simulação, insira a seguinte linha (não se esqueça do comando no diretório do workspace):
```bash
roslaunch tiago gazebo tiago gazebo.launch public sim:=true robot:=titanium world:=maze
```

- Existem ambientes simplificados para teste:
```bash
roslaunch tiago gazebo tiago gazebo.launch public sim:=true robot:=titanium world:=mazeTest1
```

```bash
roslaunch tiago gazebo tiago gazebo.launch public sim:=true robot:=titanium world:=mazeTest2
```
- Seu código deve fazer parte de um workspace diferente

## Avaliação
- O trabalho é realizado em dupla.
- Os programas (pacotes) devem ser colocados no Classroom.
- Nenhum relatório será necessário.
- Os programas devem funcionar em um ambiente diferente respeitando as mesmas regras.

## Problemas Recorrentes
## TIAGo Iniciando não orientado
O problema parece ter relação com a movimentação de posicionamento do braço robótico. Para evitar a orientação, é possível alterar o arquivo de launch do TIAGo, ajustando sua orientação `x` na inicialização.
No diretório `/tiago_public_ws/src/tiago_simulation/tiago_gazebo/launch`, alterar o arquivo `tiago_gazebo.launch`.

```bash
sudo nano tiago_gazebo.launch
```

E alterar a linha de `gzpose` para:
```xml
<arg name="gzpose" default="-x -0.5 -y 0.0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0"/>
```
