## Como ligar ao raspberry pi

1. Instalar a extensão "Raspberry Pi Pico" no Visual Studio Code.
2. Ligar o Raspberry Pi Pico ao computador, __NÃO PRESSIONAR O BOTÃO BOOTSEL__.
3. Abrir o Visual Studio Code na pasta do raspberry (esta).
4. Na barra inferior do ecrã, clicar no botão "Pico Disconnected", e deverá aparecer um novo terminal com uma mensagem a verde.
5. Para correr algum código, clicar no botão "Run" (o símbolo de play) na barra inferior do ecrã (ao lado do botão de connectar).

## Ficheiros

### accelerometer.py

Código simples para correr o acelerómetro (não usa o gyroscope controller).

### gyroscope_controller.py

Implementa a classe MPU6050 para interface com o sensor acelerómetro/giroscópio, permitindo ler os valores de aceleração e rotação.

### motor_controller.py

Contém a classe MotorController para controlar os motores através de PWM, com funções para definir velocidades e direção.

### balance_controller.py

Implementa o controlador PID para equilibrar o robot, processando os dados do acelerómetro/giroscópio e ajustando a potência dos motores.

### main.py

Programa principal que configura o controlo dos motores e a comunicação Bluetooth, executado automaticamente quando o Pico é ligado. (atualmente recebe apenas comandos simples de ligar/desligar motores)

### full.py

Implementação completa do robot auto-equilibrante, integrando todos os componentes (sensores, motores e controlo PID) (not finished)

### bluethooth_raspberry.py

Implementa a funcionalidade Bluetooth no Raspberry Pi Pico, criando um servidor BLE para receber comandos e enviar telemetria. Esta classe deve ser importada no main.py.

### bluethooth_pc.py

Script para computador que se conecta ao robô via Bluetooth, permitindo o envio de comandos e recepção de dados de telemetria.

## Configuração do hardware

- Raspberry Pi Pico
- Acelerómetro/giroscópio MPU6050
- Controlador de motor (L298N ou similar)
- Dois motores DC
- Bateria
- Chassis do robot

## Conexões

- MPU6050:

  - SDA → GP26
  - SCL → GP27
  - VCC → 3.3V
  - GND → GND
- Controlador de motor:

  - IN1 → GP2
  - IN2 → GP3
  - IN3 → GP4
  - IN4 → GP5
  - ENA → GP6
  - ENB → GP7
- Econders:

  - pinA -> 22
  - pinB -> 28
