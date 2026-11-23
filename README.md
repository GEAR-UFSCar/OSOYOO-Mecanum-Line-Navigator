# OSOYOO Mecanum Line Navigator

Seguidor de linha com PID, tratamento de cruzamentos, desvio de obstáculos e
movimento lateral com rodas mecanum para o **OSOYOO Mecanum Wheel Robotic Car
Kit V2.1**. Projeto do [GEAR/UFSCar](https://github.com/GEAR-UFSCar).

Todo o código está em um único sketch: [`line_navigator.ino`](line_navigator.ino).

## Hardware

| Item | Detalhe |
|---|---|
| Placa | Arduino Mega 2560 |
| Driver | OSOYOO Model Y (4 motores DC) |
| Rodas | 4 × mecanum |
| Linha | Sensor de 5 canais em `A4 A3 A2 A1 A0` (S1 → S5) |
| Obstáculo | 2 sensores IR: esquerdo `D3`, direito `D2` |

Servo, ultrassom, Wi-Fi e Bluetooth do kit **não** são usados.

### Pinos dos motores

| Motor | PWM | Direção |
|---|---|---|
| Frente direita | 9 | 22, 24 |
| Frente esquerda | 10 | 26, 28 |
| Trás direita | 11 | 5, 6 |
| Trás esquerda | 12 | 7, 8 |

## Como usar

1. Abra `line_navigator.ino` na Arduino IDE.
2. Placa: **Arduino Mega or Mega 2560**.
3. Grave e abra o Monitor Serial em **115200 baud** para ver o diagnóstico.

Pelo terminal, com `arduino-cli`:

```bash
arduino-cli compile -b arduino:avr:mega line_navigator.ino
arduino-cli upload  -b arduino:avr:mega -p /dev/ttyACM0 line_navigator.ino
```

## Como funciona

O `loop()` é uma máquina de estados:

```
FOLLOW_LINE ─┬─ curva fechada ──────────→ SHARP_LEFT / SHARP_RIGHT ─┐
             ├─ todos os sensores ativos → INTERSECTION ────────────┤
             ├─ IR detecta obstáculo ───→ OBSTACLE_* → ESCAPE_* ────┤
             └─ perdeu a linha ─────────────────────────────────────┴→ SEARCH_LINE
SEARCH_LINE ── achou a linha → FOLLOW_LINE   |   estourou o tempo → STOPPED
```

- **PID:** o erro é a média ponderada dos 5 sensores (pesos −200, −100, 0, 100, 200).
- **Cruzamentos:** a decisão segue `navigationStrategy` (`LEFT_HAND`, `RIGHT_HAND`
  ou `STRAIGHT_FIRST`) e cada decisão fica gravada em `intersectionMemory` (até 50).
- **Obstáculos:** o robô contorna de lado com o deslocamento lateral das rodas
  mecanum (`shiftLeft` / `shiftRight`) e depois procura a linha.
- **Busca da linha:** gira na direção em que a linha foi vista por último e para
  se não a encontrar a tempo.

## Ajustes

Tudo fica no topo do sketch:

| Parâmetro | Padrão | Efeito |
|---|---|---|
| `Kp`, `Ki`, `Kd` | 0.30, 0.00, 0.075 | Ganhos do PID |
| `BASE_SPEED` / `MAX_SPEED` / `MIN_SPEED` | 70 / 90 / 35 | Velocidades em reta (PWM 0–255) |
| `CURVE_SPEED`, `SEARCH_SPEED`, `OBSTACLE_SPEED`, `SHIFT_SPEED` | 70, 40, 38, 55 | Velocidades por manobra |
| `navigationStrategy` | `LEFT_HAND` | Regra de escolha nos cruzamentos |

Cada kit responde um pouco diferente (motores, bateria, piso). Calibre `Kp`/`Kd`
e as velocidades na pista real.

## Licença

[MIT](LICENSE)
