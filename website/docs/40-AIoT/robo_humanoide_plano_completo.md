# Projeto de Robô Humanoide para Trabalho Agrícola e de Construção Civil

**Arquitetura técnica, plano de estudos e cronograma**

---

## Nota preliminar sobre o escopo

Antes do detalhamento, três observações honestas sobre o esquema inicial, porque elas mudam o desenho de tudo o que vem depois:

1. **O esquema é forte em cognição e fraco em mecânica.** Ele descreve muito bem a pilha "percepção → raciocínio → comando", mas não menciona estrutura, atuadores de alto torque, redutores, transmissão, rigidez estrutural, dissipação térmica, proteção IP contra poeira/água nem controle de equilíbrio. Para um robô que carrega um saco de ração ou levanta um bloco de concreto, **essa é a parte difícil e cara** — não o LLM. Adicionei essas camadas.

2. **Bipedismo é a decisão de maior risco do projeto.** Um humanoide bípede de trabalho pesado é um problema de doutorado com orçamento de laboratório. Recomendo uma progressão em três versões: **v0** (braço fixo sobre bancada), **v1** (base com rodas/esteiras + tronco + dois braços — já resolve 80% das tarefas agrícolas e boa parte das de construção) e **v2** (pernas). O plano de estudos e o cronograma seguem essa progressão.

3. **`numpy` não roda "em C++ puro".** NumPy é uma biblioteca Python com núcleo em C. O equivalente C++ para o laço de controle é **Eigen** (ou Pinocchio, para dinâmica de corpos rígidos). Tratei os dois separadamente no texto.

**Cenário fio-condutor:** ao longo de todo o Tópico 1 uso um mesmo comando de usuário para exemplificar cada elemento:

> *"Colha os tomates maduros da fileira 3 da estufa e coloque na caixa azul. Não mexa nos verdes."*

---

# TÓPICO 1 — Elementos do Robô Humanoide

## Visão geral da arquitetura em camadas

O sistema é dividido em **quatro domínios temporais** — essa é a chave de organização de todo o projeto, porque cada domínio tem requisitos de latência, determinismo e hardware completamente diferentes:

| Domínio | Frequência | Hardware | Determinismo | Elementos |
|---|---|---|---|---|
| **D0 — Malha de corrente/torque** | 10–20 kHz | Driver do motor (gate driver + MCU dedicado) | Rígido (hard real-time) | FOC, PWM, sensor de corrente |
| **D1 — Malha de junta / equilíbrio** | 500 Hz – 2 kHz | Microcontrolador (STM32/ESP32), micro-ROS | Rígido | PID de posição, ZMP, IMU, encoders |
| **D2 — Percepção e planejamento de movimento** | 10–60 Hz | SBC / GPU embarcada (Jetson, Raspberry Pi) | Suave (soft real-time) | ROS 2, SLAM, visão, cinemática inversa, trajetórias |
| **D3 — Cognição e interação** | 0,2 – 2 Hz | SBC + nuvem | Não determinístico | LLM, ontologia, DeepProbLog, chatbot, planejador |

> **Regra de ouro:** nada que possa travar, esperar rede ou alocar memória dinamicamente pode viver em D0 ou D1. O LLM **nunca** comanda um motor diretamente. Ele emite *intenções simbólicas* que descem, verificadas, pela pilha.

```
┌─────────────────────────────────────────────────────────────┐
│ D3  Chatbot ─ LLM ─ Alinhador Semântico ─ Ontologia OWL     │
│     DeepProbLog (Intenção) ─ Semantic Grounding             │
│     DeepProbLog (Execução) ─ Planejador PDDL/Behavior Tree  │
└────────────────────────┬────────────────────────────────────┘
                         │ ROS 2 ACTIONS (goal/feedback/result)
┌────────────────────────▼────────────────────────────────────┐
│ D2  Percepção (RGB-D, LiDAR, SLAM, tf2) ─ Cinemática        │
│     Inversa ─ Geração de trajetória ─ ros2_control          │
│     [Jetson/RPi · Ubuntu + PREEMPT_RT · ROS 2 Lyrical]      │
└────────────────────────┬────────────────────────────────────┘
                         │ ROS 2 TOPICS (DDS) / micro-ROS / CAN
┌────────────────────────▼────────────────────────────────────┐
│ D1  MCUs de junta: PID, limites, watchdog, TinyML (LiteRT)  │
│ D0  Drivers FOC: PWM, corrente, comutação                   │
└────────────────────────┬────────────────────────────────────┘
                         │ Potência
┌────────────────────────▼────────────────────────────────────┐
│     Atuadores · Estrutura · Baterias · E-Stop (hardwired)   │
└─────────────────────────────────────────────────────────────┘
```

---

## 1. Camada mecânica e estrutural (adicionada ao esquema)

### 1.1 Estrutura e escolha de topologia

**Física envolvida:** o problema central é a razão **torque útil / massa própria**. Cada quilo adicionado a um elo distal aparece quadraticamente no momento de inércia visto pela junta proximal ($I = \int r^2\,dm$), o que exige motor maior, que pesa mais, que exige motor maior ainda — o *loop* de espiral de massa.

Dimensionamento estático de uma junta de ombro que segura uma carga $m$ na ponta de um braço de comprimento $L$:

$$\tau_{\text{junta}} = m_{\text{carga}} \cdot g \cdot L + m_{\text{braço}} \cdot g \cdot \frac{L}{2}$$

**Exemplo real no robô:** braço de 0,65 m que precisa segurar uma britadeira de 8 kg, com braço próprio de 3 kg:
$\tau = 8 \times 9{,}81 \times 0{,}65 + 3 \times 9{,}81 \times 0{,}325 \approx 51 + 9{,}6 \approx 61\ \text{N}\cdot\text{m}$ **em regime estático**. Com fator de segurança dinâmico de 2,5× (aceleração + impacto), o projeto pede ~150 N·m no ombro. Isso é a diferença entre um servo de hobby (3 N·m) e um atuador quase-direct-drive com redutor cicloidal — cerca de 40× mais caro.

**Materiais:** perfis de alumínio 6061-T6 (limite de escoamento ~276 MPa, densidade 2,70 g/cm³) para elos; chapa de aço para a base (baixar o centro de massa); polímeros impressos (PETG/nylon-CF) apenas para carenagens e suportes de sensor, nunca para caminho de carga. Em estufa/canteiro de obra, **grau de proteção IP54 mínimo** (poeira + respingos); juntas com vedação labial e diafragma.

**Rigidez:** a frequência natural do elo determina o teto da largura de banda do controlador. Se o braço ressoa a 12 Hz, o controlador de junta não pode ter ganho útil acima de ~4 Hz sem excitar a estrutura. Isso se mede empiricamente com um ensaio de martelo instrumentado e FFT do sinal do acelerômetro.

### 1.2 Redutores e transmissão

| Tipo | Redução típica | Folga (backlash) | Rendimento | Uso no robô |
|---|---|---|---|---|
| Planetário | 3:1 a 100:1 | 15–30 arcmin | 90–97% | Juntas de pulso, cabeça |
| Cicloidal | 30:1 a 120:1 | < 1 arcmin | 80–90% | Ombro, cotovelo, quadril |
| Harmonic drive | 50:1 a 160:1 | ~0 (pré-carga) | 70–85% | Juntas de precisão, caro |
| Correia + polia | 2:1 a 6:1 | baixa, elástica | > 95% | Transmitir motor para longe da junta (reduz inércia distal) |
| Fuso de esferas | linear | baixa | ~90% | Atuação linear do tronco |

**Exemplo real:** no cotovelo, motor BLDC de 200 W girando a 3000 rpm com redutor cicloidal 50:1 entrega 500 rpm ÷ 50 ≈ 60 rpm na junta e multiplica o torque por 50 × 0,85 ≈ 42,5. A **inércia refletida** do rotor na junta é $n^2 J_{\text{rotor}}$ — com $n=50$, um rotor de $2\times10^{-5}\,\text{kg·m}^2$ vira $0{,}05\ \text{kg·m}^2$ vistos pela junta, o que *domina* a dinâmica e na verdade **ajuda** na estabilidade, mas mata a capacidade de sentir forças externas pela corrente (retro-dirigibilidade).

### 1.3 Complacência: SEA e limitação de torque

Um robô que trabalha ao lado de pessoas e de plantas não pode ser infinitamente rígido. Duas soluções:

- **SEA (Series Elastic Actuator):** insere-se uma mola de constante $k$ conhecida entre o redutor e o elo. A deflexão $\Delta\theta$, lida por dois encoders (antes e depois da mola), dá o torque diretamente: $\tau = k\,\Delta\theta$. Isso transforma controle de força em controle de posição — muito mais fácil e robusto. Custo: reduz a largura de banda para $\omega_n = \sqrt{k/J}$.
- **Limitação de corrente:** o firmware do driver satura a corrente de fase, o que satura o torque. É a proteção mais barata, mas só funciona se a transmissão for retro-dirigível.

**Exemplo real:** a garra que colhe o tomate usa SEA com $k = 15\ \text{N·m/rad}$. Ao fechar sobre o fruto, a deflexão de 0,12 rad indica 1,8 N·m, convertidos por geometria em ~9 N de força de preensão — abaixo do limiar de esmagamento do tomate maduro (~15 N). A mesma garra em modo "construção" opera com batente mecânico que curto-circuita a mola e permite 80 N para segurar um tijolo.

---

## 2. Atuadores, circuitos e eletromagnetismo

### 2.1 Motores DC com escovas

**Física (eletromagnetismo):** a força de Lorentz sobre os condutores da armadura imersos no campo do estator, $\vec{F} = I\,\vec{L}\times\vec{B}$, produz o torque. As duas equações fundamentais:

$$\tau = K_t \cdot i \qquad\qquad e_{\text{fcem}} = K_e \cdot \omega$$

E o circuito elétrico da armadura:

$$V = R\,i + L\frac{di}{dt} + K_e\,\omega$$

Em unidades SI, $K_t$ [N·m/A] e $K_e$ [V·s/rad] são **numericamente iguais** — consequência direta da conservação de energia ($P_{\text{elétrica}} = P_{\text{mecânica}}$ no motor ideal).

**Constante de tempo elétrica** $\tau_e = L/R$ (tipicamente 0,1–2 ms) define a frequência mínima do PWM: para a corrente não "picotar", $f_{\text{PWM}} \gg 1/(2\pi\tau_e)$, daí PWM de 20–40 kHz (que também fica acima da faixa audível).

**Exemplo real:** as rodas da base móvel usam motores DC com escovas de 24 V, $K_t = 0{,}085\ \text{N·m/A}$. Para vencer o arrasto em solo mole de estufa exigindo 4 N·m na roda com redutor 20:1, o motor precisa de $4/(20 \times 0{,}085) \approx 2{,}35\ \text{A}$. Com $R = 0{,}9\ \Omega$, a perda joule é $I^2R \approx 5\ \text{W}$ por motor — calor que precisa sair do invólucro IP54, o que obriga a dissipador acoplado à carcaça.

### 2.2 Servomotores

Um servo é um sistema fechado: motor DC + redutor + sensor de posição + eletrônica de controle, tudo em um invólucro.

- **Servo de hobby (RC):** recebe PWM de 50 Hz com largura de pulso 1,0–2,0 ms mapeada linearmente para o ângulo. É *comando de posição sem realimentação para o host* — você não sabe se ele chegou lá. Serve apenas para protótipo.
- **Servo "inteligente" (Dynamixel, LX-16A, STS):** comunica por barramento serial half-duplex ou RS-485, reporta posição, velocidade, corrente e temperatura, aceita comando de posição, velocidade ou corrente. **Este é o caminho realista para v0/v1.**

**Exemplo real:** os 5 dedos da mão usam servos de barramento encadeados em daisy-chain; um único par de fios percorre o antebraço. O nó ROS 2 `hand_driver` lê o registro de corrente de cada dedo a 100 Hz; quando a corrente do indicador ultrapassa o limiar durante 30 ms, o firmware conclui "contato com o fruto" e publica em `/hand/contact_events` — uma forma barata de sensor tátil por *sensorless force estimation*.

### 2.3 Motores de passo

**Física:** o rotor de ímãs permanentes alinha-se com o campo produzido por fases sucessivas do estator. Um motor híbrido de 1,8°/passo tem 200 passos por volta. Com **micropassos** (16×), o driver modula as correntes de fase em quadratura, $i_A = I\cos\theta$, $i_B = I\sin\theta$, produzindo 3200 microposições/volta.

**Limitação crítica:** em malha aberta, se o torque de carga excede o torque de retenção, o motor **perde passos silenciosamente** e a posição comandada diverge da real. Em robô de campo isso é inaceitável em juntas de carga.

**Exemplo real:** usados apenas no **carro linear que ajusta a altura do tronco** e no **posicionador da cabeça/LiDAR** — cargas previsíveis, baixa velocidade, e ambos com sensor de fim de curso para homing e encoder magnético de verificação. Um motor NEMA 23 com 1,2 N·m aciona um fuso de 5 mm/volta: $F = 2\pi\eta\tau/p = 2\pi \times 0{,}9 \times 1{,}2 / 0{,}005 \approx 1357\ \text{N}$ de empuxo, suficiente para levantar o tronco de 60 kg.

### 2.4 BLDC com FOC — os atuadores principais

Para as juntas de carga, a escolha correta é **BLDC/PMSM com controle vetorial (FOC)**.

**Eletromagnetismo e matemática:** as três correntes de fase $i_a, i_b, i_c$ são levadas por duas transformadas a um referencial girante solidário ao rotor:

- **Clarke** ($abc \rightarrow \alpha\beta$, estacionário, 2 eixos)
- **Park** ($\alpha\beta \rightarrow dq$, girante com o ângulo elétrico $\theta_e$)

No referencial $dq$, $i_q$ produz torque ($\tau = \frac{3}{2}p\,\lambda_m\,i_q$) e $i_d$ produz apenas fluxo (mantido em zero abaixo da velocidade-base). O resultado: **o motor AC trifásico passa a se comportar como um motor DC ideal**, com dois PIs independentes.

**Exemplo real:** o joelho (ou, na v1, a junta de elevação do tronco) usa um driver FOC (ODrive, Moteus, SimpleFOC em STM32) rodando a malha de corrente a 20 kHz e a malha de torque a 8 kHz. O MCU de junta envia `torque_setpoint` por CAN a 1 kHz. Quando o controlador de equilíbrio em D1 detecta deslocamento do ZMP, ele modula diretamente o torque comandado — algo impossível com servo de hobby.

### 2.5 Circuitos de potência, EMI e aterramento

Esta é a parte que mais causa falhas misteriosas em robôs amadores.

- **Ponte H / inversor trifásico:** MOSFETs em meia-ponte com *gate drivers* isolados; **tempo morto** (dead-time, 200–800 ns) obrigatório entre o desligamento do MOSFET superior e a ligação do inferior, sob pena de *shoot-through* e destruição.
- **Diodos de roda-livre / recirculação:** o motor é indutivo; ao cortar a corrente, $V_L = -L\,di/dt$ gera picos de centenas de volts. Diodos Schottky (ou os diodos de corpo do MOSFET, se dimensionados) drenam essa energia.
- **Medição de corrente:** shunt de baixo valor (1–5 mΩ) + amplificador de instrumentação (INA240) ou sensor Hall isolado (ACS724). Essa medida fecha a malha de torque **e** funciona como detecção de colisão.
- **EMI:** o PWM de 20 kHz com bordas de 50 ns tem harmônicos até dezenas de MHz. Mitigações obrigatórias: cabos de potência **trançados**, separados fisicamente dos cabos de sinal; blindagem aterrada em **um só ponto**; capacitores de desacoplamento próximos a cada CI; ferrite bead nas linhas de alimentação dos sensores; **opto-isoladores ou isoladores digitais** entre o domínio lógico e o domínio de potência.
- **Terra:** uma única estrela de terra. Terra de potência e terra de lógica unidos em um ponto só, junto ao negativo da bateria. Loops de terra são a causa nº 1 de IMU ruidosa e de encoder que "pula".

**Exemplo real:** em um teste de campo, o LiDAR começou a retornar pontos fantasma sempre que os motores de tração aceleravam. Causa: o cabo USB do LiDAR corria paralelo aos cabos de potência por 40 cm. Solução: reroteamento perpendicular + ferrite + blindagem. Nenhuma linha de código resolveria isso.

---

## 3. Sensores

### 3.1 Câmera RGB e RGB-D

**Física óptica:** sensor CMOS com matriz de Bayer; cada fotossítio integra fótons gerando carga proporcional ao fluxo luminoso × tempo de exposição. Modelo pinhole com matriz intrínseca:

$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}, \qquad \tilde{p} = K\,[R\,|\,t]\,\tilde{P}_{\text{mundo}}$$

Três tecnologias de profundidade:

1. **Estéreo passivo:** triangulação. $Z = \dfrac{f \cdot B}{d}$, onde $B$ é a linha de base e $d$ a disparidade em pixels. O **erro cresce com $Z^2$**: $\sigma_Z = \dfrac{Z^2}{fB}\sigma_d$. Com $f=700$ px, $B=0{,}05$ m e $\sigma_d = 0{,}2$ px, a 1 m o erro é ~6 mm; a 4 m, ~91 mm.
2. **Luz estruturada (IR):** projeta padrão conhecido. Excelente em interiores, **inutilizável sob sol direto** — fator decisivo para uso agrícola a céu aberto.
3. **Time-of-Flight:** mede desvio de fase da luz modulada. Sofre com *multipath* e superfícies molhadas.

**Global shutter vs rolling shutter:** um robô que anda ou vibra **precisa de global shutter**. Com rolling shutter, cada linha é exposta em instante diferente, e a imagem de uma cena durante o passo fica cisalhada, o que destrói odometria visual.

**Exemplo real:** uma RealSense D435 (estéreo ativo, global shutter) montada na cabeça publica `/camera/color/image_raw` e `/camera/depth/color/points` em ROS 2 a 30 Hz. O nó `tomato_detector` roda um YOLO quantizado na GPU do Jetson, obtém a caixa 2D do tomate, projeta o centro na nuvem de pontos e publica a pose 3D em `/perception/detected_fruits` no frame `camera_link`. O `tf2` transforma para `base_link`, e daí para `map`.

### 3.2 LiDAR

**Física:** $d = \dfrac{c\,\Delta t}{2}$. Para resolução de 1 cm é preciso medir 67 ps — por isso LiDARs baratos usam modulação e medem **fase**, não tempo direto.

- **2D rotativo** (RPLIDAR, ~10 Hz, 360°): barato, ótimo para SLAM 2D e desvio de obstáculos na base.
- **3D multicanal** (Livox, Ouster, Velodyne): nuvem densa, caro, essencial para terreno irregular.
- **Comprimento de onda:** 905 nm (barato, limitado por segurança ocular Classe 1) vs 1550 nm (permite mais potência, penetra melhor névoa, muito mais caro).
- **Limitação em campo:** poeira de canteiro de obra e névoa de pulverização em estufa geram retornos espúrios. Mitigação: usar o **último retorno** (last-return) em vez do primeiro e filtrar estatisticamente outliers (Statistical Outlier Removal do PCL).

**Exemplo real:** LiDAR 2D na cintura alimenta `slam_toolbox`, que constrói o mapa de ocupação da estufa e publica a transformada `map → odom`. Sem ele, o robô não sabe que "fileira 3" corresponde à região $x \in [12{,}0;\,16{,}5]$, $y \in [3{,}1;\,3{,}9]$ do mapa — é o que dá referente espacial ao símbolo `fileira_3` da ontologia.

### 3.3 IMU

**Física (MEMS):**
- *Acelerômetro:* massa de prova suspensa por molas de silício; a aceleração desloca a massa e altera a capacitância diferencial de pentes interdigitados. Mede **aceleração própria**, ou seja, $a_{\text{medida}} = a_{\text{real}} - g$ — parado, lê 9,81 m/s² para cima.
- *Giroscópio:* estrutura vibrante; a rotação gera **força de Coriolis** $\vec{F} = -2m\,\vec{\Omega}\times\vec{v}$, perpendicular à vibração, detectada capacitivamente.
- *Magnetômetro:* efeito Hall ou magnetorresistência anisotrópica.

**Erros que dominam o projeto:** bias, *bias instability* (caracterizada pela **variância de Allan**), ruído branco de velocidade angular (ARW) e fator de escala. Integrar um giroscópio com bias de 0,01°/s produz 36° de erro em uma hora. Por isso **IMU nunca é usada sozinha**.

**Fusão:** filtro complementar (barato), Madgwick/Mahony (bom, ~O(n)), ou EKF/UKF (`robot_localization`) fundindo IMU + odometria de roda + odometria do LiDAR.

**Exemplo real:** IMU de 9 eixos no tronco a 400 Hz. O magnetômetro é *desativado* dentro da estufa porque a estrutura metálica e os próprios motores distorcem o campo local em dezenas de µT; a orientação em guinada vem do SLAM. No canteiro de obra, o mesmo problema aparece perto de armadura de aço. Esse é um exemplo de decisão puramente eletromagnética afetando a pilha de software.

### 3.4 Encoders e sensores de junta

- **Incremental em quadratura:** dois canais A/B defasados 90°; a direção vem da ordem das bordas. Resolução ×4 por decodificação de quadratura. Precisa de *homing* a cada boot.
- **Absoluto magnético** (AS5047, MT6701): um ímã diametralmente magnetizado no eixo, lido por matriz de sensores Hall, dá ângulo absoluto de 14 bits sem homing. **Escolha preferida** para um robô que pode perder energia no meio da tarefa.
- **Leitura:** o timer em modo encoder do STM32 conta bordas em hardware, sem carga de CPU — indispensável a 1 kHz com 20 juntas.

### 3.5 Microfones

Array de 2–4 MEMS com saída **PDM** (densidade de pulsos, 1 bit a ~3 MHz), decimada por filtro CIC para PCM 16 kHz. Com dois microfones separados por $d$, a diferença de tempo de chegada dá a direção: $\theta = \arcsin\left(\frac{c\,\Delta t}{d}\right)$.

**Exemplo real:** *wake word* "Ei, robô" detectada localmente por um modelo TinyML no MCU (ver §6), o que evita streaming contínuo de áudio para a nuvem — economia de energia, banda e privacidade. Um segundo modelo classifica sons de alerta (buzina de retroescavadeira, grito) e dispara parada de segurança via tópico `/safety/audio_alert` com QoS de alta prioridade.

### 3.6 Temperatura, umidade e sensores de contexto

Sensor capacitivo de umidade (SHT4x, I²C): um polímero higroscópico entre eletrodos muda a permissividade elétrica $\varepsilon_r$ com a absorção de água, alterando a capacitância. Temperatura por sensor de banda proibida no silício.

**Dois papéis distintos no robô:**
1. **Saúde do sistema:** temperatura dos enrolamentos e dos drivers. A resistência do cobre sobe ~0,39%/°C; um motor a 100 °C tem 30% mais perda joule que a 20 °C. O firmware reduz o limite de corrente progressivamente (*thermal derating*) acima de 80 °C.
2. **Contexto agronômico:** umidade relativa e temperatura da estufa entram como **fatos da base de conhecimento**, não apenas como telemetria. Se UR > 90% e T < 15 °C, uma regra simbólica bloqueia a colheita (risco de botrytis na ferida do pedúnculo) e o robô informa o operador pelo chatbot. Esse é um exemplo concreto de sensor físico alimentando o raciocinador simbólico.

---

## 4. Computação embarcada

### 4.1 Microcontroladores (Arduino / STM32 / ESP32)

**Papel:** tudo que precisa de determinismo de microssegundos.

- **Arduino Uno/Mega (AVR, 16 MHz):** adequado apenas para aprendizado e I/O simples. Sem FPU, sem DMA decente.
- **STM32 (Cortex-M4F/M7, 80–480 MHz, FPU):** a escolha de produção. Timers avançados com PWM complementar e dead-time em hardware (feitos para inversores trifásicos), ADC injetado sincronizado ao PWM (amostra a corrente exatamente no meio do período — crítico para FOC), CAN nativo, DMA.
- **ESP32:** Wi-Fi/BLE integrados; ótimo para nós sensores sem fio, ruim para controle de junta (o stack Wi-Fi introduz jitter).

**Programação em tempo real:** a malha de junta roda em uma interrupção de timer a 1 kHz. Dentro dela: sem `malloc`, sem `printf`, sem loop de duração variável. O PID discretizado (forma de velocidade, com anti-windup por *clamping* e derivada filtrada sobre a medição, não sobre o erro):

```c
// ISR @ 1 kHz — determinística, ~12 µs de execução
void TIM2_IRQHandler(void) {
    float pos   = encoder_read();              // timer em modo quadratura, custo O(1)
    float err   = setpoint - pos;
    integral   += err * DT;
    integral    = clampf(integral, -I_MAX, I_MAX);      // anti-windup
    float deriv = -(pos - pos_prev) / DT;               // derivada na medição
    deriv       = alpha*deriv + (1.0f-alpha)*deriv_prev; // filtro passa-baixa
    float u     = KP*err + KI*integral + KD*deriv + gravity_ff(pos);
    set_torque(clampf(u, -TAU_MAX, TAU_MAX));
    watchdog_kick();
}
```

O termo `gravity_ff(pos)` é **feedforward de gravidade**: $\tau_g = m g l \cos(\theta)$. Compensar analiticamente o que a física prevê reduz o erro de regime em uma ordem de grandeza sem aumentar ganhos.

**Exemplo real:** 6 STM32G4, um por membro/segmento, cada um controlando 3–4 juntas, todos em um barramento CAN a 1 Mbit/s com o Jetson. Cada MCU roda **micro-ROS** (cliente DDS-XRCE), aparecendo na rede ROS 2 como nó nativo que publica `/joint_states` e assina `/joint_commands`.

### 4.2 Raspberry Pi / computador de bordo

**Realidade técnica:** o Raspberry Pi 5 (Cortex-A76 quad, 4–16 GB) é excelente como **coordenador**, mas insuficiente como **cérebro de percepção** de um humanoide. Não tem GPU para inferência de redes neurais de visão, e o Linux padrão não é tempo real.

**Divisão recomendada:**
- **Raspberry Pi 5** → nó de coordenação, interface de rede, chatbot, logging, ontologia, ROS 2 core. Em v0 pode fazer tudo.
- **NVIDIA Jetson Orin Nano/NX** → percepção visual, inferência de detecção, eventualmente LLM local quantizado. É a peça que torna o projeto viável sem depender de nuvem.

**Tempo real no Linux:** aplicar o patch **PREEMPT_RT** (mainline desde o kernel 6.12), isolar núcleos com `isolcpus`, usar `SCHED_FIFO` com prioridade alta e `mlockall()` para travar páginas de memória. Mesmo assim, latência de pior caso de dezenas de microssegundos — bom para D2, insuficiente para D0. Daí a divisão SBC/MCU.

**Exemplo real:** ao receber "colha os tomates maduros da fileira 3", o Pi hospeda o nó do chatbot e o raciocinador; o Jetson roda detecção e estimativa de pose a 20 Hz; os STM32 seguem a trajetória a 1 kHz. Se o Wi-Fi cair no meio, os STM32 continuam executando a trajetória atual e, se o heartbeat do Jetson falhar por 200 ms, o watchdog leva as juntas para posição segura.

---

## 5. Middleware: ROS 2

### 5.1 Distribuição e conceito

**ROS 2 Lyrical Luth** (maio de 2026) é a distribuição LTS atual, pareada com Ubuntu 26.04 e suportada até ~2031; **Jazzy Jalisco** (2024, Ubuntu 24.04) permanece suportada até 2029 e tem ecossistema de pacotes de terceiros mais maduro. Para um projeto que começa agora e dura 2 anos, **Jazzy é a escolha conservadora; Lyrical é a escolha de longo prazo.**

ROS 2 não é um sistema operacional — é um conjunto de bibliotecas (`rclcpp`, `rclpy`), ferramentas de build (`colcon`) e um modelo de comunicação em grafo, rodando sobre Linux.

### 5.2 ROS 2 Topics

Publish/subscribe assíncrono, muitos-para-muitos, sem resposta. É o canal de **fluxo de dados contínuo**.

**Exemplo real no robô:**

| Tópico | Tipo | Taxa | QoS |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | Best-effort, depth 1 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 15 Hz | Best-effort, depth 1 |
| `/imu/data_raw` | `sensor_msgs/Imu` | 400 Hz | Best-effort |
| `/tf` | `tf2_msgs/TFMessage` | 100 Hz | Reliable, volatile |
| `/safety/estop_state` | `std_msgs/Bool` | 20 Hz | **Reliable, transient_local, deadline 100 ms** |
| `/perception/detected_fruits` | `vision_msgs/Detection3DArray` | 10 Hz | Reliable |

Note que a escolha de QoS é uma decisão de engenharia, não um detalhe: nuvem de pontos com *reliable* sobre Wi-Fi congestiona o enlace e atrasa tudo; e-stop com *best-effort* mata alguém.

### 5.3 ROS 2 Services e Actions

- **Service:** requisição/resposta síncrona, rápida. Ex.: `/get_ontology_fact`, `/calibrate_hand`.
- **Action:** o mecanismo certo para **tarefas longas, com feedback e cancelamento**. Tem `goal`, `feedback` periódico, `result` final e `cancel`.

**Exemplo real — definição de action do projeto:**

```
# HarvestRow.action
# --- Goal ---
string    row_id              # "fileira_3"
string    target_class        # "tomate_maduro"
string    deposit_container   # "caixa_azul"
float32   min_ripeness        # 0.80 — limiar vindo do raciocinador
---
# --- Result ---
uint32    fruits_harvested
uint32    fruits_skipped
string[]  failure_reasons
---
# --- Feedback ---
uint32    current_index
float32   percent_complete
geometry_msgs/Pose current_target
string    status_message      # "aproximando", "agarrando", "depositando"
```

O agente executor é o **action client**; o controlador de manipulação é o **action server**. Quando o operador digita "pare" no chatbot, o cliente chama `cancel_goal()`, o servidor interrompe a trajetória com desaceleração controlada e devolve `CANCELED` — semântica que um simples tópico não oferece.

### 5.4 DDS e a camada de transporte

Por baixo do ROS 2 está o **DDS** (Data Distribution Service), com protocolo de fio **RTPS**. Implementações: Fast DDS (padrão), Cyclone DDS (mais previsível), **Zenoh** (`rmw_zenoh`, recomendado para enlaces sem fio e multi-robô, pois substitui a descoberta por multicast — que funciona mal em Wi-Fi — por um roteador).

**Políticas de QoS relevantes:**
- *Reliability*: `RELIABLE` (com retransmissão) vs `BEST_EFFORT`.
- *Durability*: `TRANSIENT_LOCAL` entrega a última mensagem a assinantes tardios — essencial para mapas e estados de configuração.
- *Deadline* e *Liveliness*: contratos temporais. Se `/joint_states` não chegar dentro do deadline, um callback dispara e o supervisor de segurança age.
- *History/Depth*: profundidade 1 para sensores (só interessa o dado mais novo).

**Exemplo real:** na estufa, o robô opera em Wi-Fi 5 GHz. Descoberta DDS por multicast falha porque o AP filtra multicast. Solução adotada: `rmw_zenoh` com roteador no Pi, ou Fast DDS com *discovery server* estático. Sobre 4G/5G (campo aberto), **nenhum tráfego de controle atravessa a rede celular** — apenas telemetria e o canal do chatbot, por MQTT/WebSocket sobre TLS. Controle por rede celular é receita de acidente.

### 5.5 `tf2` e a árvore de transformadas

`tf2` mantém a árvore de referenciais com carimbo de tempo e interpolação:

```
map → odom → base_link → torso_link → shoulder_link → ... → gripper_tip
                       ↘ camera_link
                       ↘ lidar_link
```

**Exemplo real:** o tomate é detectado em `camera_link` na pose $(0{,}12,\ -0{,}04,\ 0{,}58)$. O `tf2` compõe as transformadas homogêneas $T^{\text{base}}_{\text{camera}}$ e leva a pose para `base_link`, onde a cinemática inversa consegue trabalhar. Sem `tf2` com timestamps corretos, a 30 Hz e com o robô se movendo, um atraso de 100 ms significa erro de posição de vários centímetros — e a garra fecha no ar.

### 5.6 `ros2_control`

Framework padrão para controladores: define `hardware_interface` (a ponte para os MCUs por CAN/micro-ROS), `controller_manager` e controladores plugáveis (`joint_trajectory_controller`, `effort_controllers`, `diff_drive_controller`). Usar isso em vez de escrever drivers ad-hoc economiza meses.

---

## 6. Aprendizado embarcado: TinyML, TFLite Micro / LiteRT

### 6.1 O que TinyML é e o que não é

**TinyML = inferência de redes neurais quantizadas em microcontroladores**, tipicamente com dezenas a centenas de kilobytes de RAM, sem sistema operacional e **sem alocação dinâmica**. O runtime padrão é o **TensorFlow Lite for Microcontrollers**, rebatizado **LiteRT for Microcontrollers** (a renomeação de 2024 causou — e ainda causa — confusão de nomenclatura no ecossistema; o repositório `tflite-micro` continua ativo).

**Ponto importante sobre o esquema original:** "treinamento local com TensorFlow" precisa de qualificação. Treinamento **não acontece no microcontrolador**. O fluxo real é:

```
Coleta de dados no robô → treino em PC/Jetson (TensorFlow/Keras)
    → quantização int8 (post-training ou QAT)
    → conversão .tflite → array C (xxd -i)
    → compilação junto ao firmware → inferência no MCU
```

Se você quiser adaptação *no robô* (e faz sentido: cada fazenda tem tomates diferentes), o lugar para isso é o **Jetson**, com *fine-tuning* das últimas camadas, não o MCU.

### 6.2 Quantização — a matemática

A conversão de float32 para int8 usa mapeamento afim:

$$r = S\,(q - Z)$$

onde $r$ é o real, $q$ o inteiro de 8 bits, $S$ a escala (float) e $Z$ o zero-point (int). A multiplicação de matrizes acontece inteiramente em inteiros com acumulador de 32 bits, e o requantize usa multiplicador de ponto fixo. Resultado: **4× menos memória e 2–4× mais velocidade** em Cortex-M com instruções SIMD (via CMSIS-NN), com perda de acurácia tipicamente < 1% se houver dataset de calibração representativo.

### 6.3 Aplicações concretas no robô

| Modelo | Onde roda | Tamanho | Função |
|---|---|---|---|
| Keyword spotting (CNN 1D sobre MFCC) | STM32 da cabeça | ~40 KB | Detecta "ei, robô" e "parar"; evita streaming de áudio |
| Anomalia de vibração (autoencoder sobre FFT do acelerômetro) | STM32 de cada junta | ~25 KB | Manutenção preditiva: erro de reconstrução alto = rolamento degradando |
| Classificador de contato (MLP sobre corrente + encoder) | STM32 da mão | ~8 KB | Distingue "tocou folha", "tocou fruto", "colidiu com estrutura" em < 5 ms |
| Detecção de tombamento (LSTM pequena sobre IMU) | STM32 do tronco | ~30 KB | Dispara postura de proteção antes que o SBC perceba |

**Exemplo real detalhado — o classificador de contato:** o LLM e o planejador levam ~800 ms para reagir. A malha de posição no MCU leva 1 ms. O modelo TinyML preenche a lacuna: ele roda **dentro** da ISR de 1 kHz (inferência em ~3 ms, portanto em uma tarefa de 200 Hz), e ao classificar "colidiu com estrutura" com confiança > 0,9, o próprio MCU aborta o movimento e publica o evento. O raciocinador simbólico *depois* decide o que fazer — mas a proteção física já aconteceu. **Isto é a essência do IAoT: inteligência colocada onde a latência exige, não onde é mais conveniente programar.**

---

## 7. Percepção

### 7.1 Pipeline

```
Imagem RGB + Profundidade + LiDAR + IMU + Odometria
   ↓ retificação, filtragem (voxel grid, SOR)
   ↓ Detecção 2D (YOLO / RT-DETR quantizado, TensorRT no Jetson)
   ↓ Segmentação de instância (para pegar o contorno do fruto)
   ↓ Fusão RGB-D: projeção da máscara na nuvem → cluster 3D
   ↓ Estimativa de pose (PCA do cluster / ajuste de superquádrica)
   ↓ Filtro de rastreamento (associação de dados + Kalman por objeto)
   ↓ /perception/detected_fruits (Detection3DArray com covariância)
```

### 7.2 SLAM e localização

- `slam_toolbox` (SLAM 2D baseado em grafo de poses; *scan matching* por correlação, otimização com Ceres) para o mapa da estufa.
- `rtabmap_ros` para SLAM RGB-D/visual quando o ambiente é 3D e irregular (canteiro de obra).
- `robot_localization` (EKF/UKF) fundindo IMU + odometria de rodas + odometria visual, publicando `odom → base_link`.
- **Fechamento de laço:** detecta que o robô voltou a um lugar já visitado e corrige a deriva acumulada globalmente.

**Matemática do EKF:** predição $\hat{x}_k^- = f(\hat{x}_{k-1}, u_k)$, $P_k^- = F P_{k-1} F^\top + Q$; correção com ganho $K_k = P_k^- H^\top (H P_k^- H^\top + R)^{-1}$. Ajustar $Q$ e $R$ (confiança no modelo vs nos sensores) é onde se gasta o tempo real de engenharia.

**Exemplo real:** "fileira 3" é uma *pose anotada no mapa*. O Nav2 recebe a pose como objetivo, planeja global (A*/Smac) e local (DWB/MPPI), e navega evitando obstáculos dinâmicos detectados pela nuvem de pontos projetada em `costmap_2d`. Só quando o robô chega, o braço entra em ação.

---

## 8. Cinemática, dinâmica e geração de movimento

### 8.1 Cinemática

**Direta:** produto de exponenciais (formulação de Lynch & Park, mais limpa que Denavit-Hartenberg):

$$T(\theta) = e^{[\mathcal{S}_1]\theta_1} e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} M$$

**Jacobiano:** relaciona velocidades de junta e do efetuador, $V = J(\theta)\,\dot{\theta}$. Também relaciona forças: $\tau = J^\top \mathcal{F}$ — é assim que se transforma "aplicar 9 N na garra" em torques de junta.

**Inversa (IK):** para um braço de 6–7 GDL, resolve-se numericamente por **mínimos quadrados amortecidos** (Levenberg-Marquardt), que é estável perto de singularidades:

$$\Delta\theta = J^\top (J J^\top + \lambda^2 I)^{-1}\,e$$

Bibliotecas: KDL, TracIK, `pinocchio`, ou o `MoveIt 2` que embrulha tudo com planejamento (OMPL/RRT-Connect) e verificação de colisão (FCL).

### 8.2 Dinâmica

$$M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + g(q) + \tau_{\text{atrito}} = \tau + J^\top F_{\text{ext}}$$

- $M(q)$: matriz de inércia (simétrica, positiva-definida)
- $C(q,\dot{q})$: termos de Coriolis e centrífugos
- $g(q)$: gravidade

Calculada eficientemente pelo **RNEA** (Recursive Newton-Euler, $O(n)$) — implementado em **Pinocchio**, que é a referência atual em C++ com bindings Python.

**Exemplo real:** o controlador de torque calculado (*computed torque*) usa esse modelo como feedforward: $\tau = M(q)(\ddot{q}_d + K_d\dot{e} + K_p e) + C\dot{q} + g$. O robô levanta um saco de 25 kg sem "cair" na trajetória porque o modelo *prevê* o torque necessário em vez de reagir ao erro.

### 8.3 Equilíbrio (v2, bípede)

- **LIPM (Linear Inverted Pendulum Model):** $\ddot{x} = \frac{g}{z_c}(x - p_x)$, onde $p_x$ é o ZMP.
- **ZMP (Zero Moment Point):** ponto no solo onde o momento horizontal das forças de reação é nulo. Enquanto o ZMP estiver dentro do polígono de suporte, o robô não tomba.
- **Capture Point:** $\xi = x + \dot{x}\sqrt{z_c/g}$ — onde pisar para parar. Base dos controladores modernos de recuperação de equilíbrio.
- **Whole-Body Control por QP:** resolve a cada ciclo um problema quadrático minimizando erro de tarefas sujeito a restrições (limites de torque, atrito de contato, ZMP no polígono).

### 8.4 Scripts Blender para movimentos — o que funciona e o que não funciona

**O que Blender faz bem no projeto:**
1. **Autoria e pré-visualização cinemática.** Modelar a armature (esqueleto) com a mesma topologia do URDF, animar por keyframes e IK, e exportar as trajetórias de junta.
2. **Geração de dados sintéticos.** Renderizar milhares de imagens de tomates em condições de luz e oclusão variadas, com anotação automática perfeita, para treinar o detector. **Este é provavelmente o uso de maior valor prático.**
3. **Comunicação e documentação** com o cliente e a equipe.

**O que Blender não faz:** garantir que o movimento seja **dinamicamente viável**. Blender ignora massa, inércia, limites de torque, atrito e equilíbrio. Uma animação bonita pode exigir 400 N·m em um ombro de 150 N·m, ou deslocar o CoM para fora do polígono de suporte.

**Pipeline correto:**

```python
# Exportação de trajetória do Blender (bpy) para ROS 2
import bpy, json
arm = bpy.data.objects["RobotArmature"]
joint_map = {"Bone.Shoulder": "shoulder_pitch", "Bone.Elbow": "elbow_pitch", ...}
fps = bpy.context.scene.render.fps
traj = []
for f in range(bpy.context.scene.frame_start, bpy.context.scene.frame_end + 1):
    bpy.context.scene.frame_set(f)
    positions = [arm.pose.bones[b].rotation_euler.y for b in joint_map]
    traj.append({"t": f / fps, "positions": positions})
json.dump({"joint_names": list(joint_map.values()), "points": traj},
          open("/tmp/motion_pick.json", "w"))
```

E então, **obrigatoriamente**, três etapas de validação antes de qualquer hardware:

1. **Retargeting** para o URDF real (proporções e limites de junta de verdade).
2. **Verificação de viabilidade** em Pinocchio: dinâmica inversa sobre a trajetória → torque exigido em cada junta em cada instante; rejeitar se exceder o envelope do atuador.
3. **Simulação física** (Gazebo/MuJoCo/Isaac Sim) com colisão e contato.

Só então a trajetória vira `trajectory_msgs/JointTrajectory` e é enviada ao `joint_trajectory_controller`, que faz interpolação cúbica ou quíntica entre os pontos.

**Exemplo real:** o gesto "aproximar-se do fruto por baixo, girar o pulso 30° e destacar pelo pedúnculo" foi desenhado em Blender por um animador em duas horas — algo que levaria dias para parametrizar à mão. A verificação em Pinocchio mostrou pico de 78 N·m no cotovelo (limite: 60). O movimento foi re-temporizado (mesma geometria, 1,6× mais lento), o pico caiu para 41 N·m, e aí sim foi ao robô.

### 8.5 NumPy e Eigen — onde cada um vive

| Camada | Ferramenta | Por quê |
|---|---|---|
| Prototipagem, análise offline, treino | **NumPy/SciPy** (Python) | Velocidade de desenvolvimento; núcleo em C com BLAS/LAPACK; vetorização |
| Nós ROS 2 de percepção e planejamento (D2) | NumPy ou Eigen | NumPy aceitável a 10–30 Hz se vetorizado |
| Malha de controle (D1/D0) | **Eigen (C++)** ou C puro | Sem GIL, sem GC, sem alocação; templates expandidos em tempo de compilação |

**Detalhe técnico que importa:** NumPy é *row-major* (C order) por padrão, Eigen é *column-major* por padrão. Ao trocar matrizes entre Python e C++ via `pybind11`, ignorar isso produz matrizes transpostas silenciosamente — bug clássico e caro de achar.

**Eigen em tempo real:** usar tipos de tamanho fixo (`Eigen::Matrix<double,6,6>`) que vão para a pilha, não para o heap. Definir `EIGEN_RUNTIME_NO_MALLOC` e chamar `Eigen::internal::set_is_malloc_allowed(false)` dentro do laço de controle: qualquer alocação acidental vira um assert em vez de um *deadline miss* no campo.

```cpp
// Jacobiano + IK amortecida, sem alocação dinâmica, ~15 µs
Eigen::Matrix<double,6,7> J = computeJacobian(q);
Eigen::Matrix<double,6,6> A = J*J.transpose()
                            + lambda*lambda*Eigen::Matrix<double,6,6>::Identity();
Eigen::Matrix<double,7,1> dq = J.transpose() * A.ldlt().solve(error);
```

---

## 9. Camada cognitiva neurossimbólica

Esta é a parte mais original do esquema proposto, e a que precisa de maior rigor arquitetural. A ideia central é sólida:

> **Redes neurais são boas em perceber e interpretar, e ruins em garantir. Lógica simbólica é boa em garantir, e ruim em perceber. Um robô que opera perto de pessoas e de patrimônio precisa das duas.**

### 9.1 Interação com o cliente: chatbot

Arquitetura: navegador → WebSocket (TLS) → serviço no Raspberry Pi → nó ROS 2. Alternativas: `rosbridge_suite` (JSON sobre WebSocket, prático) ou serviço próprio em FastAPI. Autenticação obrigatória: comandar um robô de 80 kg não pode ser anônimo.

**Requisito de design:** o chatbot **nunca** é o caminho de segurança. Parada de emergência é botão físico, canal duplo, normalmente fechado, cortando a alimentação dos drivers por relé de segurança.

### 9.2 LLM: system prompt, user prompt e saída estruturada

O LLM tem exatamente **uma** função nesta arquitetura: converter linguagem natural ambígua em uma **estrutura simbólica candidata**. Ele não planeja, não move, não decide sobre segurança.

**System prompt (esboço real):**

```
Você é o interpretador linguístico de um robô agrícola/construção.
Sua ÚNICA saída é um objeto JSON válido, sem texto adicional.

Vocabulário de ações permitido (fechado):
  navigate_to, harvest, inspect, transport, place, dig, lift, report, stop

Vocabulário de objetos: apenas classes da ontologia fornecida abaixo.
Você NÃO pode inventar ações nem classes.
Se o comando for ambíguo, incompleto ou fora do vocabulário,
retorne {"status":"clarify","question":"<pergunta objetiva em PT-BR>"}.

Esquema de saída:
{"status":"ok",
 "intent":"<ação>",
 "args":{...},
 "constraints":[...],
 "confidence":<0..1>}

Ontologia disponível (classes): {{ONTOLOGY_CLASSES}}
Estado atual do robô: {{ROBOT_STATE}}
Locais conhecidos no mapa: {{KNOWN_LOCATIONS}}
```

**User prompt:** `"Colhe os tomate maduro da fileira 3 e põe na caixa azul. Não mexe nos verde."`

**Saída:**

```json
{"status": "ok",
 "intent": "harvest",
 "args": {"target_class": "Tomate", "target_state": "Maduro",
          "location": "fileira_3", "deposit": "caixa_azul"},
 "constraints": [{"type": "exclude", "target_state": "Verde"}],
 "confidence": 0.91}
```

Duas opções de hospedagem: **LLM local** (modelo 7–8B quantizado em 4 bits no Jetson, ~2–4 s de latência, funciona offline — decisivo em fazenda sem sinal) ou **LLM em nuvem** (melhor qualidade, exige conectividade). A recomendação é **local com fallback para nuvem**, nunca o contrário.

### 9.3 Alinhador Semântico

**Problema:** o LLM pode devolver `"tomate"`, `"tomateiro"`, `"fruto vermelho"`, `"Solanum lycopersicum"`. A ontologia conhece apenas `:Tomate`. O alinhador é a ponte.

**Como funciona:** modelo de *embeddings* de sentenças (Sentence-BERT multilíngue) mapeia cada rótulo para $\mathbb{R}^{384}$. Cada conceito da ontologia tem seu vetor pré-computado. O alinhamento é o vizinho mais próximo por similaridade cosseno:

$$\text{sim}(a,b) = \frac{\vec{a}\cdot\vec{b}}{\|\vec{a}\|\,\|\vec{b}\|}$$

Com política de limiares: $> 0{,}85$ aceita; $0{,}60$–$0{,}85$ propõe e pede confirmação ao usuário; $< 0{,}60$ rejeita e pergunta.

**Exemplo real:** `"fruto vermelho"` → similaridade 0,79 com `:Tomate` e 0,74 com `:Pimentao`. Como ficou na faixa ambígua **e** há duas hipóteses próximas, o robô responde pelo chatbot: *"Você quer dizer tomate ou pimentão? Na fileira 3 tenho tomates cadastrados."* A pergunta é gerada com apoio da ontologia, não inventada pelo LLM.

### 9.4 Ontologia Cadastrada

Base de conhecimento formal em **OWL 2 / RDF**, editada em Protégé, consultada por SPARQL, com raciocinador (HermiT, Pellet, ou `owlready2` em Python).

```turtle
@prefix :   <http://robo.fazenda/onto#> .
@prefix owl: <http://www.w3.org/2002/07/owl#> .

:Tomate          a owl:Class ; rdfs:subClassOf :Fruto .
:Maduro          a owl:Class ; rdfs:subClassOf :EstadoMaturacao .
:Colher          a owl:Class ; rdfs:subClassOf :Acao .

:GarraMacia      a owl:Class ; rdfs:subClassOf :Efetuador .
:GarraMacia      :forcaMaxima "15.0"^^xsd:float .

:Colher :requerEfetuador :GarraMacia ;
        :requerEstado    :Maduro ;
        :forcaSegura     "9.0"^^xsd:float ;
        :duracaoEstimada "12.0"^^xsd:float .

:fileira_3 a :LocalCultivo ;
           :contemCultura :Tomate ;
           :poseMapa "12.4 3.5 0.0"^^xsd:string ;
           :comprimento "22.0"^^xsd:float .

# Regra de segurança expressa na ontologia
:Colher :proibidoSe :UmidadeAlta .
```

**Exemplo real de uso:** o planejador não precisa "saber" que colher tomate exige garra macia com 9 N. Ele **consulta**:

```sparql
SELECT ?efetuador ?forca WHERE {
  :Colher :requerEfetuador ?efetuador ;
          :forcaSegura ?forca .
}
```

Quando a fazenda passar a cultivar morango, ninguém reescreve código: adiciona-se `:Morango` com `:forcaSegura "4.0"` e o sistema inteiro se adapta. **É esse desacoplamento entre conhecimento e código que justifica a ontologia.**

### 9.5 Semantic Grounding

O **problema do grounding de símbolos** (Harnad, 1990) e o **problema de ancoragem** (Coradeschi & Saffiotti, 2003): o símbolo `tomate_maduro_07` precisa estar amarrado, de forma verificável e persistente, a um conjunto de pixels, a um cluster 3D e a uma pose no mapa.

**Estrutura de uma âncora:**

```python
@dataclass
class Anchor:
    symbol_id:    str                 # "tomate_maduro_07"
    onto_class:   str                 # ":Tomate"
    properties:   dict                # {"maturacao": 0.87, "diametro_m": 0.061}
    pose_map:     np.ndarray          # 4x4, frame "map"
    covariance:   np.ndarray          # 6x6 — incerteza importa!
    percept_ids:  list[int]           # rastreabilidade aos percepts brutos
    last_seen:    Time
    confidence:   float
```

Três operações canônicas:
- **Find:** dado o símbolo, encontrar o percepto (o robô procura o tomate).
- **Track:** manter a âncora viva entre quadros, mesmo com oclusão temporária.
- **Reacquire:** reencontrar depois de o objeto sair de vista (o robô virou e voltou).

**Exemplo real:** o detector encontra 23 frutos na fileira 3. O grounding cria 23 âncoras, cada uma com maturação estimada por um classificador (índice de cor no espaço HSV + textura). Dessas, 14 têm `maturacao > 0.80`. **Somente essas 14 viram constantes lógicas no programa DeepProbLog.** Isso reduz o espaço de raciocínio simbólico de "tudo o que existe" para "14 objetos ancorados" — que é o que torna raciocínio lógico computacionalmente viável em robótica real.

### 9.6 DeepProbLog — os dois motores de raciocínio

**O que é:** DeepProbLog (Manhaeve et al., NeurIPS 2018; AIJ 2021) estende o ProbLog — programação lógica probabilística — com o **predicado neural**: um fato probabilístico cuja probabilidade é a **saída de uma rede neural**.

**Como funciona por dentro:**
1. A consulta lógica é resolvida por SLD-resolution, produzindo a fórmula proposicional de todas as provas.
2. Essa fórmula é compilada em um **circuito aritmético** (SDD / d-DNNF), o que torna a contagem de modelos ponderada (*weighted model counting*) tratável.
3. A probabilidade da consulta é avaliada percorrendo o circuito com um semianel (soma-produto).
4. Como o circuito é **diferenciável**, o gradiente da perda retropropaga até os pesos da rede neural — treino fim-a-fim de percepção e lógica juntas.

> Nota de engenharia: DeepProbLog é uma ferramenta de pesquisa (`pip install deepproblog`, requer SWI-Prolog) e inferência exata **explode combinatoriamente**. Mantenha os programas pequenos (dezenas de fatos), use as variantes aproximadas, e execute em D3 (~1 Hz), nunca em malha de controle. Trabalhos recentes (DeepLog / DeepSoftLog, KU Leuven, 2025–2026) endereçam justamente escalabilidade e modularidade — vale acompanhar.

#### Motor 1 — Raciocinador de Intenção

Decide **o que o usuário realmente quer**, combinando a saída do LLM, o histórico e o contexto.

```prolog
% Predicado neural: confiança do LLM sobre a intenção
nn(intent_net, [Utterance], I, [harvest, inspect, transport, place, stop])
    :: intent(Utterance, I).

% Fatos probabilísticos de contexto
0.85 :: season_appropriate(tomate, colheita).
0.30 :: user_typically_vague(joao).

% Regra: a intenção é aceita se o LLM concorda E o contexto é plausível
valid_intent(U, harvest) :-
    intent(U, harvest),
    mentions_crop(U, C),
    season_appropriate(C, colheita),
    robot_has_capability(harvest, C).

% Ambiguidade explícita
needs_clarification(U) :-
    valid_intent(U, I1), valid_intent(U, I2), I1 \= I2.

query(valid_intent("colhe os tomate maduro da fileira 3", harvest)).
```

**Exemplo real de saída:** $P(\text{valid\_intent}=\text{harvest}) = 0{,}88$. Acima do limiar de 0,75 → prossegue. Se estivesse em 0,55, o sistema perguntaria, em vez de agir. Crucialmente, a **razão** da recusa é inspecionável: a árvore de prova mostra que `season_appropriate` falhou.

#### Motor 2 — Raciocinador de Execução

Decide **se e como executar cada passo**, combinando percepção neural com regras físicas e de segurança.

```prolog
% Predicados neurais alimentados pelo grounding
nn(ripeness_net, [Img], R, [verde, pintando, maduro, passado]) :: ripeness(Obj, R).
nn(occlusion_net, [Cloud], O, [livre, parcial, bloqueado]) :: occlusion(Obj, O).
nn(stability_net, [ImuWin], S, [estavel, marginal, instavel]) :: balance(S).

% Conhecimento importado da ontologia (via owlready2 → fatos Prolog)
safe_force(tomate, 9.0).
max_force(garra_macia, 15.0).

% Regra de decisão de colheita
should_harvest(Obj) :-
    ripeness(Obj, maduro),
    occlusion(Obj, livre),
    reachable(Obj),
    balance(estavel).

% Regra que nunca pode ser violada — restrição dura
safe_to_grasp(Obj, F) :-
    class_of(Obj, C), safe_force(C, Fmax), F =< Fmax,
    \+ human_in_workspace.

% Recuperação: se parcialmente ocluído, tente reposicionar antes de desistir
plan_for(Obj, reposition_then_harvest) :-
    ripeness(Obj, maduro), occlusion(Obj, parcial).

query(should_harvest(tomate_maduro_07)).
```

**Exemplo real:** para `tomate_maduro_07`, $P(\text{should\_harvest}) = 0{,}93$ → executa. Para `tomate_maduro_11`, a rede de oclusão dá `parcial` com 0,7, e a regra `plan_for` dispara `reposition_then_harvest`: o robô move a base 15 cm, reobserva, e reavalia. Para `tomate_12`, $P = 0{,}41$ (maturação 0,62) → pulado, e registrado no resultado da action como `skipped: below_ripeness_threshold`.

**Por que isso é melhor que um classificador puro:** o classificador diria "colher: 0,93" sem explicar. Aqui, se o robô erra, a árvore de prova diz exatamente qual predicado estava errado — se foi a rede de maturação ou a regra de força. Isso é **auditabilidade**, e é o que permite melhorar um sistema em produção.

### 9.7 Neurosymbolic Computing — a justificativa de projeto

Os três ganhos concretos que justificam a complexidade adicional:

1. **Eficiência de dados.** Ensinar "não colha tomate verde" com redes puras exige milhares de exemplos rotulados de erro. Como regra lógica, é uma linha.
2. **Garantias duras.** Uma restrição como `\+ human_in_workspace` é **estruturalmente inviolável** — não é um peso que o gradiente pode aprender a ignorar. Isso é indispensável para certificação de segurança.
3. **Explicabilidade.** Toda decisão vem com uma árvore de prova. Um operador pode perguntar "por que você pulou aquele tomate?" e receber uma resposta verdadeira, derivada do mecanismo real de decisão.

### 9.8 Agente Executor de Comandos (planejamento)

Recebe a intenção validada e produz uma sequência executável. Duas abordagens combináveis:

**(a) Planejamento simbólico clássico — PDDL:**

```lisp
(:action grasp_fruit
  :parameters (?f - fruit ?g - gripper ?r - robot)
  :precondition (and (at ?r (near ?f)) (reachable ?f) (empty ?g)
                     (ripe ?f) (not (occluded ?f)))
  :effect (and (holding ?g ?f) (not (empty ?g)) (not (attached ?f))))
```

Resolvido por Fast Downward / POPF. Bom para tarefas longas com dependências; frágil quando o mundo muda.

**(b) Behavior Trees (`BehaviorTree.CPP`, usado pelo Nav2):** mais robustas a falhas e reativas; são a prática dominante em robótica de campo.

```
Sequence: HarvestRow
├── Fallback
│   ├── IsAtRow("fileira_3")
│   └── NavigateTo("fileira_3")          [Action ROS 2]
├── ScanRow → produz lista de âncoras
└── Retry(3)
    └── ForEach(anchor in ripe_fruits)
        └── Sequence
            ├── ReasonShouldHarvest(anchor)   [chama DeepProbLog]
            ├── Fallback
            │   ├── ApproachAndGrasp(anchor)
            │   └── RepositionBase(0.15) → ApproachAndGrasp(anchor)
            ├── VerifyGrasped                 [corrente da garra + visão]
            └── PlaceIn("caixa_azul")
```

**Exemplo real do ciclo completo, com tempos:**

```
t=0.0s   Usuário digita no chatbot
t=0.1s   WebSocket → nó ROS 2 → LLM local (Jetson)
t=2.3s   LLM retorna JSON estruturado
t=2.4s   Alinhador Semântico: "tomate"→:Tomate (0.97), "caixa azul"→:CaixaAzul (0.94)
t=2.5s   Ontologia: :Colher requer :GarraMacia, força ≤ 9 N, :fileira_3 está em (12.4, 3.5)
t=2.8s   DeepProbLog Intenção: P(harvest)=0.88 → aprovado
t=2.9s   Planejador emite Behavior Tree; action client envia HarvestRow.goal
t=3.0s   Nav2 assume; robô navega 18 m  [feedback a 1 Hz pelo chatbot]
t=41.0s  Chegada. LiDAR+RGB-D varrem a fileira
t=44.0s  Grounding cria 23 âncoras; 14 com maturação > 0.80
t=44.5s  DeepProbLog Execução avalia âncora #1 → P=0.93
t=44.6s  MoveIt 2 resolve IK, planeja trajetória livre de colisão
t=45.0s  joint_trajectory_controller executa; STM32 seguem a 1 kHz
t=47.2s  TinyML no MCU da mão classifica "contato com fruto" em 3 ms
t=47.3s  SEA reporta 8.7 N — dentro do limite ontológico de 9.0 N
t=48.1s  Pulso gira 30°, fruto destacado; VerifyGrasped confirma por corrente
t=51.0s  Depositado na caixa azul; feedback: "1 de 14 (7%)"
...
t=9m12s  Result: {fruits_harvested: 13, fruits_skipped: 1,
                  failure_reasons: ["#11: oclusão persistente após 2 tentativas"]}
```

---

## 10. Comunicação e energia

### 10.1 Wi-Fi

**Física:** perda de percurso no espaço livre (Friis): $L_{\text{dB}} = 20\log_{10}(d) + 20\log_{10}(f) + 32{,}45$ (d em km, f em MHz).

- **2,4 GHz:** maior alcance, atravessa melhor folhagem e estruturas, mas congestionado.
- **5 GHz:** mais banda, menos interferência, alcance menor e muito atenuado por folhagem úmida — relevante numa estufa densa.

**Exemplo real:** a estufa recebeu três APs em mesh formando cobertura contínua; o robô usa *fast roaming* (802.11r) para trocar de AP sem derrubar as sessões DDS. Todo tráfego de controle é **local ao robô** (CAN + micro-ROS), de forma que uma queda de Wi-Fi degrada a telemetria, não o controle.

### 10.2 Bluetooth / BLE

Curto alcance, baixo consumo. Usos concretos: emparelhamento com o controle manual de manutenção, leitura de sensores de solo BLE espalhados no campo (o robô passa e coleta — *data mule*), e configuração inicial em campo por celular.

### 10.3 4G/5G

**Papel correto:** telemetria, atualizações de modelo, backup em nuvem, chatbot remoto, e *teleoperação supervisória* (aprovação humana de decisões, não pilotagem).

**Papel incorreto:** malha de controle. Latência de 30–150 ms com caudas de segundos e perda de pacotes tornam isso perigoso. Mesmo o 5G URLLC exige infraestrutura dedicada que uma fazenda não tem.

**Exemplo real:** modem 4G Cat-4 em USB. O robô publica a cada 30 s um resumo MQTT (bateria, posição, frutos colhidos, temperatura das juntas) para um painel. Se o sinal cai, o `store-and-forward` local acumula e reenvia. O robô **continua trabalhando offline** — com LLM local, esse requisito é atendido.

### 10.4 Energia

**Química:**

| | Li-ion NMC | LiFePO4 |
|---|---|---|
| Energia específica | 150–250 Wh/kg | 90–140 Wh/kg |
| Tensão nominal/célula | 3,6–3,7 V | 3,2 V |
| Ciclos (80% DoD) | 500–1500 | 2000–5000 |
| Segurança térmica | menor | **muito maior** |
| Recomendação | v2 (peso crítico) | **v0/v1** |

**Cálculo de orçamento energético (v1, base com rodas + 2 braços):**

| Consumidor | Potência média |
|---|---|
| 2 motores de tração | 120 W |
| 12 juntas dos braços (ciclo de trabalho ~35%) | 210 W |
| Jetson Orin Nano | 15 W |
| Raspberry Pi 5 + periféricos | 12 W |
| Sensores (LiDAR, câmeras, IMU) | 18 W |
| MCUs, drivers, ventoinhas | 25 W |
| **Total médio** | **~400 W** |

Para 4 h de autonomia: $400 \times 4 = 1600\ \text{Wh}$. Em 48 V, isso é ~33 Ah. Com LiFePO4 (~110 Wh/kg de pack, incluindo BMS e caixa), o pack pesa **~15 kg**. Essa massa entra de volta no cálculo estrutural da §1 — é o acoplamento que torna projeto de robô um problema iterativo.

**Sistema elétrico:**
- Barramento de 48 V para tração e juntas de carga (menos corrente = menos perda $I^2R$ e cabos mais finos).
- Conversores DC-DC isolados para 24 V (sensores), 12 V (LiDAR) e 5 V (lógica).
- **BMS** com balanceamento de células, corte por sobrecorrente, sub/sobretensão e temperatura.
- **Pré-carga** dos capacitores de barramento por resistor + relé, evitando arco no conector ao ligar.

---

## 11. Segurança funcional (adicionada — não negociável)

Um robô de 80–150 kg com atuadores de 150 N·m operando ao lado de pessoas é **maquinário perigoso**. A segurança não pode depender do software cognitivo.

**Camadas independentes, do mais confiável ao menos:**

1. **E-stop físico** — botão cogumelo, contato normalmente fechado, canal duplo, categoria de parada 0 ou 1, cortando a alimentação dos drivers via relé de segurança. **Não passa por nenhum processador.**
2. **Limites mecânicos** — batentes físicos e fusíveis mecânicos nas juntas.
3. **Watchdog de hardware** nos MCUs: sem heartbeat, freio + torque zero.
4. **Limites de corrente/torque** no firmware do driver.
5. **Monitoramento de velocidade e separação** — reduzir velocidade proporcionalmente à proximidade de pessoas detectadas (LiDAR + visão), conforme ISO/TS 15066.
6. **Supervisor em ROS 2** com QoS de deadline sobre `/safety/*`.
7. **Restrições lógicas duras** no DeepProbLog (`\+ human_in_workspace`) — a camada mais alta e menos confiável, mas útil.

**Normas para consultar:** ISO 12100 (avaliação de risco), ISO 10218-1/-2 (robôs industriais, revisão de 2025), ISO/TS 15066 (colaborativos), ISO 13482 (robôs de assistência pessoal), ISO 3691-4 (veículos autônomos industriais), ISO 18497 (segurança de máquinas agrícolas autônomas), IEC 61508 (segurança funcional), e no Brasil a **NR-12** (segurança em máquinas e equipamentos) e NR-31 (trabalho rural).

---
---

# TÓPICO 2 — Planejamento de Estudos Aprofundado

## Como usar este plano

São **onze módulos**. Cada um tem: o que dominar, os conceitos-chave, a bibliografia de referência, artigos científicos fundadores e recentes, cursos online e — o mais importante — um **projeto de validação**. Só considere o módulo concluído quando o projeto funcionar; ler não é aprender em robótica.

> **Sobre as referências:** os artigos estão listados por autor, título e veículo, que é o que você precisa para achá-los no Google Scholar, arXiv ou Sci-Hub institucional. Os links de curso são as páginas raiz das plataformas; confirme preço e disponibilidade, que mudam com frequência.

---

## Módulo 1 — Matemática para robótica

**Por que primeiro:** sem isso, tudo o mais vira copiar-e-colar. Você já tem mecânica clássica básica, o que ajuda muito.

**Dominar:**
- Álgebra linear: espaços vetoriais, autovalores, SVD, pseudo-inversa, decomposições (LU, QR, Cholesky), condicionamento numérico.
- Grupos de Lie: SO(3), SE(3), álgebras $\mathfrak{so}(3)$, $\mathfrak{se}(3)$, exponencial e logaritmo matricial, twists e wrenches, quatérnios unitários.
- Probabilidade: Bayes, gaussianas multivariadas, marginalização, filtro de Kalman e suas variantes.
- Otimização: mínimos quadrados, Gauss-Newton, Levenberg-Marquardt, programação quadrática (QP), multiplicadores de Lagrange e KKT.

**Bibliografia:**
- Strang, *Introduction to Linear Algebra* — ou o MIT 18.06 no OCW.
- Boyd & Vandenberghe, *Convex Optimization* (PDF gratuito na página de Stephen Boyd, Stanford).
- Solà, Deray & Atchuthan, **"A micro Lie theory for state estimation in robotics"** (arXiv:1812.01537) — **leia este primeiro; é a melhor introdução prática a grupos de Lie para robótica que existe.**
- Barfoot, *State Estimation for Robotics* (PDF gratuito na página do autor, U. Toronto).

**Cursos:**
- MIT OCW 18.06 *Linear Algebra* (Gilbert Strang) — gratuito.
- Stanford EE364A *Convex Optimization* (Boyd) — vídeos e notas gratuitos.
- 3Blue1Brown, *Essence of Linear Algebra* — para intuição geométrica.

**Projeto de validação:** implemente do zero, em NumPy, sem bibliotecas de robótica: conversão quatérnio ↔ matriz de rotação ↔ ângulos de Euler, composição de transformadas SE(3), e um filtro de Kalman estendido que funde acelerômetro e giroscópio simulados com ruído e bias. Compare com a saída de uma biblioteca pronta.

---

## Módulo 2 — Mecânica de robôs: cinemática, dinâmica e controle

**Dominar:** configuração e espaço de configurações, graus de liberdade, cinemática direta por produto de exponenciais, Jacobiano e singularidades, cinemática inversa numérica, dinâmica de Lagrange e Newton-Euler recursivo, planejamento de trajetória, controle de movimento e de força/impedância.

**Bibliografia:**
- **Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge, 2017.** Pré-print gratuito no site de Northwestern. **É a espinha dorsal deste módulo.**
- Siciliano, Sciavicco, Villani & Oriolo, *Robotics: Modelling, Planning and Control*, Springer.
- Featherstone, *Rigid Body Dynamics Algorithms*, Springer — a referência para RNEA e ABA.
- Siciliano & Khatib (eds.), *Springer Handbook of Robotics* — enciclopédia de consulta.

**Artigos:**
- Carpentier et al., "The Pinocchio C++ library: a fast and flexible implementation of rigid body dynamics algorithms", IEEE/SICE SII 2019.
- Khatib, "A unified approach for motion and force control of robot manipulators: The operational space formulation", IEEE J. Robotics & Automation, 1987. — clássico fundador do controle no espaço da tarefa.
- Hogan, "Impedance Control: An Approach to Manipulation" (3 partes), J. Dynamic Systems, 1985.
- Pratt & Williamson, "Series Elastic Actuators", IROS 1995.

**Cursos:**
- **Coursera — *Modern Robotics: Mechanics, Planning, and Control* Specialization** (Northwestern, Kevin Lynch). Seis cursos, ~4 meses a 10 h/semana, nível intermediário. Acompanha o livro e tem biblioteca de código. → `coursera.org/specializations/modernrobotics`
- **MIT 6.4210 *Robotic Manipulation* (Russ Tedrake)** — notas, vídeos e exercícios com Drake, tudo gratuito em `manipulation.csail.mit.edu`.
- ETH Zurich, *Robot Dynamics* — notas de aula públicas.

**Projeto de validação:** modele um braço de 6 GDL em URDF; implemente FK por produto de exponenciais e IK por mínimos quadrados amortecidos em NumPy; valide contra Pinocchio; calcule os torques exigidos por uma trajetória e verifique se cabem em atuadores reais que você escolheu num catálogo.

---

## Módulo 3 — Locomoção e equilíbrio (para a v2)

**Dominar:** sistemas subatuados, LIPM, ZMP, capture point, geração de padrão de marcha, MPC para locomoção, controle de corpo inteiro por QP, e a alternativa moderna por aprendizado por reforço.

**Bibliografia e cursos:**
- **Tedrake, *Underactuated Robotics* (MIT 6.832)** — livro-texto online, vídeos e notebooks gratuitos em `underactuated.mit.edu`. **Este é o melhor recurso do mundo sobre o assunto, e é gratuito.**
- Kajita et al., *Introduction to Humanoid Robotics*, Springer.
- Wieber, Tedrake & Kuindersma, "Modeling and Control of Legged Robots", capítulo do *Springer Handbook of Robotics*.

**Artigos:**
- Vukobratović & Borovac, "Zero-Moment Point — Thirty Five Years of Its Life", Int. J. Humanoid Robotics, 2004.
- Kajita et al., "Biped walking pattern generation by using preview control of zero-moment point", ICRA 2003.
- Pratt, Carff, Drakunov & Goswami, "Capture Point: A Step toward Humanoid Push Recovery", Humanoids 2006.
- Hwangbo et al., "Learning agile and dynamic motor skills for legged robots", *Science Robotics*, 2019.
- Rudin, Hoeller, Reist & Hutter, "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning", CoRL 2021.
- Radosavovic et al., "Real-World Humanoid Locomotion with Reinforcement Learning", *Science Robotics*, 2024.

**Projeto de validação:** em simulação (MuJoCo ou Drake), implemente um gerador de marcha ZMP para um modelo bípede planar, e depois treine uma política de RL para a mesma tarefa. Compare robustez a perturbações. Você vai entender por que a indústria migrou para RL.

---

## Módulo 4 — Eletrônica de potência e acionamento

**Dominar:** máquinas elétricas, inversores trifásicos, FOC, projeto de PCB com sinais rápidos, integridade de sinal, compatibilidade eletromagnética, sensoriamento de corrente, gestão térmica.

**Bibliografia:**
- Mohan, Undeland & Robbins, *Power Electronics: Converters, Applications and Design*, Wiley.
- Hughes & Drury, *Electric Motors and Drives*.
- **Ott, *Electromagnetic Compatibility Engineering*, Wiley** — o livro que evita meses de depuração de ruído.
- Horowitz & Hill, *The Art of Electronics* — referência geral de bancada.

**Artigos e material técnico:**
- Texas Instruments, *Field Oriented Control of Permanent Magnet Motors* (application note) — mais útil que muito paper.
- STMicroelectronics AN4013/AN5397 sobre timers avançados e FOC no STM32.
- Documentação do **SimpleFOC** e do **ODrive** — código aberto, legível, e você aprende lendo.

**Cursos:**
- Coursera — *Electric Power Electronics* Specialization (University of Colorado Boulder, Robert Erickson).
- edX/Coursera — *Embedded Systems Essentials with Arm*.
- Phil's Lab (YouTube) — projeto de PCB para motores e sinais mistos, excelente qualidade.

**Projeto de validação:** construa um driver FOC funcional para um gimbal BLDC usando SimpleFOC em STM32, com encoder magnético AS5600/AS5047 e medição de corrente por shunt. Feche a malha de torque e verifique com uma célula de carga que o torque comandado corresponde ao real.

---

## Módulo 5 — Sistemas embarcados e tempo real

**Dominar:** arquitetura Cortex-M, interrupções e prioridades, DMA, timers, I²C/SPI/UART/CAN, RTOS, análise de escalabilidade temporal, watchdogs, bootloader e atualização de firmware.

**Bibliografia:**
- Yiu, *The Definitive Guide to Arm Cortex-M3 and Cortex-M4 Processors*.
- Buttazzo, *Hard Real-Time Computing Systems*, Springer — teoria de escalonamento (RM, EDF, análise de resposta).
- Documentação do FreeRTOS e do Zephyr.

**Cursos:**
- Coursera — *Introduction to Embedded Systems Software and Development Environments* (U. Colorado Boulder).
- Documentação e tutoriais do **micro-ROS** (`micro.ros.org`) — ponte essencial entre MCU e ROS 2.

**Projeto de validação:** malha de controle de posição a 1 kHz em STM32, com encoder em quadratura lido por timer em hardware, comandos via CAN, watchdog ativo e **medição real do jitter** da ISR com um pino de GPIO e osciloscópio. Se o jitter passar de 5%, você ainda não entendeu o módulo.

---

## Módulo 6 — ROS 2

**Dominar:** nós, tópicos, serviços, actions, parâmetros, lifecycle nodes, QoS, executores e callback groups, `tf2`, `colcon`, URDF/Xacro, `ros2_control`, Nav2, MoveIt 2, Gazebo/Ignition, `rosbag2`, e o funcionamento do DDS por baixo.

**Bibliografia:**
- Documentação oficial: `docs.ros.org` — os tutoriais são bons e devem ser feitos integralmente, não lidos.
- Newman, *A Systematic Approach to Learning Robot Programming with ROS*.
- Especificação DDS/RTPS do OMG (leitura seletiva).

**Artigos:**
- **Macenski, Foote, Gerkey, Lalancette & Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild", *Science Robotics*, 2022.** — leitura obrigatória para entender o *porquê* das decisões de design.
- Macenski, Martín, White & Clavero, "The Marathon 2: A Navigation System" (Nav2), IROS 2020.
- Macenski & Jambrecic, "SLAM Toolbox: SLAM for the dynamic world", JOSS 2021.
- Coleman, Şucan, Chitta & Correll, "Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study", 2014.

**Cursos:**
- **The Construct** (`theconstruct.ai`) — cursos ROS 2 com simulação no navegador, sem instalar nada. É o caminho mais rápido do zero ao produtivo.
- Udemy — *ROS 2 for Beginners* (Edouard Renard), didático e barato.
- ETH Zurich — *Programming for Robotics* (material público).
- Articulated Robotics (YouTube) — série excelente de construção de robô com ROS 2 do zero.

**Projeto de validação:** um robô diferencial simulado no Gazebo, com URDF completo, `ros2_control`, navegação Nav2 num mapa criado por `slam_toolbox`, e um **action server customizado** que aceita "vá ao ponto X e volte", com feedback e cancelamento funcionando. Meça a latência ponta a ponta com `ros2 topic delay`.

---

## Módulo 7 — Percepção e visão computacional

**Dominar:** modelo de câmera e calibração, geometria epipolar, estéreo, detecção e segmentação com redes neurais, nuvens de pontos (PCL/Open3D), ICP e registro, SLAM visual, odometria visual-inercial.

**Bibliografia:**
- Hartley & Zisserman, *Multiple View Geometry in Computer Vision* — a bíblia da geometria.
- Szeliski, *Computer Vision: Algorithms and Applications* — PDF gratuito na página do autor.
- **Thrun, Burgard & Fox, *Probabilistic Robotics*, MIT Press** — fundamental para localização e mapeamento.

**Artigos:**
- Mur-Artal & Tardós, "ORB-SLAM2", IEEE T-RO 2017.
- Campos et al., "ORB-SLAM3", IEEE T-RO 2021.
- Qin, Li & Shen, "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator", IEEE T-RO 2018.
- Labbé & Michaud, "RTAB-Map as an open-source lidar and visual SLAM library", J. Field Robotics 2019.
- Kirillov et al., "Segment Anything" (SAM), ICCV 2023 — e o SAM 2, para segmentação sem treino específico.
- Madgwick, Harrison & Vaidyanathan, "Estimation of IMU and MARG orientation using a gradient descent algorithm", ICORR 2011.

**Cursos:**
- Coursera — *Robotics: Perception* (University of Pennsylvania).
- TU Munich — *Multiple View Geometry* (Daniel Cremers), aulas gravadas no YouTube.
- Cyrill Stachniss (Univ. Bonn) — canal no YouTube com cursos completos de *Mobile Sensing and Robotics*, SLAM e fotogrametria. **Recurso de altíssima qualidade e gratuito.**

**Projeto de validação:** calibre uma câmera com tabuleiro de xadrez; implemente detecção de um objeto e extraia sua pose 6D a partir de RGB-D; publique como `Detection3DArray` em ROS 2 e visualize no RViz2 com `tf2` correto. Meça o erro de posição contra uma referência medida com paquímetro.

---

## Módulo 8 — Aprendizado de máquina e TinyML

**Dominar:** redes neurais, treino e regularização, CNNs, transformers (o suficiente), quantização, pruning, destilação, *deployment* em MCU, ciclo de vida de modelos embarcados.

**Bibliografia:**
- **Warden & Situnayake, *TinyML: Machine Learning with TensorFlow Lite on Arduino and Ultra-Low-Power Microcontrollers*, O'Reilly, 2019.** — o livro do campo.
- Situnayake & Plunkett, *AI at the Edge*, O'Reilly, 2023 — mais atual, cobre MLOps de borda.
- Goodfellow, Bengio & Courville, *Deep Learning* — PDF gratuito.

**Artigos:**
- **David et al., "TensorFlow Lite Micro: Embedded Machine Learning for TinyML Systems", MLSys 2021.** — descreve a arquitetura do runtime que você vai usar.
- Jacob et al., "Quantization and Training of Neural Networks for Efficient Integer-Arithmetic-Only Inference", CVPR 2018. — a matemática da quantização int8.
- Banbury et al., "MLPerf Tiny Benchmark", NeurIPS Datasets & Benchmarks 2021.
- Lin et al., "MCUNet: Tiny Deep Learning on IoT Devices", NeurIPS 2020 — e MCUNetV2/V3, incluindo treino no dispositivo.
- Lai, Suda & Chandra, "CMSIS-NN: Efficient Neural Network Kernels for Arm Cortex-M CPUs", 2018.

**Cursos:**
- **edX/HarvardX — *Tiny Machine Learning (TinyML)* Professional Certificate** (Vijay Janapa Reddi, Harvard SEAS + Laurence Moroney, Google): três cursos — *Fundamentals of TinyML*, *Applications of TinyML*, *Deploying TinyML*. Auditável de graça; certificado pago. ~60–70 h. Há o programa complementar *Applied TinyML for Scale* (MLOps). → `edx.org/certificates/professional-certificate/harvardx-tiny-machine-learning`
- Documentação LiteRT for Microcontrollers (`ai.google.dev/edge/litert/microcontrollers`).
- Edge Impulse — tutoriais e plataforma gratuita para prototipagem TinyML (muito prática para o classificador de vibração).

**Projeto de validação:** treine um detector de anomalia de vibração com o acelerômetro de um motor real, quantize para int8, rode no STM32 e **meça o tempo de inferência e a RAM do tensor arena**. Depois, induza uma falha (desalinhe o eixo) e verifique se o modelo detecta.

---

## Módulo 9 — IA neurossimbólica, ontologias e grounding

**Dominar:** programação lógica (Prolog), programação lógica probabilística (ProbLog), predicados neurais (DeepProbLog), compilação de conhecimento (SDD/d-DNNF, weighted model counting), OWL/RDF/SPARQL, raciocinadores de descrição lógica, e o problema de ancoragem.

**Bibliografia:**
- De Raedt, Kersting, Natarajan & Poole, *Statistical Relational Artificial Intelligence: Logic, Probability, and Computation*, Morgan & Claypool.
- Hitzler, Krötzsch & Rudolph, *Foundations of Semantic Web Technologies*.
- Colledanchise & Ögren, *Behavior Trees in Robotics and AI: An Introduction*, CRC Press (também em arXiv).

**Artigos — fundadores:**
- Harnad, "The Symbol Grounding Problem", *Physica D*, 1990.
- **Coradeschi & Saffiotti, "An introduction to the anchoring problem", *Robotics and Autonomous Systems*, 2003.** — a base conceitual da sua camada de Semantic Grounding.
- De Raedt, Kimmig & Toivonen, "ProbLog: A Probabilistic Prolog and Its Application in Link Discovery", IJCAI 2007.

**Artigos — DeepProbLog e neurossimbólico:**
- **Manhaeve, Dumančić, Kimmig, Demeester & De Raedt, "DeepProbLog: Neural Probabilistic Logic Programming", NeurIPS 2018** (arXiv:1805.10872) — e a versão estendida em *Artificial Intelligence*, 2021.
- Manhaeve, Marra & De Raedt, "Approximate Inference for Neural Probabilistic Logic Programming", KR 2021 — leia se a inferência exata estourar (e vai estourar).
- Derkinderen, Manhaeve et al., "The DeepLog Neurosymbolic Machine" (arXiv:2508.13697) e "DeepLog: A Software Framework for Modular Neurosymbolic AI", KU Leuven, 2026 — o estado da arte atual da família.
- Xu, Zhang, Friedman, Liang & Van den Broeck, "A Semantic Loss Function for Deep Learning with Symbolic Knowledge", ICML 2018.
- Garcez & Lamb, "Neurosymbolic AI: The 3rd Wave", *Artificial Intelligence Review*, 2023.
- Marra, Dumančić, Manhaeve & De Raedt, "From Statistical Relational to Neurosymbolic Artificial Intelligence: A Survey", *Artificial Intelligence*, 2024.

**Artigos — ontologias em robótica:**
- Tenorth & Beetz, "KnowRob: A knowledge processing infrastructure for cognition-enabled robots", *IJRR*, 2013.
- Beetz, Beßler, Haidu, Pomarlan, Bozcuoğlu & Bartels, "KnowRob 2.0 — A 2nd Generation Knowledge Processing Framework for Cognition-Enabled Robotic Agents", ICRA 2018.
- Beßler et al., "Foundations of the Socio-Physical Model of Activities (SOMA) for Autonomous Robotic Agents", 2021.
- IEEE 1872-2015, *Standard Ontologies for Robotics and Automation* (ORA).

**Ferramentas:** SWI-Prolog, ProbLog2, `deepproblog` (PyPI, repositório `ML-KULeuven/deepproblog` no GitHub), Protégé, `owlready2`, `rdflib`, `BehaviorTree.CPP`.

**Cursos:**
- KU Leuven e Politecnico di Milano oferecem cursos gravados de programação lógica probabilística; procure "Probabilistic Logic Programming" nos canais dos grupos DTAI (KU Leuven) e AIPlan4EU.
- Stanford CS520 *Knowledge Graphs* — vídeos públicos.
- Tutoriais de Protégé da Stanford BMIR — o caminho prático para OWL.

**Projeto de validação:** escreva uma ontologia OWL pequena de uma cultura agrícola (5 classes, 10 propriedades) em Protégé; exporte fatos para Prolog via `owlready2`; escreva um programa DeepProbLog com um predicado neural (classificador de maturação treinado em 200 fotos suas) e uma regra de decisão; consulte e **inspecione a árvore de prova**. Meça o tempo de inferência — você vai descobrir na prática por que essa camada roda a 1 Hz.

---

## Módulo 10 — LLMs aplicados a robótica

**Dominar:** prompting estruturado e saída em JSON, *function/tool calling*, embeddings e busca semântica, RAG, quantização e execução local (llama.cpp, vLLM, Ollama), e a literatura de planejamento com LLM.

**Artigos — leitura essencial, em ordem:**
- **Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (SayCan), CoRL 2022** (arXiv:2204.01691) — o artigo que formaliza por que o LLM precisa ser filtrado por *affordances* reais. Diretamente aplicável à sua arquitetura.
- Huang et al., "Inner Monologue: Embodied Reasoning through Planning with Language Models", CoRL 2022.
- Liang et al., "Code as Policies: Language Model Programs for Embodied Control", ICRA 2023.
- Driess et al., "PaLM-E: An Embodied Multimodal Language Model", ICML 2023 (arXiv:2303.03378).
- Brohan et al., "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control", CoRL 2023.
- Black et al., "π₀: A Vision-Language-Action Flow Model for General Robot Control", 2024.
- NVIDIA, "GR00T N1: An Open Foundation Model for Generalist Humanoid Robots", 2025 — arquitetura de dois sistemas (VLM a ~10 Hz + módulo de difusão a ~120 Hz) que é *exatamente* a separação de domínios temporais defendida no Tópico 1.
- Reimers & Gurevych, "Sentence-BERT: Sentence Embeddings using Siamese BERT-Networks", EMNLP 2019 — a base do seu Alinhador Semântico.
- Radford et al., "Learning Transferable Visual Models From Natural Language Supervision" (CLIP), ICML 2021.
- Surveys recentes: "A Survey on Vision-Language-Action Models for Embodied AI" (arXiv:2405.14093, atualizado continuamente) e "A Survey on Vision-Language-Action Models: An Action Tokenization Perspective" (arXiv:2507.01925).

**Cursos e recursos:**
- **Hugging Face — curso LeRobot** (gratuito): robótica com aprendizado, datasets e políticas. Muito alinhado ao seu projeto.
- Hugging Face NLP Course e Agents Course — para embeddings, RAG e tool calling.
- DeepLearning.AI — cursos curtos sobre *function calling*, saída estruturada e avaliação de LLMs.

**Projeto de validação:** rode um modelo de 7–8B quantizado localmente; construa o pipeline `texto livre → JSON validado por esquema → alinhamento semântico com a ontologia → chamada de action ROS 2 simulada`. Teste com **50 frases mal escritas, ambíguas e maliciosas** e meça a taxa de saída inválida. Esse número é o que define se a camada é usável.

---

## Módulo 11 — Robótica de campo, agrícola e de construção

**Por que separado:** o que funciona no laboratório falha no barro. Este módulo é sobre o *domínio*, não sobre a tecnologia.

**Artigos e relatórios:**
- Bac, van Henten, Hemming & Edan, "Harvesting Robots for High-Value Crops: State-of-the-Art Review and Challenges Ahead", *Journal of Field Robotics*, 2014. — leitura obrigatória; explica por que colheita robótica é mais difícil do que parece.
- Duckett et al., *Agricultural Robotics: The Future of Robotic Agriculture*, UK-RAS White Paper, 2018.
- Oliveira, Moreira & Silva, "Advances in Agriculture Robotics: A State-of-the-Art Review and Challenges Ahead", *Robotics*, 2021.
- Bogue, "What are the prospects for robots in the construction industry?", *Industrial Robot*, 2018.
- Melenbrink, Werfel & Menges, "On-site autonomous construction robots: Towards unsupervised building", *Automation in Construction*, 2020.
- Kootbally et al. e a literatura de *Industry Foundation Classes* (IFC) / BIM para integração com construção.
- Embrapa e SBIAgro — literatura brasileira sobre agricultura de precisão, com dados de culturas e condições locais.

**Normas de segurança:** ISO 12100, ISO 10218-1/-2:2025, ISO/TS 15066, ISO 13482, ISO 3691-4, ISO 18497, IEC 61508; no Brasil, **NR-12** e **NR-31**. Leia pelo menos a ISO 12100 inteira — avaliação de risco é uma habilidade, não burocracia.

**Projeto de validação:** faça uma **análise formal de risco** (ISO 12100) do seu robô v1: identifique perigos, estime severidade/exposição/evitabilidade, defina medidas de redução e documente. Isso muda o projeto de hardware — e é melhor descobrir isso agora que depois de construir.

---

## Recursos transversais

- **Simuladores:** Gazebo (Harmonic/Ionic), MuJoCo (gratuito, física de contato excelente), NVIDIA Isaac Sim/Lab (RL em escala, exige GPU forte), Drake (MIT, controle e otimização), Webots.
- **Comunidades:** ROS Discourse (`discourse.ros.org`), Robotics Stack Exchange, subreddit r/robotics, fórum TinyML de Harvard, ROS Brasil.
- **Conferências para acompanhar:** ICRA, IROS, RSS, CoRL, Humanoids, FSR (Field and Service Robotics), AgEng.
- **Acompanhamento contínuo:** arXiv cs.RO (feed diário), Papers With Code, canais de Cyrill Stachniss e Russ Tedrake.


---
---

# TÓPICO 3 — Cronograma de Estudo e Construção

## Premissas

**Ponto de partida assumido:** Python e programação sólidos, mecânica clássica básica, eletromagnetismo básico.

**O que está faltando e precisa ser adquirido:** C++ moderno, álgebra linear aplicada e grupos de Lie, eletrônica prática de potência, sistemas embarcados em tempo real, ROS 2, visão computacional, e a pilha neurossimbólica.

**Dedicação base:** 15–20 h/semana. Duração: **24 meses** até a v1 funcional (base móvel + tronco + dois braços executando o cenário completo de colheita). O cronograma é sequenciado por **dependência técnica**, não por interesse — a tentação de pular para o LLM no mês 2 é o erro mais comum e mais caro.

**Princípio de organização:** cada fase tem um **portão de saída** verificável. Não avance sem passar. E a regra dos 70/30: 70% do tempo construindo e depurando, 30% estudando. Robótica não se aprende lendo.

---

## Visão geral das seis fases

| Fase | Meses | Foco | Entregável físico |
|---|---|---|---|
| **F1 — Fundamentos** | 1–4 | Matemática, C++, ROS 2, simulação | Robô simulado navegando no Gazebo |
| **F2 — Hardware básico** | 5–8 | Eletrônica, MCU, tempo real, uma junta real | Junta única com FOC e controle a 1 kHz |
| **F3 — Braço e percepção** | 9–14 | Cinemática, visão, MoveIt 2, TinyML | **v0:** braço de 6 GDL que pega objetos vistos por câmera |
| **F4 — Cognição** | 15–19 | Ontologia, DeepProbLog, LLM, grounding | Pilha cognitiva completa comandando a v0 |
| **F5 — Integração v1** | 20–24 | Base móvel, tronco, dois braços, segurança | **v1:** robô móvel executando o cenário fim a fim |
| **F6 — Bipedismo (opcional)** | 25–40+ | Locomoção, RL, whole-body control | **v2:** pernas |

---

## FASE 1 — Fundamentos (Meses 1–4)

### Mês 1 — Matemática e C++
| Semana | Estudo (10 h) | Prática (8 h) |
|---|---|---|
| 1 | MIT 18.06: espaços vetoriais, SVD, pseudo-inversa | NumPy: implementar SVD-based least squares do zero |
| 2 | Solà, *A micro Lie theory* — SO(3), SE(3), exp/log | Biblioteca própria de transformadas SE(3) com testes unitários |
| 3 | C++ moderno: RAII, templates, `std::`, CMake | Portar a biblioteca de SE(3) para C++ com Eigen |
| 4 | Eigen: tipos fixos, decomposições, aliasing | Benchmark NumPy vs Eigen; medir alocações com `set_is_malloc_allowed(false)` |

**Portão:** biblioteca SE(3) em Python e C++, com testes, produzindo resultados idênticos.

### Mês 2 — Modern Robotics, parte 1
- Coursera *Modern Robotics* Cursos 1 e 2 (Fundamentos do Movimento + Cinemática).
- Livro de Lynch & Park, capítulos 2–6.
- **Prática:** FK e IK de um braço de 6 GDL em NumPy, validados contra a biblioteca `modern_robotics`.

**Portão:** IK convergindo em < 10 ms para poses alcançáveis, com tratamento de singularidade.

### Mês 3 — ROS 2 do zero
- The Construct ou Udemy (Renard): curso completo de ROS 2 Jazzy.
- Tutoriais oficiais em `docs.ros.org` — **todos**, incluindo os avançados de QoS e executores.
- Ler Macenski et al., *Science Robotics* 2022.
- **Prática:** pacote próprio com publisher/subscriber, service, **action server com feedback e cancelamento**, lifecycle node, e parâmetros dinâmicos.

**Portão:** o action `CountTo` implementado com preempção correta, testado com `ros2 action send_goal --feedback` e cancelamento.

### Mês 4 — Simulação e URDF
- URDF/Xacro, `ros2_control`, Gazebo, RViz2, `tf2`.
- **Prática:** modelar um robô diferencial com braço em URDF; simular; mapear com `slam_toolbox`; navegar com Nav2.

**Portão de fase:** robô simulado que recebe um objetivo de navegação, chega lá, e reporta por um action customizado. Tudo com `tf2` correto e sem avisos.

> **Compras neste período:** kit STM32 Nucleo (~R$ 180), multímetro decente (~R$ 250), fonte de bancada ajustável (~R$ 600), ferro de solda com controle de temperatura (~R$ 300), osciloscópio de entrada (~R$ 1.500 — o item que mais acelera o aprendizado de hardware).

---

## FASE 2 — Hardware e tempo real (Meses 5–8)

### Mês 5 — Eletrônica de potência
- Livro de Mohan (caps. 1–8) + Ott (EMC, caps. 1–5).
- Application notes da TI sobre FOC.
- **Prática:** montar uma ponte H discreta em protoboard, acionar um motor DC com PWM de MCU, observar no osciloscópio o ripple de corrente e os picos de comutação. Adicionar snubber e ver a diferença.

### Mês 6 — Embarcado em tempo real
- Curso de sistemas embarcados (Boulder ou Arm).
- Yiu (Cortex-M) + Buttazzo (caps. 1–4).
- **Prática:** ISR a 1 kHz no STM32 com PID de posição; encoder por timer em hardware; **medir jitter com GPIO + osciloscópio**; implementar watchdog.

**Portão:** jitter da ISR < 2% e recuperação correta do watchdog ao travar deliberadamente o loop principal.

### Mês 7 — FOC e uma junta real
- SimpleFOC: estudar o código-fonte.
- **Prática:** driver FOC completo para um BLDC de gimbal com encoder AS5047; malha de corrente, velocidade e posição; caracterizar $K_t$ experimentalmente com célula de carga e braço de alavanca.

**Portão:** comandar 0,5 N·m e medir 0,5 ± 0,05 N·m reais.

### Mês 8 — CAN, micro-ROS e integração
- micro-ROS: `micro.ros.org`, agente DDS-XRCE.
- **Prática:** o STM32 vira nó ROS 2 publicando `/joint_states` e assinando `/joint_commands` via CAN + agente no Raspberry Pi. Rodar `ros2_control` com `hardware_interface` customizada.

**Portão de fase:** `ros2 topic echo /joint_states` mostrando a posição real de um motor físico a 200 Hz, comandado pelo `joint_trajectory_controller`.

> **Compras:** Raspberry Pi 5 8GB (~R$ 900), 2 BLDCs de gimbal + drivers (~R$ 700), encoders magnéticos (~R$ 150), transceivers CAN (~R$ 80), fonte 24 V 10 A (~R$ 350).

---

## FASE 3 — Braço e percepção (Meses 9–14) → **v0**

### Meses 9–10 — Dinâmica e controle avançado
- Coursera *Modern Robotics* Cursos 3 e 4 (Dinâmica + Planejamento e Controle).
- Featherstone, caps. 1–5; Pinocchio (tutoriais).
- Khatib (1987) e Hogan (1985).
- **Prática:** dinâmica inversa de um braço de 6 GDL em Pinocchio; implementar *computed torque control* em simulação; comparar com PID puro sob carga variável.

### Meses 11–12 — Construção física do braço
- **Projeto mecânico:** CAD (FreeCAD ou Fusion 360) de um braço de 5–6 GDL, alcance ~600 mm, carga útil 1,5 kg. Servos de barramento (Dynamixel ou equivalente nacional) nas juntas — **não tente BLDC customizado ainda**.
- Dimensionar torques pela §1.1; verificar em Pinocchio antes de comprar nada.
- Imprimir/usinar, montar, cablear com boas práticas de EMC.
- URDF fiel ao CAD, com massas e inércias reais (o CAD calcula).
- **Prática:** MoveIt 2 configurado; planejar e executar trajetórias livres de colisão no hardware real.

**Portão:** o braço real segue uma trajetória cartesiana com erro < 5 mm, planejada pelo MoveIt 2.

### Mês 13 — Visão e grounding geométrico
- Curso de Cyrill Stachniss (percepção) + Coursera *Robotics: Perception*.
- **Prática:** calibrar câmera RGB-D; treinar um YOLO em 300 fotos de um objeto-alvo (tomate de plástico serve para começar); extrair pose 6D; publicar `Detection3DArray`; `tf2` até `base_link`.

**Portão:** erro de posição da pose estimada < 15 mm, medido com paquímetro.

### Mês 14 — TinyML
- HarvardX *Fundamentals* + *Applications* + *Deploying TinyML*.
- **Prática:** classificador de contato (corrente + encoder → toque/colisão) rodando no MCU da garra em < 5 ms; detector de anomalia de vibração.

**Portão de fase — v0:** o braço vê um objeto, planeja, pega com força controlada detectada por TinyML, e deposita em local pré-definido. Repetibilidade ≥ 85% em 40 tentativas. **Documente e grave em vídeo — este é o marco que prova que o projeto é real.**

> **Compras:** servos de barramento para 6 juntas (~R$ 4.000–9.000, o maior custo até aqui), câmera RGB-D global shutter (~R$ 1.800), Jetson Orin Nano (~R$ 3.000), impressão 3D/usinagem (~R$ 1.500).

---

## FASE 4 — Cognição neurossimbólica (Meses 15–19)

### Mês 15 — Lógica e ontologias
- Prolog (SWI) até fluência básica; ProbLog em seguida.
- Protégé + OWL 2; Tenorth & Beetz (KnowRob).
- **Prática:** ontologia de 15–20 classes do seu domínio; consultas SPARQL; exportação para fatos Prolog com `owlready2`.

### Mês 16 — DeepProbLog
- Manhaeve et al. 2018 e a versão AIJ 2021; depois a variante aproximada (KR 2021).
- Instalar `deepproblog` + SWI-Prolog; rodar os exemplos do repositório (MNIST addition é o "hello world").
- **Prática:** o Raciocinador de Execução da §9.6 com um predicado neural real (seu classificador de maturação). **Meça o tempo de inferência em função do número de objetos** — trace a curva e descubra seu teto prático.

**Portão:** consulta com 15 objetos ancorados resolvendo em < 1 s, com árvore de prova inspecionável.

### Mês 17 — LLM e alinhamento semântico
- SayCan, Inner Monologue, Code as Policies.
- Ollama/llama.cpp com modelo 7–8B quantizado no Jetson.
- Sentence-BERT multilíngue para o alinhador.
- **Prática:** pipeline `texto → JSON validado → alinhamento → intenção`; suíte de 50 frases adversariais.

**Portão:** ≥ 95% de saídas JSON válidas; ≥ 90% de alinhamento correto; **100% de recusa segura** em comandos fora do vocabulário.

### Mês 18 — Semantic Grounding e planejamento
- Coradeschi & Saffiotti; Colledanchise & Ögren.
- **Prática:** sistema de âncoras com track/reacquire; `BehaviorTree.CPP` com nós que chamam actions ROS 2 e consultam DeepProbLog.

### Mês 19 — Chatbot e integração cognitiva
- WebSocket/FastAPI + `rosbridge`; autenticação; feedback em tempo real.
- **Prática:** Blender — modelar a armature do seu braço real, animar dois gestos, exportar, **validar torques em Pinocchio**, executar no hardware. Também: gerar 2.000 imagens sintéticas no Blender e medir o ganho no detector.

**Portão de fase:** comando em português no navegador → LLM → alinhador → ontologia → DeepProbLog → behavior tree → braço v0 executa. **Com um caso deliberadamente ambíguo em que o robô pergunta em vez de agir.**

---

## FASE 5 — Integração v1 (Meses 20–24)

### Mês 20 — Segurança primeiro
- ISO 12100 completa; ISO/TS 15066; NR-12.
- **Prática:** análise formal de risco documentada; projeto e instalação do circuito de E-stop de canal duplo com relé de segurança; watchdogs em todos os MCUs; supervisor ROS 2 com QoS de deadline.

**Portão:** E-stop corta a potência em < 100 ms, verificado com osciloscópio, **com o software travado deliberadamente**.

### Meses 21–22 — Base móvel e tronco
- Base diferencial ou de 4 rodas com tração, capaz de 80 kg e terreno irregular.
- Pack LiFePO4 48 V com BMS; DC-DCs; distribuição com fusíveis e pré-carga.
- Nav2 + `slam_toolbox` + `robot_localization` no hardware real.
- Tronco com junta de elevação (fuso + motor de passo com encoder).

**Portão:** navegação autônoma de 50 m em ambiente real com obstáculos, sem intervenção.

### Mês 23 — Segundo braço e coordenação
- Duplicar o braço; coordenação bimanual no MoveIt 2; resolução de colisão braço-braço e braço-corpo.

### Mês 24 — Teste de campo e iteração
- Levar para uma estufa ou canteiro real. **Tudo vai quebrar** — poeira, luz, vibração, Wi-Fi, terreno.
- Instrumentar: `rosbag2` de tudo; analisar falhas; iterar.

**Portão de fase — v1:** o cenário completo do Tópico 1 executado três vezes seguidas em ambiente real, com ≥ 80% de sucesso por fruto, log completo e relatório de falhas.

> **Compras:** motores de tração + drivers (~R$ 2.500), chassi e rodas (~R$ 2.000), pack LiFePO4 48 V 33 Ah + BMS (~R$ 6.000), LiDAR 2D (~R$ 1.500), segundo braço (~R$ 6.000), componentes de segurança (~R$ 800).

**Ordem de grandeza do investimento total até a v1: R$ 45.000–70.000.** Isso exclui seu tempo, e exclui as duas ou três vezes em que você vai queimar um driver.

---

## FASE 6 — Bipedismo (Meses 25–40+, opcional)

Só entre aqui se a v1 estiver funcionando de forma confiável e se houver uma razão real para pernas (terreno com degraus, canteiro de obra desestruturado). Para estufa e para a maioria das tarefas agrícolas, **rodas ou esteiras são tecnicamente superiores e infinitamente mais baratas**.

| Período | Foco |
|---|---|
| M25–28 | *Underactuated Robotics* (MIT 6.832) completo; Kajita; ZMP e capture point em simulação |
| M29–32 | RL para locomoção: Isaac Lab ou MuJoCo; Rudin et al. 2021; treinar política em simulação |
| M33–36 | Atuadores quase-direct-drive: projeto, construção e caracterização de uma perna |
| M37–40 | Duas pernas, whole-body QP, sim-to-real, teste em esteira com arnês de segurança |

Esta fase custa mais que todas as anteriores somadas.

---

## Versão comprimida: 12 meses até um MVP

Se a restrição for tempo e não profundidade, este é o caminho enxuto — sacrifica compreensão de fundamentos em troca de um sistema demonstrável:

| Mês | Foco | Atalhos aceitos |
|---|---|---|
| 1–2 | ROS 2 + simulação | Pular grupos de Lie; usar `tf2` como caixa-preta |
| 3–4 | Braço comercial pronto (ex.: kit de 6 GDL com servos de barramento) | Não projetar mecânica; comprar montado |
| 5–6 | Percepção: câmera RGB-D + detector treinado | Usar modelo pré-treinado com fine-tuning |
| 7–8 | MoveIt 2 + pick-and-place funcionando | Usar `moveit_task_constructor` pronto |
| 9–10 | LLM + JSON estruturado + ontologia simples | Ontologia em YAML em vez de OWL; regras em Python |
| 11 | DeepProbLog em um único ponto de decisão | Um predicado neural, cinco regras |
| 12 | Integração, chatbot, demonstração | Base fixa, sem mobilidade |

Custo: ~R$ 20.000. Resultado: braço fixo que entende linguagem natural e faz pick-and-place com raciocínio simbólico verificável. É uma demonstração honesta e defensável — e a base para levantar recursos para o projeto completo.

---

## Ritmo semanal sugerido (qualquer fase)

| Dia | Horas | Atividade |
|---|---|---|
| Seg | 2 | Estudo teórico (curso/livro) |
| Ter | 2 | Implementação |
| Qua | 2 | Estudo teórico |
| Qui | 2 | Implementação |
| Sex | 2 | Leitura de artigo + notas |
| Sáb | 6 | Bancada: hardware, montagem, depuração |
| Dom | 2 | Documentação, git, revisão da semana, ajuste do plano |

**Duas disciplinas não negociáveis:**
1. **Caderno de laboratório.** Toda medição, todo valor de ganho, toda falha e sua causa. Em robótica, você resolve o mesmo problema três vezes se não anotar.
2. **Git desde o primeiro dia**, com `rosbag2` dos experimentos versionados fora do repositório (LFS ou armazenamento externo).

---

## Riscos do projeto e mitigações

| Risco | Probabilidade | Mitigação |
|---|---|---|
| Espiral de massa/custo dos atuadores | Alta | Dimensionar torque **antes** de comprar; começar com servo de barramento |
| DeepProbLog não escala | Alta | Manter programas pequenos; inferência aproximada; limitar a D3 |
| Percepção falha sob sol/poeira | Alta | Estéreo ativo com global shutter; testar em campo cedo, não no mês 24 |
| EMI dos motores corrompendo sensores | Alta | Disciplina de aterramento e blindagem desde o primeiro protótipo |
| Wi-Fi instável derrubando o sistema | Média | Controle 100% local; nuvem apenas para telemetria |
| Tentação de pular para a camada cognitiva | **Muito alta** | Respeitar os portões de fase; o LLM é a parte fácil |
| Bipedismo consumir o projeto inteiro | Alta | Adiar para a F6; provar valor com rodas primeiro |

---

## Observação final

A parte do seu esquema que mais me chamou atenção — a pilha **ontologia + semantic grounding + DeepProbLog + LLM** — está tecnicamente correta e alinhada com o que grupos de ponta fazem hoje (a arquitetura de dois sistemas do GR00T N1, o filtro de affordances do SayCan, a ancoragem de Coradeschi & Saffiotti). Ela resolve o problema real de segurança e auditabilidade que nenhuma rede neural pura resolve.

O risco do projeto não está aí. Está em torque, rigidez, calor, poeira, EMI e energia — e no fato de que a colheita robótica de frutas continua sendo um problema aberto depois de trinta anos de pesquisa, não por falta de inteligência artificial, mas por causa de folhas que ocultam frutos, hastes que resistem de forma imprevisível e pele que se machuca com 15 N.

Construa a v0 primeiro. Tudo o mais decorre daí.
