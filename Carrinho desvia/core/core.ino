// -------------------- Pinos sensores --------------------
const int TRIG_F = 12;   // Frontal - TRIG
const int ECHO_F = 13;   // Frontal - ECHO

const int TRIG_R = 11;   // Direita - TRIG
const int ECHO_R = 10;   // Direita - ECHO

// -------------------- Pinos motores ---------------------
const int PIN_MOTOR_IN1 = 2;
const int PIN_MOTOR_IN2 = 3;
const int PIN_MOTOR_IN3 = 4;
const int PIN_MOTOR_IN4 = 5;

// -------------------- Parâmetros de navegação -----------
const int LIMIAR_F_CM   = 20;   // distância (cm) para considerar obstáculo à frente
const int RECUO_MS      = 300;  // quanto tempo recua antes de girar
const int GIRO_DIR_MS   = 350;  // quanto tempo gira à direita (ajuste fino)
const int LOOP_DELAY_MS = 60;   // intervalo do loop para estabilizar leituras

// -------------------- Funções de motor ------------------
// Mover o robo para a frente
void mover_frente(void){
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, HIGH);
  digitalWrite(PIN_MOTOR_IN4, LOW);
}

// Mover o robo para tras
void mover_tras(void){
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, HIGH);
}

// Parar
void parar(void){
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW);
}

// Girar à direita (pivô: roda esquerda para frente, direita para trás)
void girar_direita(void){
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, HIGH);
}

// -------------------- Ultrasom (HC-SR04) ----------------
long distancia_cm(int trigPin, int echoPin){
  // Pulso de trigger (10us)
  digitalWrite(trigPin, LOW);      delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);     delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout ~25ms (≈ 4,3 m). Se estourar, retorna "longe".
  unsigned long dur = pulseIn(echoPin, HIGH, 25000UL);
  if (dur == 0) return 400;  // fora de alcance -> 400 cm
  return (long)(dur / 58UL); // conversão aproximada para cm
}

// -------------------- Setup -----------------------------
void setup(){
  // Motores
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);
  parar();

  // Sensores ultrassom
  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  // Serial
  Serial.begin(9600);
  delay(200);
  Serial.println("Inicializado. sensores: Frente (D12/D13) e Direita (D11/D10)");
}

// -------------------- Loop principal --------------------
void loop(){
  long distF = distancia_cm(TRIG_F, ECHO_F); // Frente
  long distR = distancia_cm(TRIG_R, ECHO_R); // Direita (monitoramento)

  // Prints separados (ajuste seu Serial Monitor para 115200 baud)
  Serial.print("Frente [cm]: ");
  Serial.print(distF);
  Serial.print("  |  Direita [cm]: ");
  Serial.println(distR);

  // Lógica de desvio: se tem obstáculo à frente, recua e gira à direita
  if (distF <= LIMIAR_F_CM){
    parar();               delay(100);
    mover_tras();          delay(RECUO_MS);
    parar();               delay(100);
    girar_direita();       delay(GIRO_DIR_MS);
    parar();               delay(50);
  } else {
    // caminho livre à frente -> segue em frente
    mover_frente();
  }

  delay(LOOP_DELAY_MS);
}
