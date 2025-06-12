#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

// =======================
// Filtro Kalman Simples
// =======================
class Kalman {
  public:
    Kalman() {
      Q_angle = 0.001;
      Q_bias  = 0.003;
      R_measure = 0.03;
      angle = 0;
      bias = 0;
      P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
    }

    double getAngle(double newAngle, double newRate, double dt) {
      double rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      double S  = P[0][0] + R_measure;
      double K0 = P[0][0] / S;
      double K1 = P[1][0] / S;

      double y = newAngle - angle;
      angle += K0 * y;
      bias  += K1 * y;

      double P00_temp = P[0][0];
      double P01_temp = P[0][1];
      P[0][0] -= K0 * P00_temp;
      P[0][1] -= K0 * P01_temp;
      P[1][0] -= K1 * P00_temp;
      P[1][1] -= K1 * P01_temp;

      return angle;
    }

  private:
    double Q_angle, Q_bias, R_measure;
    double angle, bias;
    double P[2][2];
};

Kalman kalmanX;
Kalman kalmanY;

// ---------- LED RGB ----------
const uint8_t LED_R_PIN = 9;   // Anodo do vermelho
const uint8_t LED_G_PIN = 10;    // Anodo do verde
const uint8_t LED_B_PIN = 8;    // Anodo do azul (não usado aqui)

inline void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R_PIN, r ? HIGH : LOW);
  digitalWrite(LED_G_PIN, g ? HIGH : LOW);
  digitalWrite(LED_B_PIN, b ? HIGH : LOW);
}
// --------------------------------

unsigned long previousTime = 0;
double yawAngle = 0;
const double PI_F = 3.14159265358979323846;

double initialRoll  = 0;
double initialPitch = 0;

double lastRollRelative  = 0;
double horizontalAngle   = 90.0;   // começa em 90°

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configura pinos do LED
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  // Estado inicial – desconectado (vermelho)
  setLED(true, false, false);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Erro ao conectar com o MPU6050! (vermelho fixo)");
    return;                       // mantém LED vermelho e sai do setup
  }

  // Conectando (amarelo)
  setLED(true, true, false);
  Serial.println("MPU6050 conectado. Calibrando posição inicial...");

  // --------- Calibração inicial ---------
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double AccX = ax / 16384.0;
    double AccY = ay / 16384.0;
    double AccZ = az / 16384.0;

    initialRoll  += atan2(AccY, AccZ) * 180.0 / PI_F;
    initialPitch += atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI_F;

    delay(5);
  }
  initialRoll  /= 100.0;
  initialPitch /= 100.0;

  Serial.print("Referência definida. Roll: ");
  Serial.print(initialRoll);
  Serial.print(" | Pitch: ");
  Serial.println(initialPitch);

  // Pronto (verde)
  setLED(false, true, false);

  previousTime = millis();
}

void loop() {
  // Verifica se o sensor continua respondendo; se cair, volta para vermelho
  if (!mpu.testConnection()) {
    setLED(true, false, false);   // vermelho
    Serial.println("MPU6050 desconectado!");
    delay(500);
    return;
  } else {
    setLED(true, false, true);   // garante verde durante operação normal
  }

  unsigned long currentTime = millis();
  double dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double AccX = ax / 16384.0;
  double AccY = ay / 16384.0;
  double AccZ = az / 16384.0;

  double GyroX = gx / 131.0;
  double GyroY = gy / 131.0;
  double GyroZ = gz / 131.0;

  double rollAcc  = atan2(AccY, AccZ) * 180.0 / PI_F;
  double pitchAcc = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI_F;

  double roll  = kalmanX.getAngle(rollAcc, GyroX, dt);
  double pitch = kalmanY.getAngle(pitchAcc, GyroY, dt);
  yawAngle    += GyroZ * dt;

  // Corrige ângulos relativos
  double rollRelative  = roll  - initialRoll;
  double pitchRelative = pitch - initialPitch;

  // ---- Ângulo Horizontal acumulado ----
  double deltaRoll = rollRelative - lastRollRelative;
  lastRollRelative = rollRelative;

  horizontalAngle += deltaRoll;
  horizontalAngle = constrain(horizontalAngle, 0, 180.0);

  // ---- Ângulo Vertical com zona morta ----
  const float ANG_MIN = -10.0;
  const float ANG_MAX =  10.0;
  const float DEAD    =   3.0;

  auto mapDead = [&](double val) -> double {
    if (fabs(val) < DEAD) return 0.0;
    val = constrain(val, ANG_MIN, ANG_MAX);
    return (val / ANG_MAX) * 5.0;      // −5 a +5
  };

  int vertical = static_cast<int>(mapDead(pitchRelative));

  // ---- Saída Serial ----
  Serial.print("HorizontalAngle: ");
  Serial.print(horizontalAngle, 1);
  Serial.print("° | Vertical: ");
  Serial.print(vertical);
  Serial.print(" | Yaw: ");
  Serial.println(yawAngle, 1);

  delay(100);
}
