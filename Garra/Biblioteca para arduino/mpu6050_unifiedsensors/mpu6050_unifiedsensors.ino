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
      P[0][0] = 0;
      P[0][1] = 0;
      P[1][0] = 0;
      P[1][1] = 0;
    }

    double getAngle(double newAngle, double newRate, double dt) {
      double rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      double S = P[0][0] + R_measure;
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

unsigned long previousTime = 0;
double yawAngle = 0;
const float PI_F = 3.14159265358979323846;

double initialRoll = 0;
double initialPitch = 0;

double lastRollRelative = 0;
double horizontalAngle = 90.0; // começa de 90°

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Erro ao conectar com o MPU6050!");
    while (1);
  }

  Serial.println("MPU6050 conectado com sucesso.");

  delay(1000); // Estabilização
  Serial.println("Calibrando posição inicial...");

  // Média dos primeiros 100 valores para referência
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double AccX = ax / 16384.0;
    double AccY = ay / 16384.0;
    double AccZ = az / 16384.0;

    double rollAcc  = atan2(AccY, AccZ) * 180.0 / PI_F;
    double pitchAcc = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI_F;

    initialRoll += rollAcc;
    initialPitch += pitchAcc;

    delay(5);
  }

  initialRoll /= 100.0;
  initialPitch /= 100.0;

  Serial.print("Referência inicial definida. Roll: ");
  Serial.print(initialRoll);
  Serial.print(" | Pitch: ");
  Serial.println(initialPitch);

  previousTime = millis();
}

void loop() {
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
  yawAngle += GyroZ * dt;
  double yaw = yawAngle;

  // Corrige os ângulos com base na posição inicial
  double rollRelative = roll - initialRoll;
  double pitchRelative = pitch - initialPitch;

  // ===== HORIZONTAL acumulado (-180° a +180°) =====
  double deltaRoll = rollRelative - lastRollRelative;
  lastRollRelative = rollRelative;

  horizontalAngle += deltaRoll;
  horizontalAngle = constrain(horizontalAngle, 0, 180.0);

  // ===== VERTICAL (mapeado de -5 a 5 com zona morta) =====
  const float angleMin = -10.0;
  const float angleMax = 10.0;
  const float deadZone = 3.0;

  auto mapWithDeadZone = [&](double angle) {
    if (abs(angle) < deadZone) return 0.0;
    angle = constrain(angle, angleMin, angleMax);
    return (angle / angleMax) * 5.0;
  };

  int vertical = (int)mapWithDeadZone(pitchRelative);

  // ===== SAÍDA SERIAL =====
  Serial.print("HorizontalAngle: ");
  Serial.print(horizontalAngle, 1);
  Serial.print("° | Vertical: ");
  Serial.print(vertical);
  Serial.print(" | Yaw: ");
  Serial.println(yaw, 1);

  delay(100);
}
