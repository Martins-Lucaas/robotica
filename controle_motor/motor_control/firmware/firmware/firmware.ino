#include <WiFi.h>

// Defina o nome (SSID) e a senha da rede que será criada
const char* ssid     = "ESP32_Network";
const char* password = "minhaSenha";  // mínimo 8 caracteres

void setup() {
  // Inicializa a porta serial para debug
  Serial.begin(115200);
  delay(100);

  // Inicializa o modo SoftAP
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(ssid, password);
  if (ok) {
    Serial.println("✅ Ponto de acesso criado com sucesso!");
  } else {
    Serial.println("❌ Falha ao criar ponto de acesso");
  }

  // Exibe o IP do ponto de acesso
  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP do AP: ");
  Serial.println(IP);

  // Opcional: configurar limite de clientes conectados
  // WiFi.softAPsetMaxConnections(4);
}

void loop() {
  // Aqui você pode tratar clientes conectados, eventos, etc.
  // Por exemplo, ver quantos dispositivos estão conectados:
  Serial.print("Clientes atuais: ");
  Serial.println(WiFi.softAPgetStationNum());
  delay(5000);
}
