#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <RoboCore_Vespa.h>

const char* ssid = "Vespa_S1";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

VespaServo servo;
const uint16_t SERVO_MIN = 500;
const uint16_t SERVO_MAX = 3000;
int anguloAtual = 0;

unsigned long ultimoMovimento = 0;
int destinoAtual = 0;
float velocidadeAtual = 0.1; // segundos por 60 graus
bool emMovimento = false;

// === IMPLEMENTAÇÃO 04 ===
bool emImplementacao04 = false;
int passos04[] = {30, 120, 75, 100, 0};
int indexPasso = 0;
unsigned long tempoInicioPasso = 0;
bool esperandoParada = false;
float velocidade04[] = {0.34, 0.34, 0.34, 0.17};
unsigned long ultimaAtualizacaoStatus = 0;
float velocidadeRadS = 1.57; // valor padrão inicial

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Controle de Junta Rotativa</title>
  <style>
    html, body {
      margin: 0; padding: 0; height: 100%;
      font-family: Arial, sans-serif; background-color: #f5f5f5;
    }
    .container {
      height: 100%; display: flex; flex-direction: column;
      justify-content: center; align-items: center;
    }
    .card {
      background: white; padding: 20px; border-radius: 12px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.1);
      max-width: 500px; width: 90%;
    }
    h2 { text-align: center; color: teal; }
    .slider-group { margin: 16px 0; }
    input[type=range] { width: 100%; }
    .button-group {
      display: flex; gap: 10px; justify-content: center;
      margin-top: 16px;
    }
    .button-group button {
      flex: 1; background: #008080; border: none; padding: 10px;
      border-radius: 8px; color: white; font-weight: bold;
      cursor: pointer;
    }
    #status, #passoStatus {
      text-align: center; color: gray; margin-top: 12px;
      font-style: italic;
    }
    .protractor {
      position: relative; height: 200px; margin-bottom: 20px;
    }
    .needle {
      width: 4px; height: 80px; background-color: teal;
      border-radius: 2px; position: absolute; bottom: 0;
      left: 50%; transform-origin: bottom center; transform: rotate(0deg);
    }
    .custom-angle {
      margin-top: 10px; display: flex;
      justify-content: center; gap: 10px;
    }
    .custom-angle input {
      width: 60px; padding: 5px; text-align: center;
    }
    .imp04 {
      margin-top: 16px; text-align: center;
    }
    .imp04 button {
      padding: 10px 20px; background-color: darkorange;
      color: white; border: none; border-radius: 8px;
      font-weight: bold; cursor: pointer;
    }
    .velocidade { margin-top: 16px; }
    .info-status {
      margin-top: 10px; text-align: center;
      font-size: 15px; color: #444;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="card">
      <h2>Controle de Junta Rotativa</h2>
      <div class="protractor">
        <div class="needle" id="needle"></div>
      </div>
      <div class="slider-group">
        <label for="angle">Ângulo: <span id="angleLabel">0</span>°</label>
        <input type="range" id="angle" min="0" max="180" value="0">
      </div>
      <div class="button-group">
        <button onclick="setAngle(0)">0°</button>
        <button onclick="setAngle(90)">90°</button>
        <button onclick="setAngle(180)">180°</button>
      </div>
      <div class="custom-angle">
        <input type="number" id="customAngle" placeholder="Ângulo">
        <button onclick="sendCustomAngle()">Enviar</button>
      </div>
      <div class="velocidade">
        <label for="speedSlider">Velocidade (rad/s): <span id="speedLabel">1.57</span></label>
        <input type="range" id="speedSlider" min="0.15" max="10.0" step="0.01" value="1.57">
      </div>
      <div class="imp04">
        <button onclick="iniciarImp04()">Implementação 04</button>
      </div>
      <div id="status"></div>
      <div class="info-status">
        <div id="passoStatus"></div>
        <div id="anguloStatus"></div>
        <div id="velocidadeStatus"></div>
      </div>
    </div>
  </div>
  <script>
    let ws;
    const angleSlider = document.getElementById('angle');
    const angleLabel = document.getElementById('angleLabel');
    const status = document.getElementById('status');
    const passoStatus = document.getElementById('passoStatus');
    const anguloStatus = document.getElementById('anguloStatus');
    const velocidadeStatus = document.getElementById('velocidadeStatus');
    const customAngle = document.getElementById('customAngle');
    const speedSlider = document.getElementById('speedSlider');
    const speedLabel = document.getElementById('speedLabel');
    const needle = document.getElementById('needle');

    function connectWS() {
      ws = new WebSocket(`ws://${location.hostname}/ws`);

      ws.onopen = () => {
        status.innerText = "Conectado ao ESP32 via WebSocket";
      };

      ws.onerror = (err) => {
        status.innerText = "Erro na conexão WebSocket.";
        console.error("WebSocket erro:", err);
      };

      ws.onclose = () => {
        status.innerText = "Desconectado. Tentando reconectar...";
        setTimeout(connectWS, 2000);
      };

      ws.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          if (msg.passo !== undefined) {
            passoStatus.innerText = `Passo atual: ${msg.passo + 1}`;
          }
          if (msg.angulo !== undefined) {
            anguloStatus.innerText = `Ângulo atual: ${msg.angulo}°`;
            needle.style.transform = `rotate(${msg.angulo - 90}deg)`;
          }
          if (msg.velocidade !== undefined) {
            velocidadeStatus.innerText = `Velocidade: ${msg.velocidade.toFixed(2)} rad/s`;
          }
        } catch (e) {
          console.log("Mensagem inválida:", event.data);
        }
      };
    }

    connectWS();

    function setAngle(angle) {
      angle = Math.max(0, Math.min(180, angle));
      angleSlider.value = angle;
      sendToEsp(angle);
    }

    function sendToEsp(angle) {
      const data = { angulo: Math.round(angle) };
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(data));
        angleLabel.innerText = angle;
        needle.style.transform = `rotate(${angle - 90}deg)`;
        status.innerText = `Enviado: ângulo ${angle}°`;
      } else {
        status.innerText = "WebSocket não conectado.";
      }
    }

    angleSlider.addEventListener('input', () => {
      sendToEsp(angleSlider.value);
    });

    function sendCustomAngle() {
      let angle = parseInt(customAngle.value);
      if (isNaN(angle)) return;
      setAngle(angle);
    }

    speedSlider.addEventListener('input', () => {
      let value = parseFloat(speedSlider.value);
      speedLabel.innerText = value.toFixed(2);
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ velocidade: value }));
        status.innerText = `Velocidade definida: ${value.toFixed(2)} rad/s`;
      }
    });

    function iniciarImp04() {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ imp04: true }));
        status.innerText = "Executando Implementação 04...";
        passoStatus.innerText = "";
      } else {
        status.innerText = "WebSocket não conectado.";
      }
    }
  </script>
</body>
</html>
)rawliteral";

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) return;

  data[len] = 0;
  Serial.printf("Recebido via WebSocket: %s\n", (char*)data);

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, data)) return;

  if (doc.containsKey("imp04")) {
    emImplementacao04 = true;
    indexPasso = 0;
    esperandoParada = false;
    tempoInicioPasso = millis();
    destinoAtual = passos04[0];
  }

  if (doc.containsKey("angulo")) {
    destinoAtual = constrain(doc["angulo"], 0, 180);
    emMovimento = true;
    emImplementacao04 = false;
  }

  if (doc.containsKey("velocidade")) {
    float rad_s = constrain(doc["velocidade"].as<float>(), 0.15, 10.0);
    float deg_s = rad_s * (180.0 / PI);
    velocidadeAtual = 60.0 / deg_s;
    velocidadeRadS = rad_s;
    Serial.printf("Velocidade ajustada: %.2f rad/s (%.2f s por 60°)\n", rad_s, velocidadeAtual);
  }
}

void moverParaDestino() {
  if (!emMovimento) return;
  unsigned long agora = millis();
  float delayMsPorGrau = (velocidadeAtual / 60.0) * 1000.0;
  if (agora - ultimoMovimento >= delayMsPorGrau) {
    ultimoMovimento = agora;
    if (anguloAtual < destinoAtual) anguloAtual++;
    else if (anguloAtual > destinoAtual) anguloAtual--;
    servo.write(anguloAtual);
    if (anguloAtual == destinoAtual) emMovimento = false;
  }
}

void executarImplementacao04() {
  if (!emImplementacao04) return;
  unsigned long agora = millis();
  if (!esperandoParada) {
    destinoAtual = passos04[indexPasso];
    velocidadeAtual = (indexPasso < 3) ? (60.0 / (velocidade04[indexPasso] * 180.0 / PI)) : (60.0 / (0.17 * 180.0 / PI));
    emMovimento = true;
    esperandoParada = true;
  } else if (!emMovimento && (agora - tempoInicioPasso >= 2000)) {
    indexPasso++;
    tempoInicioPasso = agora;
    esperandoParada = false;
    if (indexPasso >= 5) emImplementacao04 = false;
  }
}

void enviarStatus() {
  if (millis() - ultimaAtualizacaoStatus > 500) {
    StaticJsonDocument<128> statusDoc;
    statusDoc["passo"] = indexPasso;
    statusDoc["angulo"] = anguloAtual;
    statusDoc["velocidade"] = velocidadeRadS;
    String msg;
    serializeJson(statusDoc, msg);
    ws.textAll(msg);
    ultimaAtualizacaoStatus = millis();
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) handleWebSocketMessage(arg, data, len);
}

void setup() {
  Serial.begin(115200);
  servo.attach(VESPA_SERVO_S1, SERVO_MIN, SERVO_MAX);
  servo.write(anguloAtual);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("Acesse: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", index_html);
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  moverParaDestino();
  executarImplementacao04();
  enviarStatus();
  yield();
}
