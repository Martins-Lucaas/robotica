#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <RoboCore_Vespa.h>

const char* ssid = "Martins 6";
const char* password = "17031998";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

VespaServo servo;
const uint16_t SERVO_MIN = 500;
const uint16_t SERVO_MAX = 3000;
int anguloAtual = 0;

unsigned long ultimoMovimento = 0;
int destinoAtual = 0;
float velocidadeAtual = 0.1;
bool emMovimento = false;

bool emImplementacao04 = false;
bool resetImplementacao04 = false;
int passos04[] = {30, 120, 75, 100, 0};
int indexPasso = 0;
unsigned long tempoInicioPasso = 0;
bool esperandoParada = false;
float velocidade04[] = {0.34, 0.34, 0.34, 0.17};
unsigned long ultimaAtualizacaoStatus = 0;
float velocidadeRadS = 1.57;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Controle de Junta Rotativa</title>
  <style>
    html, body {
      margin:0; padding:0; height:100%; font-family:Arial,sans-serif;
      background:#222; color:#eee;
    }
    .container {
      display:flex; align-items:center; justify-content:center; height:100%;
    }
    .card {
      background:#333; padding:30px; border-radius:12px;
      box-shadow:0 4px 12px rgba(0,0,0,0.5); width:95%; max-width:600px;
    }
    h2 {
      text-align:center; color:#0f8; margin-bottom:20px;
    }
    .protractor {
      position:relative; height:200px; margin-bottom:20px;
    }
    .needle {
      width:6px; height:90px; background:#0f8; border-radius:3px;
      position:absolute; bottom:0; left:50%; transform-origin:bottom center;
      transform:rotate(0deg);
    }
    .slider-group, .velocidade {
      margin:20px 0;
    }
    .slider-group label, .velocidade label {
      display:block; margin-bottom:8px; font-weight:bold;
    }
    input[type=range] {
      width:100%;
    }
    .button-group {
      display:flex; gap:12px; justify-content:center; margin-bottom:20px;
    }
    .button-group button {
      flex:1; padding:12px; border:none; border-radius:8px;
      background:#0f8; color:#111; font-weight:bold; cursor:pointer;
      font-size:1rem;
    }
    .custom-angle {
      display:flex; justify-content:center; gap:12px; margin-bottom:20px;
    }
    .custom-angle input {
      width:120px; padding:10px; font-size:1rem;
      border-radius:6px; border:2px solid #0f8;
      background:#111; color:#0f8; text-align:center;
    }
    .custom-angle button {
      padding:10px 20px; font-size:1rem; border:none;
      border-radius:6px; background:#0f8; color:#111;
      font-weight:bold; cursor:pointer;
    }
    .imp04 {
      text-align:center; margin-bottom:20px;
    }
    .imp04 button {
      padding:14px 40px; font-size:1.1rem; border:none;
      border-radius:8px; background:darkorange;
      color:#111; font-weight:bold; cursor:pointer;
    }
    #status {
      text-align:center; font-style:italic; margin-bottom:12px;
      color:#aaa;
    }
    .info-status {
      display:flex; justify-content:space-around;
      background:#111; padding:12px; border-radius:6px;
    }
    .digital {
      font-family: 'Courier New', monospace; font-size:1.2rem;
      background:#000; color:#0f8; padding:6px 12px;
      border-radius:4px; min-width:100px; text-align:center;
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
        <label for="angle">Ângulo:</label>
        <input type="range" id="angle" min="0" max="180" value="180">
      </div>
      <div class="button-group">
        <button onclick="setAngle(0)">0°</button>
        <button onclick="setAngle(90)">90°</button>
        <button onclick="setAngle(180)">180°</button>
      </div>
      <div class="custom-angle">
        <input type="number" id="customAngle" placeholder="Digite ângulo">
        <button onclick="sendCustomAngle()">Enviar</button>
      </div>
      <div class="velocidade">
        <label for="speedSlider">Velocidade (rad/s):</label>
        <input type="range" id="speedSlider" min="0.15" max="10.0" step="0.01" value="1.57">
      </div>
      <div class="imp04">
        <button onclick="iniciarImp04()">Implementação 04</button>
      </div>
      <div id="status">Aguardando ação...</div>
      <div class="info-status">
        <div class="digital" id="passoStatus">Passo: –</div>
        <div class="digital" id="anguloStatus">Ângulo: –°</div>
        <div class="digital" id="velocidadeStatus">Veloc.: –</div>
      </div>
    </div>
  </div>
  <script>
    let ws;
    function connectWS() {
      ws = new WebSocket(`ws://${location.hostname}/ws`);
      ws.onopen    = () => document.getElementById('status').innerText = "Conectado.";
      ws.onerror   = () => document.getElementById('status').innerText = "Erro de conexão.";
      ws.onclose   = () => { document.getElementById('status').innerText = "Desconectado."; setTimeout(connectWS,2000); };
      ws.onmessage = e => {
        const msg = JSON.parse(e.data);
        if(msg.passo !== undefined) document.getElementById('passoStatus').innerText = `Passo: ${msg.passo+1}`;
        if(msg.angulo !== undefined) {
          document.getElementById('anguloStatus').innerText = `Ângulo: ${msg.angulo}°`;
          document.getElementById('needle').style.transform = `rotate(${90 - msg.angulo}deg)`;
        }
        if(msg.velocidade !== undefined) {
          document.getElementById('velocidadeStatus').innerText = `Veloc.: ${msg.velocidade.toFixed(2)}`;
        }
      };
    }
    connectWS();

    function setAngle(angle) {
      angle = Math.max(0, Math.min(180, angle));
      document.getElementById('angle').value = 180 - angle;
      sendToEsp(angle);
    }
    function sendToEsp(angle) {
      const data = { angulo: angle };
      if(ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(data));
        document.getElementById('status').innerText = `Enviado: ${angle}°`;
        document.getElementById('needle').style.transform = `rotate(${90 - angle}deg)`;
      } else {
        document.getElementById('status').innerText = "WebSocket não conectado.";
      }
    }
    document.getElementById('angle').addEventListener('input', e => {
      const angle = 180 - parseInt(e.target.value);
      sendToEsp(angle);
    });
    function sendCustomAngle() {
      const a = parseInt(document.getElementById('customAngle').value);
      if(!isNaN(a)) setAngle(a);
    }
    document.getElementById('speedSlider').addEventListener('input', e => {
      const v = parseFloat(e.target.value);
      document.getElementById('status').innerText = `Velocidade: ${v.toFixed(2)} rad/s`;
      if(ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ velocidade: v }));
      }
    });
    function iniciarImp04() {
      if(ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ imp04: true }));
        document.getElementById('status').innerText = "Implementação 04 iniciada...";
        document.getElementById('passoStatus').innerText = "Passo: –";
      } else {
        document.getElementById('status').innerText = "WebSocket não conectado.";
      }
    }
  </script>
</body>
</html>
)rawliteral";


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  auto *info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index==0 && info->len==len && info->opcode==WS_TEXT))
    return;
  data[len]=0;
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, data)) return;

  // RESET + início da Implementação 04
  if (doc.containsKey("imp04")) {
    emImplementacao04   = true;
    resetImplementacao04 = true;         // <-- ativa a fase de reset
    indexPasso          = 0;
    esperandoParada     = false;
    // volta para 0° na velocidade máxima
    destinoAtual    = 0;
    velocidadeRadS  = 10.0f;
    {
      float deg_s       = velocidadeRadS * (180.0/PI);
      velocidadeAtual   = 60.0f / deg_s;
    }
    emMovimento      = true;
  }

  // controle manual normal (se não for imp04)
  if (doc.containsKey("angulo")) {
    destinoAtual   = constrain(doc["angulo"], 0, 180);
    emMovimento    = true;
    emImplementacao04 = false;
  }
  if (doc.containsKey("velocidade")) {
    float rad_s    = constrain(doc["velocidade"].as<float>(), 0.15f, 10.0f);
    float deg_s    = rad_s * (180.0/PI);
    velocidadeAtual = 60.0f / deg_s;
    velocidadeRadS  = rad_s;
  }
}

// --- movimentação para o destino definido ---
void moverParaDestino() {
  if (!emMovimento) return;
  unsigned long agora = millis();
  float delayMs = (velocidadeAtual/60.0f)*1000.0f;
  if (agora - ultimoMovimento >= delayMs) {
    ultimoMovimento = agora;
    if (anguloAtual < destinoAtual) anguloAtual++;
    else if (anguloAtual > destinoAtual) anguloAtual--;
    servo.write(anguloAtual);
    if (anguloAtual == destinoAtual) {
      emMovimento = false;
      tempoInicioPasso = agora;
    }
  }
}

// --- executa a sequência da Implementação 04 ---
void executarImplementacao04() {
  if (!emImplementacao04) return;
  unsigned long agora = millis();

  // 1) fase de reset: voltar a 0°
  if (resetImplementacao04) {
    // ainda não chegou a 0°?
    if (anguloAtual != 0 || emMovimento) {
      return;  // espera alcançar o 0°
    }
    // chegou a 0°, desativa reset e prepara primeiro passo
    resetImplementacao04 = false;
    esperandoParada      = false;
    indexPasso           = 0;
    // sem return, cai na lógica dos passos abaixo
  }

  // 2) aguardando pausa entre passos
  if (!emMovimento && esperandoParada && (agora - tempoInicioPasso >= 2000)) {
    indexPasso++;
    esperandoParada = false;
  }
  // 3) iniciar próximo passo, se houver
  if (!esperandoParada && indexPasso < 5) {
    destinoAtual = passos04[indexPasso];
    float rad_s  = (indexPasso < 3) ? velocidade04[indexPasso] : 0.17f;
    float deg_s  = rad_s * (180.0/PI);
    velocidadeAtual = 60.0f / deg_s;
    velocidadeRadS  = rad_s;
    emMovimento     = true;
    esperandoParada = true;
  }
  // 4) fim da sequência
  else if (indexPasso >= 5) {
    emImplementacao04 = false;
  }
}

// --- envia status periodicamente ao UI ---
void enviarStatus() {
  if (millis() - ultimaAtualizacaoStatus > 500) {
    StaticJsonDocument<128> st;
    st["passo"]      = indexPasso;
    st["angulo"]     = anguloAtual;
    st["velocidade"] = velocidadeRadS;
    String msg;
    serializeJson(st, msg);
    ws.textAll(msg);
    ultimaAtualizacaoStatus = millis();
  }
}

void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c,
               AwsEventType t, void *a, uint8_t *d, size_t l) {
  if (t == WS_EVT_DATA) handleWebSocketMessage(a, d, l);
}

void setup() {
  Serial.begin(115200);
  servo.attach(VESPA_SERVO_S1, SERVO_MIN, SERVO_MAX);
  servo.write(anguloAtual);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("Acesse: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
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