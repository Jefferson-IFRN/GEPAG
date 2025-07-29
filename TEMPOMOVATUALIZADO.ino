#include <HardwareSerial.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_SHT31.h>
#include "esp_wifi.h"
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Update.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_cntl.h"
#include <ESP32Time.h>

/* ==================== CONFIGURAÇÃO DE PINOS ==================== */
#define A7670_TX 26
#define A7670_RX 27
#define A7670_PWR_PIN 4
#define LED_VERMELHO 18
#define LED_AZUL 19
#define BAT_EN_PIN 12
#define ADC_SOLAR 36
#define BATERIA_PIN 35
#define SENSORS_PWR_PIN 23
#define I2C_SDA 21
#define I2C_SCL 22
#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13
#define BOTAO_PIN 32

/* ==================== CONSTANTES ==================== */
#define TEMPO_COLETA 60000      // 1min 60000
#define TEMPO_SEM_GPS 180000    // 10min 600000
#define TIMEOUT_SEQUENCIA 5000  // 5s 5000
#define TEMPO_DORMIR 300000     // 5min 300000

/* ===================servidor web=====================*/

#define TEMPO_WEBSERVER 300000  // 5min (alterado para tempo decorrido)
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);

// Credenciais para atualização OTA
const char* otaUsername = "admin";
const char* otaPassword = "senha123";
bool otaAuthenticated = false;

/*=====================================================*/
#define TEMPO_PRESSIONADO 3000
#define TEMPO_AMOSTRAGEM 100
#define TIMEOUT_BOTAO_WAKEUP 40000         // 40 segundos
#define TEMPO_VERIFICACAO_MOVIMENTO 10000  // 10 segundos
#define LIMIAR_VIBRACAO 1.5
#define LIMIAR_VIBRACAO_DINAMICO 0.3
#define GRAVIDADE 9.81
#define ADC_MAX 4095
#define VOLTAGE_DIVIDER_RATIO 2.2

/* ==================== VARIÁVEIS GLOBAIS ==================== */


HardwareSerial A7670(1);
MPU9250 mpu;
Adafruit_SHT31 sht30;
bool sht30_encontrado = false;
bool mpu_encontrado = false;
float gravX = 0, gravY = 0, gravZ = 0;
bool calibrado = false;
float ultimaAceleracao = 0;
unsigned long ultimoTempo = 0;

String dataGNSS = " ";
String horaGNSS = " ";

/*RTC_DATA_ATTR struct {
  uint32_t ultimoRegistro;
  float ultimaTensao;
  bool emMovimento;
} estadoSistema;
RTC_DATA_ATTR ESP32Time rtc; // Para guardar data e hora
RTC_DATA_ATTR bool tensaoCritica = false; // Salva estado de tensão crítica no RTC
RTC_DATA_ATTR int Ciclos = 0;*/

const float LIMIAR_DESLIGAMENTO = 3.3;  // Desliga abaixo de 3.3V
const float LIMIAR_RELIGAMENTO = 3.6;   // Religar acima de 3.6V
enum EstadoBotao {
  AGUARDANDO_PRIMEIRO,
  AGUARDANDO_SEGUNDO,
  SERVICO_ATIVO
};
EstadoBotao estadoBotao = AGUARDANDO_PRIMEIRO;

unsigned long tempoPressionado = 0;
bool botaoPressionado = false;
unsigned long ultimoPressionamento = 0;
unsigned long ultimoPiscar = 0;
unsigned long inicioColetaValida = 0;
unsigned long ultimaLeituraMPU = 0;
unsigned long ultimoFixValido = 0;
unsigned long tempoInicioServidor = 0;  // Adicionado para controle do tempo do servidor

bool sinalGnssValido = false;
bool primeiroFixo = false;
bool emMovimento = false;
bool semSinalGPS = false;
bool dadosSemGPSGravados = false;
bool isUpdating = false;

DNSServer dnsServer;
WebServer server(80);
const char* nomeArquivo = "/dados.txt";
uint8_t mac[6];
char macStr[18];
File arquivoSD;

/*===================================================*/
/*===================================================*/
/* ==================== FUNÇÕES ==================== */
/*===================================================*/
/*===================================================*/


void ligarSensores() {
  digitalWrite(SENSORS_PWR_PIN, HIGH);
  delay(50);
  Wire.begin(I2C_SDA, I2C_SCL);
}

void desligarSensores() {
  digitalWrite(SENSORS_PWR_PIN, LOW);
}

/* =============================== Inicializar SD =============================== */

void inicializarSD() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("Falha ao inicializar o cartão SD!");
    return;
  }
  Serial.println("Cartão SD inicializado com sucesso!");
}
//Adicionado por Breno em 26/05
/* =============================== Ler RTC =============================== */
/*String lerRTC() {
  return rtc.getDateTime("%d/%m/%Y %H:%M:%S");
}*/

/* =============================== MPU =============================== */
void inicializarMPU9250() {
  if (!mpu.setup(0x68)) {
    Serial.println("Falha ao inicializar MPU9250!");
    mpu_encontrado = false;
    return;
  }
  mpu_encontrado = true;
  mpu.calibrateAccelGyro();
  calibrarGravidade();
}

/* =============================== SHT30 =============================== */
void inicializarSHT30() {
  if (!sht30.begin(0x44)) {
    Serial.println("SHT30 não encontrado!");
    sht30_encontrado = false;
  } else {
    sht30_encontrado = true;
    Serial.println("SHT30 inicializado com sucesso");
  }
}


/* =============================== Calibração de Gravidade =============================== */
void calibrarGravidade() {
  if (mpu.update()) {
    gravX = mpu.getAccX();
    gravY = mpu.getAccY();
    gravZ = mpu.getAccZ();
    calibrado = true;
    Serial.println("Gravidade calibrada:");
    Serial.print("X: ");
    Serial.print(gravX);
    Serial.print(" | Y: ");
    Serial.print(gravY);
    Serial.print(" | Z: ");
    Serial.println(gravZ);
  }
}


/*=========================Calcula Tempo DeepSleep==========================*/
unsigned long calcularTempoSleep(float tensaoBateria) {
  if (tensaoBateria >= 4.0) return 60000;       // 300000
  else if (tensaoBateria >= 3.9) return 60000;  //600000
  else if (tensaoBateria >= 3.8) return 60000;  // 1200000
  else if (tensaoBateria >= 3.7) return 60000;  // 1800000
  else return 60000;                            //2400000
}

float lerBateria() {
  const int numLeituras = 10;
  const float maxVariacao = 0.1;
  float leituras[numLeituras];
  float soma = 0;
  float referencia = (analogRead(BATERIA_PIN) * 3.3 / ADC_MAX * VOLTAGE_DIVIDER_RATIO);

  for (int i = 0; i < numLeituras; i++) {
    float leitura = (analogRead(BATERIA_PIN) * 3.3 / ADC_MAX * VOLTAGE_DIVIDER_RATIO);
    if (abs(leitura - referencia) < maxVariacao) {
      leituras[i] = leitura;
      soma += leitura;
    } else {
      leituras[i] = referencia;
      soma += referencia;
    }
    delay(10);
  }
  return soma / numLeituras;
}

float lerSolar() {
  const int numLeituras = 10;
  const float fatorDivisor = 2.0;
  const float tensaoMaximaADC = 3.3;
  const float tensaoMaximaPainel = 24.0;

  float soma = 0;
  float referencia = (analogRead(ADC_SOLAR) * tensaoMaximaADC / ADC_MAX) * fatorDivisor;

  for (int i = 0; i < numLeituras; i++) {
    float leitura = (analogRead(ADC_SOLAR) * tensaoMaximaADC / ADC_MAX) * fatorDivisor;
    if (leitura > tensaoMaximaPainel) leitura = tensaoMaximaPainel;
    if (abs(leitura - referencia) < 0.1) {
      soma += leitura;
    } else {
      soma += referencia;
    }
    delay(10);
  }
  return (soma / numLeituras > tensaoMaximaPainel) ? tensaoMaximaPainel : soma / numLeituras;
}


/*===============================Liga GNSS==============================================*/

void ligarModuloGNSS() {
  digitalWrite(A7670_PWR_PIN, HIGH);
  delay(3000);
  delay(100);
  delay(20000);

  Serial.println("Módulo A7670 ligado. Aguardando fixo GNSS...");
  enviarComando("AT+CGNSSPWR=1", 3000);
  enviarComando("AT+CGNSSPWR=0", 2000);
  enviarComando("AT+CGNSSPWR=1", 5000);
  enviarComando("AT+CGPSHOT", 2000);
  enviarComando("AT+CGNSSINFO=1", 2000);
  enviarComando("AT+CGNSSPORTSWITCH=1,1", 1000);
}


void enviarComando(const char* comando, int espera_ms) {
  A7670.println(comando);
  Serial.print("Enviando: ");
  Serial.println(comando);
  delay(espera_ms);
  while (A7670.available()) {
    String resposta = A7670.readStringUntil('\n');
    resposta.trim();
    if (resposta.length() > 0) Serial.println(resposta);
  }
}

/*===============================Gravando data e hora no RTC do ESP32==============================================*/
void atualizarRTCApartirGNSS(String datasGNSS, String horasGNSS) {
  int dia = dataGNSS.substring(0, 2).toInt();
  int mes = dataGNSS.substring(2, 4).toInt() - 1;  // Janeiro = 0
  int ano = 2000 + dataGNSS.substring(4, 6).toInt();

  int hora = horaGNSS.substring(0, 2).toInt();
  int minuto = horaGNSS.substring(2, 4).toInt();
  int segundo = horaGNSS.substring(4, 6).toInt();

  // Configura RTC com horário UTC-3 (Brasília)
  //rtc.setTime(segundo, minuto, hora, dia, mes, ano);
  //rtc.offset = -3 * 3600; // Offset de -3 horas

  //Serial.println("RTC atualizado: " + rtc.getDateTime());
}
/*=============================================================================================*/


/* =============================== Processando dados do GNSS =============================== */
bool processarGNSS(String dadosBrutos) {
  dadosBrutos.trim();
  dadosBrutos.replace("+CGNSSINFO: ", "");

  String campos[20];
  int campoIndex = 0;
  int pos = 0;

  while (campoIndex < 20) {
    int virgula = dadosBrutos.indexOf(',', pos);
    if (virgula == -1) {
      campos[campoIndex++] = dadosBrutos.substring(pos);
      break;
    }
    campos[campoIndex++] = dadosBrutos.substring(pos, virgula);
    pos = virgula + 1;
  }

  bool dadosOK = campoIndex >= 11 && campos[5].length() > 0 && campos[7].length() > 0 && campos[9].length() == 6 && (campos[10].length() >= 6 || campos[10].indexOf('.') != -1);

  if (dadosOK) {
    dadosSemGPSGravados = false;
    String lat = campos[5] + (campos[6].length() > 0 ? campos[6] : "");
    String lon = campos[7] + (campos[8].length() > 0 ? campos[8] : "");
    String data = campos[9];
    dataGNSS = data;
    String hora = campos[10];
    horaGNSS = hora;
    String alt = (campoIndex > 11 && campos[11].length() > 0) ? campos[11] : "null";
    String vel = (campoIndex > 12 && campos[12].length() > 0) ? campos[12] : "null";

    atualizarRTCApartirGNSS(dataGNSS, horaGNSS);

    String dataFormatada = data.substring(0, 2) + "/" + data.substring(2, 4) + "/20" + data.substring(4, 6);
    if (hora.indexOf('.') != -1) hora = hora.substring(0, hora.indexOf('.'));
    String horaFormatada = hora.substring(0, 2) + ":" + hora.substring(2, 4) + ":" + hora.substring(4, 6);

    float bateriaVolts = lerBateria();
    float tensaoSolar = lerSolar();
    float temperatura = 0, umidade = 0;

    if (sht30_encontrado) {
      temperatura = sht30.readTemperature();
      umidade = sht30.readHumidity();
      if (isnan(temperatura)) temperatura = 0;
      if (isnan(umidade)) umidade = 0;
    }

    String json = "{";
    json += "\"ID\":\"" + getDeviceID() + "\",";
    json += "\"Data\":\"" + dataFormatada + "\",";
    json += "\"Hora\":\"" + horaFormatada + "\",";
    json += "\"Lat\":" + lat + ",";
    json += "\"Long\":" + lon + ",";
    json += "\"Alt\":" + alt + ",";
    json += "\"Vel\":" + vel + ",";
    json += "\"Mov\":" + String(emMovimento ? "1" : "0") + ",";
    json += "\"Temp\":" + String(temperatura, 1) + ",";
    json += "\"Umid\":" + String(umidade, 1) + ",";
    json += "\"Bat\":" + String(bateriaVolts, 2) + ",";
    json += "\"Solar\":" + String(tensaoSolar, 2);
    json += "}";

    Serial.println(json);
    escreverNoSD(json);
    return true;
  } else {
    if (!dadosSemGPSGravados) {
      float bateriaVolts = lerBateria();
      float tensaoSolar = lerSolar();
      float temperatura = 0, umidade = 0;

      if (sht30_encontrado) {
        temperatura = sht30.readTemperature();
        umidade = sht30.readHumidity();
        if (isnan(temperatura)) temperatura = 0;
        if (isnan(umidade)) umidade = 0;
      }
      String lat = campos[5] + (campos[6].length() > 0 ? campos[6] : "");
      String lon = campos[7] + (campos[8].length() > 0 ? campos[8] : "");
      String data = campos[9];
      dataGNSS = data;
      String hora = campos[10];
      horaGNSS = hora;
      String alt = (campoIndex > 11 && campos[11].length() > 0) ? campos[11] : "null";
      String vel = (campoIndex > 12 && campos[12].length() > 0) ? campos[12] : "null";

      //String dataFormatada = rtc.getTime("%d/%m/%Y");
      //String horaFormatada = rtc.getTime("%H:%M:%S");
      String dataFormatada = data.substring(0, 2) + "/" + data.substring(2, 4) + "/20" + data.substring(4, 6);
      if (hora.indexOf('.') != -1) hora = hora.substring(0, hora.indexOf('.'));
      String horaFormatada = hora.substring(0, 2) + ":" + hora.substring(2, 4) + ":" + hora.substring(4, 6);

      String json = "{";
      json += "\"ID\":\"" + getDeviceID() + "\",";
      json += "\"Data\":\"" + String(dataFormatada) + "\",";
      json += "\"Hora\":\"" + String(horaFormatada) + "\",";
      json += "\"Mov\":" + String(emMovimento ? "1" : "0") + ",";
      json += "\"Temp\":" + String(temperatura, 1) + ",";
      json += "\"Umid\":" + String(umidade, 1) + ",";
      json += "\"Bat\":" + String(bateriaVolts, 2) + ",";
      json += "\"Solar\":" + String(tensaoSolar, 2);
      json += "}";

      Serial.println(json);
      escreverNoSD(json);
      dadosSemGPSGravados = true;
    }
    return false;
  }
}

/* =============================== Escrevendo no Cartão SD =============================== */
void escreverNoSD(String dados) {
  arquivoSD = SD.open("/dados.txt", FILE_APPEND);
  if (arquivoSD) {
    arquivoSD.println(dados);
    arquivoSD.close();
    Serial.println("Dados gravados no SD");
  } else {
    Serial.println("Erro ao abrir arquivo no SD");
  }
}


String getDeviceID() {
  uint64_t chipid = ESP.getEfuseMac();
  String id = String((uint32_t)(chipid >> 24), HEX);
  id.toUpperCase();
  return id;
}

void verificarMovimento() {
  if (estadoBotao == AGUARDANDO_PRIMEIRO && millis() - ultimaLeituraMPU >= TEMPO_AMOSTRAGEM) {
    if (mpu.update() && calibrado) {
      float ax = mpu.getAccX() - gravX;
      float ay = mpu.getAccY() - gravY;
      float az = mpu.getAccZ() - gravZ;

      float aceleracaoResultante = sqrt(ax * ax + ay * ay + az * az);
      float variacao = 0;

      if (ultimoTempo > 0) {
        float deltaT = (millis() - ultimoTempo) / 1000.0;
        variacao = abs(aceleracaoResultante - ultimaAceleracao) / deltaT;
      }

      ultimaAceleracao = aceleracaoResultante;
      ultimoTempo = millis();

      if (variacao > LIMIAR_VIBRACAO_DINAMICO) {
        emMovimento = true;
        if (estadoBotao == AGUARDANDO_PRIMEIRO) digitalWrite(LED_AZUL, HIGH);
      } else {
        emMovimento = false;
        if (estadoBotao == AGUARDANDO_PRIMEIRO) digitalWrite(LED_AZUL, LOW);
      }
    }
    ultimaLeituraMPU = millis();
  }
}

bool verificarMovimentoInicial() {
  unsigned long inicioVerificacao = millis();
  bool movimentoDetectado = false;

  Serial.println("Iniciando verificação de movimento por 10s...");
  digitalWrite(LED_AZUL, HIGH);  // Liga LED durante verificação

  while (millis() - inicioVerificacao < TEMPO_VERIFICACAO_MOVIMENTO) {
    if (mpu.update() && calibrado) {
      float ax = mpu.getAccX() - gravX;
      float ay = mpu.getAccY() - gravY;
      float az = mpu.getAccZ() - gravZ;

      float aceleracaoResultante = sqrt(ax * ax + ay * ay + az * az);
      float variacao = 0;

      if (ultimoTempo > 0) {
        float deltaT = (millis() - ultimoTempo) / 1000.0;
        variacao = abs(aceleracaoResultante - ultimaAceleracao) / deltaT;
      }

      ultimaAceleracao = aceleracaoResultante;
      ultimoTempo = millis();

      if (variacao > LIMIAR_VIBRACAO_DINAMICO) {
        movimentoDetectado = true;
        Serial.println("Movimento detectado!");
        break;
      }
    }
    delay(100);
  }

  digitalWrite(LED_AZUL, LOW);  // Desliga LED após verificação
  return movimentoDetectado;
}

void showUpdatePage() {
  String html = R"rawliteral(
<!DOCTYPE html><html lang="pt-br"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Atualização OTA</title><style>body{font-family:Arial,sans-serif;background-color:#f2f2f2;text-align:center;padding:40px;}
.container{background-color:#fff;padding:30px;border-radius:10px;box-shadow:0 0 15px rgba(0,0,0,0.1);display:inline-block;}
h1{color:#333;}form{margin:20px 0;}input[type="file"]{margin:10px 0;}button{background-color:#4CAF50;color:white;padding:10px 20px;font-size:16px;border:none;border-radius:5px;cursor:pointer;}
button:hover{background-color:#45a049;}#status{margin:15px 0;font-weight:bold;}.back-button{background-color:#f44336;}.back-button:hover{background-color:#d32f2f;}
</style></head><body><div class="container"><h1>Atualização de Firmware</h1>
<form method="POST" action="http://192.168.4.1/update" enctype="multipart/form-data">
<input type="file" name="firmware"><br><br><p id="status">Selecione o arquivo .bin</p><button type="submit">Atualizar</button>
</form><p><a href="http://192.168.4.1/"><button class="back-button">Voltar</button></a></p></div></body></html>)rawliteral";
  server.send(200, "text/html", html);
}

void iniciarServidorWeb() {
  tempoInicioServidor = millis();
  WiFi.softAP("Dispositivo_MTA");
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  dnsServer.start(DNS_PORT, "*", apIP);
  digitalWrite(LED_AZUL, HIGH);


  server.on("/", []() {
    String html = R"rawliteral(
<!DOCTYPE html><html lang="pt-br"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MTA Portal</title><style>body{font-family:Arial,sans-serif;background-color:#f2f2f2;text-align:center;padding:40px;}
.container{background-color:#fff;padding:30px;border-radius:10px;box-shadow:0 0 15px rgba(0,0,0,0.1);display:inline-block;}
h1{color:#333;}button{background-color:#4CAF50;color:white;padding:15px 30px;font-size:16px;border:none;border-radius:5px;cursor:pointer;margin:5px;}
button:hover{background-color:#45a049;}.update-btn{background-color:#2196F3;}.update-btn:hover{background-color:#0b7dda;}
footer{margin-top:30px;font-size:12px;color:#777;}</style></head>
<body><div class="container"><h1>Portal do Dispositivo MTA</h1>
<p><a href="http://192.168.4.1/download"><button>📥 Baixar Dados</button></a></p>
<p><a href="http://192.168.4.1/update"><button class="update-btn">🔄 Atualizar Firmware</button></a></p>
<footer>© 2025 - projeto GEPAG</footer></div></body></html>)rawliteral";
    server.send(200, "text/html", html);
  });

  server.on("/download", []() {
    if (SD.exists(nomeArquivo)) {
      File file = SD.open(nomeArquivo, FILE_READ);
      if (file) {
        server.sendHeader("Content-Type", "application/octet-stream");
        server.sendHeader("Content-Disposition", "attachment; filename=dados.json");
        server.sendHeader("Connection", "close");
        server.streamFile(file, "application/octet-stream");
        file.close();
        return;
      }
    }
    server.send(404, "text/plain", "Arquivo não encontrado");
  });

  // Página de login OTA (será acessada quando clicar em Atualizar Firmware)
  server.on("/update", HTTP_GET, []() {
    if (otaAuthenticated) {
      showUpdatePage();
    } else {
      String loginPage = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Login OTA</title><style>body{font-family:Arial,sans-serif;background-color:#f2f2f2;text-align:center;padding:40px;}
.container{background-color:#fff;padding:30px;border-radius:10px;box-shadow:0 0 15px rgba(0,0,0,0.1);display:inline-block;}
h1{color:#333;}form{margin:20px 0;}input{padding:10px;margin:5px 0;width:90%;}button{background-color:#4CAF50;color:white;padding:10px 20px;font-size:16px;border:none;border-radius:5px;cursor:pointer;}
button:hover{background-color:#45a049;}.error{color:red;}</style></head>
<body><div class="container"><h1>Autenticação OTA</h1>
<form method="POST" action="/login">
<input type="text" name="username" placeholder="Usuário" required><br>
<input type="password" name="password" placeholder="Senha" required><br>
<button type="submit">Login</button>
</form>)rawliteral";

      if (server.hasArg("fail")) {
        loginPage += "<p class='error'>Credenciais inválidas!</p>";
      }

      loginPage += R"rawliteral(<p><a href="http://192.168.4.1/"><button style="background-color:#f44336;">Voltar</button></a></p>
</div></body></html>)rawliteral";

      server.send(200, "text/html", loginPage);
    }
  });

  // Processar login OTA
  server.on("/login", HTTP_POST, []() {
    if (server.hasArg("username") && server.hasArg("password")) {
      if (server.arg("username") == otaUsername && server.arg("password") == otaPassword) {
        otaAuthenticated = true;
        showUpdatePage();
      } else {
        server.sendHeader("Location", "/update?fail=1");
        server.send(302, "text/plain", "Redirecionando...");
      }
    } else {
      server.send(400, "text/plain", "Faltam credenciais");
    }
  });

  // Página de atualização (após login)
  server.on(
    "/update", HTTP_POST, []() {
      if (!otaAuthenticated) {
        server.sendHeader("Location", "/update");
        server.send(302, "text/plain", "Redirecionando...");
        return;
      }

      isUpdating = true;
      digitalWrite(LED_VERMELHO, HIGH);
      digitalWrite(LED_AZUL, HIGH);
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", "Atualizacao completa! Reiniciando...");
      delay(1000);
      ESP.restart();
    },
    []() {
      if (!otaAuthenticated) return;

      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        isUpdating = true;
        Serial.println("Iniciando atualizacao...");
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
          isUpdating = false;
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          Serial.printf("Atualizacao concluida: %u bytes\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });

  server.onNotFound([]() {
    String html = R"rawliteral(
<!DOCTYPE html><html><head><meta http-equiv="refresh" content="0; url=http://192.168.4.1/" />
<script>window.location.href = "http://192.168.4.1/";</script><title>Redirecionando</title></head>
<body><p style="text-align: center; margin-top: 50px;">Redirecionando... <br>
<a href="http://192.168.4.1/">Clique aqui se não for redirecionado</a></p></body></html>)rawliteral";
    server.send(200, "text/html", html);
  });

  server.on("/favicon.ico", []() {
    server.send(204);
  });
  server.begin();
  Serial.println("Servidor iniciado");
}

// Função auxiliar para mostrar a página de atualização


void finalizarServicoWeb() {
  digitalWrite(LED_VERMELHO, LOW);
  digitalWrite(LED_AZUL, LOW);
  estadoBotao = AGUARDANDO_PRIMEIRO;
  otaAuthenticated = false;
  server.stop();
  WiFi.softAPdisconnect(true);
  Serial.println("Serviço web finalizado - preparando para deep sleep");
}

void verificarSequenciaBotao() {
  static unsigned long tempoEsperaSegundo = 0;
  static bool segundoPressionamentoIniciado = false;
  int estadoAtual = digitalRead(BOTAO_PIN);

  if (estadoBotao == AGUARDANDO_SEGUNDO) {
    if (millis() - tempoEsperaSegundo > TIMEOUT_SEQUENCIA) {
      digitalWrite(LED_VERMELHO, LOW);
      estadoBotao = AGUARDANDO_PRIMEIRO;
      Serial.println("Timeout: Segundo pressionamento não realizado");
      return;
    }

    if (estadoAtual == HIGH && !segundoPressionamentoIniciado) {
      segundoPressionamentoIniciado = true;
      tempoPressionado = millis();
      Serial.println("Início do segundo pressionamento detectado");
    }

    if (segundoPressionamentoIniciado && estadoAtual == LOW) {
      unsigned long tempoDecorrido = millis() - tempoPressionado;
      segundoPressionamentoIniciado = false;

      if (tempoDecorrido >= TEMPO_PRESSIONADO) {
        digitalWrite(LED_AZUL, HIGH);
        estadoBotao = SERVICO_ATIVO;
        iniciarServidorWeb();
        Serial.println("Serviço web ativado!");
      }
    }
    return;
  }

  if (estadoAtual == HIGH && !botaoPressionado) {
    botaoPressionado = true;
    tempoPressionado = millis();
    Serial.println("Primeiro pressionamento iniciado");
  } else if (botaoPressionado && estadoAtual == LOW) {
    botaoPressionado = false;
    unsigned long tempoDecorrido = millis() - tempoPressionado;

    if (tempoDecorrido >= TEMPO_PRESSIONADO) {
      digitalWrite(LED_VERMELHO, HIGH);
      estadoBotao = AGUARDANDO_SEGUNDO;
      tempoEsperaSegundo = millis();
      Serial.println("Primeiro pressionamento completo. Aguardando segundo...");
    }
  }
}
//adicionado por Breno em 26/05
/* =============================== Proteção contra subtensão =============================== */
void configurarProtecaoEnergia() {
  // Habilita o BOD durante operação normal (1.1V é o limiar mínimo seguro)
  Serial.print("Configurando proteção de energia");
#if CONFIG_IDF_TARGET_ESP32
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,
                 RTC_CNTL_BROWN_OUT_ENA |        // Habilita detecção
                   RTC_CNTL_BROWN_OUT_RST_ENA);  // Habilita reset automático
#else
  // Para ESP32-S2/S3/C3 (com registradores diferentes)
  CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_RST_ENA_M);
  SET_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA_M);
#endif

  // Configura para manter o BOD ativo durante deep sleep
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}
/* =============================== DeepSleep =============================== */

void entrarDeepSleep(bool forcarSleep = false) {
  configurarProtecaoEnergia();
  if (!forcarSleep) {
    float tensaoBateria = lerBateria();
    unsigned long tempoSleep = calcularTempoSleep(tensaoBateria);
    Serial.print("\nTensão da bateria: ");
    Serial.print(tensaoBateria);
    Serial.print("V - Entrando em deep sleep por ");
    Serial.print(tempoSleep / 60000);
    Serial.println(" minutos");
  } else {
    Serial.println("Timeout do botão - Entrando em deep sleep");
  }

  digitalWrite(LED_VERMELHO, LOW);
  digitalWrite(LED_AZUL, LOW);
  estadoBotao = AGUARDANDO_PRIMEIRO;

  if (primeiroFixo) {
    enviarComando("AT+CGNSSPWR=0", 2000);
    delay(1000);
    digitalWrite(A7670_PWR_PIN, LOW);
  }

  if (estadoBotao == SERVICO_ATIVO) {
    server.stop();
    WiFi.softAPdisconnect(true);
  }

  desligarSensores();
  digitalWrite(BAT_EN_PIN, LOW);
  delay(1000);

  Serial.flush();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BOTAO_PIN, HIGH);
  if (!forcarSleep) {
    esp_sleep_enable_timer_wakeup(calcularTempoSleep(lerBateria()) * 1000);
  } else {
    esp_sleep_enable_timer_wakeup(30000);
  }
  esp_deep_sleep_start();
}


//adicionado por Breno em 26/05

//adicionado por Breno em 26/05
/* =============================== Gerenciamento de Energia =============================== */
void verificarEnergia() {
  static unsigned long ultimaVerificacao = 0;

  if (millis() - ultimaVerificacao > 10000) {  // Verifica a cada 10s
    ultimaVerificacao = millis();
    float tensao = lerBateria();

    if (/*!tensaoCritica && */ tensao < LIMIAR_DESLIGAMENTO) {
      escreverNoSD("Tensão crítica (" + String(tensao, 2) + "V) - entrando em deep sleep");
      //tensaoCritica = true;
      Serial.print("Tensão Crítica");
      entrarDeepSleep();
    } else if (/*tensaoCritica && */ tensao >= LIMIAR_RELIGAMENTO) {
      //tensaoCritica = false; // Reset do flag quando a tensão se normaliza
      //Serial.print("Bateria ok!");
    }
  }
}

/* =============================== SETUP =============================== */
void setup() {
  configurarProtecaoEnergia();  // Configura a proteção contra subtensão - adicionado por Breno em 26/05
    // Configura verificação de reinicialização somente se a tensão for maior que 3.6V - adicionado por Breno em 26/05

  float tensaoAtual = lerBateria();
  if (/*tensaoCritica && */ tensaoAtual < LIMIAR_RELIGAMENTO) {
    escreverNoSD("Tensão baixa (" + String(tensaoAtual, 2) + "V) - mantendo deep sleep");
    entrarDeepSleep();  // Força voltar ao deep sleep
  }
  /*=============================================================================*/

  pinMode(LED_VERMELHO, OUTPUT);
  pinMode(LED_AZUL, OUTPUT);
  pinMode(BOTAO_PIN, INPUT_PULLUP);
  pinMode(SENSORS_PWR_PIN, OUTPUT);
  pinMode(BAT_EN_PIN, OUTPUT);
  pinMode(A7670_PWR_PIN, OUTPUT);

  digitalWrite(LED_VERMELHO, HIGH);  // Liga no início
  delay(3000);                       // Mantém ligado por 3s
  digitalWrite(LED_VERMELHO, LOW);   // Desliga depois
  digitalWrite(LED_AZUL, LOW);       // Inicia desligado
  digitalWrite(SENSORS_PWR_PIN, LOW);
  digitalWrite(BAT_EN_PIN, HIGH);

  //rtc_gpio_hold_dis((gpio_num_t)LED_VERMELHO);
  delay(3000);

  Serial.begin(115200);
  A7670.begin(115200, SERIAL_8N1, A7670_RX, A7670_TX);

  inicializarSD();

  static unsigned long ultimaVerificacaoRTC = 0;


  esp_sleep_wakeup_cause_t causa_wakeup = esp_sleep_get_wakeup_cause();

  if (causa_wakeup == ESP_SLEEP_WAKEUP_EXT0) {
    unsigned long wakeTime = millis();
    bool botaoPressionado = false;

    Serial.println("Wakeup por botão - Aguardando confirmação (40s)");

    // Aguarda por 40 segundos sem acender LEDs
    while (millis() - wakeTime < TIMEOUT_BOTAO_WAKEUP) {
      if (digitalRead(BOTAO_PIN) == HIGH) {
        botaoPressionado = true;
        break;
      }
      delay(100);
    }

    if (!botaoPressionado) {
      entrarDeepSleep();
      return;
    }

    Serial.println("Botão confirmado - Iniciando modo serviço web");
    digitalWrite(A7670_PWR_PIN, LOW);
    estadoBotao = AGUARDANDO_PRIMEIRO;
  } else if (causa_wakeup == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Wakeup por timer - verificando movimento por 10s");
    ligarSensores();
    delay(100);
    inicializarMPU9250();

    // Verificar movimento por 10 segundos
    if (!verificarMovimentoInicial()) {
      Serial.println("Nenhum movimento detectado - entrando em deep sleep");
      entrarDeepSleep();
      return;  // Não continua a execução
    }

    Serial.println("Movimento detectado - ligando GNSS e sensores");
    ligarModuloGNSS();
    inicializarSHT30();
    inicioColetaValida = millis();
    ultimoFixValido = millis();
  } else {
    Serial.println("Inicialização normal - verificando movimento por 10s");
    ligarSensores();
    delay(100);
    inicializarMPU9250();

    // Verificar movimento por 10 segundos
    if (!verificarMovimentoInicial()) {
      Serial.println("Nenhum movimento detectado - entrando em deep sleep");
      entrarDeepSleep();
      return;  // Não continua a execução
    }

    Serial.println("Movimento detectado - ligando GNSS e sensores");
    ligarModuloGNSS();
    inicializarSHT30();
    ultimoFixValido = millis();
  }
}
/* =============================== LOOP =============================== */
void loop() {
  verificarEnergia();  //verifcando se há tensão suficiente - adicionado por breno em 26/05

  // 1. Sempre verificar o estado do botão
  verificarSequenciaBotao();

  // 2. Modo Serviço Web Ativo
  if (estadoBotao == SERVICO_ATIVO) {
    // Processar requisições web
    server.handleClient();
    dnsServer.processNextRequest();

    // Piscar LED de status
    if (millis() - ultimoPiscar >= 500) {
      digitalWrite(LED_VERMELHO, !digitalRead(LED_VERMELHO));
      ultimoPiscar = millis();
    }

    // Verificar timeout do servido web (tempo decorrido)
    if (!isUpdating && (millis() - tempoInicioServidor > TEMPO_WEBSERVER)) {
      finalizarServicoWeb();
      entrarDeepSleep();
    }
  }
  // 3. Modo Aguardando Segundo Pressionamento
  else if (estadoBotao == AGUARDANDO_SEGUNDO) {
    // Piscar LED enquanto aguarda
    if (millis() % 1000 < 500) {
      digitalWrite(LED_VERMELHO, !digitalRead(LED_VERMELHO));
    }
  }
  // 4. Modo Normal (Coleta de Dados)
  else {
    // Verificar se terminou o tempo de coleta
    if (primeiroFixo && (millis() - inicioColetaValida >= TEMPO_COLETA)) {
      entrarDeepSleep();
    }

    // Verificar movimento
    verificarMovimento();

    // Processar dados do GNSS
    while (A7670.available()) {
      String linha = A7670.readStringUntil('\n');
      linha.trim();

      if (linha.startsWith("+CGNSSINFO:")) {
        sinalGnssValido = processarGNSS(linha);

        if (sinalGnssValido) {
          if (!primeiroFixo) {
            primeiroFixo = true;
            inicioColetaValida = millis();
            Serial.println("Primeiro fixo válido! Iniciando contagem...");
          }
          ultimoFixValido = millis();
        }
      }
    }

    // Verificar timeout do GNSS
    if (!sinalGnssValido && (millis() - ultimoFixValido > TEMPO_SEM_GPS)) {
      Serial.println("Timeout sem sinal GPS - dormindo");
      entrarDeepSleep();
    }

    // Feedback visual do GNSS
    if (sinalGnssValido) {
      if (millis() - ultimoPiscar >= 500) {
        digitalWrite(LED_VERMELHO, !digitalRead(LED_VERMELHO));
        ultimoPiscar = millis();
      }
    } else {
      digitalWrite(LED_VERMELHO, LOW);
    }
  }
}
