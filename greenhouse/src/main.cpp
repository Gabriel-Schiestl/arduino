// ============================================
// BIBLIOTECAS NECESSÁRIAS
// ============================================
#include <Arduino.h>       // Biblioteca principal do Arduino/ESP32
#include <WiFi.h>          // Biblioteca para conexão WiFi
#include <DHT.h>           // Biblioteca para sensor DHT11 (temperatura e umidade)
#include <ArduinoJson.h>   // Biblioteca para trabalhar com JSON
#include <vector>          // Biblioteca STL para vetores dinâmicos

// ============================================
// DEFINIÇÕES DE PINOS E SENSORES
// ============================================
#define DHTPIN 4           // Pino digital conectado ao sensor DHT11
#define DHTTYPE DHT11      // Tipo do sensor (DHT11)

// Pinos analógicos dos sensores
#define SOILSENSORPIN 34   // Pino analógico do sensor de umidade do solo
#define LIGHTLEVELPIN 35   // Pino analógico do sensor de luminosidade (LDR)

// Pinos digitais dos atuadores (relés)
#define IRRIGATIONPIN 14   // Pino digital para controle da irrigação
#define VENTILATIONPIN 27  // Pino digital para controle da ventilação
#define LIGHTINGPIN 26     // Pino digital para controle da iluminação

// ============================================
// ESTRUTURAS DE DADOS
// ============================================

/**
 * Estrutura para armazenar os dados lidos dos sensores
 * Contém todas as medições ambientais da estufa
 */
struct SensorData {
  float temperature;   // Temperatura em graus Celsius
  float humidity;      // Umidade relativa do ar em porcentagem
  float soilMoisture;  // Umidade do solo em porcentagem
  float lightLevel;    // Nível de luminosidade em porcentagem
};

/**
 * Estrutura para armazenar os comandos recebidos do servidor
 * Define o estado desejado para cada atuador da estufa
 */
struct DataResponse {
  bool ventilation;    // true = ligar ventilação, false = desligar
  bool irrigation;     // true = ligar irrigação, false = desligar
  bool lighting;       // true = ligar iluminação, false = desligar
};

// ============================================
// CONFIGURAÇÕES DE REDE E SERVIDOR
// ============================================
const char* ssid = "";              // Nome da rede WiFi
const char* password = "";          // Senha da rede WiFi
const char* serverIP = "";          // Endereço IP do servidor
const uint16_t serverPort = 3000;   // Porta TCP do servidor

// ============================================
// VARIÁVEIS DE CONTROLE DE TEMPO
// ============================================
unsigned long lastSensorRead = 0;          // Marca o último momento em que os sensores foram lidos
unsigned long lastIrrigation = 0;          // Marca o último momento em que a irrigação foi ativada
bool firstIrrigation = true;        
const unsigned long sensorInterval = 600000; // Intervalo entre leituras de sensores (600000ms = 10 minutos)

// ============================================
// OBJETOS GLOBAIS
// ============================================
WiFiClient client;              // Cliente WiFi para comunicação TCP com o servidor
DHT dht(DHTPIN, DHTTYPE);       // Objeto do sensor DHT11

// ============================================
// FUNÇÕES DE COMUNICAÇÃO COM O SERVIDOR
// ============================================

/**
 * Estabelece conexão TCP com o servidor
 * Tenta conectar repetidamente até obter sucesso
 * Usa as variáveis globais serverIP e serverPort
 */
void connectToServer() {
  Serial.println("Connecting to server...");
  // Loop de tentativas de conexão até conseguir
  while(!client.connect(serverIP, serverPort)) {
    Serial.println("Connection failed, retrying...");
    delay(2000);  // Aguarda 2 segundos antes de tentar novamente
  }
  Serial.println("Connected to server");
}

// ============================================
// FUNÇÕES DE LEITURA DE SENSORES
// ============================================

/**
 * Realiza a leitura de todos os sensores da estufa
 * 
 * Sensores lidos:
 * - DHT11: temperatura e umidade do ar
 * - Sensor de umidade do solo (analógico)
 * - Sensor de luminosidade LDR (analógico)
 * 
 * @return SensorData estrutura com todos os dados lidos
 *         Retorna valores -1 em caso de erro na leitura do DHT11
 */
SensorData readDataFromSensors() {
  // Lê temperatura e umidade do sensor DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Verifica se a leitura do DHT11 foi bem-sucedida
  // isnan() retorna true se o valor for "Not a Number" (erro de leitura)
  if(isnan(temperature) || isnan(humidity)) {
    return SensorData{-1, -1, -1};  // Retorna valores inválidos em caso de erro
  }

  // Lê o valor analógico do sensor de umidade do solo
  // ESP32 trabalha com resolução de 12 bits (0-4095)
  int soilMoisture = analogRead(SOILSENSORPIN);

  // Converte o valor bruto (4095-0) para porcentagem (0-100%)
  // 4095 = solo seco, 0 = solo úmido
  float soilMoisturePercentual = map(soilMoisture, 4095, 0, 0, 100);

  // Lê o valor analógico do sensor de luminosidade (LDR)
  int lightRaw = analogRead(LIGHTLEVELPIN);
  Serial.println(lightRaw);  // Debug: imprime valor bruto
  
  // Converte o valor bruto (0-4095) para porcentagem (0-100%)
  // 0 = escuro, 4095 = muito claro
  float lightPercent = map(lightRaw, 0, 4095, 0, 100);

  // Retorna estrutura com todos os dados coletados
  return SensorData{temperature, humidity, soilMoisturePercentual, lightPercent};
}

/**
 * Converte os dados dos sensores para formato JSON
 * 
 * Cria um objeto JSON com os dados dos sensores e o serializa
 * em uma string para envio ao servidor
 * 
 * @param data Referência para estrutura SensorData com os dados a serem convertidos
 * @return std::string String contendo o JSON serializado
 * 
 * Exemplo de JSON gerado:
 * {"temperature":25.5,"humidity":60.2,"soil_humidity":45.0,"light":75.5}
 */
std::string buildJsonPayload(const SensorData& data) {
    JsonDocument doc;  // Cria documento JSON
    
    // Adiciona cada campo ao documento JSON
    doc["temperature"] = data.temperature;
    doc["humidity"] = data.humidity;
    doc["soil_humidity"] = data.soilMoisture;
    doc["light"] = data.lightLevel;

    // Serializa o documento JSON para uma string
    std::string output;
    serializeJson(doc, output);
    return output;
}

/**
 * Constrói uma requisição HTTP customizada em formato binário
 * 
 * Protocolo customizado:
 * - Bytes 0-1:   Tamanho do payload (16 bits)
 * - Bytes 2-9:   Nome do dispositivo (8 bytes)
 * - Bytes 10-13: Método HTTP (4 bytes, ex: POST, GET)
 * - Bytes 14-25: Rota da requisição (12 bytes)
 * - Bytes 26+:   Payload JSON (opcional)
 * 
 * @param sensorData Ponteiro para dados dos sensores (nullptr se não houver payload)
 * @param method Método HTTP (máx. 4 caracteres, padrão: "POST")
 * @param route Rota da API (máx. 12 caracteres, padrão: "sensor/data")
 * @return std::vector<byte> Vetor contendo a requisição completa em bytes
 */
std::vector<byte> buildRequest(SensorData* sensorData, const char* method = "POST", const char* route = "sensor/data") {
  // Valida o tamanho dos parâmetros
  if(strlen(method) > 4 || strlen(route) > 12) {
      Serial.println("Method or route string too long");
      return std::vector<byte>();  // Retorna vetor vazio em caso de erro
  }

  byte header[26];           // Array para armazenar o cabeçalho (26 bytes fixos)
  uint16_t payloadLength = 0;  // Tamanho do payload JSON
  std::string payload;

  // Se há dados de sensores, converte para JSON
  if(sensorData != nullptr) {
      payload = buildJsonPayload(*sensorData);
      payloadLength = payload.length();
  }

  // Bytes 0-1: Tamanho do payload em Big Endian (byte mais significativo primeiro)
  header[0] = payloadLength >> 8;    // Byte alto (8 bits superiores)
  header[1] = payloadLength & 0xFF;  // Byte baixo (8 bits inferiores)

  // Bytes 2-9: Nome do dispositivo (8 bytes)
  const char* name = "Device01";
  memcpy(&header[2], name, strlen(name));

  // Bytes 10-13: Método HTTP (4 bytes, preenchido com espaços se necessário)
  if(strlen(method) < 4) {
      memcpy(&header[10], method, strlen(method));  // Copia o método
      memset(&header[10 + strlen(method)], ' ', 4 - strlen(method));  // Preenche com espaços
  } else {
      memcpy(&header[10], method, 4);  // Copia exatamente 4 bytes
  }

  // Bytes 14-25: Rota da API (12 bytes, preenchido com espaços se necessário)
  if(strlen(route) < 12) {
      memcpy(&header[14], route, strlen(route));  // Copia a rota
      memset(&header[14 + strlen(route)], ' ', 12 - strlen(route));  // Preenche com espaços
  } else {
      memcpy(&header[14], route, 12);  // Copia exatamente 12 bytes
  }

  // Cria o vetor final com tamanho total (cabeçalho + payload)
  std::vector<byte> request(26 + payloadLength);
  memcpy(request.data(), header, 26);  // Copia o cabeçalho
  
  // Se há payload, adiciona após o cabeçalho
  if(payloadLength > 0) {
      memcpy(request.data() + 26, payload.c_str(), payloadLength);
  }

  return request;
}

/**
 * Solicita os parâmetros de controle ao servidor
 * 
 * Envia uma requisição GET para a rota "parameters" para obter
 * os comandos atualizados para os atuadores (ventilação, irrigação, iluminação)
 * Se não estiver conectado, tenta reconectar antes de enviar
 */
void requestParameters() {
  // Constrói requisição GET sem payload (nullptr)
  std::vector<byte> request = buildRequest(nullptr, "GET", "parameters");
  
  // Verifica se a requisição foi construída corretamente
  if(request.empty()) {
      Serial.println("Failed to build request");
      return;
  }

  // Verifica se está conectado ao servidor
  if(client.connected()) {
    // Envia a requisição
    client.write(request.data(), request.size());
  } else {
    // Se desconectado, reconecta e então envia
    connectToServer();
    client.write(request.data(), request.size());
  }
}

/**
 * Envia os dados dos sensores para o servidor
 * 
 * Constrói e envia uma requisição POST com os dados dos sensores
 * em formato JSON. Se a conexão estiver perdida, tenta reconectar
 * automaticamente. Após reconexão, também lê e processa a resposta do servidor.
 * 
 * @param data Ponteiro para estrutura SensorData com os dados a enviar
 */
void sendDataToServer(SensorData* data) {
  // Constrói requisição POST com os dados dos sensores
  std::vector<byte> request = buildRequest(data, "POST", "sensor/data");
  
  // Verifica se a requisição foi construída corretamente
  if(request.empty()) {
      Serial.println("Failed to build request");
      return;
  }

  // Verifica se está conectado ao servidor
  if(client.connected()) {
    // Se conectado, envia direto
    client.write(request.data(), request.size());
  } else {
    // Se desconectado, tenta reconectar
    Serial.println("Disconnected from server, attempting to reconnect...");
    while(!client.connect(serverIP, serverPort)) {
      Serial.println("Reconnection failed, retrying...");
      delay(2000);  // Aguarda 2 segundos entre tentativas
    }
    Serial.println("Reconnected to server");
    
    // Após reconectar, envia os dados
    client.write(request.data(), request.size());

    // Lê a resposta do servidor com os comandos para os atuadores
    DataResponse* response = readServerResponse();
    if(response != nullptr) {
      applyServerCommands(*response);  // Aplica os comandos recebidos
      delete response;                  // Libera a memória alocada
    }
  }
}

/**
 * Faz o parsing (análise) da resposta recebida do servidor
 * 
 * Protocolo de resposta do servidor:
 * - Byte 0:    Status da resposta (0x00 = sucesso, 0x01 = erro)
 * - Bytes 1-2: Tamanho do payload JSON (16 bits, Big Endian)
 * - Bytes 3+:  Payload JSON com os comandos
 * 
 * @param data Buffer contendo os bytes da resposta
 * @param len Tamanho total do buffer
 * @return DataResponse* Ponteiro para estrutura com os comandos (ou nullptr em caso de erro)
 *         IMPORTANTE: O chamador deve fazer delete do ponteiro retornado
 */
DataResponse* parseServerResponse(const uint8_t* data, size_t len) {
  // Verifica se o tamanho mínimo do cabeçalho está presente (3 bytes)
  if(len < 3) {
      Serial.println("Response too short");
      return nullptr;
  }

  // Byte 0: Status da resposta
  uint8_t status = data[0];
  if(status == 0x01) {  // 0x01 indica erro no servidor
      Serial.println("Error in server response");
      return nullptr;
  }

  // Bytes 1-2: Tamanho do payload em Big Endian
  // Combina os dois bytes: (byte_alto << 8) | byte_baixo
  uint16_t payloadLength = (uint16_t(data[1]) << 8) | uint16_t(data[2]);

  // Verifica se o buffer contém o payload completo
  if(len < 3 + payloadLength) {
      Serial.println("Incomplete payload in server response");
      return nullptr;
  }

  // Ponteiro para o início do JSON (após os 3 bytes de cabeçalho)
  const char* jsonStart = reinterpret_cast<const char*>(data + 3);

  // Aloca memória para a estrutura de resposta
  DataResponse* dataResponse = new DataResponse();

  // Cria documento JSON para fazer o parsing
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, jsonStart, payloadLength);
  
  // Verifica se houve erro no parsing do JSON
  if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
      delete dataResponse;  // Libera memória antes de retornar
      return nullptr;
  }
  
  Serial.println("Parsed JSON from server: " + String(jsonStart));
  
  // Extrai os valores booleanos do JSON
  // Usa operador | para tentar múltiplas chaves possíveis, com false como padrão
  dataResponse->ventilation = doc["TurnOnVentilation"] | doc["ventilation"] | false;
  dataResponse->irrigation = doc["TurnOnIrrigation"] | doc["irrigation"] | false;
  dataResponse->lighting = doc["TurnOnLighting"] | doc["lighting"] | false;

  // Debug: imprime os valores extraídos
  Serial.println("DATA RESPONSE " + String(dataResponse->lighting) + " " + String(dataResponse->irrigation) + " " + String(dataResponse->ventilation));
  Serial.println("------------------");

  return dataResponse;
}

/**
 * Lê e processa a resposta completa do servidor
 * 
 * Aguarda e lê os dados recebidos do servidor via TCP:
 * 1. Aguarda o cabeçalho (3 bytes) com timeout de 5 segundos
 * 2. Extrai o tamanho do payload do cabeçalho
 * 3. Lê o payload completo com timeout de 5 segundos
 * 4. Chama parseServerResponse para interpretar os dados
 * 
 * @return DataResponse* Ponteiro para estrutura com os comandos (ou nullptr em caso de erro/timeout)
 *         IMPORTANTE: O chamador deve fazer delete do ponteiro retornado
 */
DataResponse* readServerResponse() {
  // Aguarda até que pelo menos 3 bytes estejam disponíveis (cabeçalho)
  unsigned long start = millis();
  while(client.available() < 3 && millis() - start < 5000) {
      delay(10);  // Pequeno delay para não sobrecarregar o CPU
  }

  // Verifica timeout na leitura do cabeçalho
  if(client.available() < 3) {
      Serial.println("Server response timeout (header)");
      return nullptr;
  }

  // Lê os 3 bytes do cabeçalho
  uint8_t header[3];
  size_t read = client.readBytes(header, 3);
  if(read != 3) {
      Serial.println("Failed to read header");
      return nullptr;
  }

  // Extrai o tamanho do payload dos bytes 1-2 do cabeçalho
  uint16_t payloadLength = (uint16_t(header[1]) << 8) | uint16_t(header[2]);

  // Prepara buffer para armazenar resposta completa (cabeçalho + payload)
  std::vector<uint8_t> buffer;
  buffer.resize(3 + payloadLength);  // Aloca espaço necessário
  memcpy(buffer.data(), header, 3);  // Copia o cabeçalho já lido

  // Lê o payload em chunks (pedaços) até completar ou dar timeout
  size_t received = 0;  // Contador de bytes recebidos do payload
  unsigned long payloadStart = millis();
  
  while(received < payloadLength && (millis() - payloadStart) < 5000) {
      int avail = client.available();  // Quantos bytes estão disponíveis agora
      
      if(avail > 0) {
          // Calcula quantos bytes ler (o que está disponível ou o que falta)
          size_t toRead = std::min<size_t>(avail, payloadLength - received);
          
          // Lê os bytes e adiciona ao buffer após o cabeçalho e bytes já recebidos
          int r = client.read(buffer.data() + 3 + received, toRead);
          if(r > 0) received += r;  // Atualiza contador
      } else {
          delay(5);  // Aguarda um pouco antes de verificar novamente
      }
  }

  // Verifica se o payload foi recebido completamente
  if(received != payloadLength) {
      Serial.println("Server response timeout (payload)");
      return nullptr;
  }

  // Faz o parsing da resposta completa
  return parseServerResponse(buffer.data(), buffer.size());
}

// ============================================
// FUNÇÕES DE CONTROLE DE ATUADORES
// ============================================

/**
 * Aplica os comandos recebidos do servidor aos atuadores físicos
 * 
 * Controla os três sistemas da estufa baseado nos comandos do servidor:
 * - Ventilação: Liga/desliga continuamente
 * - Irrigação: Liga por 2 segundos, com intervalo mínimo de 24 horas
 * - Iluminação: Liga/desliga continuamente
 * 
 * IMPORTANTE: Os relés usam lógica invertida (LOW = ligado, HIGH = desligado)
 * 
 * @param response Referência para estrutura DataResponse com os comandos
 */
void applyServerCommands(const DataResponse& response) {
    // ===== CONTROLE DA VENTILAÇÃO =====
    // Relé de ventilação usa lógica invertida
    if(response.ventilation) {
        Serial.println("Activating ventilation system");
        digitalWrite(VENTILATIONPIN, LOW);   // LOW = liga o relé
    } else {
        Serial.println("Deactivating ventilation system");
        digitalWrite(VENTILATIONPIN, HIGH);  // HIGH = desliga o relé
    }

    // ===== CONTROLE DA IRRIGAÇÃO =====
    // Só permite irrigar se passaram pelo menos 24 horas desde a última irrigação
    // 86400000 ms = 24 horas
    if(response.irrigation && (firstIrrigation || (millis() - lastIrrigation >= 86400000))) {
        Serial.println("Activating irrigation system");
        digitalWrite(IRRIGATIONPIN, LOW);    // Liga o relé
        delay(2000);                         // Mantém ligado por 2 segundos
        digitalWrite(IRRIGATIONPIN, HIGH);   // Desliga o relé
        lastIrrigation = millis();           // Atualiza timestamp da última irrigação
        firstIrrigation = false;           
    } else {
        Serial.println("Deactivating irrigation system");
        digitalWrite(IRRIGATIONPIN, HIGH);   // Garante que está desligado
    }

    // ===== CONTROLE DA ILUMINAÇÃO =====
    // A iluminação usa lógica normal (HIGH = ligado, LOW = desligado)
    if(response.lighting) {
        Serial.println("Activating lighting system");
        digitalWrite(LIGHTINGPIN, HIGH);     // HIGH = liga a luz
    } else {
        Serial.println("Deactivating lighting system");
        digitalWrite(LIGHTINGPIN, LOW);      // LOW = desliga a luz
    }
    Serial.println("---------------------");
}

// ============================================
// FUNÇÃO SETUP - EXECUTADA UMA VEZ NA INICIALIZAÇÃO
// ============================================

/**
 * Configuração inicial do sistema
 * 
 * Executa apenas uma vez quando o ESP32 é ligado ou resetado
 * Responsável por:
 * 1. Inicializar comunicação serial
 * 2. Configurar resolução dos conversores ADC
 * 3. Configurar modo dos pinos (entrada/saída)
 * 4. Definir estado inicial dos atuadores
 * 5. Inicializar sensor DHT11
 * 6. Conectar à rede WiFi
 * 7. Estabelecer conexão TCP com o servidor
 */
void setup() {
  // Inicia comunicação serial a 115200 bps para debug
  Serial.begin(115200);
  
  // Configura resolução do ADC para 12 bits (valores de 0-4095)
  // ESP32 suporta até 12 bits de resolução
  analogReadResolution(12);
  
  // ===== CONFIGURAÇÃO DOS PINOS =====
  // Define pinos dos sensores como entrada
  pinMode(SOILSENSORPIN, INPUT);
  pinMode(LIGHTLEVELPIN, INPUT);
  
  // Define pinos dos atuadores como saída
  pinMode(IRRIGATIONPIN, OUTPUT);
  pinMode(VENTILATIONPIN, OUTPUT);
  pinMode(LIGHTINGPIN, OUTPUT);

  // ===== ESTADO INICIAL DOS ATUADORES =====
  // Define estado inicial seguro (tudo desligado)
  digitalWrite(IRRIGATIONPIN, HIGH);   // HIGH = desligado (lógica invertida)
  digitalWrite(VENTILATIONPIN, HIGH);  // HIGH = desligado (lógica invertida)
  digitalWrite(LIGHTINGPIN, LOW);      // LOW = desligado (lógica normal)

  // Inicializa o sensor DHT11
  dht.begin();

  // ===== CONEXÃO WiFi =====
  WiFi.begin(ssid, password);  // Inicia conexão WiFi
  Serial.print("Connecting to WiFi ..");
  
  // Aguarda até conectar ao WiFi
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');  // Imprime pontos enquanto tenta conectar
    delay(1000);
  }
  // Imprime o IP local obtido
  Serial.println(WiFi.localIP());

  // ===== CONEXÃO COM O SERVIDOR =====
  // Tenta conectar ao servidor TCP repetidamente até conseguir
  while(!client.connect(serverIP, serverPort)) {
    Serial.println("Connection to server failed, retrying...");
    delay(2000);  // Aguarda 2 segundos entre tentativas
  }
  Serial.println("Connected to server");
}

// ============================================
// FUNÇÃO LOOP - EXECUTADA CONTINUAMENTE
// ============================================

/**
 * Loop principal do programa
 * 
 * Executa continuamente após o setup()
 * Implementa duas rotinas principais:
 * 
 * 1. A cada 10 minutos (sensorInterval):
 *    - Lê todos os sensores
 *    - Envia dados ao servidor
 * 
 * 2. Nos intervalos entre leituras (a cada 5 segundos):
 *    - Solicita parâmetros de controle ao servidor
 *    - Aplica comandos recebidos aos atuadores
 * 
 * Também monitora e reestabelece a conexão com o servidor se necessário
 */
void loop() {
  // ===== VERIFICAÇÃO DE CONEXÃO =====
  // Se perdeu conexão com o servidor, tenta reconectar
  if (!client.connected()) {
    connectToServer();
  }

  // Obtém o tempo atual em milissegundos desde que o ESP32 ligou
  unsigned long currentTime = millis();
  
  // ===== ROTINA DE LEITURA E ENVIO DE SENSORES =====
  // Verifica se já passou o intervalo configurado (10 minutos)
  if (currentTime - lastSensorRead >= sensorInterval) {
    // Lê dados de todos os sensores
    SensorData data = readDataFromSensors();
    
    // Verifica se a leitura foi bem-sucedida (temperatura != -1 indica sucesso)
    if(data.temperature != -1) {
      // Imprime os dados lidos no monitor serial (debug)
      Serial.printf("Temperature: %.2f °C, Humidity: %.2f %%, Soil Moisture: %.2f %%, Light: %.2f %%\n", 
                    data.temperature, data.humidity, data.soilMoisture, data.lightLevel);
      
      // Envia os dados para o servidor
      sendDataToServer(&data);
    }
    
    // Atualiza o timestamp da última leitura
    lastSensorRead = currentTime;
    
  } else {
    // ===== ROTINA DE ATUALIZAÇÃO DE PARÂMETROS =====
    // Se ainda não é hora de ler sensores, solicita parâmetros de controle
    
    requestParameters();  // Envia requisição GET ao servidor
    
    // Aguarda e lê a resposta do servidor
    DataResponse* response = readServerResponse();
    
    // Se recebeu resposta válida, aplica os comandos aos atuadores
    if(response != nullptr) {
      applyServerCommands(*response);
      delete response;  // Libera a memória alocada dinamicamente
    }
  }

  // Aguarda 5 segundos antes da próxima iteração
  // Isso evita sobrecarregar o servidor e o ESP32
  delay(5000);
}



