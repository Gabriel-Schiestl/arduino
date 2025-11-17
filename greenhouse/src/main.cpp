#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <vector>

#define DHTPIN 4
#define DHTTYPE DHT11

#define SOILSENSORPIN 34
#define LIGHTLEVELPIN 35

#define IRRIGATIONPIN 14
#define VENTILATIONPIN 27
#define LIGHTINGPIN 26

struct SensorData {
  float temperature;
  float humidity;
  float soilMoisture;
  float lightLevel;
};

struct DataResponse {
  bool ventilation;
  bool irrigation;
  bool lighting;
};

const char* ssid = "";
const char* password = "";
const char* serverIP = "";
const uint16_t serverPort = 3000;

unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 600000;

WiFiClient client;

DHT dht(DHTPIN, DHTTYPE);

void connectToServer() {
  Serial.println("Connecting to server...");
  while(!client.connect(serverIP, serverPort)) {
    Serial.println("Connection failed, retrying...");
    delay(2000);
  }
  Serial.println("Connected to server");
}

SensorData readDataFromSensors() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if(isnan(temperature) || isnan(humidity)) {
    return SensorData{-1, -1, -1};
  }

  int soilMoisture = analogRead(SOILSENSORPIN);

  float soilMoisturePercentual = map(soilMoisture, 4095, 0, 0, 100);

  int lightRaw = analogRead(LIGHTLEVELPIN);
  Serial.println(lightRaw);
  float lightPercent = map(lightRaw, 0, 4095, 0, 100);

  return SensorData{temperature, humidity, soilMoisturePercentual, lightPercent};
}

std::string buildJsonPayload(const SensorData& data) {
    JsonDocument doc;
    doc["temperature"] = data.temperature;
    doc["humidity"] = data.humidity;
    doc["soil_humidity"] = data.soilMoisture;
    doc["light"] = data.lightLevel;

    std::string output;
    serializeJson(doc, output);
    return output;
}

std::vector<byte> buildRequest(SensorData* sensorData, const char* method = "POST", const char* route = "sensor/data") {
  if(strlen(method) > 4 || strlen(route) > 12) {
      Serial.println("Method or route string too long");
      return std::vector<byte>(); 
  }

  byte header[26];
  uint16_t payloadLength = 0;
  std::string payload;

  if(sensorData != nullptr) {
      payload = buildJsonPayload(*sensorData);
      payloadLength = payload.length();
  }

  header[0] = payloadLength >> 8;
  header[1] = payloadLength & 0xFF;

  const char* name = "Device01";
  memcpy(&header[2], name, strlen(name));

  if(strlen(method) < 4) {
      memcpy(&header[10], method, strlen(method));
      memset(&header[10 + strlen(method)], ' ', 4 - strlen(method));
  } else {
      memcpy(&header[10], method, 4);
  }

  if(strlen(route) < 12) {
      memcpy(&header[14], route, strlen(route));
      memset(&header[14 + strlen(route)], ' ', 12 - strlen(route));
  } else {
      memcpy(&header[14], route, 12);
  }

  std::vector<byte> request(26 + payloadLength);
  memcpy(request.data(), header, 26);
  if(payloadLength > 0) {
      memcpy(request.data() + 26, payload.c_str(), payloadLength);
  }

  return request;
}

void requestParameters() {
  std::vector<byte> request = buildRequest(nullptr, "GET", "parameters");
  if(request.empty()) {
      Serial.println("Failed to build request");
      return;
  }

  if(client.connected()) {
    client.write(request.data(), request.size());
  } else {
    connectToServer();
    client.write(request.data(), request.size());
  }
}

void sendDataToServer(SensorData* data) {
  std::vector<byte> request = buildRequest(data, "POST", "sensor/data");
  if(request.empty()) {
      Serial.println("Failed to build request");
      return;
  }

  if(client.connected()) {
    client.write(request.data(), request.size());
  } else {
    Serial.println("Disconnected from server, attempting to reconnect...");
    while(!client.connect(serverIP, serverPort)) {
      Serial.println("Reconnection failed, retrying...");
      delay(2000);
    }
    Serial.println("Reconnected to server");
    client.write(request.data(), request.size());
  }
}

DataResponse* parseServerResponse(const uint8_t* data, size_t len) {
  if(len < 3) {
      Serial.println("Response too short");
      return nullptr;
  }

  uint8_t status = data[0];
  if(status == 0x01) {
      Serial.println("Error in server response");
      return nullptr;
  }

  uint16_t payloadLength = (uint16_t(data[1]) << 8) | uint16_t(data[2]);

  if(len < 3 + payloadLength) {
      Serial.println("Incomplete payload in server response");
      return nullptr;
  }

  const char* jsonStart = reinterpret_cast<const char*>(data + 3);

  DataResponse* dataResponse = new DataResponse();

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, jsonStart, payloadLength);
  if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
      delete dataResponse;
      return nullptr;
  }

  dataResponse->ventilation = doc["TurnOnVentilation"] | doc["ventilation"] | false;
  dataResponse->irrigation = doc["TurnOnIrrigation"] | doc["irrigation"] | false;
  dataResponse->lighting = doc["TurnOnLighting"] | doc["lighting"] | false;

  return dataResponse;
}

DataResponse* readServerResponse() {
  unsigned long start = millis();
  while(client.available() < 3 && millis() - start < 5000) {
      delay(10);
  }

  if(client.available() < 3) {
      Serial.println("Server response timeout (header)");
      return nullptr;
  }

  uint8_t header[3];
  size_t read = client.readBytes(header, 3);
  if(read != 3) {
      Serial.println("Failed to read header");
      return nullptr;
  }

  uint16_t payloadLength = (uint16_t(header[1]) << 8) | uint16_t(header[2]);

  std::vector<uint8_t> buffer;
  buffer.resize(3 + payloadLength);
  memcpy(buffer.data(), header, 3);

  size_t received = 0;
  unsigned long payloadStart = millis();
  while(received < payloadLength && (millis() - payloadStart) < 5000) {
      int avail = client.available();
      if(avail > 0) {
          size_t toRead = std::min<size_t>(avail, payloadLength - received);
          int r = client.read(buffer.data() + 3 + received, toRead);
          if(r > 0) received += r;
      } else {
          delay(5);
      }
  }

  if(received != payloadLength) {
      Serial.println("Server response timeout (payload)");
      return nullptr;
  }

  return parseServerResponse(buffer.data(), buffer.size());
}

void applyServerCommands(const DataResponse& response) {
    if(response.ventilation) {
        Serial.println("Activating ventilation system");
        digitalWrite(VENTILATIONPIN, LOW);
    } else {
        Serial.println("Deactivating ventilation system");
        digitalWrite(VENTILATIONPIN, HIGH);
    }

    if(response.irrigation) {
        Serial.println("Activating irrigation system");
        digitalWrite(IRRIGATIONPIN, LOW);
        delay(2000);
        digitalWrite(IRRIGATIONPIN, HIGH);
    } else {
        Serial.println("Deactivating irrigation system");
        digitalWrite(IRRIGATIONPIN, HIGH);
    }

    if(response.lighting) {
        Serial.println("Activating lighting system");
        digitalWrite(LIGHTINGPIN, HIGH);
    } else {
        Serial.println("Deactivating lighting system");
        digitalWrite(LIGHTINGPIN, LOW);
    }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(SOILSENSORPIN, INPUT);
  pinMode(LIGHTLEVELPIN, INPUT);
  pinMode(IRRIGATIONPIN, OUTPUT);
  pinMode(VENTILATIONPIN, OUTPUT);
  pinMode(LIGHTINGPIN, OUTPUT);

  dht.begin();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());

  while(!client.connect(serverIP, serverPort)) {
    Serial.println("Connection to server failed, retrying...");
    delay(2000);
  }
  Serial.println("Connected to server");
}

void loop() {
  if (!client.connected()) {
    connectToServer();
  }

  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorRead >= sensorInterval) {
    SensorData data = readDataFromSensors();
    if(data.temperature != -1) {
      Serial.printf("Temperature: %.2f °C, Humidity: %.2f %%, Soil Moisture: %.2f %%, Light: %.2f %%\n", 
                    data.temperature, data.humidity, data.soilMoisture, data.lightLevel);
      sendDataToServer(&data);
    }
    lastSensorRead = currentTime;
  }

  requestParameters();
  DataResponse* response = readServerResponse();
  if(response != nullptr) {
    applyServerCommands(*response);
    delete response;
  }

  delay(5000);
}



