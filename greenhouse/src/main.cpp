#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <vector>

#define DHTPIN 4
#define DHTTYPE DHT11

struct SensorData {
  float temperature;
  float humidity;
  float soilMoisture;
  // float lightLevel;
};

struct DataResponse {
  bool ventilation;
  bool irrigation;
  bool lighting;
};

const char* ssid = "GTV_Schiestl";
const char* password = "";
const char* serverIP = "";
const uint16_t serverPort = 3000;

const int soilSensorPin = 34;
const int lightLevelPin = 32;

unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 600000;

WiFiClient client;

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

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
      Serial.printf("Temperature: %.2f °C, Humidity: %.2f %%, Soil Moisture: %.2f %%\n", 
                    data.temperature, data.humidity, data.soilMoisture);
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

  int soilMoisture = analogRead(soilSensorPin);

  float soilMoisturePercentual = map(soilMoisture, 4095, 0, 0, 100);

//   float lightLevel = analogRead(lightLevelPin);
//   if(isnan(lightLevel)) {
//     return -1;
//   }
    return SensorData{temperature, humidity, soilMoisturePercentual};
}

void requestParameters() {
  std::vector<byte> request = buildRequest(nullptr, "GET", "sensor/parameters");
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

std::string buildJsonPayload(const SensorData& data) {
    JsonDocument doc;
    doc["temperature"] = data.temperature;
    doc["humidity"] = data.humidity;
    doc["soil_humidity"] = data.soilMoisture;

    std::string output;
    serializeJson(doc, output);
    return output;
}

DataResponse* readServerResponse() {
  unsigned long timeout = millis() + 5000;
  while(client.available() == 0 && millis() < timeout) {
      delay(10);
  }
  
  if(client.available() == 0) {
      Serial.println("Server response timeout");
      return nullptr;
  }

  String response = "";
  while(client.available()) {
      char c = client.read();
      response += c;
  }

  return parseServerResponse(response.c_str());
}

DataResponse* parseServerResponse(const char* response) {
  if(strlen(response) < 3) {
      Serial.println("Response too short");
      return nullptr;
  }

  byte header[3] = { (byte)response[0], (byte)response[1], (byte)response[2] };
  if(header[0] == 0x01) {
      Serial.println("Error in server response");
      return nullptr;
  }

  uint16_t payloadLength = (header[1] << 8) | header[2];

  if(payloadLength > 0) {
      response += 3;
  } else {
      Serial.println("No payload in server response");
      return nullptr;
  }

  DataResponse* dataResponse = new DataResponse();

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
      delete dataResponse;
      return nullptr;
  }

  dataResponse->ventilation = doc["ventilation"] | false;
  dataResponse->irrigation = doc["irrigation"] | false;
  dataResponse->lighting = doc["lighting"] | false;

  return dataResponse;
}

void applyServerCommands(const DataResponse& response) {
    if(response.ventilation) {
        Serial.println("Activating ventilation system");
    } else {
        Serial.println("Deactivating ventilation system");
    }

    if(response.irrigation) {
        Serial.println("Activating irrigation system");
    } else {
        Serial.println("Deactivating irrigation system");
    }

    if(response.lighting) {
        Serial.println("Activating lighting system");
    } else {
        Serial.println("Deactivating lighting system");
    }
}

