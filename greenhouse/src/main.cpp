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

int timeCounter = 0;

WiFiClient client;

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

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
  SensorData data = readDataFromSensors();
  if(data.temperature != -1 && data.humidity != -1 && data.soilMoisture != -1) {
    Serial.printf("Temperature: %.2f °C, Humidity: %.2f %%, Soil Moisture: %.2f %%\n", data.temperature, data.humidity, data.soilMoisture);
    client.printf("Temperature: %.2f °C, Humidity: %.2f %%, Soil Moisture: %.2f %%\n", data.temperature, data.humidity, data.soilMoisture);
  } else {
    Serial.println("Failed to read from sensors");
  }

  while(timeCounter < 600) {
    requestParameters();
    DataResponse* response = readServerResponse();
    if(response != nullptr) {
        applyServerCommands(*response);
        delete response;
    } else {
        Serial.println("Failed to read server response");
    }
    timeCounter += 5;
    delay(5000);
  }
    timeCounter = 0;
    SensorData newData = readDataFromSensors();
    sendDataToServer(&newData);
    DataResponse* response = readServerResponse();
    if(response != nullptr) {
        applyServerCommands(*response);
        delete response;
    } else {
        Serial.println("Failed to read server response");
    }
}

SensorData readDataFromSensors() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if(isnan(temperature) || isnan(humidity)) {
    return SensorData{-1, -1, -1};
  }

  int soilMoisture = analogRead(soilSensorPin);
  if(isnan(soilMoisture)) {
    return SensorData{-1, -1, -1};
  }
  float soilMoisturePercentual = map(soilMoisture, 4095, 0, 0, 100);

//   float lightLevel = analogRead(lightLevelPin);
//   if(isnan(lightLevel)) {
//     return -1;
//   }
    return SensorData{temperature, humidity, soilMoisturePercentual};
}

void requestParameters() {
  byte* request = buildRequest(nullptr, "GET", "sensor/parameters");
  if(request == nullptr) {
      Serial.println("Failed to build request");
      return;
  }

  if(client.connected()) {
    client.write(request, sizeof(request));
  } else {
    Serial.println("Disconnected from server, attempting to reconnect...");
    while(!client.connect(serverIP, serverPort)) {
      Serial.println("Reconnection failed, retrying...");
      delay(2000);
    }
    Serial.println("Reconnected to server");
  }
}

void sendDataToServer(SensorData* data) {
  byte* request = buildRequest(data, "POST", "sensor/data");
  if(request == nullptr) {
      Serial.println("Failed to build request");
      return;
  }

  if(client.connected()) {
    client.write(request, sizeof(request));
  } else {
    Serial.println("Disconnected from server, attempting to reconnect...");
    while(!client.connect(serverIP, serverPort)) {
      Serial.println("Reconnection failed, retrying...");
      delay(2000);
    }
    Serial.println("Reconnected to server");
  }
}

byte* buildRequest(SensorData* sensorData, char* method = "POST", char* route = "sensor/data") {
  if(strlen(method) > 4 || strlen(route) > 12) {
      Serial.println("Method or route string too long");
      return nullptr;
  }

    static byte header[26];

    uint16_t payloadLength = 0;
    char* payload = nullptr;
    if(sensorData != nullptr) {
        payload = buildJsonPayload(*sensorData);
        payloadLength = strlen(payload);
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

    byte request[26 + payloadLength];
    memcpy(&request, header, 26); 
    if(payloadLength > 0) {
        memcpy(&request[26], payload, payloadLength);
    }

    return request;
}

char* buildJsonPayload(const SensorData& data) {
    JsonDocument doc;
    doc["temperature"] = data.temperature;
    doc["humidity"] = data.humidity;
    doc["soil_humidity"] = data.soilMoisture;
    // doc["light"] = data.lightLevel;

    const size_t bufferSize = measureJson(doc);
    char buffer[bufferSize];

    size_t bytesWritten = serializeJson(doc, buffer, bufferSize);
    if (bytesWritten == 0) {
        Serial.println("Erro ao serializar JSON");
        return nullptr;
    }

    return buffer;
}

DataResponse* readServerResponse() {
  while(client.available() == 0) {
      delay(10);
  }

  String response = "";
  while(client.available()) {
      char c = client.read();
      response += c;
  }

  return parseServerResponse(response.c_str());
}

DataResponse* parseServerResponse(const char* response) {
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

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, response, payloadLength);
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

