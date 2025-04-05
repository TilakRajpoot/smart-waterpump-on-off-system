#include <esp_now.h>
#include <WiFi.h>

#define RELAY_PIN 2

bool pumpState = false;

typedef struct struct_message {
  bool command;
} struct_message;

struct_message incomingData;
struct_message outgoingData;

uint8_t esp1Address[] = {0x8c, 0x4f, 0x00, 0x10, 0xd9, 0x9c}; // Replace with ESP1 MAC

// New callback signature for ESP32 core 3.x.x
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  pumpState = incomingData.command;

  digitalWrite(RELAY_PIN, pumpState);  // Turn pump ON/OFF

  // Send pump status back to ESP1
  outgoingData.command = pumpState;
  esp_now_send(esp1Address, (uint8_t *)&outgoingData, sizeof(outgoingData));
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Initial state OFF

  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register new format receive callback
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, esp1Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("ESP1 paired");
  }
}

void loop() {
  // Nothing needed here
}
