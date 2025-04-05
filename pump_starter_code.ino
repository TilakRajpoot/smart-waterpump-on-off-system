#include <esp_now.h>
#include <WiFi.h>

#define BUTTON_PIN 4
#define GREEN_LED 5
#define RED_LED 18

uint8_t esp2Address[] = {0xa0, 0xb7, 0x65, 0x25, 0xf6, 0x44}; // Replace with actual ESP2 MAC

bool pumpState = false;
bool lastButtonState = HIGH;

typedef struct struct_message {
  bool command;
} struct_message;

struct_message outgoingData;
struct_message incomingData;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, esp2Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("ESP2 paired");
  }

  // Register receive callback with new format
  esp_now_register_recv_cb([](const esp_now_recv_info_t * info, const uint8_t *data, int len) {
    memcpy(&incomingData, data, sizeof(incomingData));
    if (incomingData.command) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
    } else {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, HIGH);
    }
  });
}

void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW && lastButtonState == HIGH) {
    pumpState = !pumpState;
    outgoingData.command = pumpState;
    esp_now_send(esp2Address, (uint8_t *)&outgoingData, sizeof(outgoingData));
    delay(100);
  }

  lastButtonState = buttonState;
}
