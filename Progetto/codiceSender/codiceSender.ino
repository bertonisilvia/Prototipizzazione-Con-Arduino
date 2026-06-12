// ============================================================
//  SENDER – Doppio HC-SR04 + ESP-NOW
//  Rileva le bilie in due buche e invia:
//      goal = true, id univoco, sensorId (1 o 2)
// ============================================================

#include <WiFi.h>     // Libreria Wi-Fi dell'ESP32 (necessaria per ESP-NOW)
#include <esp_now.h>  // Libreria per protocollo ESP-NOW

// ===== PIN HC-SR04 =====
// Coppia TRIG/ECHO per ciascun sensore a ultrasuoni
const int trig1 = 13;
const int echo1 = 12;

const int trig2 = 15;
const int echo2 = 14;

// LED di stato sul Sender (lampeggia a ogni invio riuscito)
const int ledPin = 2;

// ===== MAC RECEIVER =====
// Indirizzo MAC hardware dell'ESP32 Receiver
uint8_t receiverMac[] = {0x94, 0x3C, 0xC6, 0x97, 0x4C, 0x78};

// ===== SOGLIE =====
// Soglia di distanza (in cm) per considerare una bilia presente davanti al sensore
const float triggerDistanceCm1 = 5.0;
const float triggerDistanceCm2 = 5.0;
// Tempo minimo (ms) tra due invii consecutivi per evitare conteggi multipli
const unsigned long sensorLock = 300;

// ===== STRUTTURA DATI =====
// Pacchetto che viene inviato al Receiver tramite ESP-NOW
// sensorId: 1 = buca 1, 2 = buca 2
typedef struct struct_message {
  bool    goal;      // true se è stato rilevato un "goal"
  unsigned long id;  // identificativo univoco incrementale
  uint8_t sensorId;  // identifica quale sensore/buca ha generato l'evento
} struct_message;

// Istanza del pacchetto da inviare
struct_message dataToSend;
// Struttura di configurazione del peer ESP-NOW
esp_now_peer_info_t peerInfo;

// Stato precedente dei due sensori (per rilevare il fronte di salita)
bool lastDetected1 = false;
bool lastDetected2 = false;
// Timestamp dell’ultimo invio, usato per applicare il sensorLock
unsigned long lastSendTime = 0;

// -------------------------------------------------------
// Misura la distanza in cm con un sensore HC-SR04
// Restituisce -1.0 in caso di timeout / assenza di eco
float readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long durata = pulseIn(echoPin, HIGH, 30000); // timeout 30 ms
  if (durata == 0) return -1.0;
  // conversione da tempo di volo a distanza (cm)
  return (durata * 0.0343) / 2.0;
}

// -------------------------------------------------------
// Callback di debug eseguita dopo ogni tentativo di invio ESP-NOW
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Stato invio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "ERRORE");
}

// -------------------------------------------------------
// Costruisce e invia un pacchetto di "goal" verso il Receiver
//  - nomeSensore: stringa di debug (es. "BUCA 1")
//  - sid: id numerico del sensore (1 oppure 2)
void sendGoalEvent(const char* nomeSensore, uint8_t sid) {
  dataToSend.goal     = true;  // segnala un evento valido
  dataToSend.sensorId = sid;   // memorizza quale sensore ha rilevato
  dataToSend.id++;             // incrementa l'identificativo univoco

  // Messaggio di debug sulla porta seriale
  Serial.print(nomeSensore);
  Serial.print(" (buca ");
  Serial.print(sid);
  Serial.print(") -> invio ID ");
  Serial.println(dataToSend.id);

  // Invio del pacchetto al Receiver tramite ESP-NOW
  esp_err_t result = esp_now_send(receiverMac,
                                  (uint8_t *)&dataToSend,
                                  sizeof(dataToSend));

  // Feedback visivo tramite LED in caso di invio riuscito
  if (result == ESP_OK) {
    digitalWrite(ledPin, HIGH);
    delay(80);
    digitalWrite(ledPin, LOW);
  } else {
    Serial.print("Errore esp_now_send(), codice = ");
    Serial.println(result);
  }
}

// -------------------------------------------------------
// Inizializzazione del modulo Sender
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== SENDER DOPPIO HC-SR04 + ESP-NOW ===");

  // Configurazione dei pin dei sensori e del LED
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Modalità Wi-Fi in stazione (necessaria per ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  delay(100);

  Serial.print("MAC sender: ");
  Serial.println(WiFi.STA.macAddress());

  // Inizializzazione dello stack ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Errore inizializzazione ESP-NOW");
    return;
  }

  // Registrazione della callback di invio
  esp_now_register_send_cb(onDataSent);

  // Configurazione del peer (Receiver) con il suo MAC address
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;      // canale di default
  peerInfo.encrypt = false;  // nessuna cifratura

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Errore aggiunta peer");
    return;
  }

  // Inizializzazione del pacchetto
  dataToSend.goal     = false;
  dataToSend.id       = 0;
  dataToSend.sensorId = 0;

  // Lettura iniziale dei sensori per impostare lo stato di partenza
  float d1 = readDistanceCm(trig1, echo1);
  delay(60); // piccola pausa per evitare interferenza tra i sensori
  float d2 = readDistanceCm(trig2, echo2);

  // Se una bilia è già davanti al sensore all'avvio,
  // questa condizione viene salvata ma non genera un evento
  lastDetected1 = (d1 > 0 && d1 <= triggerDistanceCm1);
  lastDetected2 = (d2 > 0 && d2 <= triggerDistanceCm2);

  Serial.print("Distanza iniziale sensore 1: ");
  if (d1 < 0) Serial.println("Fuori portata / timeout");
  else { Serial.print(d1); Serial.println(" cm"); }

  Serial.print("Distanza iniziale sensore 2: ");
  if (d2 < 0) Serial.println("Fuori portata / timeout");
  else { Serial.print(d2); Serial.println(" cm"); }

  Serial.println("Sender pronto.");
}

// -------------------------------------------------------
// Ciclo principale: lettura sensori, debug e invio eventi
void loop() {
  unsigned long now = millis();

  // ===== SENSORE 1 =====
  float distanza1 = readDistanceCm(trig1, echo1);
  // detected1 diventa true se il bersaglio è entro la soglia utile
  bool  detected1 = (distanza1 > 0 && distanza1 <= triggerDistanceCm1);

  // Piccolo delay per ridurre interferenze acustiche fra i due HC-SR04
  delay(60);

  // ===== SENSORE 2 =====
  float distanza2 = readDistanceCm(trig2, echo2);
  bool  detected2 = (distanza2 > 0 && distanza2 <= triggerDistanceCm2);

  // ===== DEBUG 2 Hz =====
  // Ogni ~500 ms stampa le distanze e gli stati dei due sensori
  static unsigned long lastDebug = 0;
  if (now - lastDebug > 500) {
    Serial.print("S1 = ");
    if (distanza1 < 0) Serial.print("timeout");
    else Serial.print(distanza1);
    Serial.print(" cm | Det1=");
    Serial.print(detected1);

    Serial.print(" || S2 = ");
    if (distanza2 < 0) Serial.print("timeout");
    else Serial.print(distanza2);
    Serial.print(" cm | Det2=");
    Serial.println(detected2);

    lastDebug = now;
  }

  // Log di debug quando cambia lo stato logico del sensore 1
  if (detected1 != lastDetected1) {
    Serial.print("CAMBIO S1: ");
    Serial.print(lastDetected1);
    Serial.print(" -> ");
    Serial.println(detected1);
  }

  // Log di debug quando cambia lo stato logico del sensore 2
  if (detected2 != lastDetected2) {
    Serial.print("CAMBIO S2: ");
    Serial.print(lastDetected2);
    Serial.print(" -> ");
    Serial.println(detected2);
  }

  // Fronte di ingresso sensore 1:
  // invia evento SOLO quando si passa da "non rilevato" a "rilevato"
  // e se è passato almeno sensorLock ms dall'ultimo invio
  if (detected1 && !lastDetected1 && (now - lastSendTime > sensorLock)) {
    sendGoalEvent("BUCA 1", 1);
    lastSendTime = now;
  }

  // Fronte di ingresso sensore 2
  if (detected2 && !lastDetected2 && (now - lastSendTime > sensorLock)) {
    sendGoalEvent("BUCA 2", 2);
    lastSendTime = now;
  }

  // Aggiorna lo stato precedente per il ciclo successivo
  lastDetected1 = detected1;
  lastDetected2 = detected2;

  // Piccolo delay per limitare la frequenza di polling
  delay(40);
}
