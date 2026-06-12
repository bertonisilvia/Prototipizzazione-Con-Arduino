// ============================================================
//  RECEIVER – Scoreboard con selezione modalità di gioco
//
//  MODALITÀ 1 (classica): ogni bilia imbucata = +1 punto
//  MODALITÀ 2 (input): quando arriva un goal si entra in
//                      modalità inserimento valore; l'utente
//                      regola il punteggio con +/- e
//                      conferma con btnTurn. Il timer si
//                      congela durante l'inserimento.
//
//  SELEZIONE ALL'AVVIO:
//    btnUp   -> evidenzia Modalità 1
//    btnDown -> evidenzia Modalità 2
//    btnTurn -> conferma
//
//  RESET (in qualsiasi momento) -> torna alla selezione modalità
// ============================================================

#include <Wire.h>               // Comunicazione I2C
#include <LiquidCrystal_I2C.h>  // Libreria per display LCD I2C
#include <WiFi.h>               // Necessaria per usare ESP-NOW su ESP32
#include <esp_now.h>            // Libreria protocollo ESP-NOW

// Oggetto display LCD all'indirizzo I2C 0x27, 16 colonne e 2 righe
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== PIN =====
// Pulsanti di controllo
const int btnUp    = 33;
const int btnDown  = 27;
const int btnTurn  = 15;
const int btnReset = 14;

// LED che indicano quale giocatore è di turno
const int ledP1    = 13;
const int ledP2    = 32;

// ===== COSTANTI =====
// Modalità 1: punteggio massimo fissato a 8
// Modalità 2: punteggio massimo fissato a 61
const int MAX_SCORE_M1 = 8;
const int MAX_SCORE_M2 = 61;

// Durata del turno: 20 secondi
const unsigned long TURN_TIME = 20000;

// Tempi di debounce e protezione
const unsigned long DEBOUNCE_DELAY   = 80;   // evita rimbalzi dei pulsanti
const unsigned long TURN_CHANGE_LOCK = 700;  // evita doppi cambi turno ravvicinati
const unsigned long DISPLAY_REFRESH  = 300;  // frequenza di refresh del display

// Range ammesso per il valore della bilia nella modalità 2
const int MIN_INPUT_VAL = 1;
const int MAX_INPUT_VAL = 9;

// ===== MODALITÀ DI GIOCO =====
// MODE_NONE: nessuna modalità ancora scelta
// MODE_1: modalità classica (+1 automatico)
// MODE_2: modalità con input del valore della bilia
enum GameMode { MODE_NONE = 0, MODE_1, MODE_2 };
GameMode gameMode         = MODE_NONE;
bool     modeSelected     = false;
GameMode highlightedMode  = MODE_1;

// ===== MACCHINA A STATI =====
// STATE_SELECT: schermata iniziale di selezione modalità
// STATE_PLAY: partita in corso
// STATE_INPUT: inserimento manuale del valore della bilia
enum GameState { STATE_SELECT, STATE_PLAY, STATE_INPUT };
GameState gameState = STATE_SELECT;

// ===== STATO GIOCO =====
// Punteggi dei due giocatori
int  punteggioP1   = 0;
int  punteggioP2   = 0;
// true = turno del giocatore 1, false = turno del giocatore 2
bool turnoP1       = true;
// Indica se la partita è terminata
bool partitaFinita = false;

// ===== INSERIMENTO VALORE =====
// Valore attualmente selezionato nella schermata di input
int  inputValore = 1;
// true se l'input è stato aperto a seguito di un goal ricevuto via ESP-NOW
// false se è stato aperto manualmente con btnUp
bool inputDaGoal = false;

// ===== BOTTONI =====
// Stato precedente dei pulsanti, utile per rilevare il fronte di pressione
bool lastUp    = HIGH;
bool lastDown  = HIGH;
bool lastTurn  = HIGH;
bool lastReset = HIGH;

// Timestamp dell'ultima variazione di stato dei pulsanti
unsigned long lastUpTime    = 0;
unsigned long lastDownTime  = 0;
unsigned long lastTurnTime  = 0;
unsigned long lastResetTime = 0;

// ===== TIMER =====
// Istante di partenza del turno corrente
unsigned long turnStartTime = 0;
// Tempo già consumato prima di una eventuale pausa
unsigned long turnElapsedBeforePause = 0;
// true se il timer è momentaneamente congelato
bool timerPaused = 0;
// Timestamp dell’ultimo cambio turno
unsigned long lastTurnChangeTime = 0;
// Timestamp dell’ultimo aggiornamento del display
unsigned long lastDisplayUpdate = 0;

// ===== ESP-NOW =====
// Struttura dei dati ricevuti dal Sender
typedef struct struct_message {
  bool goal;          // true se il sender ha rilevato un evento valido
  unsigned long id;   // identificativo progressivo dell'evento
  uint8_t sensorId;   // identifica il sensore che ha generato l'evento
} struct_message;

// Buffer in cui copiare i dati ricevuti
struct_message incomingData;

// Ultimo identificativo elaborato, utile a evitare duplicati
volatile unsigned long lastGoalId = 0;

// ===== CODA CIRCOLARE GOAL =====
// La coda permette di gestire più eventi ricevuti in rapida successione
#define MAX_PENDING 10
volatile uint8_t goalQueue[MAX_PENDING];
volatile uint8_t goalHead = 0;
volatile uint8_t goalTail = 0;

// ============================================================
// HELPER: restituisce la soglia di vittoria in base alla modalità
// ============================================================
int maxScore() {
  return (gameMode == MODE_2) ? MAX_SCORE_M2 : MAX_SCORE_M1;
}

// ============================================================
// FORWARD DECLARATIONS
// Servono a dichiarare in anticipo le funzioni usate nel codice
// ============================================================
bool pressedEdge(int pinState, bool &lastState,
                 unsigned long &lastTime, unsigned long now);
void resetGioco();
void aggiornaDisplay();
void mostraSchermataModalita();
void mostraSchermataInput();
void pauseTimer();
void resumeTimer();
int  tempoResiduoSec();

// ============================================================
// CODA GOAL
// Inserisce l'id del sensore nella coda circolare se c'è spazio
// ============================================================
void enqueueGoal(uint8_t sid) {
  uint8_t next = (goalTail + 1) % MAX_PENDING;
  if (next != goalHead) {
    goalQueue[goalTail] = sid;
    goalTail = next;
  }
}

// Estrae il prossimo goal in attesa dalla coda
bool dequeueGoal(uint8_t &sid) {
  if (goalHead == goalTail) return false; // coda vuota
  sid = goalQueue[goalHead];
  goalHead = (goalHead + 1) % MAX_PENDING;
  return true;
}

// ============================================================
// CALLBACK ESP-NOW
// Viene chiamata automaticamente quando arriva un messaggio
// ============================================================
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data, int len)
{
  // Verifica che la dimensione del pacchetto sia corretta
  if (len != sizeof(struct_message)) return;

  // Copia i dati ricevuti nella struttura locale
  memcpy(&incomingData, data, sizeof(incomingData));

  // Accetta solo i goal validi con id non ancora elaborato
  if (incomingData.goal && incomingData.id != lastGoalId) {
    lastGoalId = incomingData.id;

    // Sezione critica: inserimento in coda protetto da interrupt
    noInterrupts();
    enqueueGoal(incomingData.sensorId);
    interrupts();
  }
}

// ============================================================
// TIMER CON PAUSA
// Queste funzioni permettono di congelare e riprendere il timer
// ============================================================

// Congela il timer salvando il tempo già trascorso
void pauseTimer() {
  if (timerPaused) return;
  turnElapsedBeforePause += millis() - turnStartTime;
  timerPaused = true;
}

// Riprende il timer dal momento in cui era stato fermato
void resumeTimer() {
  if (!timerPaused) return;
  turnStartTime = millis();
  timerPaused   = false;
}

// Reinizializza completamente il timer di turno a 20 secondi
void resetTimerTurno() {
  turnElapsedBeforePause = 0;
  turnStartTime          = millis();
  timerPaused            = false;
}

// Calcola i secondi residui del turno
int tempoResiduoSec() {
  unsigned long elapsed = turnElapsedBeforePause;
  if (!timerPaused) elapsed += millis() - turnStartTime;
  if (elapsed >= TURN_TIME) return 0;
  return (TURN_TIME - elapsed + 999) / 1000;
}

// Restituisce true solo se è passato abbastanza tempo
// dall’ultimo cambio turno
bool canChangeTurn() {
  return (millis() - lastTurnChangeTime) >= TURN_CHANGE_LOCK;
}

// ============================================================
// DISPLAY SELEZIONE MODALITÀ
// Mostra la schermata iniziale in cui scegliere la modalità
// ============================================================
void mostraSchermataModalita() {
  lcd.clear();
  lcd.setCursor(0, 0);

  if (highlightedMode == MODE_1)
    lcd.print(">MOD 1  MOD 2  ");
  else
    lcd.print(" MOD 1 >MOD 2  ");

  lcd.setCursor(0, 1);
  if (highlightedMode == MODE_1)
    lcd.print("Classica (+1)  ");
  else
    lcd.print("Input valore   ");
}

// ============================================================
// DISPLAY INSERIMENTO VALORE
// Mostra la schermata usata in modalità 2 per impostare
// manualmente il valore della bilia imbucata
// ============================================================
void mostraSchermataInput() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Valore bilia:  ");
  lcd.setCursor(0, 1);
  lcd.print("< ");
  lcd.print(inputValore);
  lcd.print(" >  Conferma  ");
}

// ============================================================
// DISPLAY PARTITA
// ============================================================

// Mostra il vincitore e il punteggio finale
void mostraVincitore() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vince ");
  lcd.print(punteggioP1 >= maxScore() ? "P1" : "P2");

  lcd.setCursor(0, 1);
  lcd.print("P1:");
  lcd.print(punteggioP1);
  lcd.print(" P2:");
  lcd.print(punteggioP2);

  Serial.print("PARTITA FINITA -> Vince ");
  Serial.println(punteggioP1 >= maxScore() ? "P1" : "P2");
}

// Aggiorna il display principale durante la partita
void aggiornaDisplay() {
  // Non aggiornare il display di gioco in altri stati
  if (gameState == STATE_SELECT) return;
  if (gameState == STATE_INPUT)  return;

  // Variabili statiche usate per evitare refresh inutili
  static int  oldP1   = -1;
  static int  oldP2   = -1;
  static int  oldSec  = -1;
  static int  oldTurn = -1;
  static bool oldFine = false;

  int sec  = tempoResiduoSec();
  int turn = turnoP1 ? 1 : 2;

  // Se la partita è finita, mostra il vincitore una sola volta
  if (partitaFinita) {
    if (!oldFine) { 
      mostraVincitore(); 
      oldFine = true; 
    }
    return;
  }

  // Se nulla è cambiato, evita di ridisegnare il display
  if (punteggioP1 == oldP1 && punteggioP2 == oldP2 &&
      sec == oldSec && turn == oldTurn && !oldFine)
    return;

  // Aggiorna i valori salvati
  oldFine = false;
  oldP1 = punteggioP1;
  oldP2 = punteggioP2;
  oldSec = sec;
  oldTurn = turn;

  // Ridisegna il contenuto del display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(turnoP1 ? "P1" : "P2");
  lcd.print(" ");
  // Se il timer è in pausa, mostra "**", altrimenti i secondi residui
  lcd.print(timerPaused ? "**" : String(sec) + "s");
  lcd.print(" M");
  lcd.print(gameMode == MODE_1 ? "1" : "2");
  lcd.print("     ");

  lcd.setCursor(0, 1);
  lcd.print("P1:");
  lcd.print(punteggioP1);
  lcd.print(" P2:");
  lcd.print(punteggioP2);
  lcd.print("   ");
}

// ============================================================
// RESET GIOCO
// Riporta la partita allo stato iniziale dopo la scelta modalità
// ============================================================
void resetGioco() {
  // Svuota la coda degli eventi ricevuti
  noInterrupts();
  goalHead   = 0;
  goalTail   = 0;
  lastGoalId = 0;
  interrupts();

  // Reinizializza le variabili principali della partita
  punteggioP1   = 0;
  punteggioP2   = 0;
  turnoP1       = true;
  partitaFinita = false;
  inputValore   = 1;
  gameState     = STATE_PLAY;

  // Riavvia timer e lock del cambio turno
  resetTimerTurno();
  lastTurnChangeTime = 0;

  Serial.print("RESET GIOCO - Modalita ");
  Serial.println(gameMode == MODE_1 ? "1 (classica)" : "2 (input valore)");

  lcd.clear();
  aggiornaDisplay();

  // All’avvio della partita il turno parte dal giocatore 1
  digitalWrite(ledP1, HIGH);
  digitalWrite(ledP2, LOW);
}

// ============================================================
// ASSEGNA PUNTO
// Aggiorna il punteggio del giocatore attivo
// ============================================================
void assegnaPunto(int punti) {
  if (partitaFinita) return;

  // Aggiunge i punti al giocatore di turno senza superare il massimo
  if (turnoP1)
    punteggioP1 = min(punteggioP1 + punti, maxScore());
  else
    punteggioP2 = min(punteggioP2 + punti, maxScore());

  Serial.print("PUNTO +");
  Serial.print(punti);
  Serial.print(" -> ");
  Serial.print(turnoP1 ? "P1" : "P2");
  Serial.print(" | P1=");
  Serial.print(punteggioP1);
  Serial.print(" P2=");
  Serial.println(punteggioP2);

  // Ogni punto valido fa ripartire il timer da 20 secondi
  resetTimerTurno();
  aggiornaDisplay();
}

// ============================================================
// CAMBIO TURNO
// Passa il turno all'altro giocatore oppure chiude la partita
// ============================================================
void cambioTurno() {
  // Evita doppi cambi ravvicinati
  if (!canChangeTurn()) return;

  // Se uno dei due giocatori ha raggiunto il punteggio massimo,
  // la partita termina e viene mostrato il vincitore
  if (punteggioP1 >= maxScore() || punteggioP2 >= maxScore()) {
    partitaFinita      = true;
    lastTurnChangeTime = millis();
    mostraVincitore();
    return;
  }

  // Inverte il turno
  turnoP1 = !turnoP1;
  // Fa ripartire il timer per il nuovo giocatore
  resetTimerTurno();
  lastTurnChangeTime = millis();

  Serial.print("CAMBIO TURNO -> ");
  Serial.println(turnoP1 ? "P1" : "P2");

  aggiornaDisplay();
}

// ============================================================
// ENTRA IN MODALITÀ INPUT VALORE
// Usata nella modalità 2 quando bisogna inserire il valore bilia
// ============================================================
void entraInInput(bool daGoal) {
  gameState   = STATE_INPUT;
  inputValore = 1;
  inputDaGoal = daGoal;

  // Congela il timer mentre l'utente sceglie il valore
  pauseTimer();

  Serial.println("INIZIO INPUT VALORE");
  mostraSchermataInput();
}

// ============================================================
// CONFERMA INPUT
// Termina l'inserimento del valore e torna al gioco
// ============================================================
void confermaInput() {
  gameState = STATE_PLAY;

  // Riattiva il timer dal punto in cui era stato fermato
  resumeTimer();

  Serial.print("CONFERMATO valore = ");
  Serial.println(inputValore);

  // Aggiunge al giocatore il valore selezionato
  assegnaPunto(inputValore);

  // Controlla se il nuovo punteggio porta alla vittoria
  if (punteggioP1 >= maxScore() || punteggioP2 >= maxScore()) {
    cambioTurno();
  }

  aggiornaDisplay();
}

// ============================================================
// pressedEdge
// Rileva la pressione del pulsante sul fronte HIGH -> LOW
// applicando un semplice debounce temporale
// ============================================================
bool pressedEdge(int pinState, bool &lastState,
                 unsigned long &lastTime, unsigned long now)
{
  bool pressed = false;

  if (pinState != lastState) {
    if ((now - lastTime) > DEBOUNCE_DELAY) {
      lastTime = now;

      // Se il pulsante passa da HIGH a LOW, è stato premuto
      if (pinState == LOW && lastState == HIGH) pressed = true;

      // Aggiorna lo stato precedente
      lastState = pinState;
    }
  }
  return pressed;
}

// ============================================================
// SETUP
// Inizializzazione generale del Receiver
// ============================================================
void setup() {
  Serial.begin(115200);

  // Configurazione dei pulsanti con resistenza di pull-up interna
  pinMode(btnUp,    INPUT_PULLUP);
  pinMode(btnDown,  INPUT_PULLUP);
  pinMode(btnTurn,  INPUT_PULLUP);
  pinMode(btnReset, INPUT_PULLUP);

  // Configurazione LED
  pinMode(ledP1, OUTPUT);
  pinMode(ledP2, OUTPUT);

  // Inizializzazione del bus I2C e del display LCD
  Wire.begin(23, 22);
  lcd.init();
  lcd.backlight();

  // Schermata iniziale di avvio
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Avvio receiver");
  lcd.setCursor(0, 1);
  lcd.print("ESP-NOW...");
  delay(1000);

  // Attiva il modulo Wi-Fi in modalità stazione
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  delay(100);

  Serial.println("=== RECEIVER ESP-NOW ===");
  Serial.print("MAC receiver: ");
  Serial.println(WiFi.STA.macAddress());

  // Inizializza ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Errore inizializzazione ESP-NOW");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Errore ESP-NOW");
    return;
  }

  // Registra la callback che gestisce i pacchetti in arrivo
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Receiver pronto.");

  // Stato iniziale: selezione modalità
  gameState = STATE_SELECT;
  highlightedMode = MODE_1;
  mostraSchermataModalita();
}

// ============================================================
// LOOP
// Gestisce input utente, ricezione goal, timer e display
// ============================================================
void loop() {
  unsigned long now = millis();

  // Lettura dei pulsanti con debounce e rilevamento del fronte
  bool upPressed    = pressedEdge(digitalRead(btnUp),    lastUp,    lastUpTime,    now);
  bool downPressed  = pressedEdge(digitalRead(btnDown),  lastDown,  lastDownTime,  now);
  bool turnPressed  = pressedEdge(digitalRead(btnTurn),  lastTurn,  lastTurnTime,  now);
  bool resetPressed = pressedEdge(digitalRead(btnReset), lastReset, lastResetTime, now);

  // ============================================================
  // RESET GLOBALE
  // È sempre disponibile e riporta il sistema alla selezione modalità
  // ============================================================
  if (resetPressed) {
    Serial.println("RESET -> selezione modalita");

    noInterrupts();
    goalHead = 0;
    goalTail = 0;
    lastGoalId = 0;
    interrupts();

    punteggioP1    = 0;
    punteggioP2    = 0;
    turnoP1        = true;
    partitaFinita  = false;
    modeSelected   = false;
    gameMode       = MODE_NONE;
    gameState      = STATE_SELECT;
    highlightedMode = MODE_1;

    resetTimerTurno();
    mostraSchermataModalita();

    digitalWrite(ledP1, LOW);
    digitalWrite(ledP2, LOW);
    return;
  }

  // ============================================================
  // STATO: SELEZIONE MODALITÀ
  // ============================================================
  if (gameState == STATE_SELECT) {
    // btnUp evidenzia la modalità 1
    if (upPressed) {
      highlightedMode = MODE_1;
      mostraSchermataModalita();
    }

    // btnDown evidenzia la modalità 2
    if (downPressed) {
      highlightedMode = MODE_2;
      mostraSchermataModalita();
    }

    // btnTurn conferma la modalità selezionata
    if (turnPressed) {
      gameMode      = highlightedMode;
      modeSelected  = true;

      Serial.print("MODALITA SELEZIONATA: ");
      Serial.println(gameMode == MODE_1 ? "1 (classica)" : "2 (input valore)");

      // Mostra brevemente la modalità scelta
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Modalita ");
      lcd.print(gameMode == MODE_1 ? "1" : "2");
      lcd.setCursor(0, 1);
      lcd.print(gameMode == MODE_1 ? "Classica (+1)  " : "Input valore   ");
      delay(1200);

      // Avvia la partita
      resetGioco();
    }

    // Durante la selezione modalità i LED restano spenti
    digitalWrite(ledP1, LOW);
    digitalWrite(ledP2, LOW);
    return;
  }

  // ============================================================
  // STATO: INSERIMENTO VALORE (solo modalità 2)
  // ============================================================
  if (gameState == STATE_INPUT) {
    bool changed = false;

    // btnUp aumenta il valore selezionato
    if (upPressed) {
      if (inputValore < MAX_INPUT_VAL) {
        inputValore++;
        changed = true;
      }
    }

    // btnDown diminuisce il valore selezionato
    if (downPressed) {
      if (inputValore > MIN_INPUT_VAL) {
        inputValore--;
        changed = true;
      }
    }

    // Se il valore è cambiato, aggiorna la schermata
    if (changed) {
      Serial.print("Input valore: ");
      Serial.println(inputValore);
      mostraSchermataInput();
    }

    // btnTurn conferma il valore scelto
    if (turnPressed) {
      confermaInput();
      // Eventuali altri goal in coda saranno gestiti nel loop successivo
    }

    // Anche durante l’input i LED continuano a indicare il giocatore di turno
    if (!partitaFinita) {
      digitalWrite(ledP1, turnoP1 ? HIGH : LOW);
      digitalWrite(ledP2, turnoP1 ? LOW  : HIGH);
    }
    return;
  }

  // ============================================================
  // STATO: GIOCO
  // ============================================================
  if (!partitaFinita) {

    // 1. Gestione dei goal ricevuti via ESP-NOW
    uint8_t sid;
    noInterrupts();
    bool hasGoal = dequeueGoal(sid);
    interrupts();

    if (hasGoal) {
      if (gameMode == MODE_1) {
        // Modalità 1: assegna subito +1
        assegnaPunto(1);

        // Se si raggiunge la soglia, verifica la vittoria
        if (punteggioP1 >= maxScore() || punteggioP2 >= maxScore())
          cambioTurno();
      } else {
        // Modalità 2: entra nella schermata di inserimento valore
        entraInInput(true);
      }
    }

    // Se l'arrivo del goal ha spostato il sistema nello stato INPUT, esce
    if (gameState == STATE_INPUT) {
      if (!partitaFinita) {
        digitalWrite(ledP1, turnoP1 ? HIGH : LOW);
        digitalWrite(ledP2, turnoP1 ? LOW  : HIGH);
      }
      return;
    }

    // 2. Pulsante manuale di incremento punteggio
    if (upPressed) {
      if (gameMode == MODE_1) {
        assegnaPunto(1);
        if (punteggioP1 >= maxScore() || punteggioP2 >= maxScore())
          cambioTurno();
      } else {
        // In modalità 2 il pulsante apre manualmente l’input
        entraInInput(false);
      }
    }

    // 3. Pulsante di decremento punteggio per correggere errori
    if (downPressed && gameState == STATE_PLAY) {
      if (gameMode == MODE_1) {
        if (turnoP1 && punteggioP1 > 0) punteggioP1--;
        else if (!turnoP1 && punteggioP2 > 0) punteggioP2--;

        Serial.print("DOWN -1 -> P1=");
        Serial.print(punteggioP1);
        Serial.print(" P2=");
        Serial.println(punteggioP2);

        aggiornaDisplay();
      } else {
        // TODO: in modalità 2 si potrebbe implementare una sottrazione
        // del valore esatto della bilia, con una schermata simile all'input
      }
    }

    // 4. Cambio turno manuale
    if (turnPressed && gameState == STATE_PLAY) {
      Serial.println("CAMBIO TURNO MANUALE");
      cambioTurno();
    }

    // 5. Controllo timeout turno:
    // se il timer raggiunge i 20 secondi senza nuovi punti, cambia turno
    if (!timerPaused && (millis() - turnStartTime) + turnElapsedBeforePause >= TURN_TIME) {
      Serial.print("TIMEOUT -> turno scaduto di ");
      Serial.println(turnoP1 ? "P1" : "P2");
      cambioTurno();
    }
  }

  // Aggiornamento periodico del display per il conto alla rovescia
  if ((now - lastDisplayUpdate) >= DISPLAY_REFRESH) {
    lastDisplayUpdate = now;
    aggiornaDisplay();
  }

  // Aggiornamento LED:
  // se la partita è finita, entrambi spenti;
  // altrimenti acceso solo il LED del giocatore di turno
  if (partitaFinita) {
    digitalWrite(ledP1, LOW);
    digitalWrite(ledP2, LOW);
  } else {
    digitalWrite(ledP1, turnoP1 ? HIGH : LOW);
    digitalWrite(ledP2, turnoP1 ? LOW  : HIGH);
  }
}