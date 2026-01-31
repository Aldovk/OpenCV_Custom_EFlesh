// Mano XL-320 + OpenCM 9.04 + BT-210 + FSR en pulgar 
// Funcion grip() mejorada
#include <DynamixelSDK.h>

// ============== CONFIG===============
#define DEVICENAME "Serial1"
#define BAUDRATE   1000000
#define BLUETOOTH_BAUD 57600

int dxl_ids[5] = {121, 122, 123, 124, 125};
const int NUM_SERVOS = 5;

// Direcciones XL-320
#define ADDR_TORQUE_ENABLE     24
#define ADDR_GOAL_POSITION     30
#define ADDR_PRESENT_POSITION  36

#define TORQUE_ENABLE  1
#define TORQUE_DISABLE 0

// ============POSICIONES(calibradas) ==========
// ABIERTO
#define OPEN_POSITION121  250    // Pulgar
#define OPEN_POSITION122  250    // Índice
#define OPEN_POSITION123  250    // Medio
#define OPEN_POSITION124  950    // Anular
#define OPEN_POSITION125  400    // Meñique

// CERRADO
#define CLOSE_POSITION121  700   // Pulgar
#define CLOSE_POSITION122  600   // Índice
#define CLOSE_POSITION123  600   // Medio
#define CLOSE_POSITION124  500   // Anular
#define CLOSE_POSITION125  130   // Meñique

// ============ CONTROL DE FSR ================
#define FORCE_SENSOR_PIN      3
#define TARGET_FORCE_MIN    300     // Ajustar con pruebas
#define TARGET_FORCE_MAX    520     //
#define FORCE_DEADZONE       20
#define GRIP_SPEED_FAST      50     // Inicialmente rapido
#define GRIP_SPEED_SLOW      15     // Reduce la velocidad para fino ajuste

// ====================== CONFIG - Paquetes dynamixel ========================
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
#define btSerial Serial2

// ================= FUNCIONES=================
void set_position(int id, uint16_t pos) {
  uint8_t dxl_error = 0;
  packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, pos, &dxl_error);
}

void enable_torque() {
  uint8_t dxl_error;
  for (int i = 0; i < NUM_SERVOS; i++) {
    packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  }
}

void disable_torque() {
  uint8_t dxl_error;
  for (int i = 0; i < NUM_SERVOS; i++) {
    packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  }
}

// ================= MOVIMIENTOS ==================
void open_hand() {
  Serial.println(F("Abriendo mano..."));
  btSerial.println(F("Abriendo...")); btSerial.flush();
  set_position(121, OPEN_POSITION121);
  set_position(122, OPEN_POSITION122);
  set_position(123, OPEN_POSITION123);
  set_position(124, OPEN_POSITION124);
  set_position(125, OPEN_POSITION125);
  delay(600);
}

void close_hand() {
  Serial.println(F("Cerrando mano..."));
  btSerial.println(F("Cerrando...")); btSerial.flush();
  set_position(121, CLOSE_POSITION121);
  set_position(122, CLOSE_POSITION122);
  set_position(123, CLOSE_POSITION123);
  set_position(124, CLOSE_POSITION124);
  set_position(125, CLOSE_POSITION125);
  delay(600);
}

void wave_hand() {
  Serial.println(F("Saludando..."));
  btSerial.println(F("Saludando...")); btSerial.flush();
  for (int i = 0; i < 3; i++) {
    open_hand();  delay(600);
    close_hand(); delay(600);
  }
}

// ================= GRIP CON CONTROL DE FUERZA ===================
void grip_force_control() {
  Serial.println(F("=== GRIP POR FUERZA - Pulsa 'q' para salir ===>"));
  btSerial.println(F("GRIP activo (FSR pulgar) - 'q' salir")); btSerial.flush();

  // Posición de aproximacion al agarre del objeto
  set_position(121, OPEN_POSITION121);
  set_position(122, 300);
  set_position(123, 300);
  set_position(124, OPEN_POSITION124);
  set_position(125, OPEN_POSITION125 + 100);
  delay(700);

  uint16_t pos121 = OPEN_POSITION121;
  uint16_t pos122 = 300;
  uint16_t pos123 = 300;
  uint16_t pos124 = OPEN_POSITION124;
  uint16_t pos125 = OPEN_POSITION125 + 100;

  while (true) {
    // Lectura filtrada del FSR
    long force = 0;
    for (int i = 0; i < 10; i++) { force += analogRead(FORCE_SENSOR_PIN); delay(10); }
    force /= 10;

    int error = 0;
    int speed = GRIP_SPEED_FAST;

    if (force < TARGET_FORCE_MIN - 40) {
      error = 100;                                         // Cerrar fuerte
    } else if (force > TARGET_FORCE_MAX + 60) {
      error = -100;                                        // Abrir si muy fuerte
    } else if (force < TARGET_FORCE_MIN) {
      error = TARGET_FORCE_MIN - force;                    // Ajuste fino
      speed = GRIP_SPEED_SLOW;
    } else if (force > TARGET_FORCE_MAX) {
      error = -(force - TARGET_FORCE_MAX);
      speed = GRIP_SPEED_SLOW;
    } else {
      error = 0;
      Serial.print(F(" OK"));
      btSerial.print(F(" OK"));
    }

    if (abs(error) < FORCE_DEADZONE) error = 0;

    Serial.print(F(" F:")); Serial.print(force);
    Serial.print(F(" E:")); Serial.print(error);
    btSerial.print(F(" F:")); Serial.print(force);
    btSerial.print(F(" E:")); Serial.print(error);

    if (error != 0) {
      int step = (error > 0) ? speed : -speed;

      // Pulgar (aumentar = cerrar)
      pos121 += step;
      pos121 = constrain(pos121, OPEN_POSITION121, CLOSE_POSITION121);

      // Índice y medio (disminuir = cerrar)
      pos122 -= step;
      pos123 -= step;
      pos122 = constrain(pos122, OPEN_POSITION122, CLOSE_POSITION122);
      pos123 = constrain(pos123, OPEN_POSITION123, CLOSE_POSITION123);

      // Anular y meñique (disminuir = cerrar)
      pos124 -= step;
      pos125 -= step;
      pos124 = constrain(pos124, CLOSE_POSITION124, OPEN_POSITION124);
      pos125 = constrain(pos125, CLOSE_POSITION125, OPEN_POSITION125);

      set_position(121, pos121);
      set_position(122, pos122);
      set_position(123, pos123);
      set_position(124, pos124);
      set_position(125, pos125);
    }

    Serial.println();
    btSerial.println(); btSerial.flush();

    if (btSerial.available()) {
      char c = btSerial.read();
      while (btSerial.available()) btSerial.read();
      if (c == 'q' || c == 'Q') {
        Serial.println(F("Saliendo grip"));
        btSerial.println(F("Grip OFF")); btSerial.flush();
        break;
      }
    }
    delay(30);
  }

  Serial.println(F("Grip terminado - posición mantenida"));
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  btSerial.begin(BLUETOOTH_BAUD);
  pinMode(FORCE_SENSOR_PIN, INPUT);

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
    Serial.println(F("ERROR Dynamixel"));
    while (true);
  }

  Serial.println(F("Mano iniciada - BT listo"));
  for (int i = 0; i < 5; i++) {
    btSerial.print(F("BT OK #")); btSerial.println(i+1); btSerial.flush();
    delay(400);
  }

  enable_torque();
  open_hand();
}

void loop() {
  static unsigned long lastMenu = 0;
  if (millis() - lastMenu > 5000) {
    Serial.println(F("\n1=abrir  2=cerrar  3=saludar  g=grip_fuerza  q=off"));
    btSerial.println(F("\n1=abrir  2=cerrar  3=saludar  g=grip_fuerza  q=off")); btSerial.flush();
    lastMenu = millis();
  }

  if (btSerial.available()) {
    char cmd = btSerial.read();
    while (btSerial.available()) btSerial.read();

    switch (cmd) {
      case '1': open_hand(); break;
      case '2': close_hand(); break;
      case '3': wave_hand(); break;
      case 'g': case 'G': grip_force_control(); break;
      case 'q': case 'Q':
        disable_torque();
        Serial.println(F("Torque OFF"));
        btSerial.println(F("Apagado")); btSerial.flush();
        while(true) delay(1000);
        break;
    }
  }
  delay(50);
}