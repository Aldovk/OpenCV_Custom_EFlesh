#include <DynamixelSDK.h>

// =============== CONFIGURACIÓN ===============
#define DEVICENAME "Serial1"      // Puerto Dynamixel (UART3)
#define BAUDRATE 1000000          // Dynamixel baudrate
#define BLUETOOTH_BAUD 57600      // BT-210
#define btSerial Serial2          // Bluetooth en USART2

int dxl_ids[5] = {121, 122, 123, 124, 125};
const int NUM_SERVOS = 5;

// Direcciones EEPROM/RAM
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36

// Posiciones abiertas/cerradas
#define OPEN_POSITION 250
#define OPEN_POSITION121 250
#define OPEN_POSITION124 950
#define OPEN_POSITION125 400

#define CLOSE_POSITION 600
#define CLOSE_POSITION121 700
#define CLOSE_POSITION124 500
#define CLOSE_POSITION125 130

// Sensores
#define SENSOR_PIN1 2
#define SENSOR_PIN2 3
#define SENSOR_PIN3 4
#define SENSOR_OPEN_121_123 550
#define SENSOR_STRONG_GRIP_121_123 350
#define SENSOR_STRONG_GRIP_124 800
#define SENSOR_STRONG_GRIP_125 350
#define SENSOR_OPEN_124 650
#define SENSOR_OPEN_125 250

// Grip
#define GRIP_STEP 30
#define GRIP_DELAY 500
#define STRONG_GRIP_THRESHOLD 700
#define DESIRED_GRIP_MIN 300
#define DESIRED_GRIP_MAX 500

// =============== OBJETOS ===============
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// =============== VARIABLES GLOBALES ===============
String incoming = "";

// =============== FUNCIONES DE CONTROL ===============
void enable_torque() {
  uint8_t dxl_error;
  for (int i = 0; i < NUM_SERVOS; i++) {
    int result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (result != COMM_SUCCESS) {
      Serial.print("Error torque ON ID "); Serial.println(dxl_ids[i]);
      btSerial.print("Error torque ON ID "); btSerial.println(dxl_ids[i]); btSerial.flush();
    } else {
      Serial.print("Torque ON ID "); Serial.println(dxl_ids[i]);
      btSerial.print("Torque ON ID "); btSerial.println(dxl_ids[i]); btSerial.flush();
    }
  }
}

void disable_torque() {
  uint8_t dxl_error;
  for (int i = 0; i < NUM_SERVOS; i++) {
    int result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_TORQUE_ENABLE, 0, &dxl_error);
    if (result != COMM_SUCCESS) {
      Serial.print("Error torque OFF ID "); Serial.println(dxl_ids[i]);
      btSerial.print("Error torque OFF ID "); btSerial.println(dxl_ids[i]); btSerial.flush();
    } else {
      Serial.print("Torque OFF ID "); Serial.println(dxl_ids[i]);
      btSerial.print("Torque OFF ID "); btSerial.println(dxl_ids[i]); btSerial.flush();
    }
  }
}

void set_position(int dxl_id, uint16_t position) {
  uint8_t dxl_error;
  int result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (result != COMM_SUCCESS) {
    Serial.print("Error pos ID "); Serial.println(dxl_id);
    btSerial.print("Error pos ID "); btSerial.println(dxl_id); btSerial.flush();
  } else {
    Serial.print("Pos "); Serial.print(position); Serial.print(" → ID "); Serial.println(dxl_id);
    btSerial.print("Pos "); btSerial.print(position); btSerial.print(" → ID "); btSerial.println(dxl_id); btSerial.flush();
  }
}

uint16_t read_position(int dxl_id) {
  uint16_t position = 0;
  uint8_t dxl_error = 0;
  int result = packetHandler->read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, &position, &dxl_error);
  if (result != COMM_SUCCESS) {
    Serial.print("Error read pos ID "); Serial.println(dxl_id);
    btSerial.print("Error read pos ID "); btSerial.println(dxl_id); btSerial.flush();
  }
  return position;
}

// ============== FUNCIONES MIN Y MAX ===============
// Funciones min/max seguras para uint16_t
uint16_t my_min(uint16_t a, uint16_t b) {
  return (a < b) ? a : b;
}

uint16_t my_max(uint16_t a, uint16_t b) {
  return (a > b) ? a : b;
}

// =============== FUNCIONES DE DEDOS ===============
void open_finger(int id) {
  int pos = (id == 121) ? OPEN_POSITION121 :
            (id == 124) ? OPEN_POSITION124 :
            (id == 125) ? OPEN_POSITION125 : OPEN_POSITION;
  set_position(id, pos);
  delay(200);
}

void close_finger(int id) {
  int pos = (id == 121) ? CLOSE_POSITION121 :
            (id == 124) ? CLOSE_POSITION124 :
            (id == 125) ? CLOSE_POSITION125 : CLOSE_POSITION;
  set_position(id, pos);
  delay(200);
}

void open_hand() {
  Serial.println("Abriendo mano...");
  btSerial.println("Abriendo mano..."); btSerial.flush();
  open_finger(121); open_finger(122); open_finger(123); open_finger(124); open_finger(125);
  delay(200);
}

void close_hand() {
  Serial.println("Cerrando mano...");
  btSerial.println("Cerrando mano..."); btSerial.flush();
  close_finger(121); close_finger(122); close_finger(123); close_finger(124); close_finger(125);
  delay(200);
}

void wave_hand() {
  Serial.println("Saludando...");
  btSerial.println("Saludando..."); btSerial.flush();
  int k = 0;
  while (k<3) {
    open_finger(121); open_finger(122); open_finger(123); open_finger(124); open_finger(125);
    delay(600);
    close_finger(121); close_finger(122); close_finger(123); close_finger(124); close_finger(125);
    delay(600);
    k++;
  }
}

// =============== MODO GRIP ===============
void grip() {
  Serial.println("Modo grip activado. Presione 'q' para salir.");
  btSerial.println("Modo grip activado. Presione 'q' para salir."); btSerial.flush();

  open_finger(121); open_finger(122); open_finger(123);
  set_position(124, OPEN_POSITION124);
  set_position(125, OPEN_POSITION125);
  delay(500);

  uint16_t pos122 = read_position(122);
  uint16_t pos121 = read_position(121);
  uint16_t pos124 = read_position(124);
  uint16_t pos125 = read_position(125);
  bool gripping = true;

  while (true) {
  // check de smd bluetooth
  if (btSerial.available()) {
    char input = btSerial.read();
    if (input == 'q') {
      Serial.println("Saliendo de Grip...");
      btSerial.println("Exiting grip function..."); 
      btSerial.flush();
      delay(2500);
      break; // Exit the loop
    }
  }

  int sensor3 = analogRead(SENSOR_PIN3);
  Serial.print("Sensor3: "); Serial.println(sensor3);
  btSerial.print("Sensor3: "); btSerial.println(sensor3); 
  btSerial.flush();

  if (gripping) {
    if (pos122 < CLOSE_POSITION) { pos122 = my_min(pos122 + GRIP_STEP, CLOSE_POSITION); set_position(122, pos122); }
    if (pos121 < CLOSE_POSITION121) { pos121 = my_min(pos121 + GRIP_STEP, CLOSE_POSITION121); set_position(121, pos121); }
    if (pos124 > CLOSE_POSITION124) { pos124 = my_max(pos124 - GRIP_STEP, CLOSE_POSITION124); set_position(124, pos124); }
    if (pos125 > CLOSE_POSITION125) { pos125 = my_max(pos125 - GRIP_STEP, CLOSE_POSITION125); set_position(125, pos125); }
    delay(GRIP_DELAY);

    if (sensor3 > STRONG_GRIP_THRESHOLD) {
      Serial.println("Agarre fuerte → liberando...");
      btSerial.println("Agarre fuerte → liberando..."); 
      btSerial.flush();
      gripping = false;
    }
  } else {
    if (pos122 > OPEN_POSITION) { pos122 = my_max(pos122 - GRIP_STEP, OPEN_POSITION); set_position(122, pos122); }
    if (pos121 > OPEN_POSITION121) { pos121 = my_max(pos121 - GRIP_STEP, OPEN_POSITION121); set_position(121, pos121); }
    if (pos124 < OPEN_POSITION124) { pos124 = my_min(pos124 + GRIP_STEP, OPEN_POSITION124); set_position(124, pos124); }
    if (pos125 < OPEN_POSITION125) { pos125 = my_min(pos125 + GRIP_STEP, OPEN_POSITION125); set_position(125, pos125); }
    delay(GRIP_DELAY);

    if (sensor3 >= DESIRED_GRIP_MIN && sensor3 <= DESIRED_GRIP_MAX) {
      Serial.println("Rango deseado alcanzado. Grip detenido.");
      btSerial.println("Rango deseado alcanzado. Grip detenido."); 
      btSerial.flush();
      break;
    }
  } delay(100);
  }
}

// =============== PROCESAR COMANDO ===============
void processCommand(String cmd) {
  cmd.trim();
  Serial.println("Comando: " + cmd);
  btSerial.println("Recibido: " + cmd); btSerial.flush();

  if (cmd == "open_finger1") open_finger(121);
  else if (cmd == "close_finger1") close_finger(121);
  else if (cmd == "open_finger2") open_finger(122);
  else if (cmd == "close_finger2") close_finger(122);
  else if (cmd == "open_finger3") open_finger(123);
  else if (cmd == "close_finger3") close_finger(123);
  else if (cmd == "open_finger4") open_finger(124);
  else if (cmd == "close_finger4") close_finger(124);
  else if (cmd == "open_finger5") open_finger(125);
  else if (cmd == "close_finger5") close_finger(125);
  else if (cmd == "open_all") open_hand();
  else if (cmd == "close_all") close_hand();
  else if (cmd == "grip") grip();
  else if (cmd == "wave_hand") wave_hand();
  else if (cmd == "q") {
    disable_torque();
    Serial.println("Torque OFF. Fin.");
    btSerial.println("Torque OFF. Fin."); btSerial.flush();
    while (true);
  }
  else {
    btSerial.println("Comando desconocido"); btSerial.flush();
  }
}

// =============== SETUP ===============
void setup() {
  Serial.begin(115200);
  btSerial.begin(BLUETOOTH_BAUD);

  pinMode(SENSOR_PIN1, INPUT);
  pinMode(SENSOR_PIN2, INPUT);
  pinMode(SENSOR_PIN3, INPUT);

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (!portHandler->openPort()) {
    Serial.println("Error: Puerto Dynamixel");
    btSerial.println("Error: Puerto Dynamixel"); btSerial.flush();
    while (true);
  }
  if (!portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Error: Baud Dynamixel");
    btSerial.println("Error: Baud Dynamixel"); btSerial.flush();
    while (true);
  }

  Serial.println("Sistema iniciado. BT-210 listo.");
  btSerial.println("BT-210 listo. Envia comandos como open_finger1"); btSerial.flush();

  for (int i = 0; i < 3; i++) {
    btSerial.print("Prueba BT #"); btSerial.println(i + 1); btSerial.flush();
    delay(500);
  }

  enable_torque();
  open_hand();
}

// =============== LOOP ============ 
void loop() {
  // Leer Bluetooth
  while (btSerial.available()) {
    char c = btSerial.read();
    if (c == '\n' || c == '\r') {
      if (incoming.length() > 0) {
        processCommand(incoming);
        incoming = "";
      }
    } else {
      incoming += c;
    }
  }

  // Menú cada 5 segundos
  static unsigned long lastMenu = 0;
  if (millis() - lastMenu > 5000) {
    btSerial.println("\nComandos: open_finger1..5, close_finger1..5, open_all, close_all, grip, q");
    lastMenu = millis();
  }

  delay(100);
}