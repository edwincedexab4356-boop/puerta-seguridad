/*
  PUERTA DE SEGURIDAD - Arduino UNO R4 WiFi
  Entra con tarjeta RFID o con codigo (PIN) desde el simulador por Bluetooth HC-05.

  Librerias (Gestor de librerias): "MFRC522" (GithubCommunity), "LiquidCrystal I2C" (Frank de Brabander)
  y "Servo". SPI, Wire, EEPROM y Arduino_LED_Matrix ya vienen con la placa.

  CONEXIONES
  HC-05       VCC->5V  GND->GND  TXD->D0  RXD<-D1 (con divisor: D1-220-RXD-220-220-GND)
  RC522       3.3V->3.3V (NO a 5V)  GND->GND  RST->3.3V  SDA->D10  SCK->D13  MOSI->D11  MISO->D12  IRQ sin usar
  LCD 16x2 I2C VCC->5V  GND->GND  SDA->SDA  SCL->SCL
  Servo SG90  cafe->GND  rojo->5V  naranja->D9
  LED blanco  D6 -> 220 -> 220 -> LED(+) ; LED(-) -> GND
  LED verde   D5 -> 220 -> 220 -> LED(+) ; LED(-) -> GND
  LED rojo    D4 -> 220 -> 220 -> LED(+) ; LED(-) -> GND
  Buzzer      (+)->D2 o D3  (-)->GND   (pasivo: el sonido se genera a mano, sin tone())
  HC-SR04     VCC->5V  TRIG->D7  ECHO->D8  GND->GND

  ORDENES (9600, por Bluetooth o USB, una por linea)
  CODIGO:1234        entrar con el codigo
  CAMBIARPIN:viejo:nuevo   (4 a 6 numeros)
  ABRIR / ABRIR:0 (abierta hasta CERRAR) / CERRAR
  BLOQUEAR / DESBLOQUEAR   (bloqueada: solo abre una tarjeta de administrador o el simulador)
  PANICO / SILENCIAR
  REGISTRAR:NOMBRE[:1]  (la siguiente tarjeta se guarda; :1 = administrador)  CANCELAR
  TARJ:UID:admin:activa:NOMBRE   BORRAR:UID   BORRARTODO   LISTA
  TIEMPO:n (s abierta 2-60)  BLOQUEO:n (s 10-600)  INTENTOS:n (2-9)  ANGULO:n (30-180)
  LUZ:0/1/2 (blanca apagada/prendida/automatica)  SONIDO:0/1  PITAR  PRUEBA  ESTADO  REINICIAR
  DIST:n (cm para detectar a alguien, 5-200)  MERODEO:n (s parado frente a la puerta para avisar, 0 = no avisar)

  MENSAJES QUE ENVIA
  ST,estado,abierta,restante,verde,rojo,blanca,modoLuz,fallos,intentos,bloqueoRest,nTarj,sonido,
     tiempo,bloqueoSeg,angulo,lcdOk,rfidOk,registrando,mantener,distancia,presencia,distDetectar,merodeo,ultraOk
     (cada 1 s y con cada cambio; distancia 999 = nada al frente)
     estado: 0 cerrada, 1 abierta, 2 bloqueo por intentos, 3 bloqueada a mano, 4 alarma (panico)
  EV,texto        lo que pasa
  TJ,uid,ok,nombre,motivo     cada tarjeta pasada
  CD,ok,motivo                cada codigo probado
  LC,linea1,linea2            lo que muestra la pantalla
  TL,uid,admin,activa,nombre ... TL,FIN   lista de tarjetas
*/
#include <SPI.h>
#include <Wire.h>
#include <MFRC522.h>
#include <EEPROM.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include "Arduino_LED_Matrix.h"

#define BT Serial1

const byte P_BUZ2 = 3;   // el sonido sale por D2 y D3: el buzzer puede ir en cualquiera de los dos
const byte P_BUZ = 2, P_ROJO = 4, P_VERDE = 5, P_BLANCA = 6, P_TRIG = 7, P_ECHO = 8, P_SERVO = 9, P_SS = 10;
const byte MAX_TARJ = 10;
const byte FIRMA = 0xD2;

MFRC522 rfid(P_SS, MFRC522::UNUSED_PIN);
Servo servo;
LiquidCrystal_I2C *lcd = nullptr;
byte lcdDir = 0;
bool rfidOk = false;
ArduinoLEDMatrix matriz;

// ---------- ajustes (EEPROM) ----------
struct Tarjeta { byte len; byte uid[7]; byte admin; byte activa; char nombre[11]; };
struct Ajustes {
  byte firma;
  char pin[7];
  byte tiempo, intentos, angulo, modoLuz, sonido;
  uint16_t bloqueo;
  byte dist, merodeo;
  byte n;
  Tarjeta t[MAX_TARJ];
} A;

void guardar() { EEPROM.put(0, A); }
void porDefecto() {
  memset(&A, 0, sizeof(A));
  A.firma = FIRMA; strcpy(A.pin, "1234");
  A.tiempo = 5; A.intentos = 3; A.angulo = 90; A.modoLuz = 2; A.sonido = 1; A.bloqueo = 30; A.dist = 40; A.merodeo = 20; A.n = 0;
  guardar();
}
void cargar() {
  EEPROM.get(0, A);
  if (A.firma != FIRMA || A.n > MAX_TARJ) porDefecto();
}

// ---------- estado ----------
enum { CERRADA, ABIERTA, BLOQ_INTENTOS, BLOQ_MANUAL, ALARMA };
byte estado = CERRADA;
bool mantener = false;           // ABRIR:0
unsigned long cerrarEn = 0, bloqueoHasta = 0, blancaHasta = 0, regHasta = 0;
byte fallos = 0;
bool registrando = false, regAdmin = false;
char regNombre[11] = "";
int anguloActual = 0;
unsigned long servoSoltar = 0;
String linUsb, linBt;
bool cambio = true;
// ---- ultrasonico ----
int distancia = 999;            // cm (999 = nada al frente)
bool presencia = false, ultraOk = true, avisoMerodeo = false;
unsigned long presenciaDesde = 0, sinEco = 0;

// ---------- salida a los dos puertos ----------
void enviar(const String &s) { Serial.println(s); BT.println(s); }
void evento(const String &s) { enviar("EV," + s); }

// ---------- buzzer pasivo ----------
// El sonido se hace a mano (sin tone()), porque en el R4 tone() y el servo usan
// los mismos temporizadores y, despues de mover el servo, tone() deja de sonar.
void onda(unsigned int ms, unsigned int frec = 2000) {
  unsigned long medio = 500000UL / frec, fin = millis() + ms;
  while ((long)(millis() - fin) < 0) {
    digitalWrite(P_BUZ, HIGH); digitalWrite(P_BUZ2, HIGH); delayMicroseconds(medio);
    digitalWrite(P_BUZ, LOW);  digitalWrite(P_BUZ2, LOW);  delayMicroseconds(medio);
  }
}
void silencio(unsigned int ms) { digitalWrite(P_BUZ, LOW); digitalWrite(P_BUZ2, LOW); delay(ms); }
// 1 acceso, 2 error, 4 cierre, 5 registro, 6 tecla, 7 abre (puerta se activa)
void sonar(byte p) {
  if (!A.sonido) return;
  switch (p) {
    case 1: onda(120, 2000); silencio(60); onda(120, 2000); silencio(60); onda(250, 2600); break;  // la puerta se activa
    case 2: onda(600, 900); break;
    case 4: onda(150, 1600); break;
    case 5: for (byte i = 0; i < 3; i++) { onda(80, 2400); silencio(100); } break;
    case 6: onda(25, 2400); break;
  }
}
// sirena de alarma y de bloqueo (sube y baja de tono)
void buzzer() {
  bool sirena = estado == ALARMA || (estado == BLOQ_INTENTOS && bloqueoHasta - millis() > (unsigned long)A.bloqueo * 1000UL - 10000UL);
  if (!sirena || (!A.sonido && estado != ALARMA)) { digitalWrite(P_BUZ, LOW); digitalWrite(P_BUZ2, LOW); return; }
  static unsigned long t = 0; static bool alto = false;
  if (millis() - t >= 450) { t = millis(); alto = !alto; onda(250, alto ? 2600 : 1800); }
}

// ---------- matriz del R4 ----------
const char *F_CERR[8] = {"....####....","...#....#...","...#....#...","..########..","..###..###..","..###..###..","..########..","............"};
const char *F_ABIE[8] = {"....####....","...#....#...","........#...","..########..","..###..###..","..###..###..","..########..","............"};
const char *F_X[8]    = {"..#......#..","...#....#...","....#..#....",".....##.....",".....##.....","....#..#....","...#....#...","..#......#.."};
const char *F_TARJ[8] = {"............",".##########.",".#........#.",".#.###....#.",".#.###....#.",".#........#.",".##########.","............"};
byte dibujoActual = 255;
void dibujar(byte k) {
  if (k == dibujoActual) return;
  dibujoActual = k;
  const char **f = k == 0 ? F_CERR : k == 1 ? F_ABIE : k == 2 ? F_X : F_TARJ;
  uint8_t b[8][12];
  for (byte y = 0; y < 8; y++) for (byte x = 0; x < 12; x++) b[y][x] = f[y][x] == '#';
  matriz.renderBitmap(b, 8, 12);
}

// ---------- pantalla LCD ----------
String l1 = "", l2 = "";
unsigned long msgHasta = 0;
void pantalla(String a, String b) {
  while (a.length() < 16) a += ' ';
  while (b.length() < 16) b += ' ';
  a = a.substring(0, 16); b = b.substring(0, 16);
  if (a == l1 && b == l2) return;
  l1 = a; l2 = b;
  if (lcd) { lcd->setCursor(0, 0); lcd->print(l1); lcd->setCursor(0, 1); lcd->print(l2); }
  enviar("LC," + l1 + "," + l2);
}
void mensaje(const String &a, const String &b, unsigned int ms) { msgHasta = millis() + ms; pantalla(a, b); }
void buscarLcd() {
  for (byte d = 0x20; d <= 0x3F && !lcdDir; d++) {
    if (d > 0x27 && d < 0x38) continue;
    Wire.beginTransmission(d);
    if (Wire.endTransmission() == 0) lcdDir = d;
  }
  if (!lcdDir) return;
  lcd = new LiquidCrystal_I2C(lcdDir, 16, 2);
  delay(50);
  lcd->init(); lcd->init();
  lcd->backlight();
  lcd->clear();
}

// ---------- servo (suave y se suelta para que no zumbe) ----------
int anguloMeta = 0;
unsigned long tServo = 0;
void moverServo() {
  if (millis() - tServo < 12) return;
  tServo = millis();
  if (anguloActual != anguloMeta) {
    if (!servo.attached()) servo.attach(P_SERVO);
    anguloActual += anguloActual < anguloMeta ? 3 : -3;
    if (abs(anguloActual - anguloMeta) < 3) anguloActual = anguloMeta;
    servo.write(anguloActual);
    servoSoltar = millis() + 500;
  } else if (servo.attached() && millis() > servoSoltar) servo.detach();
}

// ---------- acciones ----------
void luzBlanca(unsigned int s) { blancaHasta = millis() + s * 1000UL; }

void abrir(const String &quien, bool fijo) {
  estado = ABIERTA; mantener = fijo; fallos = 0;
  cerrarEn = millis() + A.tiempo * 1000UL;
  anguloMeta = A.angulo;
  luzBlanca(A.tiempo + 5);
  sonar(1);
  dibujar(1);
  mensaje("ACCESO PERMITIDO", quien, 2000);
  evento("Puerta ABIERTA: " + quien + (fijo ? " (se queda abierta)" : " (" + String(A.tiempo) + " s)"));
  cambio = true;
}
void cerrar(const String &por) {
  if (estado != ABIERTA) return;
  estado = CERRADA; mantener = false;
  anguloMeta = 0;
  sonar(4);
  evento("Puerta CERRADA (" + por + ")");
  cambio = true;
}
void negar(const String &motivo) {
  fallos++;
  sonar(2);
  dibujar(2);
  if (fallos >= A.intentos) {
    estado = BLOQ_INTENTOS; anguloMeta = 0;
    bloqueoHasta = millis() + A.bloqueo * 1000UL;
    mensaje("!! BLOQUEADA !!", String(fallos) + " intentos mal", 3000);
    evento("BLOQUEO: " + String(fallos) + " intentos fallidos. Bloqueada " + String(A.bloqueo) + " s");
  } else {
    mensaje("ACCESO DENEGADO", motivo + " " + String(fallos) + "/" + String(A.intentos), 2000);
    evento("Acceso denegado (" + motivo + "). Intento " + String(fallos) + " de " + String(A.intentos));
  }
  luzBlanca(5);
  cambio = true;
}

String uidTexto(const byte *u, byte len) {
  String s;
  for (byte i = 0; i < len; i++) { if (u[i] < 16) s += '0'; s += String(u[i], HEX); }
  s.toUpperCase();
  return s;
}
int buscarTarjeta(const String &uid) {
  for (byte i = 0; i < A.n; i++) if (uidTexto(A.t[i].uid, A.t[i].len) == uid) return i;
  return -1;
}

void tarjetaLeida(const byte *u, byte len) {
  String uid = uidTexto(u, len);
  luzBlanca(8);
  if (registrando) {
    registrando = false;
    if (buscarTarjeta(uid) >= 0) { enviar("TJ," + uid + ",0,,YAEXISTE"); mensaje("Esa tarjeta ya", "esta registrada", 2000); sonar(2); evento("La tarjeta " + uid + " ya estaba registrada"); }
    else if (A.n >= MAX_TARJ) { enviar("TJ," + uid + ",0,,LLENO"); mensaje("Memoria llena", "max 10 tarjetas", 2000); sonar(2); evento("No caben mas tarjetas (maximo 10)"); }
    else {
      Tarjeta &t = A.t[A.n++];
      t.len = len > 7 ? 7 : len; memcpy(t.uid, u, t.len);
      t.admin = regAdmin || A.n == 1; t.activa = 1;
      strncpy(t.nombre, regNombre[0] ? regNombre : "Tarjeta", 10); t.nombre[10] = 0;
      guardar();
      enviar("TJ," + uid + ",1," + String(t.nombre) + ",REGISTRADA");
      mensaje("Tarjeta guardada", String(t.nombre), 2500); sonar(5);
      evento("Tarjeta registrada: " + String(t.nombre) + " (" + uid + ")" + (t.admin ? " como ADMINISTRADOR" : ""));
      enviarLista();
    }
    cambio = true; return;
  }
  int i = buscarTarjeta(uid);
  String nom = i >= 0 ? String(A.t[i].nombre) : "";
  bool admin = i >= 0 && A.t[i].admin;
  if (estado == ALARMA && !admin) { enviar("TJ," + uid + ",0," + nom + ",ALARMA"); evento("Tarjeta " + uid + " rechazada: hay una alarma (solo administrador)"); sonar(2); return; }
  if (estado == BLOQ_INTENTOS && !admin) {
    enviar("TJ," + uid + ",0," + nom + ",BLOQUEADA");
    mensaje("BLOQUEADA", "espera " + String((bloqueoHasta - millis()) / 1000 + 1) + " s", 1500);
    evento("Tarjeta " + (nom.length() ? nom : uid) + " ignorada: sistema bloqueado"); sonar(2); return;
  }
  if (estado == BLOQ_MANUAL && !admin) {
    enviar("TJ," + uid + ",0," + nom + ",CERRADA");
    mensaje("PUERTA BLOQUEADA", "solo administr.", 1500);
    evento("Tarjeta " + (nom.length() ? nom : uid) + " rechazada: puerta bloqueada a mano"); sonar(2); return;
  }
  if (i < 0) { enviar("TJ," + uid + ",0,,DESCONOCIDA"); evento("Tarjeta desconocida " + uid); negar("Desconocida"); return; }
  if (!A.t[i].activa) { enviar("TJ," + uid + ",0," + nom + ",DESACTIVADA"); evento("Tarjeta desactivada: " + nom); negar("Desactivada"); return; }
  enviar("TJ," + uid + ",1," + nom + ",OK");
  if (estado == ALARMA || estado == BLOQ_INTENTOS || estado == BLOQ_MANUAL) evento("Administrador " + nom + " quita el bloqueo/alarma");
  abrir(nom, false);
}

void probarCodigo(const String &c) {
  luzBlanca(8);
  if (estado == ALARMA) { enviar("CD,0,ALARMA"); evento("Codigo ignorado: hay una alarma (usa SILENCIAR)"); sonar(2); return; }
  if (estado == BLOQ_INTENTOS) { enviar("CD,0,BLOQUEADA"); evento("Codigo ignorado: sistema bloqueado"); sonar(2); return; }
  if (estado == BLOQ_MANUAL) { enviar("CD,0,CERRADA"); evento("Codigo ignorado: puerta bloqueada a mano"); sonar(2); return; }
  if (c == String(A.pin)) { enviar("CD,1,OK"); abrir("Codigo correcto", false); }
  else { enviar("CD,0,MAL"); evento("Codigo incorrecto"); negar("Codigo mal"); }
}

void enviarLista() {
  for (byte i = 0; i < A.n; i++)
    enviar("TL," + uidTexto(A.t[i].uid, A.t[i].len) + "," + String(A.t[i].admin) + "," + String(A.t[i].activa) + "," + String(A.t[i].nombre));
  enviar("TL,FIN");
}

void enviarEstado() {
  unsigned long ahora = millis();
  int rest = estado == ABIERTA && !mantener ? (int)((cerrarEn - ahora) / 1000) + 1 : 0;
  int bres = estado == BLOQ_INTENTOS ? (int)((bloqueoHasta - ahora) / 1000) + 1 : 0;
  enviar("ST," + String(estado) + "," + String(anguloMeta > 0) + "," + String(rest) + "," +
         String(digitalRead(P_VERDE)) + "," + String(digitalRead(P_ROJO)) + "," + String(digitalRead(P_BLANCA)) + "," +
         String(A.modoLuz) + "," + String(fallos) + "," + String(A.intentos) + "," + String(bres) + "," +
         String(A.n) + "," + String(A.sonido) + "," + String(A.tiempo) + "," + String(A.bloqueo) + "," +
         String(A.angulo) + "," + String(lcd != nullptr) + "," + String(rfidOk) + "," + String(registrando) + "," + String(mantener) + "," +
         String(distancia) + "," + String(presencia) + "," + String(A.dist) + "," + String(A.merodeo) + "," + String(ultraOk));
}

String limpiar(String s) { s.replace(",", " "); s.trim(); return s.substring(0, 10); }
bool soloNumeros(const String &s) { for (unsigned i = 0; i < s.length(); i++) if (!isDigit(s[i])) return false; return s.length() >= 4 && s.length() <= 6; }

void prueba() {
  evento("Prueba de luces, buzzer y servo");
  mensaje("PRUEBA", "luces y servo", 2500);
  for (byte p : {P_BLANCA, P_VERDE, P_ROJO}) { digitalWrite(p, HIGH); delay(300); digitalWrite(p, LOW); }
  onda(150);
  if (estado != ABIERTA) { anguloMeta = A.angulo; unsigned long t = millis(); while (millis() - t < 1200) moverServo(); anguloMeta = 0; }
}

void orden(String s) {
  s.trim();
  String may = s; may.toUpperCase();
  int p = may.indexOf(':');
  String cmd = p < 0 ? may : may.substring(0, p);
  String arg = p < 0 ? "" : s.substring(p + 1);
  int n = arg.toInt();
  if (cmd == "CODIGO" || cmd == "PIN") probarCodigo(arg);
  else if (cmd == "CAMBIARPIN") {
    int q = arg.indexOf(':');
    String viejo = arg.substring(0, q), nuevo = arg.substring(q + 1);
    if (q < 0 || viejo != String(A.pin)) { evento("No se cambio el codigo: el codigo actual no es correcto"); sonar(2); }
    else if (!soloNumeros(nuevo)) evento("El codigo nuevo debe tener de 4 a 6 numeros");
    else { strcpy(A.pin, nuevo.c_str()); guardar(); evento("Codigo cambiado"); sonar(1); }
  }
  else if (cmd == "ABRIR") {
    if (estado == BLOQ_INTENTOS || estado == BLOQ_MANUAL || estado == ALARMA) evento("Desbloqueo desde el simulador");
    abrir("Desde simulador", arg == "0");
  }
  else if (cmd == "CERRAR") { if (estado == ABIERTA) cerrar("desde simulador"); else evento("La puerta ya esta cerrada"); }
  else if (cmd == "BLOQUEAR") { estado = BLOQ_MANUAL; mantener = false; anguloMeta = 0; sonar(4); mensaje("PUERTA BLOQUEADA", "solo administr.", 2000); evento("Puerta BLOQUEADA a mano: solo abre un administrador"); }
  else if (cmd == "DESBLOQUEAR") {
    if (estado == BLOQ_MANUAL || estado == BLOQ_INTENTOS) { estado = CERRADA; fallos = 0; sonar(1); evento("Puerta desbloqueada"); }
    else evento("La puerta no estaba bloqueada");
  }
  else if (cmd == "PANICO") { estado = ALARMA; mantener = false; anguloMeta = 0; mensaje("!!! ALARMA !!!", "PANICO", 3000); evento("ALARMA DE PANICO activada"); }
  else if (cmd == "SILENCIAR") {
    if (estado == ALARMA || estado == BLOQ_INTENTOS) { estado = CERRADA; fallos = 0; evento("Alarma silenciada"); }
    else evento("No hay alarma que silenciar");
  }
  else if (cmd == "REGISTRAR") {
    int q = arg.indexOf(':');
    String nom = limpiar(q < 0 ? arg : arg.substring(0, q));
    regAdmin = q >= 0 && arg.substring(q + 1) == "1";
    strncpy(regNombre, nom.c_str(), 10); regNombre[10] = 0;
    registrando = true; regHasta = millis() + 15000;
    luzBlanca(15); sonar(4);
    mensaje("Acerque tarjeta", "nueva: " + nom, 15000);
    evento("Esperando tarjeta nueva para " + (nom.length() ? nom : String("(sin nombre)")) + (regAdmin ? " (administrador)" : "") + " - 15 s");
  }
  else if (cmd == "CANCELAR") { if (registrando) { registrando = false; msgHasta = 0; evento("Registro cancelado"); } }
  else if (cmd == "TARJ") {   // TARJ:UID:admin:activa:NOMBRE
    int a = arg.indexOf(':'), b = arg.indexOf(':', a + 1), c = arg.indexOf(':', b + 1);
    if (a < 0 || b < 0 || c < 0) { evento("Formato: TARJ:UID:admin:activa:NOMBRE"); return; }
    String uid = arg.substring(0, a); uid.toUpperCase();
    int i = buscarTarjeta(uid);
    if (i < 0) { evento("No existe la tarjeta " + uid); return; }
    A.t[i].admin = arg.substring(a + 1, b) == "1";
    A.t[i].activa = arg.substring(b + 1, c) == "1";
    String nom = limpiar(arg.substring(c + 1));
    if (nom.length()) { strncpy(A.t[i].nombre, nom.c_str(), 10); A.t[i].nombre[10] = 0; }
    guardar();
    evento("Tarjeta " + String(A.t[i].nombre) + ": " + (A.t[i].activa ? "activa" : "DESACTIVADA") + (A.t[i].admin ? ", administrador" : ""));
    enviarLista();
  }
  else if (cmd == "BORRAR") {
    String uid = arg; uid.toUpperCase();
    int i = buscarTarjeta(uid);
    if (i < 0) { evento("No existe la tarjeta " + uid); return; }
    String nom = A.t[i].nombre;
    for (byte k = i; k + 1 < A.n; k++) A.t[k] = A.t[k + 1];
    A.n--; guardar();
    evento("Tarjeta borrada: " + nom);
    enviarLista();
  }
  else if (cmd == "BORRARTODO") { A.n = 0; guardar(); evento("Se borraron todas las tarjetas. La proxima que registres sera administrador"); enviarLista(); }
  else if (cmd == "LISTA") enviarLista();
  else if (cmd == "TIEMPO") { A.tiempo = constrain(n, 2, 60); guardar(); evento("La puerta se queda abierta " + String(A.tiempo) + " s"); }
  else if (cmd == "BLOQUEO") { A.bloqueo = constrain(n, 10, 600); guardar(); evento("Bloqueo por intentos: " + String(A.bloqueo) + " s"); }
  else if (cmd == "INTENTOS") { A.intentos = constrain(n, 2, 9); guardar(); evento("Intentos antes del bloqueo: " + String(A.intentos)); }
  else if (cmd == "ANGULO") { A.angulo = constrain(n, 30, 180); guardar(); if (estado == ABIERTA) anguloMeta = A.angulo; evento("Angulo de apertura del servo: " + String(A.angulo) + " grados"); }
  else if (cmd == "LUZ") { A.modoLuz = constrain(n, 0, 2); guardar(); evento(String("Luz blanca: ") + (A.modoLuz == 0 ? "apagada" : A.modoLuz == 1 ? "prendida" : "automatica")); }
  else if (cmd == "SONIDO") { A.sonido = n ? 1 : 0; guardar(); evento(A.sonido ? "Sonido activado" : "Sonido silenciado (la alarma de panico si suena)"); }
  else if (cmd == "PITAR") { bool s = A.sonido; A.sonido = 1; onda(400); A.sonido = s; evento("Pitido de prueba"); }
  else if (cmd == "DIST") { A.dist = constrain(n, 5, 200); guardar(); evento("Detecta personas a menos de " + String(A.dist) + " cm"); }
  else if (cmd == "MERODEO") { A.merodeo = constrain(n, 0, 250); guardar(); evento(A.merodeo ? "Aviso si alguien se queda " + String(A.merodeo) + " s frente a la puerta" : "Aviso de merodeo apagado"); }
  else if (cmd == "TECLA") { sonar(6); luzBlanca(10); }          // alguien esta escribiendo el codigo
  else if (cmd == "PRUEBA") prueba();
  else if (cmd == "REINICIAR") { evento("Ajustes de fabrica: codigo 1234, sin tarjetas"); porDefecto(); estado = CERRADA; fallos = 0; enviarLista(); }
  else if (cmd == "ESTADO") {
    evento(String("Puerta de seguridad lista. LCD ") + (lcd ? "OK" : "NO ENCONTRADA") + ", RFID " + (rfidOk ? "OK" : "NO ENCONTRADO") + ", ultrasonico " + (ultraOk ? "OK" : "SIN LECTURA") + ", " + String(A.n) + " tarjetas");
    l1 = ""; pantalla(l1, l2);
  }
  else if (cmd.length()) { evento("Orden desconocida: " + s); return; }
  cambio = true;
}

void leer(Stream &p, String &lin) {
  while (p.available()) {
    char c = p.read();
    if (c == '\n' || c == '\r') { if (lin.length()) orden(lin); lin = ""; }
    else if (lin.length() < 60) lin += c;
  }
}

// ---------- sensor ultrasonico ----------
int medir() {
  digitalWrite(P_TRIG, LOW); delayMicroseconds(3);
  digitalWrite(P_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(P_TRIG, LOW);
  unsigned long d = pulseIn(P_ECHO, HIGH, 15000UL);   // hasta unos 2,5 m
  if (d == 0) return -1;                               // sin eco
  return d / 58;
}
void ultrasonico() {
  static unsigned long t = 0;
  static int m[3] = {999, 999, 999};
  static byte im = 0;
  static unsigned long cercaDesde = 0, lejosDesde = 0;
  unsigned long ahora = millis();
  if (ahora - t < 80) return;
  t = ahora;
  int d = medir();
  if (d < 0) { d = 999; if (!sinEco) sinEco = ahora; } else sinEco = 0;
  if (d < 2) d = 999;
  // si nunca hay eco en 30 s puede estar desconectado (o no hay nada delante)
  bool ok = !(sinEco && ahora - sinEco > 30000);
  if (ok != ultraOk) { ultraOk = ok; cambio = true; if (!ok) evento("El sensor ultrasonico no da lecturas desde hace 30 s (revisa TRIG D7 y ECHO D8)"); else evento("Sensor ultrasonico funcionando"); }
  m[im] = d; im = (im + 1) % 3;
  int a = m[0], b = m[1], c = m[2];
  distancia = max(min(a, b), min(max(a, b), c));        // mediana: un eco falso no cuenta
  bool cerca = distancia <= A.dist;
  if (cerca) { lejosDesde = 0; if (!cercaDesde) cercaDesde = ahora; }
  else { cercaDesde = 0; if (!lejosDesde) lejosDesde = ahora; }
  // 0,3 s cerca para detectar, 1,5 s lejos para que se fue
  if (!presencia && cercaDesde && ahora - cercaDesde > 300) {
    presencia = true; presenciaDesde = ahora; avisoMerodeo = false; cambio = true;
    luzBlanca(10);
    evento("Persona frente a la puerta (" + String(distancia) + " cm)");
    if (estado == CERRADA && !registrando && ahora > msgHasta) { mensaje("   BIENVENIDO", "Pase su tarjeta", 3000); sonar(6); }
  }
  if (presencia && lejosDesde && ahora - lejosDesde > 1500) {
    presencia = false; cambio = true;
    evento("La persona se fue (estuvo " + String((ahora - presenciaDesde) / 1000) + " s)");
  }
  if (presencia) {
    luzBlanca(5);
    if (A.merodeo && !avisoMerodeo && estado != ABIERTA && ahora - presenciaDesde > A.merodeo * 1000UL) {
      avisoMerodeo = true; cambio = true;
      sonar(5);
      mensaje("Persona sin", "identificarse", 3000);
      evento("AVISO: una persona lleva " + String(A.merodeo) + " s frente a la puerta sin entrar");
    }
  }
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(P_BUZ, OUTPUT); pinMode(P_BUZ2, OUTPUT); pinMode(P_ROJO, OUTPUT); pinMode(P_VERDE, OUTPUT); pinMode(P_BLANCA, OUTPUT);
  pinMode(P_TRIG, OUTPUT); pinMode(P_ECHO, INPUT); digitalWrite(P_TRIG, LOW);
  cargar();
  matriz.begin();
  dibujar(0);
  Wire.begin();
  buscarLcd();
  pantalla(" PUERTA SEGURA", "  iniciando...");
  SPI.begin();
  rfid.PCD_Init();
  rfid.PCD_SetAntennaGain(rfid.RxGain_max);
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  rfidOk = !(v == 0x00 || v == 0xFF);
  servo.attach(P_SERVO); servo.write(0); delay(400); servo.detach();
  digitalWrite(P_BLANCA, HIGH); digitalWrite(P_VERDE, HIGH); digitalWrite(P_ROJO, HIGH);
  onda(150); delay(600);
  digitalWrite(P_BLANCA, LOW); digitalWrite(P_VERDE, LOW); digitalWrite(P_ROJO, LOW);
  evento("Puerta de seguridad lista");
  evento(String("Sensor ultrasonico: ") + (medir() < 0 ? "sin eco (nada delante o desconectado)" : "listo"));
  evento(rfidOk ? "Lector RFID listo (" + String(A.n) + " tarjetas)" : "No se encuentra el lector RFID: revisa sus cables (va a 3.3V)");
  if (!lcd) evento("No se encuentra la pantalla LCD: revisa SDA, SCL y 5V");
  if (A.n == 0) evento("No hay tarjetas: la primera que registres sera administrador");
  enviarLista();
}

void loop() {
  unsigned long ahora = millis();
  leer(Serial, linUsb);
  leer(BT, linBt);

  if (rfid.PICC_IsNewCardPresent()) {
    if (rfid.PICC_ReadCardSerial()) {
      tarjetaLeida(rfid.uid.uidByte, rfid.uid.size);
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    } else {
      static unsigned long tMal = 0;
      if (ahora - tMal > 3000) { tMal = ahora; evento("Se detecto una tarjeta pero no se pudo leer: dejala quieta y pegada al lector"); }
    }
  }
  // el lector a veces se cuelga: se reinicia cada 10 s si no responde
  static unsigned long tRfid = 0;
  if (ahora - tRfid > 10000) {
    tRfid = ahora;
    byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    bool ok = !(v == 0x00 || v == 0xFF);
    if (!ok) { rfid.PCD_Init(); rfid.PCD_SetAntennaGain(rfid.RxGain_max); }
    else if (rfid.PCD_GetAntennaGain() != rfid.RxGain_max) { rfid.PCD_Init(); rfid.PCD_SetAntennaGain(rfid.RxGain_max); evento("Lector RFID reiniciado"); }
    if (ok != rfidOk) { rfidOk = ok; evento(ok ? "Lector RFID conectado" : "Se perdio el lector RFID"); cambio = true; }
  }

  // si la pantalla no estaba al encender, se busca otra vez cada 5 s
  static unsigned long tLcd = 0;
  if (!lcd && ahora - tLcd > 5000) {
    tLcd = ahora;
    buscarLcd();
    if (lcd) { evento("Pantalla LCD encontrada"); String a = l1, b = l2; l1 = ""; pantalla(a, b); cambio = true; }
  }
  if (registrando && ahora > regHasta) { registrando = false; msgHasta = 0; evento("Registro cancelado (no se acerco ninguna tarjeta)"); cambio = true; }
  ultrasonico();
  if (estado == ABIERTA && !mantener && (long)(ahora - cerrarEn) >= 0) {
    if (presencia && distancia <= 15) {               // alguien pasando justo por la puerta
      cerrarEn = ahora + 2000;
      static unsigned long tAviso = 0;
      if (ahora - tAviso > 5000) { tAviso = ahora; evento("No se cierra: hay alguien pasando por la puerta"); }
      mensaje("PUERTA OCUPADA", "esperando...", 1500);
    } else cerrar("tiempo cumplido");
  }
  if (estado == BLOQ_INTENTOS && (long)(ahora - bloqueoHasta) >= 0) { estado = CERRADA; fallos = 0; evento("Fin del bloqueo: se puede intentar de nuevo"); cambio = true; }
  if (estado != ABIERTA) anguloMeta = 0;
  moverServo();

  // ---- luces ----
  bool parpadeo = (ahora / 250) % 2;
  bool verde = estado == ABIERTA;
  bool rojo = estado == CERRADA || estado == BLOQ_MANUAL || ((estado == BLOQ_INTENTOS || estado == ALARMA) && parpadeo);
  bool blanca = A.modoLuz == 1 || (A.modoLuz == 2 && ahora < blancaHasta) || estado == ALARMA;
  if (registrando) verde = parpadeo, rojo = false;
  digitalWrite(P_VERDE, verde); digitalWrite(P_ROJO, rojo); digitalWrite(P_BLANCA, blanca);
  buzzer();

  // ---- pantalla y matriz ----
  if (ahora > msgHasta) {
    switch (estado) {
      case CERRADA: pantalla(" PUERTA SEGURA", "Tarjeta o codigo"); dibujar(registrando ? 3 : 0); break;
      case ABIERTA: pantalla("  PUERTA ABIERTA", mantener ? "hasta cerrar" : "Cierra en " + String((cerrarEn - ahora) / 1000 + 1) + " s"); dibujar(1); break;
      case BLOQ_INTENTOS: pantalla("!! BLOQUEADA !!", "Espera " + String((bloqueoHasta - ahora) / 1000 + 1) + " s"); dibujar(2); break;
      case BLOQ_MANUAL: pantalla("PUERTA BLOQUEADA", "Solo administr."); dibujar(0); break;
      case ALARMA: pantalla("!!! ALARMA !!!", parpadeo ? "  PANICO" : ""); dibujar(parpadeo ? 2 : 0); break;
    }
  }

  static unsigned long tSt = 0;
  if (cambio || ahora - tSt >= 1000) { tSt = ahora; cambio = false; enviarEstado(); }
}
