#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include "ph_grav.h"        //LIBRERIAS MODIFICADAS PARA ESP32
#include "do_grav.h"


#define TINY_GSM_MODEM_SIM7000 //SELECT MODEM
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialMon Serial
#define SerialAT Serial1  //SERIAL 2 ESP32
#define TINY_GSM_DEBUG SerialMon


#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
//const char apn[] = "movistar.pe";
//const char gprsUser[] = "movistar@datos";
//const char gprsPass[] = "movistar";

const char apn[] = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
const char wifiSSID[] = "YourSSID";
const char wifiPass[] = "YourWiFiPass";

// MQTT details
const char* broker = "ec2-3-16-154-178.us-east-2.compute.amazonaws.com";  //aqui habiamos creado broker mosquitto con aitenticacion
String username_mqtt = "sammy";
String password_mqtt = "HyMServer@21";
int port_mqtt = 1883;

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;


#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <TinyGsmClientSIM70xx.h>

// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient mqtt(client);


#define UART_BAUD   9600
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4  //SIM700g
#define RST_PIN     5  //RST SIM700G

#define LED_PIN     12

#define PIN_VCC_DO   2  //pin para alimentar sensor DO
#define PIN_VCC_PH   32  //pin para alimentar sensor PH
#define PIN_VCC_TEMP 2  //pin para alimentar sensor de Temperatura
#define PIN_SIGNAL_TEMP 14
#define PIN_SIGNAL_DO   15  //pin analogico
#define PIN_SIGNAL_PH   13  //pin Analogico

Gravity_pH pH = Gravity_pH(PIN_SIGNAL_PH);
Gravity_DO DO = Gravity_DO(PIN_SIGNAL_DO);
uint8_t user_bytes_received = 0;
const uint8_t bufferlen = 32;
char user_data[bufferlen];

OneWire ourWire(PIN_SIGNAL_TEMP);                //Se establece el pin 25  como bus OneWire
DallasTemperature sensors(&ourWire); //Se declara una variable u objeto para nuestro sensor


uint32_t lastReconnectAttempt = 0;

//VARIABLES DE SISTEMA
String modemInfo;
String modemIMEI;
String modemOP;
String StrACK;
String TopicOut;
String TopicIn;
int cont_desc;
float Temperatura;
float PH_DATA;
float DO_DATA;
////////////////////
#define uS_TO_S_FACTOR 1000ULL
int Time = 3600;//3600 Segundos 1 hora

void publicar_JSON(String ACK = "") {
   StrACK = ACK;
   encender_sensores();
   delay(1000); //estabilizar alimentacion
   setCpuFrequencyMhz(80);  //SET FRECUENCIA 80MHZ, problemas con lectura ds18b20 no mover
   //leer analogicos...
   Serial.print("pH: ");
   PH_DATA = pH.read_ph();
   Serial.print(PH_DATA);
   Serial.print(", DO: ");
   DO_DATA = DO.read_do_percentage();
   Serial.println(DO_DATA );
   leer_sensor_temperatura();
   apagar_sensores();
   setCpuFrequencyMhz(10);  //SET FRECUENCIA 10MHZ

   StaticJsonDocument<200> doc;
   doc["ID"] = modemIMEI;
   doc["DO(%)"] = DO_DATA;
   doc["PH"] = PH_DATA;
   doc["TEMP(C)"] = Temperatura;
   int Batt=analogRead(35);
   Batt=Batt*1.7395;
   doc["BAT(mV)"] = Batt;
   doc["ACK"] = StrACK;

   String output ;
   serializeJson(doc, output);
   Serial.println(output);

  mqtt.publish(TopicOut.c_str(), output.c_str());
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String str_topic = String(topic); //de char arra a string contienen nombre de topico
  String str_payload; //contiene payload e string

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    str_payload += (char)payload[i];
  }
  Serial.println();

  StaticJsonDocument<96> doc;
  DeserializationError error = deserializeJson(doc, str_payload);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  String str_command = doc["COMMAND"];
  Serial.println(str_command);

  if (str_command == "AT")
  {
    //do..
    publicar_JSON("OK");
    return;
  }else if (str_command == "AT+RST")
  {
    //do..
    publicar_JSON("OK:RST");
    ESP.restart();
    return;
  }else if (str_command == "AT+CAL:DO")
  {
    //do..
    encender_sensores();
    delay(1000);
    DO.cal();
    SerialMon.println("DO CALIBRATED");
    publicar_JSON("OK:DO");
    return;
  } else if (str_command == "AT+CAL:DO:ERASE")
  {
    //do..
    encender_sensores();
    DO.cal_clear();
    delay(1000);
    SerialMon.println("CALIBRATION CLEARED");
    publicar_JSON("OK:DO:ERASE");
    return;
  } else if (str_command == "AT+CAL:PH:10")
  {
    //do..
    encender_sensores();
    delay(1000);
    pH.cal_high(); 
    SerialMon.println("HIGH CALIBRATED");
    publicar_JSON("OK:PH:10");
    return;
  } else if (str_command == "AT+CAL:PH:7")
  {
    //do..
    encender_sensores();
    delay(1000);
    pH.cal_mid();
    SerialMon.println("MID CALIBRATED");
    publicar_JSON("OK:PH:7");
    return;
  } else if (str_command == "AT+CAL:PH:4")
  {
    //do..
    encender_sensores();
    delay(1000);
    pH.cal_low(); 
    publicar_JSON("OK:PH:4");
    SerialMon.println("LOW CALIBRATED");
    return;
  } else if (str_command == "AT+CAL:PH:ERASE")
  {
    //do..
    encender_sensores();
    delay(1000);
    pH.cal_clear();
    SerialMon.println("PH CALIBRATION CLEARED");
    publicar_JSON("OK:PH:ERASE");
    return;
  } else if (str_command == "AT+TIME:1M")
  {
    //do..
    Time = 60;
    publicar_JSON("OK:TIME:1M");
    return;
  }
  else if (str_command == "AT+TIME:10M")
  {
    //do..
    Time = 600;
    publicar_JSON("OK:TIME:10M");
    return;
  } else if (str_command == "AT+TIME:30M")
  {
    //do..
    Time = 1800;
    publicar_JSON("OK:TIME:30M");
    return;
  } else if (str_command == "AT+TIME:1H")
  {
    //do..
    Time = 3600;
    publicar_JSON("OK:TIME:1H");
    return;
  } else if (str_command == "AT+LED:1")
  {
    //do..
    digitalWrite(LED_PIN, HIGH);
    publicar_JSON("OK:LED:1");
    return;
  } else if (str_command == "AT+LED:0")
  {
    //do..
    digitalWrite(LED_PIN, LOW);
    publicar_JSON("OK:LED:0");
    return;
  }
  else {
    //do..
    publicar_JSON("ERROR");
    return;
  }

}


void reconnect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = modemIMEI;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt.connect(clientId.c_str(), username_mqtt.c_str() , password_mqtt.c_str() )) {
      //if (mqtt.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // ... and resubscribe

      mqtt.subscribe(TopicIn.c_str());
      cont_desc = 0;
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying

      // MQTT Broker BACKUP
      cont_desc ++;
      if (cont_desc == 3) {
        Serial.println(">>Reconectando a broker.mqtt-dashboard");
        mqtt.setServer("broker.mqtt-dashboard.com", port_mqtt);
        username_mqtt = "";
        password_mqtt = "";
      }
      if (cont_desc == 10) { //No conexion, Reiniciar
        Serial.println("No RED");
        Serial.println(">>Reiniciar");
        ESP.restart();
      }
      delay(5000);
    }
  }
}

void encender_sensores() {
  digitalWrite(PIN_VCC_DO, HIGH);
  digitalWrite(PIN_VCC_PH, HIGH);
  digitalWrite(PIN_VCC_TEMP, HIGH);
  delay(100);
}
void apagar_sensores() {
  digitalWrite(PIN_VCC_DO, LOW);
  digitalWrite(PIN_VCC_PH, LOW);
  digitalWrite(PIN_VCC_TEMP, LOW);
  delay(100);
}

void leer_sensor_temperatura() {
  //sensors.begin();   //Se inicia el sensor
  sensors.requestTemperatures();
  Temperatura = sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
  SerialMon.print("Temperatura= ");
  SerialMon.print(Temperatura);
  SerialMon.println(" C");

}

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  Serial1.begin(9600, SERIAL_8N1, 18, 5 ); // A GPS NEO 6M
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_VCC_DO, OUTPUT);  //pin para alimentar sensor DO
  pinMode(PIN_VCC_PH, OUTPUT);  //pin para alimentar sensor PH
  pinMode(PIN_VCC_TEMP, OUTPUT); //pin para alimentar sensor de Temperatura
  pinMode(RST_PIN, OUTPUT);//PIN RESET SIM700G
  
  WiFi.mode( WIFI_MODE_NULL );
  btStop();
  WiFi.disconnect(true);  // Disconnect from the network
  WiFi.mode(WIFI_OFF);    // Switch WiFi off
  setCpuFrequencyMhz(80);  //SET FRECUENCIA
  SerialMon.print("CPU Freq: ");
  SerialMon.println(getCpuFrequencyMhz());

  SerialMon.println("");
  SerialMon.println("INICIANDO...");
  encender_sensores();
  if(DO.begin()){
    SerialMon.println("Loaded DO EEPROM");
  }
  if (pH.begin()) {                                     
    SerialMon.println("Loaded pH EEPROM");
  }
  SerialMon.println();
  sensors.begin();   //Se inicia el sensor
  delay(1000);
   Serial.print("pH: ");
   PH_DATA = pH.read_ph();
   Serial.print(PH_DATA);
   Serial.print(", DO: ");
   DO_DATA = DO.read_do_percentage();
   Serial.println(DO_DATA );
  leer_sensor_temperatura();
  apagar_sensores();

  setCpuFrequencyMhz(10);  //SET FRECUENCIA

  pinMode(PWR_PIN, OUTPUT);
  for (int x = 0; x <= 10; x++)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(RST_PIN, LOW);
  
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(PWR_PIN, HIGH);
  delay(4000);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  SerialMon.println(">>Modem Encendido");
  SerialMon.println(">>Inicializando...");

  //TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  // SerialAT.begin(9600);
  modem.restart();
  for (int x = 0; x <= 10; x++)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
  // 2 Automatic
  // 13 GSM only
  // 38 LTE only
  // 51 GSM and LTE only
  Serial.println( modem.setNetworkMode(51));
  SerialMon.println(" ");
  Serial.print("Network Mode: ");
  Serial.println(modem.getNetworkModes());
  Serial.println(modem.getNetworkMode());

  // 1 CATM
  // 2 NB-IOT
  // 3 CAT-M & NBIOT
  Serial.println( modem.setPreferredMode(1));
  Serial.print("Prefered  Mode: ");
  Serial.println(modem.getPreferredModes());
  Serial.println(modem.getPreferredMode());
  SerialMon.println(" ");

  //modem.init();
  SerialMon.println(">>Modem Inicializado");
  modemInfo = modem.getModemInfo();
  SerialMon.print(">>Modem Info: ");
  SerialMon.println(modemInfo);
  modemIMEI = modem.getIMEI();
  SerialMon.print("Modem IMEI: ");
  SerialMon.println(modemIMEI);
  modemOP = modem.getOperator();
  SerialMon.print("Modem OPERATOR: ");
  SerialMon.println(modemOP);
  TopicOut = modemIMEI + "/out";
  TopicIn = modemIMEI + "/in";
  //  String modemBAT = modem.getBattVoltageImpl();
  //  SerialMon.print("Modem Voltage: ");
  //  SerialMon.println(modemBAT);


#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }
#endif

  SerialMon.print(">>Esperando Conexion a Red...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    digitalWrite(LED_PIN, LOW);
    delay(10000);
    ESP.restart();
    return;
  }
  SerialMon.println(">>CONECTADO! ");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F(">>Conectando A:  "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(">>No Conexion..");
    digitalWrite(LED_PIN, LOW);
    delay(10000);
    ESP.restart();
    return;
  }
  SerialMon.println(" Conectado!");

  if (modem.isGprsConnected()) {
    SerialMon.println(">>CONECTADo A RED MOVIL");
  }
#endif

  // MQTT Broker setup
  mqtt.setServer(broker, port_mqtt);
  mqtt.setCallback(callback);
  for (int x = 0; x <= 5; x++)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(300);
  }
  digitalWrite(LED_PIN, HIGH);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  mqtt.loop();

  unsigned long now = millis();
  if (now - lastMsg > Time * uS_TO_S_FACTOR) {
    lastMsg = now;

    Serial.println("PUBLICAR: ");
    digitalWrite(LED_PIN, LOW);

    publicar_JSON();
    digitalWrite(LED_PIN, HIGH);
  }
  esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR); // ESP32 wakes up every 5 seconds
  //Serial.println("Going to light-sleep now");
  Serial.flush();
  esp_light_sleep_start();
  //Serial.println("on");

}
