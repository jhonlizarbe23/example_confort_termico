#if !( defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) \
      || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) \
      || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) \
      || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(__SAMD51__) || defined(__SAMD51J20A__) \
      || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__)  \
      || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \
      || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \
      || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__) )
  #error This code is designed to run on SAMD21/SAMD51 platform! Please check your Tools->Board setting.
#endif

#define USING_TIMER_TC3         true      // Only TC3 can be used for SAMD51
#define USING_TIMER_TC4         false     // Not to use with Servo library
#define USING_TIMER_TC5         false
#define USING_TIMER_TCC         false
#define USING_TIMER_TCC1        false
#define USING_TIMER_TCC2        false     // Don't use this, can crash on some boards

#include "config.h"

/* Librerias para WiFi y envío de datos a ThingSpeak */
#include <WiFiNINA.h>
#include <ThingSpeak.h>

/* Librerias sensor DHT11 */
#include <DHT.h>

/* Librerias sensor BMP180 */
#include <SFE_BMP180.h>
#include <Wire.h>

/* Librerias sensor SCD41 CO2 */
#include <SparkFun_SCD4x_Arduino_Library.h>

/* Librerias SAMD Timer */
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"

// Configura la red WiFi
const char* ssid = MY_WIFI_SSID;
const char* password = MY_WIFI_PASSWORD;

// ThingSpeak
WiFiClient client;
unsigned long myChannelNumber = MY_ID_CHANNEL; // ID del canal de ThingSpeak
const char* myWriteAPIKey = MY_API_KEY; // API Key de escritura de tu canal
const bool TS_ENABLED = false;

/* Configuración del DHT11 */
#define DHTPIN 2        // Pin digital DHT11
#define DHTTYPE DHT11   // Tipo de sensor DHT
DHT dht(DHTPIN, DHTTYPE); // Inicializamos el sensor DHT

/* Configuración para lectura DHT11 */
unsigned long previousMillis = 0;
const long interval = 3000; 

/* Configuración BMP180 */
SFE_BMP180 bmp180;
double PresionNivelMar=1013.25; //presion sobre el nibel del mar en mbar

/* Configuración SCD41 CO2 */
SCD4x scd41;

#define HW_TIMER_INTERVAL_MS      10

///////////////////////////////////////////////

#if (TIMER_INTERRUPT_USING_SAMD21)

  #if USING_TIMER_TC3
    #define SELECTED_TIMER      TIMER_TC3
  #elif USING_TIMER_TC4
    #define SELECTED_TIMER      TIMER_TC4
  #elif USING_TIMER_TC5
    #define SELECTED_TIMER      TIMER_TC5
  #elif USING_TIMER_TCC
    #define SELECTED_TIMER      TIMER_TCC
  #elif USING_TIMER_TCC1
    #define SELECTED_TIMER      TIMER_TCC1
  #elif USING_TIMER_TCC2
    #define SELECTED_TIMER      TIMER_TCC
  #else
    #error You have to select 1 Timer  
  #endif

#else

  #if !(USING_TIMER_TC3)
    #error You must select TC3 for SAMD51
  #endif
  
  #define SELECTED_TIMER      TIMER_TC3

#endif  

// Init selected SAMD timer
SAMDTimer ITimer(SELECTED_TIMER);

////////////////////////////////////////////////

// Init SAMD_ISR_Timer
SAMD_ISR_Timer ISR_Timer;

/* Configuración ejecución tarea */
#define TIMER_INTERVAL_3S             3000L // 3 SEG
#define TIMER_INTERVAL_5S             5000L // 5 SEG
#define TIMER_INTERVAL_7S             7000L // 7 SEG

const float PRESSURE_THRESHOLD = 1000.0; // Umbral de presión en mb para BMP180
const float TEMP_THRESHOLD = 30.0;       // Umbral de temperatura en grados Celsius para DHT11
const float HUMIDITY_THRESHOLD = 80.0;   // Umbral de humedad en % para DHT11
const uint16_t CO2_THRESHOLD = 900;      // Umbral de CO2 en ppm para SCD41

/* Fields definidos en ThingSpeak */
const int TYPE_FIELD_TEMPERATURA = 1;
const int TYPE_FIELD_HUMEDAD = 2;
const int TYPE_FIELD_PRESION = 3;
const int TYPE_FIELD_CO2 = 4;

void TimerHandler(void)
{
  ISR_Timer.run();
}

/* TAREAS */
void lecturaDHT11() {

  // Lectura de la humedad y la temperatura
  float humedad = dht.readHumidity();
  float temperatura = dht.readTemperature();

  // Verificamos si hay error en la lectura
  if (isnan(humedad) || isnan(temperatura)) {
    Serial.println("Error al leer el sensor DHT11");
    return;
  }
    
  sendDataToThingspeak(temperatura, TYPE_FIELD_TEMPERATURA);
  sendDataToThingspeak(humedad, TYPE_FIELD_HUMEDAD);

  if(humedad > HUMIDITY_THRESHOLD){
    Serial.print(F("ALERT.Humedad.")); Serial.println(round(humedad));
  }else if(temperatura > TEMP_THRESHOLD){
    Serial.print(F("ALERT.Temperatura.")); Serial.println(round(temperatura));
  }else{
     Serial.print(F("OK.Temperatura.")); Serial.println(round(temperatura));
      Serial.print(F("OK.Humedad.")); Serial.println(round(humedad));
  }

}

void lecturaBMP180(){

  char status;
  double T,P;
  
  status = bmp180.startTemperature();//Inicio de lectura de temperatura
  if (status != 0){   
    delay(status); //Pausa para que finalice la lectura
    status = bmp180.getTemperature(T); //Obtener la temperatura
    if (status != 0){
      status = bmp180.startPressure(3);//Inicio lectura de presión
      if (status != 0){        
        delay(status);//Pausa para que finalice la lectura        
        status = bmp180.getPressure(P,T);//Obtenemos la presión
        if (status != 0){
          sendDataToThingspeak((float) P, TYPE_FIELD_PRESION);
          if(P > PRESSURE_THRESHOLD){
            Serial.print(F("ALERT.Presion.")); Serial.println(round(P));
          }else{
            Serial.print(F("OK.Presion.")); Serial.println(round(P));
          }              
        }      
      }      
    }   
  } 
}

void lecturaSCD41(){
   if (scd41.readMeasurement()) {  // Revisa si hay nuevos datos
      sendDataToThingspeak((float) scd41.getCO2(), TYPE_FIELD_CO2);
    if(scd41.getCO2() > CO2_THRESHOLD){
       Serial.print(F("ALERT.CO2.")); Serial.println(round(scd41.getCO2()));
    }else{
       Serial.print(F("OK.CO2.")); Serial.println(round(scd41.getCO2()));
    }
  } else {
     Serial.print(F("ALERT.CO2.")); Serial.println(round(0));
  }
}

void sendDataToThingspeak(float valor, int typeField) {

  if(TS_ENABLED){
    ThingSpeak.setField(typeField, valor);

    // Enviar los datos
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    delay(15000);
  }
    
}

/* SETUP */
void setup(){
  Serial.begin(9600);
  delay(100);

  if(TS_ENABLED){
    // Verificación conexión WiFi
    if (WiFi.status() != WL_CONNECTED) {
          Serial.println("WiFi desconectado. Reconectando...");
          while (WiFi.status() != WL_CONNECTED) {
              WiFi.begin(ssid, password);
              delay(5000);
          }
          Serial.println("Reconectado al WiFi");
    }

    ThingSpeak.begin(client);
  }
  
  dht.begin(); // Inicialización DHT11
  bmp180.begin(); // Inicialización BMP180

  // Inicialización SCD41
  Wire.begin();
  scd41.begin();

  // Interval in millisecs
  if (ITimer.attachInterruptInterval_MS(HW_TIMER_INTERVAL_MS, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  // Just to demonstrate, don't use too many ISR Timers if not absolutely necessary
  // You can use up to 16 timer for each ISR_Timer
  ISR_Timer.setInterval(TIMER_INTERVAL_5S,  lecturaBMP180);
  ISR_Timer.setInterval(TIMER_INTERVAL_7S,  lecturaSCD41);
}

/* LOOP */
void loop(){
  /* Lectura DHT11 : Nothing to do all is done by hardware. Even no interrupt required. */
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    lecturaDHT11();
  }
}
