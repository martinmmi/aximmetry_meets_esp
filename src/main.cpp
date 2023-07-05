//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define LED_PIN_INTERNAL    25        //Accumulator Function Declaration

#define SEALEVELPRESSURE_HPA (1013.25)

unsigned long delayTime = 0;

float temperature;
float pressure;
float altitude;
float humidity;

int boottime;

char buf_boottime[20];
char buf_temperature[20];
char buf_pressure[20];
char buf_humidity[20];

//The udp library class
WiFiUDP udp;

//set up to connect to an existing network
const char* ssid = "AirDoLan";
const char* password = "mmg1199#";

//const char* ssid = "Karli68";
//const char* password = "xxxxx";

//IP address to send UDP data to:
const char *udpAddress = "192.168.1.75";
const int udpPort = 3333;

Adafruit_BME280 bme; // I2C

//////////////////////////////////////////////////////////////////////

void initWiFi() {
  Serial.println("Setup WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println("Setup done");
}

//////////////////////////////////////////////////////////////////////

void setup()
{

  Serial.begin(115200);                   // initialize serial
  while (!Serial);

  Serial.println("UDP Sender starts");
  
  Serial.println("BME280 Selftest!");

  bool bme_status = bme.begin(0x76); 

  if (!bme_status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  pinMode(LED_PIN_INTERNAL, OUTPUT);

  initWiFi();

}

//////////////////////////////////////////////////////////////////////

void loop()
{ 

  if ((millis() - delayTime > 5000)) {
    
    temperature = bme.readTemperature() - 1;
    int temperature_int = (int) temperature;
    float temperature_float = (abs(temperature) - abs(temperature_int)) * 100;
    int temperature_fra = (int)temperature_float;
    sprintf (buf_temperature, "%d.%d", temperature_int, temperature_fra);


    pressure = bme.readPressure() / 100.0F;
    int pressure_int = (int) pressure;
    float pressure_float = (abs(pressure) - abs(pressure_int)) * 100;
    int pressure_fra = (int)pressure_float;
    sprintf (buf_pressure, "%d.%d", pressure_int, pressure_fra);


    humidity = bme.readHumidity();
    int humidity_int = (int) humidity;
    float humidity_float = (abs(humidity) - abs(humidity_int)) * 100;
    int humidity_fra = (int)humidity_float;
    sprintf (buf_humidity, "%d.%d", humidity_int, humidity_fra);


    boottime = millis() / 1000;
    sprintf (buf_boottime, "%d", boottime);


    if (WiFi.status() == WL_CONNECTED){
      //Send a packet

      Serial.println("Sends UDP Packet:");

      udp.beginPacket(udpAddress, udpPort);

      Serial.print("Seconds since boot: "); Serial.print(boottime); Serial.print(" Temperature: "); Serial.print(temperature); Serial.print(" C "); Serial.print(" Pressure: "); Serial.print(pressure); Serial.print(" hPa "); Serial.print(" Humidity: "); Serial.print(humidity); Serial.println(" %");
      
      udp.printf("Seconds since boot: ");
      udp.printf(buf_boottime);
      udp.printf(" Temperature: ");
      udp.printf(buf_temperature);
      udp.printf(" C");
      udp.printf(" Pressure: ");
      udp.printf(buf_pressure);
      udp.printf(" hPa");
      udp.printf(" Humidity: ");
      udp.printf(buf_humidity);
      udp.printf(" %");
      udp.endPacket();
      
      digitalWrite(LED_PIN_INTERNAL, HIGH);
      delay(50);
      digitalWrite(LED_PIN_INTERNAL, LOW);
    }

    if (WiFi.status() != WL_CONNECTED){
      Serial.println("Something goes wrong with the wifi :/");
    }

    delayTime = millis();
  }

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////