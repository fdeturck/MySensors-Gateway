
/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * The ESP8266 MQTT gateway sends radio network (or locally attached sensors) data to your MQTT broker.
 * The node also listens to MY_MQTT_TOPIC_PREFIX and sends out those messages to the radio network
 *
 * LED purposes:
 * - To use the feature, uncomment WITH_LEDS_BLINKING in MyConfig.h
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/esp8266_gateway for wiring instructions.
 * nRF24L01+  ESP8266
 * VCC        VCC
 * CE         GPIO4
 * CSN/CS     GPIO15
 * SCK        GPIO14
 * MISO       GPIO12
 * MOSI       GPIO13
 *
 * Not all ESP8266 modules have all pins available on their external interface.
 * This code has been tested on an ESP-12 module.
 * The ESP8266 requires a certain pin configuration to download code, and another one to run code:
 * - Connect REST (reset) via 10K pullup resistor to VCC, and via switch to GND ('reset switch')
 * - Connect GPIO15 via 10K pulldown resistor to GND
 * - Connect CH_PD via 10K resistor to VCC
 * - Connect GPIO2 via 10K resistor to VCC
 * - Connect GPIO0 via 10K resistor to VCC, and via switch to GND ('bootload switch')
 *
  * Inclusion mode button:
 * - Connect GPIO5 via switch to GND ('inclusion switch')
 *
 * Hardware SHA204 signing is currently not supported!
 *
 * Make sure to fill in your ssid and WiFi password below for ssid & pass.
 */

#include <EEPROM.h>
#include <SPI.h>
#include <SimpleDHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Enable debug prints to serial monitor
#define MY_DEBUG

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 9600

// Enables and select radio type (if attached)
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_NODE_ID 2

#define MY_GATEWAY_MQTT_CLIENT
#define MY_GATEWAY_ESP8266

// Set this nodes subscripe and publish topic prefix
#define MY_MQTT_PUBLISH_TOPIC_PREFIX "mygateway2-out"
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "mygateway2-in"

// Set MQTT client id
#define MY_MQTT_CLIENT_ID "gateway-nonrf2"

// Enable these if your MQTT broker requires usenrame/password
//#define MY_MQTT_USER "username"
//#define MY_MQTT_PASSWORD "password"

// Set WIFI SSID and password
//#define MY_ESP8266_SSID "TP-LINK_POCKET_3020_473BAF"
#define  MY_ESP8266_SSID "Maison"
#define MY_ESP8266_PASSWORD "internet"

// Set the hostname for the WiFi Client. This is the hostname
// it will pass to the DHCP server if not static.
// #define MY_ESP8266_HOSTNAME "mqtt-sensor-gateway"

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
#define MY_IP_ADDRESS 192,168,1,86

// If using static ip you need to define Gateway and Subnet address as well
#define MY_IP_GATEWAY_ADDRESS 192,168,1,1
#define MY_IP_SUBNET_ADDRESS 255,255,255,0


// MQTT broker ip address.
#define MY_CONTROLLER_IP_ADDRESS 192, 168, 1, 156

// The MQTT broker port to to open
#define MY_PORT 1883

 /*
// Flash leds on rx/tx/err
#define MY_LEDS_BLINKING_FEATURE
// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
#define MY_INCLUSION_MODE_BUTTON_PIN  3

#define MY_DEFAULT_ERR_LED_PIN 16  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  16  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  16  // the PCB, on board LED
*/

#include <ESP8266WiFi.h>
#include <MySensors.h>

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_CONTACTEUR 2
#define CONTACTEUR_PIN D3          // Broche sur laquelle est attaché le contacteur
#define DHT11_PIN 2 //D4

#define SENSOR_TEMP_OFFSET -1
// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint16_t FORCE_UPDATE_N_READS = 5000;


MyMessage msgContacteur(CHILD_ID_CONTACTEUR, V_TRIPPED);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
SimpleDHT11 dht11;
boolean etatPrecedent = false;
byte humidity = 0;
byte temperature = 0;
byte lastTemp;
byte lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;
uint8_t etat;

// SCL GPIO5
// SDA GPIO4
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);

/*#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };
*/

static const unsigned char PROGMEM logo_bmp[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
0x80, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0x80, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xF8, 0xF8, 0xFC, 0x7E, 0x7E, 0x3F, 0x1F, 0x1F,
0x0F, 0x0F, 0x07, 0x07, 0x07, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07, 0x07, 0x07, 0x0F, 0x0F, 0x1F,
0x1F, 0x3F, 0x7E, 0x7E, 0xFC, 0xF8, 0xF8, 0xF0, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xF8, 0xFE, 0xFF, 0xFF, 0xFF, 0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xFE, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0xFE, 0x02, 0x02, 0x02, 0x02, 0x04,
0xF8, 0x00, 0x00, 0xFE, 0x20, 0x30, 0xC8, 0x84, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0x07, 0x1F, 0x3F, 0xFF, 0xFF, 0xFC, 0xF8, 0xE0, 0xE0, 0xC0, 0x80, 0x80,
0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x02, 0x02, 0x02, 0x01,
0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
0x80, 0xC0, 0xE0, 0xE0, 0xF0, 0xFC, 0xFF, 0xFF, 0x7F, 0x1F, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x07, 0x07, 0x0F, 0xFF, 0xFF,
0xFF, 0xFF, 0xFE, 0xCC, 0xE0, 0xE0, 0xF0, 0xF0, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8,
0xF8, 0xF8, 0xF8, 0xF8, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7E, 0x3E, 0x3F, 0x1F, 0x1F, 0x1F,
0x0F, 0x0F, 0x07, 0x07, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07,
0x0F, 0x0F, 0x0F, 0x07, 0x07, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const int CLOCK_SPEED = 50;
unsigned long lastDraw = 0;
int s=0;
int sec=0;
int hrs=0;
int minutes=0;
int initialHours = 00;//variable to initiate hours
int initialMins = 0;//variable to initiate minutes
int initialSecs = 00;//variable to initiate seconds
bool timeReceived = false;
unsigned long lastUpdate=0, lastRequest=0, lastTempTime=0, lastHumTime=0;

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)

/////////////////////// SETUP //////////////////

void presentation() {
  sendSketchInfo("GatewayWemos+DHT+MQTT-nonNRF", "1.0");
  Serial.println("sendSketchInfo(GatewayWemos+DHT+MQTT-nonNRF, 1.0)");
  present(CHILD_ID_CONTACTEUR, S_DOOR);
  Serial.println("CHILD_ID_CONTACTEUR, S_DOOR");
  present(CHILD_ID_HUM, S_HUM);
  Serial.println("CHILD_ID_HUM, S_HUM");
  present(CHILD_ID_TEMP, S_TEMP);
  Serial.println("CHILD_ID_TEMP, S_TEMP");
  metric = getControllerConfig().isMetric;
}

void receiveTime(unsigned long controllerTime) {
  // Ok, set incoming time
  Serial.print("Time value received: ");
  Serial.println(controllerTime);
  //RTC.set(controllerTime); // this sets the RTC to the time from controller - which we do want periodically
  initialHours = numberOfHours(controllerTime);
  initialMins = numberOfMinutes(controllerTime);
  initialSecs = numberOfSeconds(controllerTime);
  timeReceived = true;

}

void setup() {
  pinMode(CONTACTEUR_PIN, INPUT);
  // Activate internal pull-ups
  digitalWrite(CONTACTEUR_PIN, HIGH);

  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  wait(2000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  // init done

  //display.drawBitmap(0, 0,  logo_bmp, 64, 48, 1);
  display.display();
  wait(2000);

  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

}
/////////////////////// SETUP //////////////////

void loop() {

  unsigned long now = millis();

  if ((!timeReceived && (now-lastRequest) > (10UL*1000UL*10UL))
    || (timeReceived && (now-lastRequest) > (60UL*1000UL*60UL))) {
    // Request time from controller.
    Serial.println("requesting time");
    requestTime();
    lastRequest = now;
  }


  // Lit l'état du contacteur
  etat = digitalRead( CONTACTEUR_PIN );
  if (etat != etatPrecedent) {
    etatPrecedent = etat;

    Serial.print("Etat du contacteur ");
    Serial.println(etat ? "Ouvert " : "Ferme" );
    send( msgContacteur.set( etat ? "1" : "0" ) );
  }

   ////////  TEMP HUM ////////
  // Force reading sensor, so it works also after sleep()
  //dht.readSensor(true);

  ////////  TEMP ////////
  // Get temperature from DHT library

  if (lastTempTime + UPDATE_INTERVAL < now){
    lastTempTime = now;

    if (dht11.read(DHT11_PIN, &temperature, &humidity, NULL)) {
      Serial.print("Read DHT11 failed.1");
    }
    //vu qu'on relit l'humidité après alors on fait une pause ici
    //pour éviter de faire une deuxième lecture trop rapidement
    wait(1000UL);

    #ifdef MY_DEBUG
    Serial.print("T brut: ");
    Serial.println(temperature);
    #endif
    if(isnan(temperature)){
      Serial.println("Failed reading temperature from DHT");
    }else{
      temperature = temperature + SENSOR_TEMP_OFFSET;
    }

    #ifdef MY_DEBUG
    Serial.print("T corr.: ");
    Serial.println(temperature);
    #endif

    //sending values
    if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
      // Only send value if it changed since the last measurement or if we didn't send an update for n times
      lastTemp = temperature;
      // Reset no updates counter
      nNoUpdatesTemp = 0;
      send(msgTemp.set(temperature, 1));
    } else {
      // Increase no update counter if the humidity stayed the same
      nNoUpdatesTemp++;
    }
  }

  ////////  HUMIDITY ////////
  // Get humidity from DHT library
  if (lastHumTime + UPDATE_INTERVAL < now){
    lastHumTime = now;

    if (dht11.read(DHT11_PIN, &temperature, &humidity, NULL)) {
      Serial.print("Read DHT11 failed.2");
    }

    if(isnan(humidity)){
      Serial.println("Failed reading humidity from DHT");
    }else{
      humidity = humidity+10;
    }
    #ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(humidity);
    #endif

    //send value
    if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
      // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
      lastHum = humidity;
      // Reset no updates counter
      nNoUpdatesHum = 0;
      send(msgHum.set(humidity, 1));
    } else {
      // Increase no update counter if the humidity stayed the same
      nNoUpdatesHum++;
    }
  }


  if (lastDraw + CLOCK_SPEED < millis())
  {
    lastDraw = millis();
    // Add a second, update minutes/hours if necessary:
    display.clearDisplay();
    // Draw the screen:
    digitalDisplay();
    display.display(); // Draw the memory buffer
  }

  wait(50);
}

int seconds()
{
    s = initialHours*3600;
    s = s+(initialMins*60);
    s = s+initialSecs;
    s = s+(millis()/1000);
    return s;
}
//this method is for hours
int hours()
{
    hrs = seconds();
    hrs = hrs/3600;
    hrs = hrs%24;
    return hrs;
}
//this method is for minutes
int mins()
{
    minutes = seconds();
    minutes = minutes/60;
    minutes = minutes%60;
    return minutes;
}

int secs()
{
    sec = seconds();
    sec = sec%60;
    return sec;
}

void printDigits(byte digits){
    if(digits < 10)
        display.print('0');
    display.print(digits);
}
char sep()
{
    s = millis()/1000;
    if(s%2==0)
    {
        display.print(":");
    }
    else {
        display.print(" ");
    }
}
void digitalDisplay(){
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(1,0);
    display.println("ip: .86");
    display.print("P3: ");
    display.println(etat ? "Ouvert" : "Ferme");
    display.print("T: ");
    display.println(temperature);
    display.print("H: ");
    display.println(humidity);
    display.drawLine(0, display.height()-10, display.width()-1, display.height()-10, WHITE);
    display.setCursor(1,40);
    printDigits(
    hours());
    sep();
    printDigits(mins());
    sep();
    printDigits(secs());
    display.drawLine(0, display.height()-1, display.width()-1, display.height()-1, WHITE);
}
