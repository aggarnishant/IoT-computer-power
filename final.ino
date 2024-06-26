#define LEDPinIndi 16
// -----Servo configuration -----

#include <Servo.h>
Servo myServo;

const int servoPin = 12;
const int defaultServoAngle = 0;
int servoAngle =defaultServoAngle;

//------AHT10 setup------
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

//---IoT Setup------
#include <ESP8266WiFi.h>
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* WiFi Access Point *********************************/

#define WLAN_SSID "Airtel_nishant"
#define WLAN_PASS "nishant200429"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER "io.adafruit.com"

// Using port 8883 for MQTTS
#define AIO_SERVERPORT 8883

// Adafruit IO Account Configuration


#define AIO_USERNAME "aggarnishant"
#define AIO_KEY "aio_SdMX90BF4cXAMOGyfU6ipZEAGPbK"

/************ Global State (you don't need to change this!) ******************/

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);


// io.adafruit.com root fingerprint
static const char *fingerprint PROGMEM = "4E C1 52 73 24 A8 36 D6 7A 4C 67 C7 91 0C 0A 22 B9 2D 5B CA";

/****************************** Feeds ***************************************/

// Setup a feed called 'test' for publishing and 'test2' for subscription.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish LED = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/computer.led-status");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/computer.humidity");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/computer.temperature");
Adafruit_MQTT_Subscribe SERVO = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/computer.servo-angle");
Adafruit_MQTT_Publish state = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/computer.servo-state");

/*************************** Sketch Code ************************************/

// set pin numbers
const int ledPin = 14;  // the number of the led pin



void setup() {
   // Attach the servo to the specified pin and set its pulse width range
  myServo.attach(servoPin);

  pinMode(LEDPinIndi,OUTPUT);
  digitalWrite(LEDPinIndi,LOW);
  Serial.begin(115200);
  delay(10);

  Serial.println(F("Adafruit IO MQTTS (SSL/TLS) Example"));

  // Connect to WiFi access point.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  delay(1000);
  while (! aht.begin()) {
    Serial.println("Could not find Temperature sensor? Retrying...");
    delay(10);
  }
  Serial.println("Temperature sensor found");

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  digitalWrite(LEDPinIndi,HIGH);
  delay(500);
  digitalWrite(LEDPinIndi,LOW);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set Adafruit IO's root CA
 client.setFingerprint(fingerprint);

// register callback for feed
  SERVO.setCallback(servoCallback);
  // Setup MQTT subscription for time feed.
  mqtt.subscribe(&SERVO);
  // initialize the LED pin as an output
  pinMode(ledPin, INPUT);
 


}

// uint32_t x=0;


void loop() {

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
 
   // wait 10 seconds for subscription messages
  mqtt.processPackets(5000);
   Serial.print("Angle: ");
  Serial.println(servoAngle);
  if(digitalRead(ledPin)==0)
  {
    if(servoAngle==180)
    {
  myServo.write(servoAngle);
  delay(600);
  state.publish(1);
  myServo.write(0);
    }
    else
    {
      myServo.write(0);
      state.publish(0);

    }
  }
  myServo.write(0);
  // wait a couple seconds to avoid rate limit
  delay(2000);
  indicatorPublish();


}

void servoCallback(char* message, uint16_t len) {
  char messageBuffer[40];
  snprintf(messageBuffer, sizeof(messageBuffer), "Servo status is :: %s, len :: %u", message, len);
  Serial.println(messageBuffer);
  Serial.println(message);
  String inString = message;//sotre the message to String
 
  servoAngle  =  inString.toInt();//convert the message to Integer
  if(servoAngle >180 || servoAngle < 0)
  {
    servoAngle =0;
  }

}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {  // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1)
        ;
    }
  }

  Serial.println("MQTT Connected!");
  digitalWrite(LEDPinIndi,HIGH);
  delay(500);
  digitalWrite(LEDPinIndi,LOW);
}

void indicatorPublish()
{
  if(digitalRead(ledPin)==HIGH)
  LED.publish(1);
  else
  LED.publish(0);
  if(digitalRead(ledPin)==HIGH)
  {
  sensors_event_t humidity1, temp1;
  aht.getEvent(&humidity1, &temp1);// populate temp and humidity objects with fresh data
  float humValue = humidity1.relative_humidity;
  float tempValue = temp1.temperature;

  if(isnan(humValue) || isnan(tempValue))
  {
    Serial.println("Failed to read from the Sensor");
    return;
  }

  if(!temperature.publish(tempValue))
  {
    Serial.println(F("Failed"));
  }
  else
  {
    Serial.print("Temperature: ");
    Serial.println(tempValue);

  }
  if(!humidity.publish(humValue))
  {
    Serial.println(F("Failed"));
  }
  else
  {
    Serial.print("Humidity: ");
    Serial.println(humValue);

  }
  }
  else if(digitalRead(ledPin)==LOW)
  {
    Serial.println("Computer is switched off");
    //temperature.publish(0);
    //humidity.publish(0);
  }

}