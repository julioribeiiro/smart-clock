/**
 * Created by K. Suwatchai (Mobizt)
 * 
 * Email: k_suwatchai@hotmail.com
 * 
 * Github: https://github.com/mobizt
 * 
 * Copyright (c) 2021 mobizt
 *
*/

/** This example will show how to authenticate using 
 * the legacy token or database secret with the new APIs (using config and auth data).
*/
#include <SoftwareSerial.h>
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif

//Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

//Pinos de comunicacao serial com a ST Núcleo
#define Pin_ST_NUCLEO_RX    5  //Pino D1 da placa Node MCU
#define Pin_ST_NUCLEO_TX    4  //Pino D2 da placa Node MCU

#define MAX_MESSAGE_LENGTH  100

SoftwareSerial SSerial(Pin_ST_NUCLEO_RX, Pin_ST_NUCLEO_TX);

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Manoel Casa"
#define WIFI_PASSWORD "manoel1500"

/* 2. If work with RTDB, define the RTDB URL and database secret */
#define DATABASE_URL "smart-clock-a7cc8-default-rtdb.firebaseio.com" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define DATABASE_SECRET "TsIsshMi1YgUJWa1ES5Rl1vLorF8IH3029w4Gpbq"

/* 3. Define the Firebase Data object */
FirebaseData fbdo;

/* 4, Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* Define the FirebaseConfig data for config data */
FirebaseConfig config;

int alarm = 0;
int button_state = 0;
String output = "";
static char message[100];

void setup()
{

    Serial.begin(115200);
    SSerial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the certificate file (optional) */
    //config.cert.file = "/cert.cer";
    //config.cert.file_storage = StorageType::FLASH;

    /* Assign the database URL and database secret(required) */
    config.database_url = DATABASE_URL;
    config.signer.tokens.legacy_token = DATABASE_SECRET;

    Firebase.reconnectWiFi(true);

    /* Initialize the library with the Firebase authen and config */
    Firebase.begin(&config, &auth);

    //Or use legacy authenticate method
    //Firebase.begin(DATABASE_URL, DATABASE_SECRET);

        
}

void loop()
{

  Firebase.getInt(fbdo, "/ALARM");
  alarm = fbdo.intData();
  if (alarm){
    SSerial.write('l');
  } else {
    SSerial.write('d');
  }
  delay(1000);
 

  if (SSerial.available()){
    Serial.write(SSerial.read());
    
    if(SSerial.read() == 10) {
      Firebase.setInt(fbdo, "/ALARM", 0);
    }
  }
  if (Serial.available()){
    SSerial.write(Serial.read());
  }

}
