// Ánimo Platte
// ESP8266 Communication Controller
//
// Handles Wi-Fi/Blynk connectivity and forwards virtual-pin
// commands to the ATmega328P over the serial interface.

#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Animo Platte"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"

#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// Wi-Fi credentials
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";

// Must match the ATmega328P firmware
#define BAUD_RATE 115200


// ---------------------------------------------------------------
// Serial command protocol
// ---------------------------------------------------------------
// Commands are transmitted as:
//
//     V1:VALUE\n
//     V3:VALUE\n
//     V7:VALUE\n
//     V9:VALUE\n
//     V10:VALUE\n
//
// The ATmega328P parses these commands and applies the requested
// manual control to the corresponding subsystem.
// ---------------------------------------------------------------


// Light master PWM
BLYNK_WRITE(V1)
{
    int value = param.asInt();

    Serial.println("V1:" + String(value));

    Blynk.logEvent(
        "light_update",
        "Light PWM set to " + String(value)
    );
}


// Noise PWM
BLYNK_WRITE(V3)
{
    int value = param.asInt();

    Serial.println("V3:" + String(value));

    Blynk.logEvent(
        "noise_update",
        "Noise PWM set to " + String(value)
    );
}


// Cooling fan PWM
BLYNK_WRITE(V7)
{
    int value = param.asInt();

    Serial.println("V7:" + String(value));

    Blynk.logEvent(
        "fan_update",
        "Fan PWM set to " + String(value)
    );
}


// Air purifier ON/OFF
BLYNK_WRITE(V9)
{
    int value = param.asInt();

    Serial.println("V9:" + String(value));

    Blynk.logEvent(
        "air_purifier_update",
        String("Air Purifier state set to ") +
        (value ? "ON" : "OFF")
    );
}


// Ventilation servo position
BLYNK_WRITE(V10)
{
    int value = param.asInt();

    Serial.println("V10:" + String(value));

    Blynk.logEvent(
        "vent_update",
        "Ventilation Servo angle set to " + String(value)
    );
}


void setup()
{
    Serial.begin(BAUD_RATE);
    delay(10);

    Serial.println();
    Serial.println("--- Animo Platte ESP8266 Controller ---");

    Blynk.begin(
        BLYNK_AUTH_TOKEN,
        ssid,
        pass
    );
}


void loop()
{
    Blynk.run();
}
