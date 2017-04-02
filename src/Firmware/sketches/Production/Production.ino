#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoNunchuk.h>

/*
 * Configuration
 *
 */

#define FIRMWARE_VERSION                        "1.0.3"

#define DEBUG                                   0

#define WIFI_SSID                               "Bastis Cruiserboard"
#define WIFI_PASSWORD                           "cruiser1337"
#define WIFI_CONNECTION_RETRIES                 20

#define PIN_STATUSLED                           LED_BUILTIN

#define UDP_SERVER_HOST                         "10.0.1.1"
#define UDP_SERVER_PORT                         8888

// We just send 4 bytes
#define UDP_BUFFER_SIZE                         4

// Send every N milliseconds a packet to receiver
#define PACKET_SEND_INTERVAL                    100

// Set the "real" minimum/maximum of x axis that the joystick is able to reach
#define NUNCHUK_MIN_VALUE_X                     31
#define NUNCHUK_MAX_VALUE_X                     232

// Set the "real" minimum/maximum of y axis that the joystick is able to reach
#define NUNCHUK_MIN_VALUE_Y                     29
#define NUNCHUK_MAX_VALUE_Y                     218

#define PIN_BATTERYSENSOR                       A0

/*
 * Main program
 *
 */


ArduinoNunchuk nunchuk = ArduinoNunchuk();
int connectionFailCouter = WIFI_CONNECTION_RETRIES;
WiFiUDP udpClient = WiFiUDP();

void setup()
{
    Serial.begin(115200);
    delay(250);

    setupPins();
    setupNunchuk();
    setupWifi();
}

void setupPins()
{
    pinMode(PIN_STATUSLED, OUTPUT);
    pinMode(PIN_BATTERYSENSOR, INPUT);
}

void setupNunchuk()
{
    nunchuk.init();
}

void setupWifi()
{
    Serial.printf("Connecting to to Wi-Fi access point '%s'...\n", WIFI_SSID);

    // Do not store Wi-Fi config in SDK flash area
    WiFi.persistent(false);

    // Disable auto Wi-Fi access point mode
    WiFi.mode(WIFI_STA);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    bool wasSuccessful = true;

    while (WiFi.status() != WL_CONNECTED)
    {
        if (connectionFailCouter <= 0)
        {
            wasSuccessful = false;
            break;
        }

        Serial.print(F("."));

        // Blink 2 times when connecting
        blinkStatusLED(2);
        delay(300);

        connectionFailCouter--;
    }

    Serial.println();

    if (wasSuccessful)
    {
        Serial.print(F("Connected to Wi-Fi access point. Obtained IP address: "));
        Serial.println(WiFi.localIP());

        // Enable LED constantly
        digitalWrite(PIN_STATUSLED, LOW);
    }
    else
    {
        Serial.printf("Connection failed after %i tries! Giving up.\n", WIFI_CONNECTION_RETRIES);

        // Disable LED constantly
        digitalWrite(PIN_STATUSLED, HIGH);

        startDeepSleep();
    }
}

void blinkStatusLED(const int times)
{
    for (int i = 0; i < times; i++)
    {
        // Enable LED
        digitalWrite(PIN_STATUSLED, LOW);
        delay(100);

        // Disable LED
        digitalWrite(PIN_STATUSLED, HIGH);
        delay(100);
    }
}

void startDeepSleep()
{
    Serial.println(F("Shutting down. Going to deep sleep..."));
    ESP.deepSleep(0);

    Serial.println(F("Deep sleep failed!"));
}

void loop()
{
    sendCurrentControlStatus();
    delay(PACKET_SEND_INTERVAL);
}

void sendCurrentControlStatus()
{
    nunchuk.update();

    const uint8_t rawValueX = nunchuk.analogX;
    const uint8_t rawValueY = nunchuk.analogY;

    const uint8_t normalizedValueX = normalizeValueX(rawValueX);
    const uint8_t normalizedValueY = normalizeValueY(rawValueY);

    const uint8_t mappedValueX = map(normalizedValueX, NUNCHUK_MIN_VALUE_X, NUNCHUK_MAX_VALUE_X, 0, 255);
    const uint8_t mappedValueY = map(normalizedValueY, NUNCHUK_MIN_VALUE_Y, NUNCHUK_MAX_VALUE_Y, 0, 255);

    const float batteryVoltage = readBatteryVoltage();
    const int multipliedBatteryVoltage = batteryVoltage * 10;

    #if DEBUG == 1
        Serial.print(F("Nunchuk X axis raw value: "));
        Serial.println(rawValueX);

        Serial.print(F("Nunchuk X axis normalized value: "));
        Serial.println(normalizedValueX);

        Serial.print(F("Nunchuk X axis mapped value: "));
        Serial.println(mappedValueX);

        Serial.println();

        Serial.print(F("Nunchuk Y axis raw value: "));
        Serial.println(rawValueY);

        Serial.print(F("Nunchuk Y axis normalized value: "));
        Serial.println(normalizedValueY);

        Serial.print(F("Nunchuk Y axis mapped value: "));
        Serial.println(mappedValueY);

        Serial.println();

        Serial.print(F("Nunchuk Z button pressed: "));
        Serial.println(nunchuk.zButton);

        Serial.print(F("Nunchuk C button pressed: "));
        Serial.println(nunchuk.cButton);

        Serial.println();

        Serial.print(F("Battery voltage: "));
        Serial.println(batteryVoltage);

        Serial.print(F("Battery multiplied voltage: "));
        Serial.println(multipliedBatteryVoltage);

        Serial.println();
        Serial.println();
    #endif

    uint8_t buttonsPressedState = 0b00000000;

    if (nunchuk.zButton == 1)
    {
        buttonsPressedState |= 0b10000000;
    }

    if (nunchuk.cButton == 1)
    {
        buttonsPressedState |= 0b01000000;
    }

    byte message[UDP_BUFFER_SIZE] = {0};
    message[0] = mappedValueX;
    message[1] = mappedValueY;
    message[2] = buttonsPressedState;
    message[3] = multipliedBatteryVoltage;

    sendUDPPacket(message, sizeof(message));
}

uint8_t normalizeValueX(const uint8_t rawValueX)
{
    // Be sure the X value is not greater or smaller than the given limits:
    const uint8_t normalizedValue = constrain(rawValueX, NUNCHUK_MIN_VALUE_X, NUNCHUK_MAX_VALUE_X);

    return normalizedValue;
}

uint8_t normalizeValueY(const uint8_t rawValueY)
{
    // Be sure the Y value is not greater or smaller than the given limits:
    const uint8_t normalizedValue = constrain(rawValueY, NUNCHUK_MIN_VALUE_Y, NUNCHUK_MAX_VALUE_Y);

    return normalizedValue;
}

float readBatteryVoltage()
{
    /*

    10kOhm 2,2kOhm resistors

    1,0V becomes 0,18V

    0.0 -> 0.00
    1.0 -> 0.18
    3.2 -> 0.58
    4.2 -> 0.76
    5.0 -> 0.90
    */

    const int analogBatterySensorLevel = analogRead(PIN_BATTERYSENSOR);
    const int reducedBatteryVoltageInMillivolt = map(analogBatterySensorLevel, 0, 1023, 0, 1000);;
    const float batteryVoltage = (float) reducedBatteryVoltageInMillivolt / 180.0;

    return batteryVoltage;
}

void sendUDPPacket(const byte payload[], const uint16_t length)
{
    udpClient.beginPacket(UDP_SERVER_HOST, UDP_SERVER_PORT);
    udpClient.write(payload, length);
    udpClient.endPacket();
}
