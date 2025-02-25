// © Jakub Jandus 2023

 #include <mqtt.h>
 #include <credentials.h>

// #define DEBUG

WiFiClient wifiClient;
PubSubClient client(wifiClient);

//------------------------------------//---messages
const uint16_t maxDataLength = 65535, maxMessageLength = 128;
const char *overseerCommandPath = "ShiveWorks/overseer/command";
const char *overseerReturnPath = "ShiveWorks/overseer/return";
// auto-assigned is the unique MAC address of the ESP32 backwards in HEX
// this ID gets called once more when subscribing to the MQTT broker
const char *segmentPathMain = "ShiveWorks/segment/";

bool sendOverseerMessage(const char *message)
{
    return client.publish(overseerReturnPath, message);
}
bool sendSegmentStatus(const char *message)
{
    String topic = "ShiveWorks/segment/" + String(ESP.getEfuseMac(), HEX) + "/status";
    return client.publish(topic.c_str(), message);
}
bool sendSegmentData(const char *message)
{
    String topic = "ShiveWorks/segment/" + String(ESP.getEfuseMac(), HEX) + "/return";
    return client.publish(topic.c_str(), message);
}

String getSegmentCommandPath()
{
    return segmentPathMain + String(ESP.getEfuseMac(), HEX) + "/command";
}

String getSegmentDataPath()
{
    return segmentPathMain + String(ESP.getEfuseMac(), HEX) + "/data";
}

//------------------------------------//---wifi

const uint32_t KEEP_ALIVE_INTERVAL = 30; // seconds to keep the connection alive, do not set to less than 15

uint32_t lastWiFiConnectionAttempt = 0;
uint32_t wifiConnectionRetryInterval = 7000; // milliseconds, do not set to less than 5000
bool isConnecting = false;
uint8_t wifiConnectionAttempts = 0;

uint8_t getConnectionAttemptsWiFi()
{
    return wifiConnectionAttempts;
}

/*
 * Connect to WiFi network
 * @param forceReconnect: if true, force a reconnect
 * @return true if connected, false otherwise
 */

bool wifi_reconnect(bool forceReconnect)
{
    // if we're not connected, try to connect
    if (WiFi.status() != WL_CONNECTED)
    {
        // try to connect, and if we failed try again after a delay
        if (millis() - lastWiFiConnectionAttempt > wifiConnectionRetryInterval || forceReconnect)
        {
#ifdef DEBUG
            Serial.print("\nMy MAC address is: ");
            Serial.print(WiFi.macAddress());

            Serial.print("\nConnecting to: ");
            Serial.println(ssid);
#endif

            // attempt to connect to WiFi network
            lastWiFiConnectionAttempt = millis();
            // WiFi.mode(WIFI_STA); // set the ESP32 to be a WiFi client
            WiFi.begin(ssid, password);

            isConnecting = true;
            wifiConnectionAttempts++;
        }

        // if we're still not connected, let us know we're still trying by printing a dot
        static uint32_t lastDot = 0;
        if (millis() - lastDot > 500)
        {
            lastDot = millis();
#ifdef DEBUG
            Serial.print('.');
#endif
        }
    }

    // if we're connected, let us know
    if (WiFi.status() == WL_CONNECTED)
    {
        // if we've been trying to connect, let us know we succeeded
        if (isConnecting)
        {
#ifdef DEBUG
            Serial.println("\nConnected with IP address: ");
            Serial.println(WiFi.localIP());
#endif
            isConnecting = false;
        }
        return true;
    }

    return false;
}

/*
 * Connect to MQTT broker
 * @return true if connected, false otherwise
 */

uint32_t lastConnectionAttempt = 0;
uint32_t connectionRetryInterval = 2500; // milliseconds, do not set to less than 2000
uint8_t MQTTConnectionAttempts = 0;

uint8_t getConnectionAttemptsMQTT()
{
    return MQTTConnectionAttempts;
}

bool mqtt_reconnect()
{
    // try to reconnect once to the MQTT broker
    if (!client.connected())
    {
        if (millis() - lastConnectionAttempt > connectionRetryInterval)
        {
            lastConnectionAttempt = millis();

            client.setServer(mqtt_server, mqtt_port);
            client.setKeepAlive(KEEP_ALIVE_INTERVAL);
            client.setBufferSize(maxDataLength);
#ifdef DEBUG
            Serial.println("MQTT connecting...");
#endif

            // Create a client ID unique to this very chip the code is uploaded to
            String clientId = String(ESP.getEfuseMac(), HEX);

#ifdef DEBUG
            Serial.print("Connecting as: ");
            Serial.println(clientId);
#endif

            // Attempt to connect
            if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD))
            {
                client.setCallback(callbackMSG); // set the callback function that will handle incoming messages               )
#ifdef DEBUG
                Serial.println("Connected to broker");
#endif
                return true;
            }

            else
            {
#ifdef DEBUG
                Serial.print("Failed, Error = ");
                Serial.print(client.state());
                Serial.println("; will retry");
#endif

                MQTTConnectionAttempts++;
                return false;
            }
        }

        return false;
    }

    return true;
}

String receivedTopic = "";    // shared buffer for received topic
bool messageReceived = false; // flag for received message

byte *receivedMessage = new byte[maxMessageLength + 1]; //  shared buffer pointer for received messages
// messages are null-terminated strings, so we need to allocate one extra byte for the null character
uint16_t receivedMessageLength = 0; // length of received message

byte *receivedData = new byte[maxDataLength]; // shared buffer pointer for received data
uint16_t receivedDataLength = 0;              // length of received data

void callbackMSG(char *topic, byte *payload, unsigned int length)
{
#ifdef DEBUG
    Serial.print(topic);
    Serial.print(" | ");
    if (length <= 32)
    {
        Serial.write(payload, length);
    }
    else
    {
        Serial.write(payload, 32);
        Serial.print("...");
    }
    Serial.print("   | length: ");
    Serial.print(length);
    Serial.println();
#endif

    // send a warning status if the message is over maxDataLength so as to not overflow the buffer
    if (length > maxDataLength - 1)
    {
        sendSegmentStatus("Warning: message is over maxDataLength set in mqtt.cpp");
#ifdef DEBUG
        Serial.println("Warning: message is over maxDataLength");
#endif
    }

    else
    {
        receivedTopic = String(topic); // using this the type of the payload can be handled differently
        // split incoming messages into data and other command messages
        if (receivedTopic == getSegmentDataPath())
        {
            // data payload will be a byte array
            for (uint16_t i = 0; i < length; i++)
            {
                receivedData[i] = (byte)payload[i];
            }
            receivedDataLength = length;
        }

        else
        {
            // command payload will always be a string
            for (int i = 0; i < length; i++)
            {
                receivedMessage[i] = (byte)payload[i];
            }
            receivedMessage[length] = '\0'; // null terminate the string
            receivedMessageLength = length;
        }
        messageReceived = true; // set the flag that a message was received
    }
}