#include <WiFi.h>
#include "kiss_utils.h"
#include "kiss_protocol.h"
#include "configuration.h"
#include "station_utils.h"
#include "board_pinout.h"
#include "utils.h"
#include "ESPmDNS.h"


extern Configuration        Config;

#define MAX_CLIENTS 4
#define INPUT_BUFFER_SIZE (2 + MAX_CLIENTS)

#define TNC_PORT 8001

WiFiClient* clients[MAX_CLIENTS];

WiFiServer tncServer(TNC_PORT);

String inputServerBuffer[INPUT_BUFFER_SIZE];
String inputSerialBuffer = "";


#if (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)) &&  defined (KISS_USB_OTG)
    #if ARDUINO_USB_MODE == 1
        #warning This build should be used when USB is in OTG mode
    #else
        #include "USB.h"

        #if !ARDUINO_USB_CDC_ON_BOOT
            USBCDC USBSerial;
            #define TNC_UART USBSerial
        #endif

        static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
            if (event_base == ARDUINO_USB_EVENTS) {
                arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
                switch (event_id) {
                case ARDUINO_USB_STARTED_EVENT: Serial.println("USB PLUGGED"); break;
                case ARDUINO_USB_STOPPED_EVENT: Serial.println("USB UNPLUGGED"); break;
                case ARDUINO_USB_SUSPEND_EVENT: Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en); break;
                case ARDUINO_USB_RESUME_EVENT:  Serial.println("USB RESUMED"); break;

                default: break;
                }
            } else if (event_base == ARDUINO_USB_CDC_EVENTS) {
                arduino_usb_cdc_event_data_t *data = (arduino_usb_cdc_event_data_t *)event_data;
                switch (event_id) {
                case ARDUINO_USB_CDC_CONNECTED_EVENT:    Serial.println("CDC CONNECTED"); break;
                case ARDUINO_USB_CDC_DISCONNECTED_EVENT: Serial.println("CDC DISCONNECTED"); break;
                case ARDUINO_USB_CDC_LINE_STATE_EVENT:   Serial.printf("CDC LINE STATE: dtr: %u, rts: %u\n", data->line_state.dtr, data->line_state.rts); break;
                case ARDUINO_USB_CDC_LINE_CODING_EVENT:
                    Serial.printf(
                    "CDC LINE CODING: bit_rate: %lu, data_bits: %u, stop_bits: %u, parity: %u\n", data->line_coding.bit_rate, data->line_coding.data_bits,
                    data->line_coding.stop_bits, data->line_coding.parity
                    );
                    break;
                case ARDUINO_USB_CDC_RX_EVENT:
                    Serial.printf("CDC RX [%u]:", data->rx.len);
                    {
                    uint8_t buf[data->rx.len];
                    size_t len = USBSerial.read(buf, data->rx.len);
                    Serial.write(buf, len);
                    }
                    Serial.println();
                    break;
                case ARDUINO_USB_CDC_RX_OVERFLOW_EVENT: Serial.printf("CDC RX Overflow of %d bytes", data->rx_overflow.dropped_bytes); break;

                default: break;
                }
            }
        }
    #endif

#elif defined(TNC_RXD) && defined(TNC_TXD)

    // This adds separate serial port for TNC to spleet debug and KISS TNC output
    // It requires additional USB-to-Serial interface board,
    // connected to TNC_RXD and TNC_TXD pins defined for your board
    HardwareSerial  tncSerial(2);
    #define TNC_UART tncSerial

#else

    // If there is neither native USB OTG nor additional USB-to-Serial adapter
    // this will make TNC_UART work in any case (using or not USB)
    #if ARDUINO_USB_CDC_ON_BOOT
        #define TNC_UART Serial0
    #else
        #define TNC_UART Serial
    #endif
#endif

namespace TNC_Utils {
    
    void setup() {
        if (Config.tnc.enableServer && Config.digi.ecoMode == 0) {
            tncServer.stop();
            tncServer.begin();
            //DNS-SD TNC announcement
            MDNS.begin("kiss-tnc");
            MDNS.addService("kiss-tnc", "tcp", TNC_PORT);
        }
    #if (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)) &&  defined (KISS_USB_OTG)
        // https://github.com/espressif/arduino-esp32/blob/master/libraries/USB/examples/USBSerial/USBSerial.ino
        //Serial.setDebugOutput(true);  //Change nothing?
        
        USB.onEvent(usbEventCallback);
        TNC_UART.onEvent(usbEventCallback);
        TNC_UART.begin();
        USB.begin();
        //TNC_UART.println("KISS TNC is on USBCDC");

    #elif defined(TNC_RXD) && defined(TNC_TXD)

        TNC_UART.setPins(TNC_RXD, TNC_TXD);
        TNC_UART.begin(115200);
        //TNC_UART.println("KISS TNC is on TNC_RXD+TNC_TXD pins");

    #else

        TNC_UART.println("KISS TNC is on Serial. So is all debug output to Serial!");
        // It will not work as intended till you replace in the project code
        // AND IN ALL LIBS USED all "Serial.print" with "Utils::print"...
        
    #endif
    }

    void checkNewClients() {
        WiFiClient new_client = tncServer.accept();
        if (new_client.connected()) {
            for (int i = 0; i < MAX_CLIENTS; i++) {
                WiFiClient* client = clients[i];
                if (client == nullptr) {
                    clients[i] = new WiFiClient(new_client);
                    Utils::println("New TNC client connected");
                    break;
                }
            }
        }
    }

    void handleInputData(char character, int bufferIndex) {
        String* data = (bufferIndex == -1) ? &inputSerialBuffer : &inputServerBuffer[bufferIndex];
        if (data->length() == 0 && character != (char)FEND) return;

        data->concat(character);

        if (character == (char)FEND && data->length() > 3) {
            bool isDataFrame = false;
            const String& frame = decodeKISS(*data, isDataFrame);

            if (isDataFrame) {
                if (bufferIndex != -1) {
                    Utils::print("<--- Got from TNC      : ");
                    Utils::println(frame);
                }

                String sender = frame.substring(0,frame.indexOf(">"));

                if (Config.tnc.acceptOwn || sender != Config.callsign) {
                    STATION_Utils::addToOutputPacketBuffer(frame);
                } else {
                    Utils::println("Ignored own frame from KISS");
                }
            }
            data->clear();
        }

        if (data->length() > 255) {
            data->clear();
        }
    }

    void readFromClients() {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            auto client = clients[i];
            if (client != nullptr) {
                if (client->connected()) {
                    while (client->available() > 0) {
                        char character = client->read();
                        handleInputData(character, 2 + i);
                    }
                } else {
                    delete client;
                    clients[i] = nullptr;
                }
            }
        }
    }

    void readFromSerial() {
        while (TNC_UART.available() > 0) {
            char character = TNC_UART.read();
            handleInputData(character, -1);
        }
    }

    void sendToClients(const String& packet) {
        String cleanPacket = packet.substring(3);

        const String kissEncoded = encodeKISS(cleanPacket);

        for (int i = 0; i < MAX_CLIENTS; i++) {
            auto client = clients[i];
            if (client != nullptr) {
                if (client->connected()) {
                    client->print(kissEncoded);
                    client->clear();
                } else {
                    delete client;
                    clients[i] = nullptr;
                }
            }
        }
        Utils::print("---> Sent to TNC       : ");
        Utils::println(cleanPacket);
    }

    void sendToSerial(const String& packet) {
        String cleanPacket = packet.substring(3);
        TNC_UART.print(encodeKISS(cleanPacket));
        TNC_UART.flush();
    }

    void loop() {
        if (Config.digi.ecoMode == 0) {
            if (Config.tnc.enableServer) {
                checkNewClients();
                readFromClients();
            }
            if (Config.tnc.enableSerial) {
                readFromSerial();
            }
        }
    }
}