#if WIFI_LOGGER || WIFI_UPDATER

#include "wifi_logger.h"

#include <Arduino.h>
#include <freertos/ringbuf.h>
#include <ESPmDNS.h>
#if WIFI_MANAGER
#include <WiFiManager.h>
#endif
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <HTTPUpdate.h>

#include "msp.h"
#include "platform.h"
#include "targets.h"

#define QUEUE_SIZE 256

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "ExpressLRS AP"
#endif
#ifndef WIFI_AP_PSK
#define WIFI_AP_PSK "expresslrs"
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif

MDNSResponder mdns;
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
#define WEBSOCKET_BROADCASET(text) webSocket.broadcastTXT(text)
#define WEBSOCKET_SEND(sock, text) webSocket.sendTXT(sock, text)

/*************************************************************************/

#if WIFI_LOGGER && !WIFI_UPDATER
static QueueHandle_t DRAM_ATTR receive_queue = NULL;
static QueueHandle_t DRAM_ATTR send_queue = NULL;
#endif // WIFI_LOGGER

class CtrlSerialPrivate: public CtrlSerial
{
public:
  void set_queue_rx(QueueHandle_t q) {
    p_queue_rx = q;
  }
  void set_queue_tx(QueueHandle_t q) {
    p_queue_tx = q;
  }

  size_t available(void) {
    if (p_queue_rx == NULL)
      return 0;
    return uxQueueMessagesWaiting(p_queue_rx);
  }
  uint8_t read(void) {
    uint8_t out;
    if (p_queue_rx && xQueueReceive(p_queue_rx, &out, (TickType_t)1)) {
      return out;
    }
    return 0;
  }

  void write(uint8_t * buffer, size_t size) {
    if (p_queue_tx == NULL)
      return;
    while (size--)
      xQueueSend(p_queue_tx, (void *)buffer++, (TickType_t)0);
  }

private:
  QueueHandle_t p_queue_rx = NULL;
  QueueHandle_t p_queue_tx = NULL;
};

CtrlSerialPrivate ctrl_msp_send;
CtrlSerialPrivate ctrl_msp_receive;
CtrlSerial& ctrl_serial = ctrl_msp_send;

/*************************************************************************/

String inputString = "";
String my_ipaddress_info_str = "NA";

static const char PROGMEM GO_BACK[] = R"rawliteral(
<!DOCTYPE html>
<html>
    <head>
    </head>
    <body>
        <script>
            javascript:history.back();
        </script>
    </body>
</html>
)rawliteral";

#if WIFI_UPDATER

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
    <title>ESP update</title>
    <style>
        body {
            background-color: #1E1E1E;
            font-family: Arial, Helvetica, Sans-Serif;
            Color: #69cbf7;
        }

        textarea {
            background-color: #252525;
            Color: #C5C5C5;
            border-radius: 5px;
            border: none;
        }
    </style>
</head>
<body>
  <center>
    <div>
      <form method='POST' action='/update' enctype='multipart/form-data'>
          Update Firmware:
          <input type='file' accept='.bin' name='firmware'>
          <input type='submit' value='Upload and Flash'>
      </form>
    </div>

  </center>
</body>
</html>
)rawliteral";

#elif WIFI_LOGGER
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>

<head>
    <meta name="viewport" content="width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
    <title>TX Log Messages</title>
    <style>
        body {
            background-color: #1E1E1E;
            font-family: Arial, Helvetica, Sans-Serif;
            Color: #69cbf7;
        }

        textarea {
            background-color: #252525;
            Color: #C5C5C5;
            border-radius: 5px;
            border: none;
        }
    </style>
    <script>
        var websock;
        function start() {
            document.getElementById("logField").scrollTop = document.getElementById("logField").scrollHeight;
            websock = new WebSocket('ws://' + window.location.hostname + ':81/');
            websock.onopen = function (evt) {
              console.log('websock open');
            };
            websock.onclose = function(e) {
              console.log('Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
              setTimeout(function() {
                start();
              }, 1000);
            };
            websock.onerror = function (evt) { console.log(evt); };
            websock.onmessage = function (evt) {
                //console.log(evt);
                var text = evt.data;
                if (text.startsWith("ELRS_setting_")) {
                  var res = text.replace("ELRS_setting_", "");
                  res = res.split("=");
                  setting_set(res[0], res[1]);
                } else {
                  var logger = document.getElementById("logField");
                  var autoscroll = document.getElementById("autoscroll").checked;
                  var date = new Date();
                  var n=new Date(date.getTime() - (date.getTimezoneOffset() * 60000)).toISOString();
                  logger.value += n + ' ' + text + '\n';
                  if (autoscroll)
                    logger.scrollTop = logger.scrollHeight;
                }
            };
        }

        function saveTextAsFile() {
            var textToWrite = document.getElementById('logField').value;
            var textFileAsBlob = new Blob([textToWrite], { type: 'text/plain' });

            var downloadLink = document.createElement("a");
            downloadLink.download = "tx_log.txt";
            downloadLink.innerHTML = "Download File";
            if (window.webkitURL != null) {
                // Chrome allows the link to be clicked without actually adding it to the DOM.
                downloadLink.href = window.webkitURL.createObjectURL(textFileAsBlob);
            } else {
                // Firefox requires the link to be added to the DOM before it can be clicked.
                downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
                downloadLink.onclick = destroyClickedElement;
                downloadLink.style.display = "none";
                document.body.appendChild(downloadLink);
            }

            downloadLink.click();
        }

        function destroyClickedElement(event) {
            // remove the link from the DOM
            document.body.removeChild(event.target);
        }

        function setting_set(type, value) {
          var elem = document.getElementById(type);
          if (elem) {
            if (type == "region_domain") {
              var domain_info = "Regulatory domain UNKNOWN";
              if (value == "0")
                domain_info = "Regulatory domain 915MHz";
              else if (value == "1")
                domain_info = "Regulatory domain 868MHz";
              else if (value == "2")
                domain_info = "Regulatory domain 433MHz";
              else if (value == "3")
                domain_info = "Regulatory domain ISM 2400";
              elem.innerHTML = domain_info;

              // update rate options
              var rates = document.getElementById("rates_input");
              while (rates.length > 0) {
                rates.remove(rates.length-1);
              }
              var options = [];
              if (value == "3") {
                options = ['250Hz', '125Hz', '50Hz'];
              } else {
                options = ['200Hz', '100Hz', '50Hz'];
              }
              for (i = 0; i < options.length; i++) {
                var option = document.createElement("option");
                option.text = options[i];
                option.value = i;
                rates.add(option);
              }
            } else {
              value = value.split(",");
              if (1 < value.length) {
                var max_value = parseInt(value[1], 10);
                if (elem.options[0].value == "R")
                  max_value = max_value + 1; // include reset
                var i;
                // enable all
                for (i = 0; i < elem.length; i++) {
                  elem.options[i].disabled = false;
                }
                // disable unavailable values
                for (i = (elem.length-1); max_value < i; i--) {
                  //elem.remove(i);
                  elem.options[i].disabled = true;
                }
              }
              elem.selectedIndex = [...elem.options].findIndex (option => option.value === value[0]);
            }
          }
        }

        function setting_send(type, elem=null) {
          if (elem) {
            websock.send(type + "=" + elem.value);
          } else {
            websock.send(type + "?");
          }
        }

    </script>
</head>

<body onload="javascript:start();">
  <center>
    <h2>TX Log Messages</h2>
    <textarea id="logField" rows="40" cols="100" style="margin: 0px; height: 621px; width: 968px;"></textarea>
    <br>
    <button type="button" onclick="saveTextAsFile()" value="save" id="save">Save log to file...</button> |
    <input type="checkbox" id="autoscroll" checked><label for="autoscroll"> Auto scroll</label>
    <hr/>
    <h2>Settings</h2>
    <table>
      <tr>
        <td style="padding: 1px 20px 1px 1px;" colspan="3" id="region_domain">
          Regulatory domain UNKNOWN
        </td>
      </tr>
      <tr>
        <td style="padding: 1px 20px 1px 1px;">
          Rate:
          <select name="rate" onchange="setting_send('S_rate', this)" id="rates_input">
            <option value="0">200Hz</option>
            <option value="1">100Hz</option>
            <option value="2">50Hz</option>
          </select>
        </td>

        <td style="padding: 1px 20px 1px 20px;">
          Power:
          <select name="power" onchange="setting_send('S_power', this)" id="power_input">
            <option value="R">Reset</option>
            <option value="0">Dynamic</option>
            <option value="1">10mW</option>
            <option value="2">25mW</option>
            <option value="3">50mW</option>
            <option value="4">100mW</option>
            <option value="5">250mW</option>
            <option value="6">500mW</option>
            <option value="7">1000mW</option>
            <option value="8">2000mW</option>
          </select>
        </td>

        <td style="padding: 1px 1px 1px 20px;">
          Telemetry:
          <select name="telemetry" onchange="setting_send('S_telemetry', this)" id="tlm_input">
            <option value="R">Reset</option>
            <option value="0">Off</option>
            <option value="1">1/128</option>
            <option value="2">1/64</option>
            <option value="3">1/32</option>
            <option value="4">1/16</option>
            <option value="5">1/8</option>
            <option value="6">1/4</option>
            <option value="7">1/2</option>
          </select>
        </td>
      </tr>
      <!--
      <tr>
        <td style="padding: 1px 1px 1px 20px;">
        VTX Settings
        </td>
        <td style="padding: 1px 1px 1px 20px;">
          Freq:
          <select name="vtx_freq" onchange="setting_send('S_vtx_freq', this)" id="vtx_f_input">
            <option value="5740">F1</option>
            <option value="5760">F2</option>
            <option value="5780">F3</option>
            <option value="5800">F4</option>
            <option value="5820">F5</option>
            <option value="5840">F6</option>
            <option value="5860">F7</option>
            <option value="5880">F8</option>
          </select>
        </td>
        <td style="padding: 1px 1px 1px 20px;">
          Power:
          <select name="vtx_pwr" onchange="setting_send('S_vtx_pwr', this)" id="vtx_p_input">
            <option value="0">Pit</option>
            <option value="1">0</option>
            <option value="2">1</option>
            <option value="3">2</option>
          </select>
        </td>
      </tr>
      -->
    </table>

    <hr/>
      <h2>Danger Zone</h2>
    <div>
      <form method='POST' action='/update' enctype='multipart/form-data'>
          Self Firmware:
          <input type='file' accept='.bin' name='firmware'>
          <input type='submit' value='Upload and Flash Self'>
      </form>
    </div>

  </center>
  <hr/>
  <pre>
The following command can be used to connect to the websocket using curl, which is a lot faster over the terminal than Chrome.

curl --include \
     --output - \
     --no-buffer \
     --header "Connection: Upgrade" \
     --header "Upgrade: websocket" \
     --header "Host: example.com:80" \
     --header "Origin: http://example.com:80" \
     --header "Sec-WebSocket-Key: SGVsbG8sIHdvcmxkIQ==" \
     --header "Sec-WebSocket-Version: 13" \
     http://elrs_tx.local:81/
  </pre>
</body>
</html>
)rawliteral";

static uint8_t settings_rate = 1;
static uint8_t settings_power = 4, settings_power_max = 8;
static uint8_t settings_tlm = 7;
static uint8_t settings_region = 0;
String settings_out;

MSP msp_handler;
mspPacket_t msp_out;

void SettingsWrite(uint8_t * buff, uint8_t len)
{
    // Fill MSP packet
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.payloadSize = len;
    msp_out.function = ELRS_INT_MSP_PARAMS;
    memcpy((void*)msp_out.payload, buff, len);
    // Send packet
    msp_handler.sendPacket(&msp_out, &ctrl_msp_receive);
}

void SettingsGet(void)
{
    uint8_t buff[] = {0, 0};
    SettingsWrite(buff, sizeof(buff));
}

void handleSettingRate(const char * input, int num)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_rates_input=";
    settings_out += settings_rate;
  } else if (*input == '=') {
    input++;
    settings_out = "[L] Setting rate: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {1, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    WEBSOCKET_SEND(num, settings_out);
  else
    WEBSOCKET_BROADCASET(settings_out);
}

void handleSettingPower(const char * input, int num)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_power_input=";
    settings_out += settings_power;
    settings_out += ",";
    settings_out += settings_power_max;
  } else if (*input == '=') {
    input++;
    settings_out = "[L] Setting power: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {3, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    WEBSOCKET_SEND(num, settings_out);
  else
    WEBSOCKET_BROADCASET(settings_out);
}

void handleSettingTlm(const char * input, int num)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_tlm_input=";
    settings_out += settings_tlm;
  } else if (*input == '=') {
    input++;
    settings_out = "[L] Setting telemetry: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {2, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    WEBSOCKET_SEND(num, settings_out);
  else
    WEBSOCKET_BROADCASET(settings_out);
}

void handleSettingDomain(const char * input, int num)
{
  settings_out = "[ERROR] Domain set is not supported!";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_region_domain=";
#if 1
    settings_out += settings_region;
#else
#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_FCC_915)
        settings_out += 0;
#elif defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_EU_868_R9)
        settings_out += 1;
#elif defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
        settings_out += 2;
#elif defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_800kHz)
        settings_out += 3;
#else
        settings_out += 0xff;
#endif
#endif
  }
  if (0 <= num)
    WEBSOCKET_SEND(num, settings_out);
  else
    WEBSOCKET_BROADCASET(settings_out);
}

void SettingsSendWS(int num)
{
  handleSettingDomain(NULL, num);
  handleSettingRate(NULL, num);
  handleSettingPower(NULL, num);
  handleSettingTlm(NULL, num);
}

void MspVtxWrite(void)
{
  // TODO, FIXME: VTX power & freq
  uint8_t power = 1;
  uint16_t freq = 5880; //(uint16_t)msg[0] * 8 + msg[1]; // band * 8 + channel
  uint8_t vtx_cmd[] = {
    (uint8_t)(freq >> 8), (uint8_t)freq,
    power,
    (power == 0), // pit mode
  };

  // Fill MSP packet
  msp_out.type = MSP_PACKET_V1_CMD;
  msp_out.flags = MSP_VERSION | MSP_STARTFLAG;
  msp_out.function = MSP_VTX_SET_CONFIG;
  msp_out.payloadSize = sizeof(vtx_cmd);
  memcpy((void*)msp_out.payload, vtx_cmd, sizeof(vtx_cmd));
  // Send packet
  msp_handler.sendPacket(&msp_out, &ctrl_msp_receive);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  char * temp;
  //Serial.printf("webSocketEvent(%d, %d, ...)\r\n", num, type);

  switch (type)
  {
  case WStype_DISCONNECTED:
    //Serial.printf("[%u] Disconnected!\r\n", num);
    break;
  case WStype_CONNECTED:
  {
    //IPAddress ip = webSocket.remoteIP(num);
    //Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);

    WEBSOCKET_SEND(num, my_ipaddress_info_str);
    // Send settings
    SettingsSendWS(num);
  }
  break;
  case WStype_TEXT:
    //Serial.printf("[%u] get Text: %s\r\n", num, payload);
    // send data to all connected clients
    //WEBSOCKET_BROADCASET(payload, length);

    temp = strstr((char*)payload, "S_rate");
    if (temp) {
      handleSettingRate(&temp[6], num);
      break;
    }
    temp = strstr((char*)payload, "S_power");
    if (temp) {
      handleSettingPower(&temp[7], num);
      break;
    }
    temp = strstr((char*)payload, "S_telemetry");
    if (temp) {
      handleSettingTlm(&temp[11], num);
    }

    break;
  case WStype_BIN:
    //Serial.printf("[%u] get binary length: %u\r\n", num, length);
    //hexdump(payload, length);

    // echo data back to browser
    webSocket.sendBIN(num, payload, length);
    break;
  default:
    //Serial.printf("Invalid WStype [%d]\r\n", type);
    //WEBSOCKET_BROADCASET("Invalid WStype: " + type);
    break;
  }
}

#endif // WIFI_LOGGER

void sendReturn()
{
  server.send_P(200, "text/html", GO_BACK);
}

void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleNotFound()
{
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void wifi_setup(void)
{
  IPAddress my_ip;
#if defined(WIFI_SSID) && defined(WIFI_PSK)
  uint32_t iter, jter, timeout;
#endif
  //wifi_station_set_hostname("elrs_tx");

#if defined(WIFI_SSID) && defined(WIFI_PSK)
  //Serial.begin(115200);
  Serial.print("Connecting to wifi ");

  timeout = WIFI_TIMEOUT * 2;
  timeout /= 10;

  for (iter = 0; (iter < timeout) && (WiFi.status() != WL_CONNECTED); iter++) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.persistent(false);
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PSK);

      jter = 0;
      while (++jter <= 50) {
        vTaskDelay(100);
        if (WiFi.status() == WL_CONNECTED) {
          break;
        } else if ((jter % 10) == 0) {
          Serial.print(".");
        }
      }
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    my_ip = WiFi.localIP();
    Serial.println(" CONNECTED!");
  } else
#elif WIFI_MANAGER
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);
  if (wifiManager.autoConnect(WIFI_AP_SSID" ESP32 TX")) {
    // AP found, connected
    my_ip = WiFi.localIP();
  }
  else
#endif // WIFI_MANAGER
  {
    Serial.println(" FAILED! Starting AP...");
    // No WiFi found, start access point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID" ESP32 TX", WIFI_AP_PSK);
    my_ip = WiFi.softAPIP();
  }

  if (mdns.begin("elrs_tx"))
  {
    mdns.addService("http", "tcp", 80);
    mdns.addService("ws", "tcp", 81);
  }

  my_ipaddress_info_str = "[L] My IP address = ";
  my_ipaddress_info_str += my_ip.toString();

  Serial.print("Connect to http://elrs_tx.local or http://");
  Serial.println(my_ip);

  server.on("/", handleRoot);
  server.on("/return", sendReturn);

  /* handling uploading firmware file (OTA update) */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    //server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    if (Update.hasError()) {
      //server.sendHeader("Connection", "close");
      server.send(200, "text/plain", "OTA flash failed!");
    } else {
      server.send(200, "text/html", "<HEAD><meta http-equiv=\"refresh\" content=\"0;url=/\"></HEAD>");
    }
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      platform_radio_force_stop();
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.onNotFound(handleRoot);
  server.begin();

#if WIFI_LOGGER && !WIFI_UPDATER
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
#endif // WIFI_LOGGER
}

#if WIFI_LOGGER && !WIFI_UPDATER
int serialEvent(QueueHandle_t queue)
{
  char inChar;
  // process inputs
  uint32_t numBytes = uxQueueMessagesWaiting(queue);
  while (numBytes--)
  {
    if (xQueueReceive(queue, &inChar, (TickType_t)10)) {
      if (msp_handler.processReceivedByte(inChar)) {
        WEBSOCKET_BROADCASET("[L] MSP received");
        // msp fully received
        mspPacket_t &msp_in = msp_handler.getPacket();
        if (msp_in.type == MSP_PACKET_V1_ELRS) {
          switch (msp_in.function) {
            case ELRS_INT_MSP_PARAMS: {
              WEBSOCKET_BROADCASET("[L] ELRS params resp");
              uint8_t * payload = (uint8_t*)msp_in.payload;
              settings_rate = payload[0];
              settings_tlm = payload[1];
              settings_power = payload[2];
              settings_power_max = payload[3];
              settings_region = payload[4];
              SettingsSendWS(-1); // send to all clients
              break;
            }
          };
        }
        msp_handler.markPacketFree();

      } else if (!msp_handler.mspOngoing()) {
        if (inChar == '\r') {
          continue;
        } else if (inChar == '\n') {
          return 0;
        }
        inputString += inChar;
      }
    } else {
      break; // no more input data
    }
  }
  return -1;
}
#endif // WIFI_LOGGER

void wifi_loop(QueueHandle_t queue)
{
#if WIFI_LOGGER && !WIFI_UPDATER
  if (0 <= serialEvent(queue))
  {
    WEBSOCKET_BROADCASET(inputString);
    inputString = "";
  }
#else
  (void)queue;
#endif // WIFI_LOGGER

  server.handleClient();
#if WIFI_LOGGER && !WIFI_UPDATER
  webSocket.loop();
#endif // WIFI_LOGGER
}

TaskHandle_t wifiTask = NULL;

void httpsTask(void *pvParameters)
{
  QueueHandle_t queue = (QueueHandle_t)pvParameters;

  Serial.println("HTTP task about to start...");
  wifi_setup();
  for(;;) {
    wifi_loop(queue);
  }

#if WIFI_LOGGER && !WIFI_UPDATER
  /* delete the input queue */
  receive_queue = NULL;
  vQueueDelete(queue);

  /* and output queue */
  queue = send_queue;
  send_queue = NULL;
  vQueueDelete(queue);
#endif // WIFI_LOGGER

  /* remove task */
  vTaskDelete( NULL );
  wifiTask = NULL;
  Serial.println("HTTP task exit");
}

void wifi_stop(void)
{

}

void wifi_start(void)
{
  if (wifiTask != NULL) {
    Serial.println("HTTP task already started");
    return;
  }

#if WIFI_LOGGER && !WIFI_UPDATER
  receive_queue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
  send_queue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));

  ctrl_msp_send.set_queue_rx(send_queue);
  ctrl_msp_send.set_queue_tx(receive_queue);

  ctrl_msp_receive.set_queue_rx(receive_queue);
  ctrl_msp_receive.set_queue_tx(send_queue);
#else // WIFI_LOGGER
#define receive_queue NULL
  Serial.println("Logger disabled!");
#endif // WIFI_LOGGER

  uint8_t taskPriority = 10;
  xTaskCreatePinnedToCore(
    httpsTask,              //Function to implement the task
    "wifiTask",             //Name of the task
    4096,                   //Stack size in words
    receive_queue,          //Task input parameter
    taskPriority,           //Priority of the task
    &wifiTask, 0);
  Serial.println("HTTP task started");
}

/*************************************************************************/

#if WIFI_LOGGER && !WIFI_UPDATER
int DebugSerial::available(void)
{
  if (receive_queue == NULL)
    return 0;
  return uxQueueMessagesWaiting(receive_queue);
}

int DebugSerial::availableForWrite(void)
{
  if (receive_queue == NULL)
    return 0;
  return uxQueueSpacesAvailable(receive_queue);
}

int DebugSerial::peek(void)
{
  return 0;
}

int DebugSerial::read(void)
{
  uint8_t out;
  if (receive_queue && xQueueReceive(receive_queue, &out, (TickType_t)1)) {
    return out;
  }
  return 0;
}

void DebugSerial::flush(void)
{
  /* do nothing */
}

size_t DebugSerial::write(uint8_t data)
{
  return write(&data, 1);
}

size_t DebugSerial::write(const uint8_t *buffer, size_t size)
{
  size_t num = 0;
  if (receive_queue == NULL)
    return 0;
  while (size--) {
    xQueueSend(receive_queue, (void *)buffer++, (TickType_t)0);
    num++;
  }
  return num;
}

DebugSerial wifi_logger_serial;
#endif // WIFI_LOGGER

#endif // WIFI_LOGGER || WIFI_UPDATER