#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#if WIFI_MANAGER
#include <WiFiManager.h>
#endif /* WIFI_MANAGER */
#include <ESP8266HTTPUpdateServer.h>
#include <FS.h>
#ifdef USE_LITTLE_FS
#include <LittleFS.h>
#define FILESYSTEM LittleFS
#else
#define FILESYSTEM SPIFFS
#endif

#include "stm32Updater.h"
#include "msp.h"

#ifdef ESP_NOW
#ifndef ESP_NOW_PEERS
#undef ESP_NOW
#endif // ESP_NOW_PEERS
#endif // ESP_NOW


#define STRINGIFY(s) #s
#define STRINGIFY_TMP(A) STRINGIFY(A)
#define CONCAT(A, B) A##B

//reference for spiffs upload https://taillieu.info/index.php/internet-of-things/esp8266/335-esp8266-uploading-files-to-the-server

//#define INVERTED_SERIAL // Comment this out for non-inverted serial

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "ExpressLRS AP"
#endif
#ifndef WIFI_AP_PSK
#define WIFI_AP_PSK "expresslrs"
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif

#if !ESP_NOW
#define WIFI_CHANNEL 0 // Not defined
#if defined(ESP_NOW_CHANNEL)
#undef ESP_NOW_CHANNEL
#endif
#define ESP_NOW_CHANNEL 1

#else // ESP_NOW
#define WIFI_CHANNEL 2
#ifndef ESP_NOW_CHANNEL
#define ESP_NOW_CHANNEL 1
#endif
#if (ESP_NOW_CHANNEL == WIFI_CHANNEL)
#error "WiFi Channel config error! ESPNOW and WiFi must be on different channels"
#endif
#endif // ESP_NOW


MDNSResponder mdns;

ESP8266WebServer server(80);

WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266HTTPUpdateServer httpUpdater;

File fsUploadFile; // a File object to temporarily store the received file
FSInfo fs_info;
String uploadedfilename; // filename of uploaded file

uint32_t TotalUploadedBytes;

uint8_t socketNumber;

String inputString = "";
String my_ipaddress_info_str = "NA";

#if ESP_NOW
String espnow_init_info = "";
#endif
String bootlog = "";

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
                domain_info = "Regulatory domain ISM 2400 (BW 0.8MHz)";
              else if (value == "4")
                domain_info = "Regulatory domain ISM 2400 (BW 1.6MHz)";
              elem.innerHTML = domain_info;

              // update rate options
              var rates = document.getElementById("rates_input");
              while (rates.length > 0) {
                rates.remove(rates.length-1);
              }
              var options = [];
              if (value == "4") {
                options = ['500Hz', '250Hz', '125Hz', '50Hz'];
              } else if (value == "3") {
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

        function command_stm32(type) {
          websock.send("stm32_cmd=" + type);
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
        <!--
        <td style="padding: 1px 1px 1px 20px;">
          Power:
          <select name="vtx_pwr" onchange="setting_send('S_vtx_pwr', this)" id="vtx_p_input">
            <option value="0">Pit</option>
            <option value="1">0</option>
            <option value="2">1</option>
            <option value="3">2</option>
          </select>
        </td>
        -->
      </tr>
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
    <br>
    <div>
      <form method='POST' action='/upload' enctype='multipart/form-data'>
          R9M Firmware:
          <input type='file' accept='.bin' name='filesystem'>
          <input type='submit' value='Upload and Flash R9M'>
      </form>
    </div>
    <br>
    <div>
    <button onclick="command_stm32('reset')">R9M Reset</button>
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

/*************************************************************************/

class CtrlSerialPrivate: public CtrlSerial
{
public:
  size_t available(void) {
    return Serial.available();
  }
  uint8_t read(void) {
    return Serial.read();
  }

  void write(uint8_t * buffer, size_t size) {
    Serial.write(buffer, size);
  }
};

CtrlSerialPrivate my_ctrl_serial;
CtrlSerial& ctrl_serial = my_ctrl_serial;

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
  msp_handler.sendPacket(&msp_out, &my_ctrl_serial);
}

void SettingsGet(void)
{
  uint8_t buff[] = {0, 0};
  SettingsWrite(buff, sizeof(buff));
}

void handleSettingRate(const char * input, int num = -1)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_rates_input=";
    settings_out += settings_rate;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting rate: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {1, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void handleSettingPower(const char * input, int num = -1)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_power_input=";
    settings_out += settings_power;
    settings_out += ",";
    settings_out += settings_power_max;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting power: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {3, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void handleSettingTlm(const char * input, int num = -1)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_tlm_input=";
    settings_out += settings_tlm;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting telemetry: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {2, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void handleSettingDomain(const char * input, int num = -1)
{
  settings_out = "[ERROR] Domain set is not supported!";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_region_domain=";
    settings_out += settings_region;
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void MspVtxWrite(const char * input, int num = -1)
{
  (void)num;
  (void)input;
  //uint8_t power = 1;
  uint16_t freq = 5840; //(uint16_t)msg[0] * 8 + msg[1]; // band * 8 + channel

  if (input[0] == '=') {
    freq = (input[1] - '0');
    freq = freq*10 + (input[2] - '0');
    freq = freq*10 + (input[3] - '0');
    freq = freq*10 + (input[4] - '0');
  }

  uint8_t vtx_cmd[] = {
    (uint8_t)freq, (uint8_t)(freq >> 8),
    //power,
    //(power == 0), // pit mode
  };

  // Fill MSP packet
  msp_out.type = MSP_PACKET_V1_CMD;
  msp_out.flags = MSP_VERSION | MSP_STARTFLAG;
  msp_out.function = MSP_VTX_SET_CONFIG;
  msp_out.payloadSize = sizeof(vtx_cmd);
  memcpy((void*)msp_out.payload, vtx_cmd, sizeof(vtx_cmd));
  // Send packet
  msp_handler.sendPacket(&msp_out, &my_ctrl_serial);
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
    socketNumber = num;

    webSocket.sendTXT(num, my_ipaddress_info_str);
#if ESP_NOW
    webSocket.sendTXT(num, espnow_init_info);
#endif

    // Send settings
    SettingsGet();
  }
  break;
  case WStype_TEXT:
    //Serial.printf("[%u] get Text: %s\r\n", num, payload);
    // send data to all connected clients
    //webSocket.broadcastTXT(payload, length);

    temp = strstr((char*)payload, "stm32_cmd=");
    if (temp) {
      // Command STM32
      if (strstr((char*)&temp[10], "reset")) {
        // Reset STM32
        reset_stm32_to_app_mode();
      }
    } else {
      // ExLRS setting commands
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
      temp = strstr((char*)payload, "S_vtx_freq");
      if (temp) {
        MspVtxWrite(&temp[10], num);
      }
    }
    break;
  case WStype_BIN:
    //Serial.printf("[%u] get binary length: %u\r\n", num, length);
    hexdump(payload, length);

    // echo data back to browser
    webSocket.sendBIN(num, payload, length);
    break;
  default:
    //Serial.printf("Invalid WStype [%d]\r\n", type);
    //webSocket.broadcastTXT("Invalid WStype: " + type);
    break;
  }
}

void sendReturn()
{
  server.send_P(200, "text/html", GO_BACK);
}

void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}


bool flashR9M()
{
  webSocket.broadcastTXT("R9M Firmware Flash Requested!");
  stm32flasher_hardware_init();
  webSocket.broadcastTXT("Going to flash the following file: " + uploadedfilename);
  bool result = esp8266_spifs_write_file(uploadedfilename.c_str());
  if (result == 0)
    reset_stm32_to_app_mode(); // boot into app
  Serial.begin(460800);
  return result;
}

void handleFileUpload()
{ // upload a new file to the SPIFFS
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {

    if (FILESYSTEM.info(fs_info))
    {
      String output;
      output += "Filesystem: used: ";
      output += String(fs_info.usedBytes);
      output += " / free: ";
      output += String(fs_info.totalBytes);
      webSocket.broadcastTXT(output);
    }
    else
    {
      webSocket.broadcastTXT("SPIFFs Failed to init!");
      return;
    }
    uploadedfilename = upload.filename;

    webSocket.broadcastTXT("Uploading file: " + uploadedfilename);

    if (!uploadedfilename.startsWith("/"))
    {
      uploadedfilename = "/" + uploadedfilename;
    }
    fsUploadFile = FILESYSTEM.open(uploadedfilename, "w"); // Open the file for writing in SPIFFS (create if it doesn't exist)
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (fsUploadFile)
    {
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
      TotalUploadedBytes += int(upload.currentSize);
      String output = "";
      output += "Uploaded: ";
      output += String(TotalUploadedBytes);
      output += " bytes";
      webSocket.broadcastTXT(output);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (fsUploadFile)
    {                       // If the file was successfully created
      fsUploadFile.close(); // Close the file again
      String totsize = String(upload.totalSize);
      webSocket.broadcastTXT("Total uploaded size: " + totsize);
      TotalUploadedBytes = 0;
      server.send(100);
      if (flashR9M())
      {
        server.sendHeader("Location", "/return"); // Redirect the client to the success page
        server.send(303);
        webSocket.broadcastTXT("Update Sucess!!!");
      }
      else
      {
        server.sendHeader("Location", "/return"); // Redirect the client to the success page
        server.send(303);
        webSocket.broadcastTXT("Update Failed!!!");
      }
    }
    else
    {
      server.send(500, "text/plain", "500: couldn't create file");
      FILESYSTEM.format();
    }
  }
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

void handleMacAddress()
{
  String message = "WiFi STA MAC: ";
  message += WiFi.macAddress();
  message += "\n  - channel in use: ";
  message += wifi_get_channel();
  message += "\n  - mode: ";
  message += (uint8_t)WiFi.getMode();
  message += "\n\nWiFi SoftAP MAC: ";
  message += WiFi.softAPmacAddress();
  message += "\n  - IP: ";
  message += WiFi.softAPIP().toString();
  message += "\n";
  server.send(200, "text/plain", message);
}

/******************* ESP-NOW *********************/
#if ESP_NOW

#include <espnow.h>

class CtrlSerialEspNow: public CtrlSerial
{
public:
  size_t available(void) {
    return 0; // not used
  }
  uint8_t read(void) {
    return 0; // not used
  }

  void write(uint8_t * buffer, size_t size) {
    while (size-- && p_iterator < sizeof(p_buffer)) {
      p_buffer[p_iterator++] = *buffer++;
    }
    if (p_iterator >= sizeof(p_buffer))
      p_iterator = UINT8_MAX; // error, not enough space
  }

  void reset(void) {
    p_iterator = 0;
  }
  void send_now(mspPacket_t *msp_in) {
    reset();
    if (msp_handler.sendPacket(msp_in, this) && p_iterator <= sizeof(p_buffer)) // check overflow
    {
      msp_handler.sendPacket(msp_in, this);
      esp_now_send(NULL, (uint8_t*)p_buffer, p_iterator);
      //webSocket.broadcastTXT("MSP sent!");
    }
  }

private:
  uint8_t p_buffer[128];
  uint8_t p_iterator = 0;
};

CtrlSerialEspNow esp_now_sender;

void esp_now_recv_cb(uint8_t *mac_addr, uint8_t *data, uint8_t data_len)
{
  /* No data or peer is unknown => ignore */
  if (!data_len || !esp_now_is_peer_exist(mac_addr))
    return;

  webSocket.broadcastTXT("ESP NOW message received!");

  // Pass data to ERLS
  // Note: accepts only correctly formatted MSP packets
  Serial.write((uint8_t*)data, data_len);
}

void esp_now_send_cb(uint8_t *mac_addr, u8 status) {
#if 0
  String temp = "ESPNOW Sent: ";
  temp += (status ? "FAIL" : "SUCCESS");
  webSocket.broadcastTXT(temp);
#endif
}

void init_esp_now(void)
{
  espnow_init_info = "ESP NOW init... ";

  if (esp_now_init() != 0) {
    espnow_init_info += "ESP NOW init failed!";
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(esp_now_send_cb);
  esp_now_register_recv_cb(esp_now_recv_cb);

#ifdef ESP_NOW_PEERS
#define ESP_NOW_ETH_ALEN 6
  uint8_t peers[][ESP_NOW_ETH_ALEN] = ESP_NOW_PEERS;
  uint8_t num_peers = sizeof(peers) / ESP_NOW_ETH_ALEN;
  for (uint8_t iter = 0; iter < num_peers; iter++) {
    //esp_now_del_peer(peers[iter]);
    if (esp_now_add_peer(peers[iter], ESP_NOW_ROLE_COMBO, ESP_NOW_CHANNEL, NULL, 0) != 0) {
      espnow_init_info += ", PEER ";
      espnow_init_info += iter;
      espnow_init_info += " FAIL";
    }
  }
#endif // ESP_NOW_PEERS

  espnow_init_info += " - Init DONE!";
}
#endif // ESP_NOW
/*************************************************/

void setup()
{
  IPAddress my_ip;
  uint8_t sta_up = 0;

#ifdef INVERTED_SERIAL
  Serial.begin(460800, SERIAL_8N1, SERIAL_FULL, 1, true); // inverted serial
#else
  Serial.begin(460800); // non-inverted serial
#endif

  FILESYSTEM.begin();

  wifi_station_set_hostname("elrs_tx");

#if defined(WIFI_SSID) && defined(WIFI_PSK)
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PSK, WIFI_CHANNEL);
  }
  uint32_t i = 0;
#define TIMEOUT (WIFI_TIMEOUT * 10)
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    if (++i > TIMEOUT) {
      break;
    }
  }
  sta_up = (WiFi.status() == WL_CONNECTED);

#elif WIFI_MANAGER
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);
  if (wifiManager.autoConnect(WIFI_AP_SSID " R9M")) {
    // AP found, connected
    sta_up = 1;
  }
#endif /* WIFI_MANAGER */

  if (!sta_up)
  {
    // WiFi not connected, Start access point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID " R9M", WIFI_AP_PSK, ESP_NOW_CHANNEL);
  }

#if ESP_NOW
  init_esp_now();
#endif // ESP_NOW

  my_ip = (sta_up) ? WiFi.localIP() : WiFi.softAPIP();

  if (mdns.begin("elrs_tx", my_ip))
  {
    mdns.addService("http", "tcp", 80);
    mdns.addService("ws", "tcp", 81);
  }
  my_ipaddress_info_str = "My IP address = ";
  my_ipaddress_info_str += my_ip.toString();

  //Serial.print("Connect to http://elrs_tx.local or http://");
  //Serial.println(my_ip);

  server.on("/", handleRoot);
  server.on("/return", sendReturn);
  server.on("/mac", handleMacAddress);

  server.on(
      "/upload", HTTP_POST,       // if the client posts to the upload page
      []() { server.send(200); }, // Send status 200 (OK) to tell the client we are ready to receive
      handleFileUpload            // Receive and save the file
  );

  server.onNotFound(handleRoot);
  httpUpdater.setup(&server);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

int serialEvent()
{
  char inChar;
  while (Serial.available())
  {
    inChar = (char)Serial.read();

    if (msp_handler.processReceivedByte(inChar)) {
      webSocket.broadcastTXT("MSP received");
      // msp fully received
      mspPacket_t &msp_in = msp_handler.getPacket();
      if (msp_in.type == MSP_PACKET_V1_ELRS) {
        uint8_t * payload = (uint8_t*)msp_in.payload;
        switch (msp_in.function) {
          case ELRS_INT_MSP_PARAMS: {
            webSocket.broadcastTXT("ELRS params resp");
            settings_rate = payload[0];
            settings_tlm = payload[1];
            settings_power = payload[2];
            settings_power_max = payload[3];
            settings_region = payload[4];

            handleSettingDomain(NULL);
            handleSettingRate(NULL);
            handleSettingPower(NULL);
            handleSettingTlm(NULL);
            break;
          }
        };
      }

#if ESP_NOW
      // Send received MSP packet to clients
      esp_now_sender.send_now(&msp_in);
#endif

      msp_handler.markPacketFree();
    } else if (!msp_handler.mspOngoing())
    {
      if (inChar == '\r') {
        continue;
      } else if (inChar == '\n') {
        return 0;
      }
      inputString += inChar;
    }
  }
  return -1;
}

void loop()
{
  if (0 <= serialEvent())
  {
    webSocket.broadcastTXT(inputString);
    inputString = "";
  }

  server.handleClient();
  webSocket.loop();
  mdns.update();
}
