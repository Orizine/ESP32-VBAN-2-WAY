//    .sample_rate =  newSampleRate,        //int initialSampleRate  = 44100
//    .dma_buf_len = newDmaBufLen,          //uint16_t initialSamplesPerPacket
//    .dma_buf_count = newDmaBufCount * 8,  //uint8_t Initiam NetworkQuality
//Connect the pin to the GPIO matrix.
//PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_number], PIN_FUNC_GPIO);
//Set the direction. GPIO_MODE_INPUT always makes it an input, GPIO_MODE_OUTPUT always makes it an output, GPIO_MODE_INPUT_OUTPUT lets the peripheral decide what direction it has. You usually want the last one.
//gpio_set_direction(bus_config->miso_io_num, GPIO_MODE_INPUT_OUTPUT);
//Connect the output functionality of a peripheral to the pin you want. This allows a peripheral to set the direction of the pin (in case it's configured as GPIO_INPUT_OUTPUT) and set the output value.
//gpio_matrix_out(pin_number, io_signal_out, false, false);
//Connect the input functionality of a peripheral to the pin. This allows the peripheral to read the signal indicated from this pin.
//gpio_matrix_in(pin_number, io_signal_in, false);
//   gpio_set_direction(0, GPIO_MODE_OUTPUT);
//   PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
//   REG_WRITE(PIN_CTRL,0);

#include "hal_i2c.h"
#include "Arduino.h"
#include <driver/i2s.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include "VBAN.h"
#include "mywifi.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "packet.h"
#include "Audio.h"
#include <stdio.h>
#include <string.h>
//#include "SPIFFS.h"
//#include "IotWebConf.h"
//#include "lwip/apps/sntp.h"


//LED
#include <NeoPixelBus.h>

//TIME
#include <time.h>
#include <NTPClient.h>
//#include <ArduinoOTA.h>
//#include <ESPmDNS.h>

//WEBSERIAL
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

AsyncWebServer server(80);


//
#define VBAN_BIT_RESOLUTION_MAX 2
#define EINVAL 22

//AMPLI
#define GPIO_PA_EN        GPIO_NUM_21
#define GAIN GPIO_NUM_23  

//CONF
uint8_t networkQuality = 4; 
uint32_t overflow_counter = 0;
const uint8_t ptt_pin = 27;

WiFiUDP udpIn, udpOut, ntpUDP;
VBan vbanIn, vbanOut;
NTPClient timeClient(ntpUDP);

//////////////////////////////
// NeoPixel led control
/////////////////////////////
#define PixelCount 1
#define PixelPin 22
RgbColor RED(255, 0, 0);
RgbColor GREEN(0, 255, 0);
RgbColor BLUE(0, 0, 255);
RgbColor YELLOW(255, 128, 0);
RgbColor WHITE(255, 255, 255);
RgbColor BLACK(0, 0, 0);

RgbColor REDL(64, 0, 0);
RgbColor GREENL(0, 64, 0);
RgbColor BLUEL(0, 0, 64);
RgbColor YELLOWL(64, 32, 0);
RgbColor WHITEL(64, 64, 64);
RgbColor BLACKL(0, 0, 0);
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

int b0 = -1, b1 = -1, b2 = -1;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// WEBSERIAL MONITOR AND CMD from
//////////////////////////////////////////////////////////////////////////
void SetVolume(){
//gpio_set_level(GAIN,newvol);
    WebSerial.print("volume réglé sur ");
//    WebSerial.println(newvol);
}

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Sended Data...");
  Serial.println("Received Data...");

  String d = "";
  
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  
  if (d.startsWith("vol") ) {   //&& d.indexOf(volvalue, from)
int newvol = d.charAt(3);
  WebSerial.println(d);
    WebSerial.println(newvol);
  Serial.print(d);
}  else  {
      WebSerial.println("Commande incorrecte");
  }

}

 // return 0;
//  if (d == "ON"){
//    digitalWrite(LED, HIGH);
//  }
//  if (d=="OFF"){
//    digitalWrite(LED, LOW);
//    
//  }
//}


///// task for battery monitoring /////

#define BATGOOD 2300
#define BATXXXX 2200
#define BATXXX 2100
#define BATXX 2000
#define BATX 1900
#define BATLOW 1800

static void battery(void* pdata)
{
  int val;
  while(1)
  {
   val = adc1_get_raw(ADC1_GPIO33_CHANNEL);
   printf("Battery : %d\n", val);
   WebSerial.println(timeClient.getFormattedTime());
   WebSerial.print("Battery : ");
   WebSerial.println(val);
   
   if(val >= BATGOOD){
    strip.SetPixelColor(0, GREEN);
      WebSerial.print("GREEN");    
   }
   if(val < BATGOOD && val > BATXXXX){
    strip.SetPixelColor(0, GREENL);
      WebSerial.print("GREENL");    
   }
   if(val < BATXXXX && val > BATXXX){
    strip.SetPixelColor(0, BLUEL);
      WebSerial.print("BLUEL");    
   }
   if(val < BATXXX && val > BATXX){
    strip.SetPixelColor(0, WHITEL);
      WebSerial.print("WHITEL");    
   }
   if(val < BATXX && val > BATX){
    strip.SetPixelColor(0, YELLOWL);
      WebSerial.print("YELLOWL");    
   }
   if(val < BATX){
    strip.SetPixelColor(0, RED);
      WebSerial.print("RED");    
   }
   ;
   strip.Show();
  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());   
   delay(10000);
   }  
}


adc1_channel_t adc_channel = ADC1_CHANNEL_0;

const uint8_t  vban_out_sample_rate_selector = SAMPLE_RATE_44100_HZ;   // SET ADC SAMPLE RATE TO VALID VBAN RATE HERE
const uint16_t adc_sample_freq = VBanSRList[vban_out_sample_rate_selector];

const uint32_t initialSampleRate        = 44100;  // samples/sec; this must match VBAN Outgoing Stream SampleRate parameter
const uint16_t initialSamplesPerPacket  = 256;

const uint16_t i2s_in_dma_buffer_len = 64;
const uint16_t i2s_read_buffer_len = i2s_in_dma_buffer_len;
uint16_t* i2s_read_buff = (uint16_t*)calloc(i2s_read_buffer_len, sizeof(uint16_t));
uint16_t noDataToTransmitCounter = 0;
const uint16_t idleTransmissionSignal[VBAN_PACKET_MAX_SAMPLES] = {32768};
unsigned long millisStartTransmission;
unsigned long prevPacketNbr = 0;
uint16_t packetCountThisTransmission = 0;

TaskHandle_t audioReadTaskHandle;
#define AUDIO_READ_TASK_PRI 4
static void audioReadTask(void * pvParameters);
static void vbanSendPacket();

i2s_port_t I2SOUT = I2S_NUM_1;
i2s_port_t I2SIN = I2S_NUM_0;
i2s_config_t i2sIN_config, i2sOUT_config;
i2s_pin_config_t i2sIN_pin_config, i2sOUT_pin_config;

void configure_i2sOUT(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount);
void update_i2sOUT(int newSampleRate, uint16_t , uint8_t newDmaBufCount);

static int packet_pcm_check(char const* buffer, size_t size);
int vban_packet_check(char const* streamname, char const* buffer, size_t size);
uint8_t VBANINBuffer[VBAN_PROTOCOL_MAX_SIZE];

#define DEBUG             (bool)true
#define DEBUGTIME         10000          // time in ms

uint32_t last_millis = 0;
uint32_t cycle_time = 0;

//////////////////////////   CONFIGURE VBAN PACKET   ////////////////////////////////
/* VBAN "out":  from esp32 to PC */
void configure_vban_out() {
  // Set vban packet header, counter, and data frame pointers to respective parts of packet:
  vbanOut.hdr           = (VBanHeader*) &vbanOut.packet[0];
  vbanOut.packet_number = (uint32_t*)   &vbanOut.packet[VBAN_PACKET_HEADER_BYTES];
  vbanOut.data_frame    = (int16_t*)    &vbanOut.packet[VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES];
  
  // Setup the packet header:
  strncpy(vbanOut.hdr->preamble, "VBAN", 4);
  vbanOut.hdr->sample_rate      = VBAN_PROTOCOL_AUDIO | vban_out_sample_rate_selector;  // Set protocol and sample rate
  vbanOut.hdr->num_samples      = i2s_in_dma_buffer_len-1;                      // actual length = num_samples +1
  vbanOut.hdr->num_channels     = 0;                                            // actual channels = num_channels +1
  vbanOut.hdr->sample_format    = VBAN_BITFMT_16_INT | VBAN_CODEC_PCM;          // int16 PCM
  strncpy(vbanOut.hdr->stream_name, StreamOut, 16);

  *vbanOut.packet_number = 0;  // initialize packet counter

  vbanOut.data_bytes = (vbanOut.hdr->num_samples+1) * (vbanOut.hdr->num_channels+1) * ((vbanOut.hdr->sample_format & VBAN_BIT_RESOLUTION_MASK)+1);
  vbanOut.total_bytes = VBAN_PACKET_HEADER_BYTES + VBAN_PACKET_COUNTER_BYTES + vbanOut.data_bytes;
}
//////////////////////////   SECU WIFI   ////////////////////////////////
/*void CoWIFI(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info){
  Serial.println("WiFi connected");
  Serial.print("WiFi IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}*/
void RecoWIFI(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info){
  Serial.println("[-] Disconnected from WiFi");
  WiFi.config(ip, gateway, subnet); // manual ip config
  WiFi.mode(WIFI_STA);
  Serial.print("WiFi Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("WiFi IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}
void triggerPTT(){
  digitalWrite(ptt_pin, HIGH);
}

void releasePTT(){
  digitalWrite(ptt_pin, LOW);
}
//////////////////////////   SETUP   ////////////////////////////////
void setup() {
/*   gpio_set_direction (GPIO18, GPIO_MODE_INPUT_OUTPUT);
   PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, I2S1I_BCK_out );
   REG_WRITE(PIN_CTRL,GPIO18);

   gpio_set_direction(GPIO19, GPIO_MODE_INPUT_OUTPUT);
   PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, I2S1I_WS_out);
   REG_WRITE(PIN_CTRL,GPIO19);

   gpio_set_direction (GPIO23, GPIO_MODE_INPUT_OUTPUT);
   PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, I2SOI_WS_i);
   REG_WRITE(PIN_CTRL,GPIO23);

   gpio_set_direction(GPIO22, GPIO_MODE_INPUT_OUTPUT);
   PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, I2S0I_BCK_i); 
   REG_WRITE(PIN_CTRL,GPIO22);
*/
  pinMode(ptt_pin,OUTPUT);
  digitalWrite(ptt_pin, LOW);
  Serial.begin(115200);
  // Enable amplifier
  Serial.println("Enable amplifier");
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);
  
    WiFi.onEvent(PrintWiFiEvent);  
//    WiFi.onEvent(CoWIFI, SYSTEM_EVENT_STA_CONNECTED);
    WiFi.onEvent(RecoWIFI, SYSTEM_EVENT_STA_DISCONNECTED);
    
  Serial.println("WiFi begin");
  WiFi.config(ip, gateway, subnet); // manual ip config
  WiFi.mode(WIFI_STA);
  Serial.print("WiFi Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  server.begin();
/*  ArduinoOTA.setHostname("TALKIE1");
    ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.println("Progress: %u%%\r");
    Serial.print((progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.println("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();  
*/
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("WiFi IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  timeClient.begin();
///////////////////////////////////////////////////////////////
  strip.Begin();  

////////////////////////////////////////////////////////////////
// init ADC interface for battery survey
/////////////////////////////////////////////////////////////////
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_11);
////////////////////////////////////////////////////////////////
//task managing the battery
  xTaskCreate(battery, "battery", 5000, NULL, 1, NULL);  

gpio_reset_pin(GAIN);
gpio_set_direction(GAIN, GPIO_MODE_OUTPUT);
//gpio_set_pull_mode(GAIN, GPIO_PULLDOWN_ONLY);
//gpio_set_level(GAIN,4);
  Serial.println("Configuring I2SIN...");
  esp_err_t err;

  // The I2S config as per the example
i2sIN_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN), // Receive, not transfer
      .sample_rate = adc_sample_freq,                         // 16KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 32bits could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // although the SEL config should be left, it seems to transmit on right
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = 0,     // Interrupt level 1
      .dma_buf_count = 32,                           // NetworkQuality number of buffers
      .dma_buf_len = i2s_in_dma_buffer_len,                     // BLOCK_SIZE 128 samples per buffer
      .use_apll = false 
  };
  
adc1_config_channel_atten(adc_channel, ADC_ATTEN_11db);
adc1_config_width(ADC_WIDTH_12Bit);
i2s_set_adc_mode(ADC_UNIT_1, adc_channel);

  // The pin config as per the setup
i2sIN_pin_config = {
      .bck_io_num = 5,   // BCKL
      .ws_io_num = 25,    // LRCL
      .data_out_num = -1, // not used (only for speakers)
      .data_in_num = 36   // DOUT
      };

 err = i2s_driver_install(I2SIN, &i2sIN_config, 0, NULL);
 if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2SIN, &i2sIN_pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }
  Serial.println("I2SIN  installed.");
 
  Serial.println("Start I2SOUT driver");
  configure_i2sOUT (initialSampleRate, initialSamplesPerPacket, networkQuality);  
  delay(250);

  Serial.println("start VBAN WiFi receptor");
  udpIn.begin(VBANINPort);
        udpIn.flush();

  configure_vban_out();  
  delay(250);
      Serial.print("  Starting udp-out..");
      udpOut.flush();
      udpOut.begin(VBANOUTPort);
  Serial.println("Starting read tasks");
  xTaskCreatePinnedToCore(audioReadTask,  "audioReadTask",  10000, NULL, AUDIO_READ_TASK_PRI,  &audioReadTaskHandle,  1 );
  Serial.println("Setup done");

}
//Si Event Wifi, afficher le message correspondant
void PrintWiFiEvent(WiFiEvent_t event) {
    switch (event) {

    case SYSTEM_EVENT_WIFI_READY:
        Serial.println("SYSTEM_EVENT_WIFI_READY");
        break;

    case SYSTEM_EVENT_SCAN_DONE:
        Serial.println("SYSTEM_EVENT_SCAN_DONE");
        break;

    case SYSTEM_EVENT_STA_START:
        Serial.println("SYSTEM_EVENT_STA_START");
        break;

    case SYSTEM_EVENT_STA_STOP:
        Serial.println("SYSTEM_EVENT_STA_STOP");
        break;

    case SYSTEM_EVENT_STA_CONNECTED://or STARTED ?
        Serial.println("SYSTEM_EVENT_STA_CONNECTED");
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("SYSTEM_EVENT_STA_DISCONNECTED");
        break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        Serial.println("SYSTEM_EVENT_STA_AUTHMODE_CHANGE");
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("SYSTEM_EVENT_STA_GOT_IP");
        break;

    case SYSTEM_EVENT_STA_LOST_IP:
        Serial.println("SYSTEM_EVENT_STA_LOST_IP");
        break;

    case SYSTEM_EVENT_AP_START:
        Serial.println("SYSTEM_EVENT_AP_START");
        break;

    case SYSTEM_EVENT_AP_STOP:
        Serial.println("SYSTEM_EVENT_AP_STOP");
        break;

    case SYSTEM_EVENT_AP_STACONNECTED:
        Serial.println("SYSTEM_EVENT_AP_STACONNECTED");
        break;

    case SYSTEM_EVENT_AP_STADISCONNECTED:
        Serial.println("SYSTEM_EVENT_AP_STADISCONNECTED");
        break;

    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        Serial.println("SYSTEM_EVENT_AP_STAIPASSIGNED");
        break;

    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        Serial.println("SYSTEM_EVENT_AP_PROBEREQRECVED");
        break;

    default:
        Serial.println("UNKNOWN EVENT: " + event);
        break;
    }
}

uint8_t VBANSample;
uint32_t* VBANData;
uint16_t* VBANData1Ch;
uint32_t VBANValue;

//////////////////////////   MAIN LOOP   ////////////////////////////////
void loop() {
  cycle_time = micros();

  // if there's data available, read a packet
  int packetSize = udpIn.parsePacket();
  if (packetSize) {
    if (udpIn.remoteIP() == fromIp)
    {
      // read the packet into VBANINBuffer
      udpIn.read((char*)VBANINBuffer, VBAN_PROTOCOL_MAX_SIZE);
      if (vban_packet_check(StreamIn, (char*)VBANINBuffer, packetSize) == 0)
      {
        struct VBanINHeader const* const hdr = PACKET_HEADER_PTR((char*)VBANINBuffer);

        //do not call set_sample_rates on every received packet, on the esp32 it causes audio glitches
        //i2s_set_sample_rates(I2SOUT, (int)VBanSRList[hdr->format_SR & VBAN_SR_MASK]);
        int rate_temp = (int)VBanSRList[hdr->format_SR & VBAN_SR_MASK];
        if (i2sOUT_config.sample_rate != rate_temp)
        {
          update_i2sOUT(rate_temp, (uint8_t) hdr->format_nbs + 1, networkQuality);
        }

        //copy the received data to i2s buffer
        VBANData = (uint32_t*)(((char*)VBANINBuffer) + VBAN_HEADER_SIZE);
        VBANData1Ch = (uint16_t*)VBANData;

        size_t bytes_written = 0;

        switch (hdr->format_nbc)
        {
          case 0:    //one channel
            for (VBANSample = 0; VBANSample < hdr->format_nbs; VBANSample++)
            {
              VBANValue = 0;
              VBANValue = (uint32_t)VBANData1Ch[VBANSample] << 16 | VBANData1Ch[VBANSample];
              i2s_write(I2SOUT, &VBANValue, sizeof(uint32_t), &bytes_written, 0);
              if (bytes_written != 4)
              {
                overflow_counter++;
              }
            }
            break;

          case 1:   //two channels
            i2s_write(I2SOUT, VBANData, (hdr->format_nbs + 1) * sizeof(uint32_t), &bytes_written, 0);
            overflow_counter = overflow_counter + ((hdr->format_nbs + 1) - (bytes_written / 4));
            break;

          default:  //channel # not supported
            Serial.println("wrong number of channels");
            break;
        }

      }
      else
      {
        Serial.println("VBAN Error");
        WebSerial.println("VBAN Error");
      }
    }
    else
    {
      Serial.println("received packet from wrong IP address");
      WebSerial.println("received packet from wrong IP address");
    }
  }

  //Buttons--------------------------

//  volUpBtn.read();
//  volDoBtn.read();

  //---------------------------------

  if (DEBUG)
  {

    cycle_time = micros() - cycle_time;

    if ((millis() - last_millis) > DEBUGTIME)
    {
      Serial.println("--------------------------------");
      Serial.println("Debug: ");
      Serial.print("WIFI: ");
      Serial.print(WiFi.status());
      Serial.print(WiFi.SSID());
      Serial.print(" (");
        Serial.println((String)"[+] RSSI : " + WiFi.RSSI() + " dB");
        Serial.println(WiFi.gatewayIP());
        Serial.print("[+] Subnet Mask : ");      
//      Serial.print("Volume: ");
//      Serial.println(volume);
      Serial.print("Cycle-Time: ");
      Serial.print(cycle_time);
      Serial.println("us");
      Serial.print("Overflow counter: ");
      Serial.println(overflow_counter);
      WebSerial.print("Overflow counter: ");
      WebSerial.println(overflow_counter);
      Serial.println();
      last_millis = millis();
    }
  }
  yield();
}

static int packet_pcm_check(char const* buffer, size_t size)
{
  /** the packet is already a valid vban packet and buffer already checked before */

  struct VBanINHeader const* const hdr = PACKET_HEADER_PTR(buffer);
  enum VBanBitResolution const bit_resolution = (VBanBitResolution)(hdr->format_bit & VBAN_BIT_RESOLUTION_MASK);
  int const sample_rate   = hdr->format_SR & VBAN_SR_MASK;
  int const nb_samples    = hdr->format_nbs + 1;
  int const nb_channels   = hdr->format_nbc + 1;
  size_t sample_size      = 0;
  size_t payload_size     = 0;

  //logger_log(LOG_DEBUG, "%s: packet is vban: %u, sr: %d, nbs: %d, nbc: %d, bit: %d, name: %s, nu: %u",
  //    __func__, hdr->vban, hdr->format_SR, hdr->format_nbs, hdr->format_nbc, hdr->format_bit, hdr->streamname, hdr->nuFrame);

  if (bit_resolution >= VBAN_BIT_RESOLUTION_MAX)
  {
    Serial.println("invalid bit resolution");
    return -EINVAL;
  }

  if (sample_rate >= VBAN_SR_MAXNUMBER)
  {
    Serial.println("invalid sample rate");
    return -EINVAL;
  }

  sample_size = VBanBitResolutionSize[bit_resolution];
  payload_size = nb_samples * sample_size * nb_channels;

  if (payload_size != (size - VBAN_HEADER_SIZE))
  {
    //    logger_log(LOG_WARNING, "%s: invalid payload size, expected %d, got %d", __func__, payload_size, (size - VBAN_HEADER_SIZE));
    Serial.println("invalid payload size");
    return -EINVAL;
  }

  return 0;
}
//////////////////////////   CONFIG I2S   ////////////////////////////////
void configure_i2sOUT(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount)
{
  Serial.printf("Configuring I2SOUT driver with new sample rate %u and buffer length %u\n", newSampleRate, newDmaBufLen);
     i2sOUT_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate =  newSampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = newDmaBufCount * 8,     // buffer size is multiplicated by 8
    .dma_buf_len = newDmaBufLen,
    .use_apll = true,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t i2sOUT_pin_config = {
    .bck_io_num = 5,
    .ws_io_num = 25,
    .data_out_num = 26,
    .data_in_num = -1
  };

  i2s_driver_install(I2SOUT, &i2sOUT_config, 0, NULL);
  i2s_set_pin(I2SOUT, &i2sOUT_pin_config);
  i2s_zero_dma_buffer(I2SOUT);

  //enable MCLK on GPIO0
//  REG_WRITE(PIN_CTRL, 0xFF0);
//  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

}
//////////////////////////   UPDATE I2S   ////////////////////////////////
void update_i2sOUT(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount)
{
  Serial.printf("update i2s driver with new sample rate %u and buffer length %u\n", newSampleRate, newDmaBufLen);

  i2s_stop(I2SOUT);
  i2s_driver_uninstall(I2SOUT);

  i2sOUT_config.sample_rate = newSampleRate;
  i2sOUT_config.dma_buf_len = newDmaBufLen;
  i2sOUT_config.dma_buf_count = newDmaBufCount * 8;  // buffer size is multiplicated by 8

  i2s_driver_install(I2SOUT, &i2sOUT_config, 0, NULL);
  i2s_set_pin(I2SOUT, &i2sOUT_pin_config);
  i2s_zero_dma_buffer(I2SOUT);

  //enable MCLK on GPIO0
//  REG_WRITE(PIN_CTRL, 0xFF0);
//  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

}
int vban_packet_check(char const* streamname, char const* buffer, size_t size)
{
  struct VBanINHeader const* const hdr = PACKET_HEADER_PTR(buffer);
  enum VBanProtocol protocol = VBAN_PROTOCOL_UNDEFINED_4;
  enum VBanCodec codec = (VBanCodec)VBAN_BIT_RESOLUTION_MAX;

  if ((streamname == 0) || (buffer == 0))
  {
    Serial.println("null pointer argument");
    return -EINVAL;
  }

  if (size <= VBAN_HEADER_SIZE)
  {
    Serial.println("packet too small");
    return -EINVAL;
  }

  if (hdr->vban != VBAN_HEADER_FOURC)
  {
    Serial.println("invalid vban magic fourc");
    return -EINVAL;
  }

  if (strncmp(streamname, hdr->streamname, VBAN_STREAM_NAME_SIZE))
  {
    Serial.println("different streamname");
    return -EINVAL;
  }

  /** check the reserved bit : it must be 0 */
  if (hdr->format_bit & VBAN_RESERVED_MASK)
  {
    Serial.println("reserved format bit invalid value");
    return -EINVAL;
  }

  /** check protocol and codec */
  protocol        = (VBanProtocol)(hdr->format_SR & VBAN_PROTOCOL_MASK);
  codec           = (VBanCodec)(hdr->format_bit & VBAN_CODEC_MASK);

  switch (protocol)
  {
    case VBAN_PROTOCOL_AUDIO:
      return (codec == VBAN_CODEC_PCM) ? packet_pcm_check(buffer, size) : -EINVAL;
      Serial.println("switch protocol");
      /*Serial.print("OK: VBAN_PROTOCOL_AUDIO");
        Serial.print(": nuFrame ");
        Serial.print(hdr->nuFrame);
        Serial.print(": format_SR ");
        Serial.print(hdr->format_SR);
        Serial.print(": format_nbs ");B
        Serial.print(hdr->format_nbs);
        Serial.print(": format_nbc ");
        Serial.println(hdr->format_nbc);
      */
      return 0;
    case VBAN_PROTOCOL_SERIAL:
    case VBAN_PROTOCOL_TXT:
    case VBAN_PROTOCOL_UNDEFINED_1:
    case VBAN_PROTOCOL_UNDEFINED_2:
    case VBAN_PROTOCOL_UNDEFINED_3:
    case VBAN_PROTOCOL_UNDEFINED_4:
      /** not supported yet */
      return -EINVAL;

    default:
      Serial.println("packet with unknown protocol");
      WebSerial.println("packet with unknown protocol");
      return -EINVAL;
  }
}
/////////////////////   SAMPLING TASK AND FUNCTIONS  ////////////////////
static void audioReadTask(void * pvParameters){
  size_t bytes_read;
  Serial.println("Receiving task started");
  for( ;; ){
      i2s_read(I2SIN, (void*)i2s_read_buff, i2s_read_buffer_len*sizeof(uint16_t), &bytes_read, portMAX_DELAY);
      if(bytes_read>0){
        vbanSendPacket();
     } else {
      Serial.print("x");
      vTaskDelay(1);
    }

  }
}

static void vbanSendPacket(){
  int16_t adc_data[VBAN_PACKET_MAX_SAMPLES];

  //// Per esp32.com forum topic 11023, esp32 swaps even/odd samples,
  ////   i.g. samples 0 1 2 3 4 5 are stored as 1 0 3 2 5 4 ..
  ////   Have to deinterleave manually; use xor "^1" to leap frog indices
  //// Also need to mask upper 4 bits which contain channel info (see gitter chat between me-no-dev and bzeeman)  
  for(int i=0; i<VBAN_PACKET_MAX_SAMPLES; i++){  // caution: this is not robust to odd buffer lens
    adc_data[i^1]   = (int16_t)(i2s_read_buff[i] & 0x0FFF) - 2048;
  }
  memcpy(vbanOut.data_frame, adc_data, vbanOut.data_bytes);

  // Send packet
    udpOut.beginPacket(destIP, VBANOUTPort);
    udpOut.write((uint8_t*)&vbanOut.packet, vbanOut.total_bytes);
    udpOut.endPacket();  
  
  
  (*vbanOut.packet_number)++;   // increment packet counter
}
