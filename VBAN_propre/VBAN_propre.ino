#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include "driver/i2s.h"
#include "VBAN.h"
#include "packet.h"
#include "config.h"

WiFiUDP udpIn, udpOut;
VBan vbanIn, vbanOut;

//////////////////   AMPLI

#define GPIO_PA_EN        GPIO_NUM_21
#define GAIN GPIO_NUM_23  

//////////////////   CONFIG

uint8_t networkQuality = 4; 
uint32_t overflow_counter = 0;

//////////////////   VARIABLE VBAN

#define VBAN_BIT_RESOLUTION_MAX 2
#define EINVAL 22

//////////////////   CONFIGURATION VBAN OUT ////////////////// ESP > PC

const uint8_t  vban_out_sample_rate_selector = SAMPLE_RATE_44100_HZ;   // SET ADC SAMPLE RATE TO VALID VBAN RATE HERE
const uint16_t adc_sample_freq = VBanSRList[vban_out_sample_rate_selector];

const uint16_t i2s_in_dma_buffer_len = 64;
uint8_t newINDmaBufCount = 32;
const uint16_t i2s_read_buffer_len = i2s_in_dma_buffer_len;
uint16_t* i2s_read_buff = (uint16_t*)calloc(i2s_read_buffer_len, sizeof(uint16_t));
uint16_t noDataToTransmitCounter = 0;
const uint16_t idleTransmissionSignal[VBAN_PACKET_MAX_SAMPLES] = {32768};
unsigned long millisStartTransmission;
unsigned long prevPacketNbr = 0;
uint16_t packetCountThisTransmission = 0;

//////////////////   CONFIGURATION VBAN IN ////////////////// PC > ESP 

uint8_t VBANSample;
uint32_t* VBANData;
uint16_t* VBANData1Ch;
uint32_t VBANValue;

static int packet_pcm_check(char const* buffer, size_t size);
int vban_packet_check(char const* streamname, char const* buffer, size_t size);
uint8_t VBANINBuffer[VBAN_PROTOCOL_MAX_SIZE];

////////////////////   CONFIGURATION I2S////////////////////

i2s_port_t I2SOUT = I2S_NUM_1;
i2s_port_t I2SIN = I2S_NUM_0;
i2s_config_t i2sIN_config, i2sOUT_config;
i2s_pin_config_t i2sIN_pin_config, i2sOUT_pin_config;

const uint32_t initialSampleRate        = 44100;  // samples/sec; this must match VBAN Outgoing Stream SampleRate parameter
const uint16_t initialSamplesPerPacket  = 256;

void configure_i2sOUT(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount);
void update_i2sOUT(int newSampleRate, uint16_t , uint8_t newDmaBufCount);
void configure_i2sIN(int adc_sample_freq, uint16_t i2s_in_dma_buffer_len, uint8_t newINDmaBufCount);
void update_i2sIN(int adc_sample_freq, uint16_t i2s_in_dma_buffer_len, uint8_t newINDmaBufCount);

//////////////////    CHAN MICRO & LECTURE

adc1_channel_t adc_channel = ADC1_CHANNEL_0;

TaskHandle_t audioReadTaskHandle;
#define AUDIO_READ_TASK_PRI 4
static void audioReadTask(void * pvParameters);
static void vbanSendPacket();


//////////////////////////   SETUP   ////////////////////////////////
#define DEBUG             (bool)true
#define DEBUGTIME         10000          // time in ms

uint32_t last_millis = 0;
uint32_t cycle_time = 0;

void setup() {
  Serial.begin(115200);
  esp_err_t err;
// Activer l'Ampli  
  Serial.println("Enable amplifier");
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);
//

// Evenements WIFI
  WiFi.onEvent(PrintWiFiEvent);  
  WiFi.onEvent(RecoWIFI, SYSTEM_EVENT_STA_DISCONNECTED);  

// Connexion WIFI
  Serial.println("WiFi begin");
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

// DEMARRAGE DES I2S
  Serial.println("Start I2SIN driver");
  configure_i2sIN (adc_sample_freq, i2s_in_dma_buffer_len, newINDmaBufCount);  //CHANGER par Variable I2SIN
  delay(250);  
  Serial.println("Start I2SOUT driver"); // ATTENTION APRES I2SIN !!
  configure_i2sOUT (initialSampleRate, initialSamplesPerPacket, networkQuality);  
  delay(250);


  configure_vban_out();  
// RECEPTION et EMISSION DES PACKETS
  Serial.println("start VBAN WiFi receptor");
  udpIn.flush();
  udpIn.begin(VBANINPort);
    delay(250);
  udpOut.flush();
  udpOut.begin(VBANOUTPort);
    delay(250);  

// LECTURE MICRO
  Serial.println("Starting read tasks");
  xTaskCreatePinnedToCore(audioReadTask,  "audioReadTask",  10000, NULL, AUDIO_READ_TASK_PRI,  &audioReadTaskHandle,  1 );

 // END SETUP 
  Serial.println("Setup done");
}

/////////////////////   MAIN  ////////////////////
void loop() {
    cycle_time = micros();
 
  int packetSize = udpIn.parsePacket(); // if there's data available, read a packet
  if (packetSize) {
    if (udpIn.remoteIP() == fromIp)
    {

      udpIn.read((char*)VBANINBuffer, VBAN_PROTOCOL_MAX_SIZE); // read the packet into VBANINBuffer
      if (vban_packet_check(StreamIn, (char*)VBANINBuffer, packetSize) == 0)
      {
        struct VBanINHeader const* const hdr = PACKET_HEADER_PTR((char*)VBANINBuffer);

//        do not call set_sample_rates on every received packet, on the esp32 it causes audio glitches
//        i2s_set_sample_rates(I2SOUT, (int)VBanSRList[hdr->format_SR & VBAN_SR_MASK]);
        int rate_temp = (int)VBanSRList[hdr->format_SR & VBAN_SR_MASK];
        if (i2sOUT_config.sample_rate != rate_temp)
        {
          update_i2sOUT(rate_temp, (uint8_t) hdr->format_nbs + 1, networkQuality);
        }
       
        VBANData = (uint32_t*)(((char*)VBANINBuffer) + VBAN_HEADER_SIZE); //copy the received data to i2s buffer
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
//        WebSerial.println("VBAN Error");
      }
    }
    else
    {
      Serial.println("received packet from wrong IP address");
//      WebSerial.println("received packet from wrong IP address");
    }
  }

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
//      WebSerial.print("Overflow counter: ");
//      WebSerial.println(overflow_counter);
      Serial.println();
      last_millis = millis();
    }
  }
  yield();
}

/////////////////////   ENREGISTREMENT MICRO  ////////////////////

static void audioReadTask(void * pvParameters){
  size_t bytes_read;
  Serial.println("Receiving task started");
  for( ;; ){
      i2s_read(I2SIN, (void*)i2s_read_buff, i2s_read_buffer_len*sizeof(uint16_t), &bytes_read, portMAX_DELAY);
      if(bytes_read>0){vbanSendPacket();} 
      else {Serial.print("x");
      vTaskDelay(1);}
  }
}

/////////////////////   ENVOI DES PACKET VBAN  ////////////////////

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

//////////////////////////   CONFIG I2S   ////////////////////////////////

void configure_i2sOUT(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount){
   esp_err_t err; 
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

 err = i2s_driver_install(I2SOUT, &i2sOUT_config, 0, NULL);
 if (err != ESP_OK) {
    Serial.printf("Erreur I2SOUT Driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2SOUT, &i2sOUT_pin_config);
  if (err != ESP_OK) {
    Serial.printf("Erreur I2SOUT pin: %d\n", err);
    while (true);
  }
  i2s_zero_dma_buffer(I2SOUT);
  Serial.println("I2SOUT  installed.");
}

void configure_i2sIN(int adc_sample_freq, uint16_t i2s_in_dma_buffer_len, uint8_t newINDmaBufCount){ // Changer par variable I2SIN
  esp_err_t err;
  Serial.printf("Configuring I2SIN driver with new sample rate %u and buffer length %u\n", adc_sample_freq, i2s_in_dma_buffer_len);
     i2sIN_config = {
     .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN), // Receive, not transfer
     .sample_rate = adc_sample_freq,                         // 16KHz
     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 32bits could only get it to work with 32bits
     .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // although the SEL config should be left, it seems to transmit on right
     .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
     .intr_alloc_flags = 0,     // Interrupt level 1
     .dma_buf_count = newINDmaBufCount,                           // NetworkQuality number of buffers
     .dma_buf_len = i2s_in_dma_buffer_len,                     // BLOCK_SIZE 128 samples per buffer
     .use_apll = false 
     };
  
adc1_config_channel_atten(adc_channel, ADC_ATTEN_11db);
adc1_config_width(ADC_WIDTH_12Bit);
i2s_set_adc_mode(ADC_UNIT_1, adc_channel);

i2sIN_pin_config = {
      .bck_io_num = 5,   // BCKL
      .ws_io_num = 25,    // LRCL
      .data_out_num = -1, // not used (only for speakers)
      .data_in_num = 36   // DOUT
      };

 err = i2s_driver_install(I2SIN, &i2sIN_config, 0, NULL);
 if (err != ESP_OK) {
    Serial.printf("Erreur I2SIN Driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2SIN, &i2sIN_pin_config);
  if (err != ESP_OK) {
    Serial.printf("Erreur I2SIN pin: %d\n", err);
    while (true);
  }
  i2s_zero_dma_buffer(I2SIN);
  Serial.println("I2SIN  installed.");
}

//////////////////////////   UPDATE I2S   ////////////////////////////////

void update_i2sOUT(int newSampleRate, uint16_t newDmaBufLen, uint8_t newDmaBufCount){
  esp_err_t err;
  Serial.printf("update i2sOUT driver with new sample rate %u and buffer length %u\n", newSampleRate, newDmaBufLen);

  i2s_stop(I2SOUT);
  i2s_driver_uninstall(I2SOUT);

  i2sOUT_config.sample_rate = newSampleRate;
  i2sOUT_config.dma_buf_len = newDmaBufLen;
  i2sOUT_config.dma_buf_count = newDmaBufCount * 8;  // buffer size is multiplicated by 8

 err = i2s_driver_install(I2SOUT, &i2sOUT_config, 0, NULL);
 if (err != ESP_OK) {
    Serial.printf("Erreur I2SOUT Driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2SOUT, &i2sOUT_pin_config);
  if (err != ESP_OK) {
    Serial.printf("Erreur I2SOUT pin: %d\n", err);
    while (true);
  }
  i2s_zero_dma_buffer(I2SOUT);
  Serial.println("I2SOUT Restarted.");
}

void update_i2sIN(int adc_sample_freq, uint16_t i2s_in_dma_buffer_len, uint8_t newINDmaBufCount){
  esp_err_t err;
  Serial.printf("update i2sIN driver with new sample rate %u and buffer length %u\n", adc_sample_freq, i2s_in_dma_buffer_len);

  i2s_stop(I2SIN);
  i2s_driver_uninstall(I2SIN);

  i2sIN_config.sample_rate = adc_sample_freq;
  i2sIN_config.dma_buf_len = i2s_in_dma_buffer_len;
  i2sIN_config.dma_buf_count = newINDmaBufCount;  // buffer size is multiplicated by 8

 err = i2s_driver_install(I2SIN, &i2sIN_config, 0, NULL);
 if (err != ESP_OK) {
    Serial.printf("Erreur I2SIN Driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2SIN, &i2sIN_pin_config);
  if (err != ESP_OK) {
    Serial.printf("Erreur I2SIN pin: %d\n", err);
    while (true);
  }
  i2s_zero_dma_buffer(I2SIN);
  Serial.println("I2SIN Restarted.");
}

//////////////////////////   CONFIGURE VBAN PACKET ENVOYES  ////////////////////////////////

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


//////////////////////////   CONTROLE DES PACKET RECU VBAN   ////////////////////////////////

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
//      WebSerial.println("packet with unknown protocol");
      return -EINVAL;
  }
}

//////////////////////////   RECONNECTION WIFI   ////////////////////////////////

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

//////////////////////////   Event Wifi, afficher le message correspondant   ////////////////////////////////

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
