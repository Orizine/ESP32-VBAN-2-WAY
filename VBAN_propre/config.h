// VOLUME
static uint8_t volume = 31;
static uint8_t volume_step = 1;

const char *ssid = "wifiname"; // Nom du Wifi
const char *password = "wifipwd"; // Mot de passe Wifi

IPAddress ip(192, 168, 1, 98);
IPAddress dns(192, 168, 1, 254);
IPAddress gateway(192, 168, 1, 253);
IPAddress subnet(255, 255, 255, 0);

char StreamIn[16] = "StreamIN";          // Nom du stream à lui envoyer dans VBAN
IPAddress fromIp (192, 168, 1, 99);   // L'IP du PC qui envoie
uint16_t VBANINPort = 6980;           // Le Port du stream dans VBAN

char StreamOut[16] = "StreamOUT";   // Stream recu par le PC dans VBAN
IPAddress destIP(192,168,1,99); // IP du PC qui recoit
uint16_t VBANOUTPort = 6981;    // Port du stream reçu par le PC, dans VBAN