
// VOLUME
static uint8_t volume = 31;
static uint8_t volume_step = 1;
//const char* ssid = "Livebox-CFDE";
//const char* password = "nJiPEyszeSUY2ZKzL4";
const char *ssid = "Continuum-escape-game"; // Nom du Wifi
const char *password = "timetraveler"; // Mot de passe Wifi

IPAddress ip(192, 168, 1, 98);
IPAddress dns(192, 168, 1, 254);
IPAddress gateway(192, 168, 1, 253);
IPAddress subnet(255, 255, 255, 0);

char StreamIn[16] = "SHTEL";          // Nom du stream à lui envoyer dans VBAN
IPAddress fromIp (192, 168, 1, 99);   // L'IP du PC qui envoie
uint16_t VBANINPort = 6980;           // Le Port du stream dans VBAN

char StreamOut[16] = "TELSH";   // Stream recu par le PC dans VBAN
IPAddress destIP(192,168,1,99); // IP du PC qui recoit
uint16_t VBANOUTPort = 6981;    // Port du stream reçu par le PC, dans VBAN

/* TEST DEDE
IPAddress ip(192, 168, 1, 111);
IPAddress dns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

char StreamIn[16] = "TALKIEIN";      // incoming VBAN Stream Name
IPAddress fromIp (192, 168, 1, 11);   // only receive VBAN Packets from this IP
uint16_t VBANINPort = 6980;           // local port to listen on


char StreamOut[16] = "TALKIEOUT";
IPAddress destIP(192,168,1,11);
uint16_t VBANOUTPort = 6981;
*/
