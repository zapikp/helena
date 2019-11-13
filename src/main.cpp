//datum posledn9 zmeny 17.11.2017 - zvyseni hodnoty maxtemp
//datum posledn9 zmeny 17.10.2018 - oprava synchronizace casu, nacteni wifi az po nastaveni jasu
//datum posledn9 zmeny 28.10.2018 - nastaveni pouziti NMI v pwm-new.h
//datum posledni zmeny 5.11.2018 - nastaveni násobitele na zpomaleni PWM_MULTIPLY
//datum posledni zmeny 5.11.2018 - nastaveni testovani stejnych hashu
//datum posledni zmeny 11.2.2019  - nastaveni komunikace pres DNS jmeno (dale zmena promene time)
//datum posledni zmeny 4.10.2019  - kontrola na nulovy unixtime
//datum posledni zmeny 4.10.2019  - opravena blba chyba v eepromwrite / mensi pole
//datum posledni zmeny 4.10.2019  - doplnena OTA
//datum posledni zmeny 12.10.2019 -100 doplneno verzovani
//prevod na Git
//datum posledni zmeny 28.10.2019 -101 doplnen posun casu rozedneni a setmeni pri zmene zony

#include "pwm-new.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <EEPROM.h>
//#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//na miste
//  #define HELENA

//skript pro ukladani teplot a stavu
#ifdef HELENA
  #define SKRIPTV "sonda/helena.php"
#else
  #define SKRIPTV "sonda/helena1.php"
#endif


//#define SKRIPTV "sonda/helena.php"
//#define SKRIPTV "sonda/helena1.php"

//verze
//1.0.0 14.10.2019
#define VERSION 101




//definice pinu
#define PWM1  D5
#define PWM2  D6
#define PWM3  D7
#define TEPLOMER  D1  //pin pro DS18B20
#define TOP_PIN  D2 //pin pro pripojeni rele topeni
#define POCET_DS 3    //maximalni ocekavany pocet DS18B20
#define TEPLOMER_TIME 1 //interval prevodu teplromeru
//prideleni pameti EEPROM, rastr 4byte
#define ROZED 0    //adresa v eeprom pro ulozeni casu rozednivani
#define DELKAROZED 4  //delka rozednivani
#define STMIV 8   //adresa v eeprom pro ulozeni casu stmivani
#define DELKASTMIV 12 //delka stmivani
#define JASDEN 16 //jas v denn9m rezim
#define JASNOC 20 //jas v nocnim rezimu
#define TIMEZONE 24 //1Byte na casovou zonu
#define TOPENI  25 //1Byte teplota pro spinani topeni
#define EE_NOC  27 //zacatek pole pro prepinace channel_night 3byte
#define EE_DIS  30 //zacatek pro pole prepinace channel_disable 3byte
//#define TOPENI_HASH 212 //hash cidla podle ktere se bude regulovat topeni
//#define TOPENI_HASH 5 //hash cidla podle ktere se bude regulovat topeni
#define TOPENI_HASH 184
//zapnuti posunu casu rozedneni a setmeni podle zvolene casove zony
#define ABS_TIME 1

#define konstanta 1000 //pocet ms na s



#define PWM_CHANNELS 3
#define PWM_MULTIPLY 10
const uint32_t period = 1023*PWM_MULTIPLY; // * 200ns ^= 5 kHz
uint32 io_info[PWM_CHANNELS][3] = {
	// MUX, FUNC, PIN
  {PERIPHS_IO_MUX_MTMS_U,  FUNC_GPIO14, 14}, //D5
	{PERIPHS_IO_MUX_MTDI_U,  FUNC_GPIO12, 12}, //D6
	//{PERIPHS_IO_MUX_MTDO_U,  FUNC_GPIO15, 15},
	{PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13, 13}, //D7
	//{PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5 ,  5},
  };
  // initial duty: all off
  uint32 pwm_duty_init[PWM_CHANNELS] = {0,0,0};

//hlavičky funkci
OneWire temp(TEPLOMER);
ESP8266WebServer server(80);
//Ticker regul;
Ticker temper;
Ticker synchro;
Ticker minuta;
Ticker meziprechod;
WiFiUDP udp;

void prevod_teplomeru();
void handleRoot();
void handleNotFound();
void handleSubmit();
void index();
void zobraz();
void odesli();
void sendNTPpacket(IPAddress& address);
uint8_t zjistiNTP();
void syncenable();
void set_minuta();
void write_to_eeprom(uint32_t integer, uint8_t adresa);
uint32_t read_from_eeprom(uint8_t adresa);
void to_day();
void to_night();
void set_pwm (uint16_t pwm);
void search_temp (void);

//promenne pro pripojeni k wifi
#include "password.h"
//const char* ssid     = "xxxxx";
//const char* password = "yyyyyy";

const char* host = "zapadlo.name";
IPAddress host_IP_fallback(91,139,59,185);
IPAddress host_IP;
//const uint8_t host_IP[]={109,69,211,132};
const char* host_ntp="tik.cesnet.cz";
//promenne NTP
unsigned int localPort = 2390;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
IPAddress timeServerIP;
unsigned long epoch; //unix time
//promenne pro web server
const int httpPort = 80;
unsigned long last = 0;
//promenne pro teplotni cidla
byte addrT[POCET_DS][8];    //adresy teplotnich cidel
uint8_t pocetT = 0;      //pocet teplotnich cidel
float teploty[POCET_DS];    //hodnoty teplotnich cidel
byte hash[POCET_DS];    //identifikace teplotnich cidel (hashe)
uint8_t valid_temp_read[POCET_DS]; //pocet spravnych cteni na teplotnich cidlech
boolean ds_flag=false;
//promena pro chybu
int err=0;
//promenne pro cas
uint32_t testtime=0;
uint32_t temptime=0;
uint32_t time_z=0;
uint8_t syncnow=1;
uint8_t sync=0;
uint8_t timezone=0;
uint8_t flag_minuta=0;

//promene pro PWM
uint16_t pwm1=0;
uint16_t pwm2=0;
uint16_t pwm3=0; //promenne pro aktualni hodnoty
uint16_t pwmmax; //maximalni hodnota pwm
uint16_t pwmmin; //minimalni hodnota pwm
//promenne pro ulozeni casu pro pwm
uint32_t cas_rozedneni;
uint32_t cas_setmeni; // startovni casy  rozedneni a stmivani
uint32_t delka_rozedneni;
uint32_t delka_setmeni;
//rizeni prechodu
boolean prechod=false;
boolean prvni_pruchod=false;
uint16_t pocitadlo_mezi=0; //promena pro aktualni stav PWM
uint16_t pocet_mezi=0; //pocitadlo pro aktualni pozici pri setmeni
//promenne pro ladeni
uint32_t start_unixtime=0;
uint32_t prubezny_unixtime=0;
uint16_t pocitadlo_odeslani=0;
//promenne pro TOPENI
uint8_t topeni_temp=0; //hodnota je teplota*10
uint8_t topeni_spin=0;
uint8_t topeni_index=255; //index cidla podle kterho se reguluje
//
IPAddress tst;
//promenne pro rizeni kanalu
uint8_t channel_night[]={0,0,0};
uint8_t channel_disable[]={0,0,0};



String INDEX_HTML;
String INDEX_HTML_1 =
"<!DOCTYPE HTML>"
"<html>"
"<head>"
"<meta charset=\"UTF-8\">"
"<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">"
"<title>ESP8266 Web Form Settings</title>"
"<style>"
"\"body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }\""
"</style>"
"</head>"
"<body>"
"<h1>Nastavení svícení</h1>"
"<FORM action=\"/\" method=\"post\">"
"<P>"
"<br>";

String INDEX_HTML_3 =
"<tr><td></td><td><INPUT type=\"submit\" value=\"Uložit\"></td></tr>"
"</table>"
"</P>"
"</FORM>"
"</body>"
"</html>";



void setup() {
  Serial.begin(115200);
  delay(10);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TOP_PIN, OUTPUT);
  digitalWrite(TOP_PIN, 1);
//  analogWriteFreq(10000);


  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pwm_init(period, pwm_duty_init, PWM_CHANNELS, io_info);
  Serial.println("start1");
  pwm_start();
  Serial.println("start2");
  set_pwm(0);
  Serial.print("Version: ");
  Serial.println(VERSION);

  //inicializace EEPROM
  EEPROM.begin(512);




  //inicializace teplotnich cidel, test zda nejsou 2hashe stejne
  for (uint8_t i=0;i <3 ;i++ ){
      pocetT = 0;
      search_temp();
      uint8_t nogood_T=0;
      for (uint8_t j=0; j < pocetT; j++){
        for (uint8_t k=0; k < pocetT; k++){
          if (j != k){
            if (hash[j]==hash[k]){
              nogood_T++;
            }
          }
        }
      }
      if (nogood_T == 0){
        Serial.print("Vysledek testu na shodne hashe:");
        Serial.print(nogood_T);
        Serial.print(" Cyklus:");
        Serial.println(i);
        break;
      }
  }




//nacteni hodnot z eeprom
cas_rozedneni=read_from_eeprom(ROZED);
if (cas_rozedneni > 86400) {
  cas_rozedneni=25200; //7 rano
  write_to_eeprom(cas_rozedneni,ROZED);
}
delka_rozedneni=read_from_eeprom(DELKAROZED);
if ((delka_rozedneni > 7200)||(delka_rozedneni==0)){
  delka_rozedneni=1800; //pul hodiny
  write_to_eeprom(delka_rozedneni,DELKAROZED);
}
cas_setmeni=read_from_eeprom(STMIV);
if ((cas_setmeni >  86400)||(cas_setmeni<cas_rozedneni)){
  cas_setmeni=64800; //18 hodiny
  write_to_eeprom(cas_setmeni,STMIV);
}
delka_setmeni=read_from_eeprom(DELKASTMIV);
if ((delka_setmeni > 7200)||(delka_setmeni==0)){
  delka_setmeni=1800;
  write_to_eeprom(delka_setmeni,DELKASTMIV);
}
pwmmax=read_from_eeprom(JASDEN);
if (pwmmax > 1023){
  pwmmax=1023;
  write_to_eeprom(pwmmax,JASDEN);
}
pwmmin=read_from_eeprom(JASNOC);
if(pwmmin > 1023){
  pwmmin=0;
  write_to_eeprom(pwmmin,JASNOC);
}
timezone=EEPROM.read(TIMEZONE);
if ((timezone>2)||(timezone<1)){
  timezone=1; //zimni cas
  EEPROM.write(TIMEZONE,timezone);
  EEPROM.commit();
}
topeni_temp=EEPROM.read(TOPENI);
//naceteni poli channel_night a channel_disable
for (uint8_t i=0;i<3;i++){
  channel_night[i]=EEPROM.read(EE_NOC+i);
  channel_disable[i]=EEPROM.read(EE_DIS+i);
}
//start PWm na nocni rezimu
set_pwm(pwmmin);

// We start by connecting to a WiFi network


Serial.println();
Serial.println();
Serial.print("Connecting to ");
Serial.println(ssid);

WiFi.begin(ssid, password);
WiFi.persistent(false);
WiFi.mode(WIFI_STA);

while (WiFi.status() != WL_CONNECTED) {
  digitalWrite(LED_BUILTIN,0);
  delay(50);
  Serial.print(".");
  digitalWrite(LED_BUILTIN,1);
  delay(350);
}

Serial.println("");
Serial.println("WiFi connected");
Serial.println("IP address: ");
Serial.println(WiFi.localIP());
tst = WiFi.localIP();
//OTA
/*
ArduinoOTA.onStart([]() {
  Serial.println("Start");
});
ArduinoOTA.onEnd([]() {
  Serial.println("\nEnd");
});
ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
});
ArduinoOTA.onError([](ota_error_t error) {
  Serial.printf("Error[%u]: ", error);
  if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  else if (error == OTA_END_ERROR) Serial.println("End Failed");
});
*/
ArduinoOTA.setHostname("ptacci");
ArduinoOTA.begin();


//start web server
if (MDNS.begin("ptacci")) {
   Serial.println("MDNS responder started");
 }

 server.on("/", handleRoot);
 server.onNotFound(handleNotFound);
 server.begin();
 Serial.println("HTTP server started");
 //zjisteni NTP casu
 Serial.println("Starting UDP");
 udp.begin(localPort);
 Serial.print("Local port: ");
 Serial.println(udp.localPort());
 for (uint8_t i=0;i<3; i++){
   if (zjistiNTP()==0){
     break;
   }
 }
 if (sync==0) {
      synchro.attach (120, syncenable);
 } else {
      synchro.attach (43000, syncenable);
 }
 start_unixtime=prubezny_unixtime;
//start prevod teplot z cidel DS18B20
temper.attach(TEPLOMER_TIME, prevod_teplomeru);
//start minutovych intervalu
minuta.attach(60, set_minuta);
/*Serial.println("test zapisu EEPROM");
uint32_t ts=86400;
Serial.println (ts);
write_to_eeprom(ts, ROZED);
Serial.println(read_from_eeprom(ROZED));*/
//zjisteni IP adresy
for (uint8_t i=0; i<3;i++){
  if (!WiFi.hostByName(host, host_IP)){
    Serial.println("DNS resolve failed");
  } else {
    Serial.print("IP adresa serveru je:");
    Serial.println(host_IP);
    break;
  }
  delay(1000);
}


}



void loop() {
  //obsluha OTA
  ArduinoOTA.handle();
  //obsluha web clientu
  server.handleClient();
  //vterinove intervaly
  testtime = millis ();
  if ((testtime - temptime) > konstanta)
    {
      for (uint8_t i = 0; i < ((testtime - temptime) / konstanta); i++)
	     {
	        time_z++;
	        temptime = temptime + konstanta;
	       }
    if (time_z >= 86400)
	     {			//vynulovani dne
	        time_z = time_z - 86400;
          pocitadlo_odeslani=0;
	     }
    }

    //synchronizace s NTP
    if (syncnow == 0) {
      for (uint8_t i=0;i<3; i++){
        if (zjistiNTP()==0){
          break;
        }
      }
    syncnow = 1;
    if (sync==0) {
         synchro.attach (120, syncenable);
    } else {
         synchro.attach (43000, syncenable);
    }
    /*Serial.print ("Syncnow: ");
    Serial.println (syncnow);
    Serial.print ("Sync: ");
    Serial.println (sync);*/
    }
    //minutovy interval / odeslani, zobrazeni
    if (flag_minuta==1){
      flag_minuta=0;
      zobraz();

      //spinani rele pro regulaci
      if  (topeni_index != 255){
        //existuje cidlo podle ktareho se reguluje
        Serial.print("Index cidla: ");
        Serial.println(topeni_index);
        Serial.print("Teplota regulovana: ");
        Serial.println(float(topeni_temp)/10);
        Serial.print("Teplota prostredi: ");
        Serial.println(teploty[topeni_index]);
        if ((float(topeni_temp)/10) > teploty[topeni_index]){

          if ((teploty[topeni_index] > 1) && (teploty[topeni_index] < 35)) {
            if (valid_temp_read[topeni_index] > 0) {
              digitalWrite(TOP_PIN, 0);
            } else {digitalWrite(TOP_PIN,1);}
          } else {
            digitalWrite(TOP_PIN,1);
          }
        } else {
          digitalWrite(TOP_PIN,1);
        }
      } else {
        digitalWrite(TOP_PIN,1);
      }
     odesli();
    }
    //rizeni PWM
    if ((time_z > cas_rozedneni)&&(time_z <cas_rozedneni+delka_rozedneni)&&(prechod==false)){
      //prvni vstup do rozedneni
      prechod=true;
      prvni_pruchod=true;
      uint16_t rozdil_pwm=pwmmax-pwmmin;
      uint32_t att_temp=(1000*(time_z-cas_rozedneni+delka_rozedneni))/rozdil_pwm;
      meziprechod.attach_ms(att_temp, to_day);
      pocitadlo_mezi=pwmmin;
      pocet_mezi=0;
    }
    if ((time_z > cas_setmeni)&&(time_z<cas_setmeni+delka_setmeni)&&(prechod==false)){
      //prvni vstup do setmeni
      prechod=true;
      prvni_pruchod=true;
      uint16_t rozdil_pwm=pwmmax-pwmmin;
      uint32_t att_temp=(1000*(time_z-cas_setmeni+delka_setmeni))/rozdil_pwm;
      meziprechod.attach_ms(att_temp, to_night);
      pocitadlo_mezi=pwmmax;
      pocet_mezi=0;
    }
    //prvni zapnuti
    if (prvni_pruchod==false){
      prvni_pruchod=true;
      if (time_z < cas_rozedneni){
        set_pwm(pwmmin);
        pocitadlo_mezi=pwmmin;
      }
      if (time_z >(cas_rozedneni+delka_rozedneni)){
        set_pwm(pwmmax);
        pocitadlo_mezi=pwmmax;
      }
      if (time_z > (cas_setmeni+delka_setmeni)){
        set_pwm(pwmmin);
        pocitadlo_mezi=pwmmin;
      }

    }

}






//------------------------podprogramy--------------------

void to_day(){
  set_pwm(pocitadlo_mezi);
  pocitadlo_mezi++;
  pocet_mezi++;
  if (pocet_mezi >= (pwmmax-pwmmin)){
    //konec prechodu
    prechod=false;
    meziprechod.detach();
    set_pwm(pwmmax);
  }
}

void to_night(){
  set_pwm(pocitadlo_mezi);
  pocitadlo_mezi--;
  pocet_mezi++;
  if (pocet_mezi >= (pwmmax-pwmmin)){
    //konec prechodu
    prechod=false;
    meziprechod.detach();
    set_pwm(pwmmin);
  }
}

//prevod teplomeru
void prevod_teplomeru() {
  //nacteni teplot  z cidel
  #define MAXTEMP 60
  #define MINTEMP -30
//  uint8_t present = 0;
  uint8_t data[12];

  for (uint8_t i = 0; i < (pocetT); i++) {

    if (ds_flag== false) {
      temp.reset();
      temp.select(addrT[i]);
      temp.write(0x44, 1);       // start conversion, with parasite power on at the end
    } else {
      temp.reset();
      temp.select(addrT[i]);
      temp.write(0xBE);
      for (int j = 0; j < 9; j++) {           // we need 9 bytes
        data[j] = temp.read();
      }
      if ( temp.crc8( data, 8) != data[8]) {
        Serial.print(i);
        Serial.print(" CRC is not valid!  ");
        Serial.print( temp.crc8( data, 8), HEX);
        Serial.print(" ");
        Serial.println (data[8], HEX);
      } else {
        int16_t raw = (data[1] << 8) | data[0];
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
        //test zda teplota je v ocekavanem rozsahu
        if ((MAXTEMP > (float)raw / 16.0)&&(MINTEMP< (float)raw / 16.0)){
          teploty[i] = (float)raw / 16.0;
          valid_temp_read[i]++;
        } else {
          Serial.println("Teplota mimo rozsah");
        }
      }
    }
  }
  if (ds_flag==false) {
    ds_flag=true;
  } else {
    ds_flag=false;
  }
}

void handleRoot() {
  digitalWrite(LED_BUILTIN, 0);
  if (server.hasArg("JASDEN")) {
    handleSubmit();
  } else {
  index();
  server.send(200, "text/html", INDEX_HTML);
  }
  digitalWrite(LED_BUILTIN, 1);
}
void handleNotFound(){
  digitalWrite(LED_BUILTIN, 0);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(LED_BUILTIN, 1);
}

void handleSubmit() {
  String argument = server.arg("JASDEN");
  uint16_t jasden_temp = argument.toInt();
  argument = server.arg("JASNOC");
  uint16_t jasnoc_temp = argument.toInt();
  argument = server.arg("RSH");
  uint8_t rsh = argument.toInt();
  argument = server.arg("RSM");
  uint8_t rsm = argument.toInt();
  argument = server.arg("RSTH");
  uint8_t rsth = argument.toInt();
  argument = server.arg("RSTM");
  uint8_t rstm = argument.toInt();
  argument = server.arg("SSH");
  uint8_t ssh = argument.toInt();
  argument = server.arg("SSM");
  uint8_t ssm = argument.toInt();
  argument = server.arg("SSTH");
  uint8_t ssth = argument.toInt();
  argument = server.arg("SSTM");
  uint8_t sstm = argument.toInt();
  argument = server.arg("tz");
  uint8_t tz_temp = argument.toInt();
  argument = server.arg("REG");
  uint8_t topeni_docas = (argument.toFloat()*10);
  //kontrola night a disabled
  for (uint8_t i=0; i<3; i++){
    String prom = "NOC";
    prom +=i;
    if (server.arg(prom)=="ano"){
      if (channel_night[i] == 0){
          EEPROM.write(EE_NOC+i, 1);
          EEPROM.commit();
      }
      channel_night[i]=1;
    } else {
      if (channel_night[i] == 1){
          EEPROM.write(EE_NOC+i, 0);
          EEPROM.commit();
      }
      channel_night[i]=0;
   }
   prom = "DIS";
   prom +=i;
   if (server.arg(prom)=="ano"){
     if (channel_disable[i] == 0){
         EEPROM.write(EE_DIS+i, 1);
         EEPROM.commit();
     }
     channel_disable[i]=1;

   } else {
     if (channel_disable[i] == 1){
         EEPROM.write(EE_DIS+i, 0);
         EEPROM.commit();
     }
     channel_disable[i]=0;
    }
  }
  //kontrola jasu pres den
  if ((jasden_temp < 1024)&&(jasden_temp > jasnoc_temp)){
    if (jasden_temp!=pwmmax){
      pwmmax=jasden_temp;
      write_to_eeprom(pwmmax, JASDEN);
    }
  }
  //kontrola  jasu pres noc
  if ((jasnoc_temp < 1024)&&(jasden_temp > jasnoc_temp)){
    if (jasnoc_temp!=pwmmin){
      pwmmin=jasnoc_temp;
      write_to_eeprom(pwmmin, JASNOC);
    }
  }
  //kontrola  casu rozedneni
  uint32_t cas_roz_temp=rsh*3600+rsm*60;
  uint32_t cas_set_temp=ssh*3600+ssm*60;
  uint32_t cas_roz_kon_temp=rsth*3600+rstm*60;
  uint32_t cas_set_kon_temp=ssth*3600+sstm*60;

  if ((cas_roz_temp<cas_set_temp)&&(cas_roz_temp<86400)){
    if (cas_roz_temp!=cas_rozedneni){
      cas_rozedneni=cas_roz_temp;
      write_to_eeprom(cas_rozedneni, ROZED);
      prechod=false;
      prvni_pruchod=false;
    }
  }
  //kontrola casu setmeni
  if ((cas_roz_temp<cas_set_temp)&&(cas_set_temp<86400)){
    if (cas_set_temp!=cas_setmeni){
      cas_setmeni=cas_set_temp;
      write_to_eeprom(cas_setmeni, STMIV);
      prechod=false;
      prvni_pruchod=false;
    }
  }
  //kontrola delky rozedneni
  if ((cas_roz_kon_temp > cas_roz_temp)&&(cas_roz_kon_temp<cas_set_temp)){
    uint32_t delka_temp=cas_roz_kon_temp-cas_roz_temp;
    delka_rozedneni=delka_temp;
    write_to_eeprom(delka_rozedneni, DELKAROZED);
    prechod=false;
    prvni_pruchod=false;
  }
  //kontrola delky stmivani
  if ((cas_set_kon_temp > cas_set_temp)&&(cas_roz_kon_temp<86400)){
    uint32_t delka_temp=cas_set_kon_temp-cas_set_temp;
    delka_setmeni=delka_temp;
    write_to_eeprom(delka_setmeni, DELKASTMIV);
    prechod=false;
    prvni_pruchod=false;
  }
  //kontrola casove zony
  if ((tz_temp > 0)&&(tz_temp <3)){
    if (tz_temp != timezone){
      timezone=tz_temp;
      EEPROM.write(TIMEZONE,timezone);
      EEPROM.commit();
      zjistiNTP();
      //prepocitani okamziku rozedneni a stmivani do abslutni casove  polohy
      if (ABS_TIME==1){
        if (timezone==1){
          //jdeme ze zony 2 -> 1
          //odecitame 1hod
          cas_rozedneni=cas_rozedneni-3600;
          cas_setmeni=cas_setmeni-3600;
          write_to_eeprom(cas_rozedneni, ROZED);
          write_to_eeprom(cas_setmeni, STMIV);
        }
        if (timezone==2){
          //jdeme ze zony 1 -> 2
          //pricitame 1 hod
          cas_rozedneni=cas_rozedneni+3600;
          cas_setmeni=cas_setmeni+3600;
          write_to_eeprom(cas_rozedneni, ROZED);
          write_to_eeprom(cas_setmeni, STMIV);
        }
      }
    }
  }
  if (topeni_docas != topeni_temp){
    topeni_temp = topeni_docas;
    EEPROM.write(TOPENI, topeni_temp);
    EEPROM.commit();
  }

  server.sendHeader("Connection", "close");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  index();
  server.send(200, "text/html", INDEX_HTML);



}
//vytvoreni souboru index.html
void index(){
INDEX_HTML = INDEX_HTML_1;
if (pocetT ==0){
  INDEX_HTML += "Nenalezeno žádné teplotní čidlo! <br>\n";
}
INDEX_HTML += "<table>";
for (uint8_t i=0; i< pocetT; i++){
  INDEX_HTML += "<tr><td>";
  if (i == topeni_index){
    INDEX_HTML += "<b>";
  }
  INDEX_HTML += "Čidlo " ;
  INDEX_HTML += i;
  INDEX_HTML += " id=";
  INDEX_HTML +=hash[i];
  INDEX_HTML += " teplota:  </td><td>";
  INDEX_HTML += teploty[i];
  if (i==topeni_index){
      INDEX_HTML += "</b>";
  }
  INDEX_HTML +="</td><td></td></<tr>\n";

}
INDEX_HTML +="<br>\n";
INDEX_HTML += "<tr><td>Regulovana teplota:</td><td><INPUT type=\"text\" size=4 name=\"REG\" value=";
INDEX_HTML += float(topeni_temp)/10;
INDEX_HTML +="></td><td></td></tr><tr></tr>\n";
INDEX_HTML += "<tr><td>Aktuální topení:</td><td>";
if (digitalRead(TOP_PIN) == 0) {
  INDEX_HTML += "Ano";
} else {
  INDEX_HTML += "Ne";
}
INDEX_HTML +="</td><td></td></tr><tr></tr>\n";
INDEX_HTML += "<tr><td>Aktuální úroveň svícení:</td><td>";
INDEX_HTML += pocitadlo_mezi;
INDEX_HTML +="</td><td></td></tr><tr></tr>\n";
INDEX_HTML +="<tr><td>Jas přes den:</td><td><INPUT type=\"text\" size=4 name=\"JASDEN\" value=";
INDEX_HTML += pwmmax;
INDEX_HTML += "></td><td></td></tr>\n";
INDEX_HTML +="<tr><td>Jas v noci:</td><td><INPUT type=\"text\" size=4 name=\"JASNOC\" value=";
INDEX_HTML += pwmmin;
INDEX_HTML += "></td><td></td></tr><tr></tr>\n";
INDEX_HTML +="<tr><td>Čas začátku rozednění: </td><td>";
INDEX_HTML +="<INPUT type=\"text\" size=2 name=\"RSH\" value=";
INDEX_HTML += uint8_t( cas_rozedneni/3600);
INDEX_HTML += ">:";
INDEX_HTML +="<INPUT type=\"text\" size=2 name=\"RSM\" value=";
INDEX_HTML += uint8_t((cas_rozedneni%3600)/60);
INDEX_HTML += "></td><td></td></tr>\n";
INDEX_HTML +="<tr><td>Čas konce rozednění: </td><td>";
INDEX_HTML +="<INPUT type=\"text\" size=2 name=\"RSTH\" value=";
INDEX_HTML += uint8_t( (cas_rozedneni+delka_rozedneni)/3600);
INDEX_HTML += ">:";
INDEX_HTML +="<INPUT type=\"text\" size=2 name=\"RSTM\" value=";
INDEX_HTML += uint8_t((cas_rozedneni+delka_rozedneni)%3600/60);
INDEX_HTML += "></td><td></td></tr>\n";
INDEX_HTML += "<tr></tr>";
//
INDEX_HTML +="<tr><td>Čas začátku setmění: </td><td>";
INDEX_HTML +="<INPUT type=\"text\" size=2 name=\"SSH\" value=";
INDEX_HTML += uint8_t( cas_setmeni/3600);
INDEX_HTML += ">:";
INDEX_HTML +="<INPUT type=\"text\" size=2 name=\"SSM\" value=";
INDEX_HTML += uint8_t((cas_setmeni%3600)/60);
INDEX_HTML += "></td><td></td></tr>\n";
INDEX_HTML +="<tr><td>Čas konce setmění: </td><td>";
INDEX_HTML +="<INPUT type=\"text\" size=2 name=\"SSTH\" value=";
INDEX_HTML += uint8_t( (cas_setmeni+delka_setmeni)/3600);
INDEX_HTML += ">:";
INDEX_HTML +="<INPUT type=\"text\" size=2 name=\"SSTM\" value=";
INDEX_HTML += uint8_t((cas_setmeni+delka_setmeni)%3600/60);
INDEX_HTML += "></td><td></td></tr>\n";
INDEX_HTML += "<tr></tr>";
//cas
INDEX_HTML += "<tr><td>Aktuální čas:</td><td>";
INDEX_HTML += uint8_t(time_z/3600);
INDEX_HTML +=":";
if (((time_z%3600)/60) < 10){
  INDEX_HTML +="0";
}
INDEX_HTML += uint8_t((time_z%3600)/60);
INDEX_HTML += "</td><td></td></tr>\n";
//nastaveni casove zony
INDEX_HTML += "<tr><td>Časová zóna UTC + </td><td><select name=\"tz\" size=\"1\">";
for (uint8_t i = 1; i < 3; i++){
    INDEX_HTML += "<option ";
    if (i == timezone) {
      INDEX_HTML += "selected";
    }

    INDEX_HTML += ">";
    INDEX_HTML += i;
    INDEX_HTML += "</option> \n ";
  }
INDEX_HTML +="</select></td><td></td></tr>\n";
INDEX_HTML +="<tr><td>Chybový kód</td><td>";
INDEX_HTML +=err;
INDEX_HTML +="</td><td></td></tr>\n";
INDEX_HTML +="<tr><td>Verze firmware</td><td>";
INDEX_HTML +=VERSION;
INDEX_HTML +="</td><td></td></tr>\n";
INDEX_HTML +="<tr><td>Číslo kanálu:</td><td>Pouze noc: </td><td> Povolen:</td></tr>\n<tr>";
for (uint8_t i=0; i<3;i++){
  INDEX_HTML +="<td>";
  INDEX_HTML +=i;
  INDEX_HTML +="</td><td>";
  INDEX_HTML +="<input  type=\"checkbox\" name=\"NOC";
  INDEX_HTML +=i;
  INDEX_HTML +="\" value=\"ano\"";
  if (channel_night[i]==1){
    INDEX_HTML +=" checked";
    }
  INDEX_HTML +="></td><td>";
  INDEX_HTML +="<input  type=\"checkbox\" name=\"DIS";
  INDEX_HTML +=i;
  INDEX_HTML +="\" value=\"ano\"";
  if (channel_disable[i]==1){
    INDEX_HTML +=" checked";
    }
  INDEX_HTML +=">";
  INDEX_HTML +="</td></tr>\n";
}
//dokonceni formulare, tlacitko submit
INDEX_HTML += INDEX_HTML_3;

}


//odeslani dat na PI
void odesli (){
      pocitadlo_odeslani++;
      long rssi = WiFi.RSSI();
      WiFiClient client;
      err=client.connect(host, httpPort);
      if (err != 1) {
        Serial.print("connection over DNS name failed error:");
        Serial.println(err);
        err=client.connect(host_IP, httpPort);
        if (err != 1) {
            Serial.print("connection over IP resolved failed error:");
            Serial.println(err);
            err=client.connect(host_IP_fallback, httpPort);
            if (err != 1) {
                Serial.print("connection over IP fallback failed error:");
                Serial.println(err);
                return;
              }
        }

      }
      String url = "http://";
      url += host;
      url += "/";
      url += SKRIPTV;
      url += "?rssi=";
      url += rssi;
      for (uint8_t i=0; i< pocetT; i++){
        url +="&hash";
        url +=i;
        url +="=";
        url +=hash[i];
        url += "&temp";
        url +=i;
        url +="=";
        url +=teploty[i];
        url +="&valid";
        url +=i;
        url +="=";
        url +=valid_temp_read[i];
        valid_temp_read[i]=0;
      }
      url += "&jas=";
      url += pocitadlo_mezi;
      url += "&sync=";
      url += sync;
      url += "&pocitadlo=";
      url += pocitadlo_odeslani;
      url += "&unixtime=";
      url += start_unixtime;
      url += "&topeni=";

      if (digitalRead(TOP_PIN)==0){
        url +="1";
      } else {
        url +="0";
      }
      url += "&ip=";
      tst = WiFi.localIP();
      url += tst[3];
      url += "&reg=";
      url += topeni_temp;
      url += "&time=";
      url += time_z;

      Serial.println (url);
      client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                   "Host: " + host + "\r\n" +
                   "Connection: close\r\n\r\n");
}
//pprg pro zjisteni casu startu
void sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}



uint8_t zjistiNTP(){
  unsigned long epoch; //unix time
  WiFi.hostByName(host_ntp, timeServerIP);
  Serial.println(timeServerIP);
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
    sync = 0;
    return 1;
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    if (epoch < 1570215826){ //epoch je divny
      Serial.println("Unix time je v minulosti = divne");
      sync = 0;
      return 1;
    }
    //ulozeni unixtime
    prubezny_unixtime=epoch;
    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    uint32_t hod = (epoch  % 86400L)/3600;
    Serial.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    uint32_t min =  (epoch  % 3600)/60;
    uint16_t sec = (epoch % 60);
    uint32_t timenow = hod*3600 + min*60 + sec ;
    Serial.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
    Serial.print("interni cas (UTC):");
    Serial.print(timenow);
    Serial.print(" ");
    Serial.print(hod);
    Serial.print(":");
    Serial.print(min);
    Serial.print(":");
    Serial.println(sec);
    time_z=timenow+timezone*3600;
    if (time_z >= 86400){
      time_z=time_z-86400;
    }
    sync = 1;
    return 0;
  }
}

//funkce pro  ticker nastaveni flagu pro synchro z NTP
void syncenable(){
  syncnow=0;
  //Serial.println("Syncnow ticker");
}

//funkce vypisu casu
void zobraz() {
  uint8_t hod=time_z/3600;
  uint8_t min=(time_z -hod*3600)/60;
  uint8_t sec=(time_z -hod*3600 - min*60);
  Serial.print("Local time: ");
  Serial.print(hod);
  Serial.print(":");
  Serial.print(min);
  Serial.print(":");
  Serial.println(sec);
}
//minutovy interval
void set_minuta(){
  flag_minuta=1;
}

//zapis uint32_t do EEPROM
void write_to_eeprom(uint32_t integer, uint8_t adresa){
  for (uint8_t i=0; i<4;i++){
    EEPROM.write((adresa+i), (uint8_t) integer);
    EEPROM.commit();
    //Serial.println((uint8_t)integer);
    integer= integer >> 8;
  }
}
//nacteni uint32_t z EEPROM
uint32_t read_from_eeprom(uint8_t adresa){
  uint8_t pole [4];
  for (uint8_t i=0; i<4;i++){
    pole[i]=EEPROM.read(adresa+i);
  }
  /*for (uint8_t test=0; test<4; test++){
    Serial.println(pole[test]);
  }*/
return ((uint32_t)((pole[3] << 24) | (pole[2] << 16) | (pole[1] << 8) | pole[0]));
}

void set_pwm (uint16_t pwm){
  //uint16_t pwm_temp=1023-pwm;
  //analogWrite(PWM1,pwm_temp);
  //analogWrite(PWM2,pwm_temp);
  //analogWrite(PWM3,pwm_temp);
  for (uint8_t i=0; i<3;i++){
    uint16_t pwm_temp=1023-pwm;
    if (channel_disable[i]==0){
      pwm_temp=1023;
    } else {
      if (channel_night[i]==1){
        if (pwm !=pwmmin){
          pwm_temp=1023;
        }
      }


    }
    //Serial.print("Aktualni hodnota pwm=");
    //Serial.println(pwm_temp);
    pwm_set_duty(pwm_temp*PWM_MULTIPLY, i);
  }
  //pwm_set_duty(pwm_temp, 0); //D5
  //pwm_set_duty(pwm_temp, 1); //D6
  //pwm_set_duty(pwm_temp, 2); //D7
  pwm_start();
  //Serial.println(" ");
}

//vyhledani teplotnich cidel
void search_temp (void) {
  for (int i = 0; i < POCET_DS; i++ ) {
    if ( !temp.search(addrT[i])) {
      addrT[i][0] = 0;
      addrT[i][1] = 0;
      addrT[i][2] = 0;
      addrT[i][3] = 0;
      addrT[i][4] = 0;
      addrT[i][5] = 0;
      addrT[i][6] = 0;
      addrT[i][7] = 0;
      hash[i] = addrT[i][0] ^ addrT[i][1] ^ addrT[i][2] ^ addrT[i][3] ^ addrT[i][4] ^ addrT[i][5] ^ addrT[i][6] ^ addrT[i][7];
      break;
    }
    else {
      pocetT++;
      hash[i] = addrT[i][0] ^ addrT[i][1] ^ addrT[i][2] ^ addrT[i][3] ^ addrT[i][4] ^ addrT[i][5] ^ addrT[i][6] ^ addrT[i][7];
      Serial.print("Hash cidla ");
      Serial.print(hash[i]);
      Serial.println();
      //hledame cidlo podle ktereho se reguluje
      if (hash[i] == TOPENI_HASH) {
        //nalezli jsme regulacni cidlo
        topeni_index=i;
        Serial.print("Nalezeno regulacni cidlo, index;" );
        Serial.print(topeni_index);
        Serial.print(" Hash:");
        Serial.println(hash[i]);
      }

    }
    delay(700);
  }
  Serial.print("Nalezeno cidel teploty: ");
  Serial.println(pocetT);
}
