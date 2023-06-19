#include <Ethernet.h>
#include <ArduinoJson.h>
#include <SPI.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 86, 2); //Laita laite samaan ali verkkoon ja anna oma yksilöllinen ip osoite
EthernetServer server(80);

EthernetClient client = server.available();
StaticJsonDocument<200> doc;
String message, device, command;

void sendMsg() {
  doc["device"] = device;
  doc["command"] = command;
  serializeJson(doc, message);
  client.println(message);
  loop();
}

void compareMSG(String device, String command) {
  if (device != "1") {
    loop();
  }

  else if (command == "test") {
      device = "0", command = "success";
  } 

  else { 
   device = "0", command = "invalid command";
  }

  sendMsg();
}


//purkaa JSON viestin sekä tarkistaa onko viesti oikeassa muodossa
void deserializeJSON(String message) {
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    device = "vietin purku epäonnistui: ";
    command = error.c_str();
    sendMsg();
  }

  String device = doc["device"];
  String command = doc["command"];

  compareMSG(device, command);
}


//alustaa ethernet yhteyden
void setup() {
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.begin(9600);
  Serial.println("Server started, waiting for client...");
}

void loop() {  
  if (client) {
    IPAddress clientIP = client.remoteIP();
    Serial.print("Uusi laite yhdistetty, IP osoite: ");
    Serial.println(clientIP);
    
    while (client.connected()) {
      while (client.available()) {
        //tallentaa viestin muuttujaan ja kirjoittaa viestin siihen.
        message = "";
        message = client.readStringUntil('\n');

        Serial.print("Received: ");
        Serial.println(message);

        deserializeJSON(message);
      }
    }
  }
  
  //Ilmoittaa jos ethernet yhteys on päällä tai pois päältä
  else if (Ethernet.linkStatus() == LinkON) {
    Serial.println("Ethernet connection established!");
  }

  else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet connection lost!");
  }
}
