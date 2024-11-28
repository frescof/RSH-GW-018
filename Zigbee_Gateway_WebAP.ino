#define _IN_ADDR_T_DECLARED
#define LWIP_NETIF_HOSTNAME             1
// vedi anche netbios_opts.h


#include "sys_api.h"

#include <WiFi.h>
// #include <lwip/apps/netbiosns.h>
#include "wifi_conf.h"
#include "lwip_netconf.h"
#include <lwip/sockets.h>
// #include <platform_stdlib.h>

#include "serial_api.h"
#include "serial_ex_api.h"
// #include "inet.h"

#include <FlashStorage_RTL8720.h>

#include <OTA.h>
#include <AmebaMDNS.h>

 // #include <WebServer.h>

#define OTA_PORT 8082

#define SERVER_PORT 80            // Porta del server TCP
#define BUFFER_SIZE 512           // Dimensione buffer

#define WBRG1_LOGRX       AMB_D25 //PA8
#define WBRG1_LOGTX       AMB_D24 // PA7
#define WBRG1_RTS0        AMB_D1 // PA16
#define WBRG1_TX0         AMB_D0 // PA18
#define WBRG1_RX0         AMB_D8 // PA19
#define WBRG1_TX1         AMB_D4 // PA12
#define WBRG1_RX1         AMB_D3 // PA13
#define WBRG1_CTS0        AMB_D2  // PA17
#define WBRG1_PA28        AMB_D26 // PA28
#define WBRG1_PB7         AMB_D17// PB7
#define WBRG1_PB4         AMB_D14 // PB4
#define G01_LED_B         AMB_D11 // PB22
#define G01_LED_R         AMB_D6  //PA25
#define G01_BUTTON        AMB_D10 //PB23
#define UART2_REG_BASE    0x48012000
#define UART2         ((UART_TypeDef *) UART2_REG_BASE)


#define ZIGBEE_NRESET     WBRG1_PA28
#define ZIGBEE_BOOTLOAD   WBRG1_PB4

#define UART_TX_PIN    	PA_18	//UART0  TX
#define UART_RX_PIN    	PA_19	//UART0  RX
#define UART_RTS_PIN    PA_16	//UART0  RTS
#define UART_CTS_PIN    PA_17	//UART0  CTS


#define ZIGBEE_HOST_NAME "ZB_GW1"
#define MY_SSID "YourWifi"
#define MY_SSID_PASS "WifiPassword"

char ssid[25] = MY_SSID;         // your network SSID (name)
char pass[25] = MY_SSID_PASS;    // your network password (use for WPA, or use as key for WEP)

char txrx_buffer[BUFFER_SIZE];

WiFiServer server(80);

serial_t sobj;  // Oggetto per la gestione della UART


// Funzione principale
      
 void setup()
{
    // ----------------- INIT ----------------
    Serial.begin(115200);           // initialize serial communication
    while (!Serial) {}
    // Inizializza la UART
    uart_init();
    Serial.print("UART done\n");
    pinMode(ZIGBEE_NRESET, OUTPUT);digitalWrite(ZIGBEE_NRESET, HIGH);
    pinMode(ZIGBEE_BOOTLOAD, OUTPUT);digitalWrite(ZIGBEE_BOOTLOAD, HIGH);
    pinMode(G01_LED_R, OUTPUT);digitalWrite(G01_LED_R, HIGH);
    pinMode(G01_LED_B, OUTPUT); digitalWrite(G01_LED_B, LOW);
    pinMode(G01_BUTTON, INPUT_PULLUP);
    // ---------------- ssid e password da flash ------------------------
    get_credential_from_flash(); 
    // ------------------------------------------------------------------
    int but_click=button_return(G01_BUTTON,7000);
    switch(but_click) {
      case 3: // this is a request for OTA
          Serial.print("\n\r Request to OTA ...");
          digitalWrite(G01_LED_B, HIGH);
          digitalWrite(G01_LED_R, LOW);
          go_ota();
        break;
      case 2:
        // force the bootload mode of the zigbee module for update
        digitalWrite(ZIGBEE_NRESET, LOW);
        digitalWrite(ZIGBEE_BOOTLOAD, LOW);
        Serial.print("\n\r ZIGBEE_NRESET and ZIGBEE_BOOTLOAD set to LOW");
        delay(250);
        digitalWrite(ZIGBEE_NRESET, HIGH);
        Serial.print("\n\r ZIGBEE_NRESET set to HIGH");
        delay(500);
        digitalWrite(ZIGBEE_BOOTLOAD, HIGH);
        Serial.print("\n\r ZIGBEE_BOOTLOAD set to HIGH \n\r");
        for (int i=0;i<5;i+=1) {digitalWrite(G01_LED_R,LOW);digitalWrite(G01_LED_B,HIGH);delay(500);digitalWrite(G01_LED_B,LOW);digitalWrite(G01_LED_R, HIGH);delay(500);}
        break;
      case  1:
          Serial.print("\n\r Request to reboot uartburn\n\r");
          cmd_reboot_uartburn();
      case  0: if (strcmp(ssid,MY_SSID)!=0) break;
      case -1:
          digitalWrite(G01_LED_B, HIGH);
          Serial.print("\n\r Request to configure ...");
          activate_ap_mode();
          // salviamo le nuove credenziali
          save_credential_to_flash();
          digitalWrite(G01_LED_B, LOW);
          // --------------- al momento niente di meglio -------------------
          sys_reset(); 
        
      default:;
        // code block
    }
    
    // ---------------------- Main APP ---------------------------

    app_wifi_init();
    
   
      // Crea il task del server TCP
    /* if (xTaskCreate(tcp_server_task, "tcp_server_task", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
          Serial.print("Errore: Creazione del task TCP fallita!\n");
      } */
    // Serial.print("Task passata\n");
      // Avvio del scheduler FreeRTOS
    //  vTaskStartScheduler();
    
}

void app_wifi_init(){
  // so far did not find a better workaround to set he hostname
     extern struct netif xnetif[];
    // Inizializza il WiFi in modalità client (RTW_MODE_STA)
    LwIP_Init();
    delay(1000);
    
      Serial.print("\n\rWiFi off...");
      wifi_off();
      delay(1000);
      Serial.print("\n\rWiFi on ... ");
      xnetif[0].hostname=ZIGBEE_HOST_NAME;
     int result = wifi_on(RTW_MODE_STA);
     if (result != RTW_SUCCESS) {
        printf("[app_wfi_init] Wifi on error n.: %d\n", result);
      }
      // netbios_set_name("rtlweb");
      wifi_set_autoreconnect(1);
      Serial.print("\n\rStarting connection ...");
      // Connetti al WiFi (aggiungi qui le tue credenziali WiFi)
      if (wifi_connect(ssid, RTW_SECURITY_WPA2_AES_PSK, pass, strlen(ssid), strlen(pass), -1, NULL) == RTW_SUCCESS) {
          LwIP_DHCP(0, DHCP_START);  // Avvia il DHCP client per ottenere un IP
          Serial.print("\n\rWiFi connected...");
      } else {
          Serial.print("\n\r[app_wfi_init] Error: WiFi connect failded");
          for (int i=0;i<5;i+=1) {digitalWrite(G01_LED_R, LOW);delay(1000);digitalWrite(G01_LED_R, HIGH);delay(1000);}
          sys_reset();
      }

}

// Funzione per configurare la UART
void uart_init()
{
    serial_init(&sobj, UART_TX_PIN, UART_RX_PIN);
    serial_baud(&sobj, 115200);  // Imposta il baud rate a 115200
    serial_format(&sobj, 8, ParityNone, 1);  // 8 bit di dati, nessuna parità, 1 bit di stop
    serial_rx_fifo_level(&sobj, FifoLvHalf);
    //serial_set_flow_control(&sobj, FlowControlNone, UART_RTS_PIN, UART_CTS_PIN);    // Pin assignment is NOT ignored 
    serial_set_flow_control(&sobj, FlowControlRTSCTS, UART_RTS_PIN, UART_CTS_PIN);    // Pin assignment is NOT ignored 
}

void loop() {
  tcp_server_task(NULL);
  // if we get back here better reset the ZigBee module
  digitalWrite(ZIGBEE_BOOTLOAD, HIGH);
  digitalWrite(ZIGBEE_NRESET, LOW);
  vTaskDelay(500);
  digitalWrite(ZIGBEE_NRESET, HIGH);
  vTaskDelay(1500);
  if (wifi_get_last_error()!=RTW_SUCCESS) {
   vTaskDelay(1500);
   sys_reset();
  }
 
}


// Funzione per inviare dati dalla UART al client TCP
int uart_to_tcp(int client_fd)
{
    // char uart_buffer[BUFFER_SIZE];
    int bytes_read = 0;
    int bytetx=-1;

    // Legge i dati dalla UART verso TCP
    if (serial_readable(&sobj)) { 
           digitalWrite(G01_LED_B, HIGH);
           //Serial.print("\n\r->");
        }
    while (serial_readable(&sobj)) {
        txrx_buffer[bytes_read] = serial_getc(&sobj);
        //Serial.print(txrx_buffer[bytes_read],HEX);
        bytes_read++;
        if (bytes_read >= BUFFER_SIZE) {
            break;
        }
    }

    // Se ci sono dati letti dalla UART, inviali al client TCP
    if (bytes_read > 0) {
        bytetx=lwip_send(client_fd, txrx_buffer, bytes_read, 0);
    }
    digitalWrite(G01_LED_B, LOW);
    return bytetx;
}

// Funzione per ricevere dati dal client TCP e inviarli alla UART
int tcp_to_uart(int client_fd)
{
    // char tcp_buffer[BUFFER_SIZE];
    int bytes_received = lwip_recv(client_fd, txrx_buffer, BUFFER_SIZE, MSG_DONTWAIT);

    // Se ci sono dati ricevuti dal TCP, inviali alla UART
    if (bytes_received > 0) {
       digitalWrite(G01_LED_B, HIGH);
        //Serial.print("\n\r<-");
        for (int i = 0; i < bytes_received; i++) {
            serial_putc(&sobj,txrx_buffer[i]);
            //Serial.print(txrx_buffer[i],HEX);
        }
    }
    digitalWrite(G01_LED_B, LOW);
    return bytes_received;
}

// Funzione per gestire la connessione TCP e il bridge seriale
void tcp_serial_bridge(int client_fd)

{
    int ret = 0,  sock_err = 0;
		size_t err_len = sizeof(sock_err);
   
    Serial.print("Client connesso...\n\r");
    while (1) {
     
        // Gestisci i dati dal TCP alla UART
        ret=tcp_to_uart(client_fd);
        // Gestisci i dati dalla UART al TCP
        uart_to_tcp(client_fd);
        getsockopt(client_fd, SOL_SOCKET, SO_ERROR, &sock_err, &err_len);
        // Controlla se il client è ancora connesso
        // delay(1);
        if ((ret<=0) && ( (ret != -1) || ((sock_err!=EAGAIN) && (sock_err!=0)) )) {  
            
           Serial.print("\n\rRet = ["); Serial.print(ret,HEX); Serial.print("]");Serial.print(" sock_err = <"); Serial.print(sock_err); Serial.print(">\n\r");
        // if (recv(client_fd, NULL, 0, MSG_PEEK) == 0) {
      
            Serial.print("\n\rClient disconnesso\n\r");
            close(client_fd);
            break;
        }
        if (wifi_get_last_error()!=RTW_SUCCESS) break;

    }
}

// Funzione per inizializzare il server TCP
void tcp_server_task(void *param)
{
    if (param == NULL) {}
    struct sockaddr_in server_addr, client_addr;
    int server_fd =-1, client_fd=-1;
    socklen_t client_addr_size = sizeof(client_addr);
    Serial.print("\n\rMain task running ...");
    while ((wifi_get_last_error()!=RTW_NO_ERROR)) {vTaskDelay(500);}
    // Crea il socket TCP
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        Serial.print("Errore: Impossibile creare il socket\n");
        // vTaskDelete(NULL);
        return;
    }
    Serial.print("Socket ok ... ");
    // Configura l'indirizzo del server
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    // Associa il socket all'indirizzo e alla porta
    if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
        Serial.print("\n\rErrore: Bind fallito");
        if (server_fd>= 0) close(server_fd);
        // vTaskDelete(NULL);
        return;
    }
    Serial.print("Bind ok ...");
    // Imposta il socket per ascoltare le connessioni
    if (listen(server_fd, 1) != 0) {
        Serial.print("Error: Listen failure\n");
        if (server_fd>= 0) close(server_fd);
        // vTaskDelete(NULL);
        return;
    }

    Serial.print("\n\rServer TCP waiting on port: ");Serial.println(SERVER_PORT);

    while (1) {
        serial_set_flow_control(&sobj, FlowControlNone, UART_RTS_PIN, UART_CTS_PIN);    // Pin assignment is NOT ignored 
        // Accetta una nuova connessione da un client
        client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_size);
        if (client_fd >= 0) {
            // Avvia il bridge seriale con il client connesso
            tcp_serial_bridge(client_fd);
        } else {
            Serial.print("\n\rError: Accept connection failure");
        }
    }
    // Chiudi il socket del server (non si dovrebbe mai arrivare qui)
    if (client_fd>= 0) close(client_fd); 
    if (server_fd>= 0) close(server_fd);

  //  vTaskDelete(NULL);
}


void mini_web_server(){
 
 #undef read
 #undef write
 
 while(1) { 
    // Controlla se ci sono nuovi client connessi
  WiFiClient client = server.available();
  if (client.connected()) {
    Serial.println("New client connected");
    String currentLine = "";
    String postData = "";  // Dove immagazziniamo i dati POST
    bool isPost = false;
    int contentLength = 0;
      // Leggi la richiesta HTTP
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);

        // Raccogli la linea corrente della richiesta
        if (c == '\n') {
          // Se la riga è vuota, significa che l'intestazione HTTP è finita
          if (currentLine.length() == 0) {
            if (isPost && contentLength > 0) {
              // Se è una richiesta POST, inizia a leggere il corpo
              for (int i = 0; i < contentLength; i++) {
                char c = client.read();
                postData += c;  // Aggiungi i dati al buffer postData
              }
              break;  // Fine lettura del corpo POST
            } else {
              // Altrimenti, inviamo la pagina HTML (per GET)
              sendForm(client,ssid,pass);
              break;
            }
          }

          // Controlla se siamo in una richiesta POST
          if (currentLine.startsWith("POST")) {
            isPost = true;
          }

          // Controlla la lunghezza del contenuto nella richiesta POST
          if (currentLine.startsWith("Content-Length:")) {
            contentLength = currentLine.substring(16).toInt();
          }

          // Resetta la linea corrente
          currentLine = "";
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    

    // Se abbiamo ricevuto dati POST, li analizziamo
    if (isPost && postData.length() > 0) {
      Serial.println("Dati POST ricevuti:");
      Serial.println(postData);

      // Analizza i dati POST per ottenere i campi
      String campo1,campo2;
      campo1 = getValue(postData, "campo1=");
      campo2 = getValue(postData, "campo2=");
      strcpy(ssid, campo1.c_str());
      strcpy(pass, campo2.c_str());
     

      // Risposta al client con i dati ricevuti
      String response = "<html><body><h2>Received data:</h2>";
      response += "<p>Campo 1: " + campo1 + "</p>";
      response += "<p>Campo 2: " + campo2 + "</p>";
      response += "</body></html>";

      client.println("HTTP/1.1 200 OK");
      client.println("Content-type:text/html");
      client.println();
      client.print(response);
      client.stop();  // Chiude la connessione al client
      Serial.println("Client disconnected");
      break; // out of main loop
    }
    // give the web browser time to receive the data
    delay(10);
    client.stop();  // Chiude la connessione al client
    Serial.println("Client disconnected");
  }
 }
}


// Funzione di utilità per inviare la pagina HTML con il form
void sendForm(WiFiClient &client, String c1, String c2  ) {
  // Definisci i valori di default
  String defaultCampo1 = c1;
  String defaultCampo2 = c2;

  // Inizio della pagina HTML
  String html = "<!DOCTYPE html><html><head>";
  html += "<style>";
  
  // Stile CSS per layout responsivo
  html += "body { font-family: Arial, sans-serif; background-color: #f0f8ff; margin: 0; padding: 20px; }";
  html += "h1 { color: #333; text-align: center; font-size: 36px; margin-bottom: 20px; }";
  html += "form { width: 100%; max-width: 600px; margin: 0 auto; padding: 20px; background-color: #fff; border-radius: 15px; box-shadow: 0px 0px 20px rgba(0, 0, 0, 0.2); }";
  html += "input[type='text'] { width: 100%; padding: 15px; margin: 15px 0; border: 2px solid #ccc; border-radius: 10px; background-color: #e6f7ff; font-size: 18px; box-sizing: border-box; }";
  html += "input[type='submit'] { width: 100%; padding: 15px; background-color: #4CAF50; color: white; border: none; border-radius: 10px; font-size: 20px; cursor: pointer; margin-top: 20px; box-sizing: border-box; }";
  html += "input[type='submit']:hover { background-color: #45a049; }";
  
  // Media query per schermi più piccoli (mobile)
  html += "@media (max-width: 600px) {";
  html += "  h1 { font-size: 28px; }";  // Riduci dimensione del titolo su dispositivi piccoli
  html += "  form { padding: 15px; }";  // Riduci padding del form
  html += "  input[type='text'], input[type='submit'] { font-size: 16px; padding: 12px; }";  // Adatta campi di input
  html += "}";

  html += "</style>";
  html += "</head><body>";

  
  // Titolo della pagina
  html += "<h1>Zigbee Gateway WBRG1+Z3SL</h1>";
  html += "<p style='text-align:center;font-size:20px;'>Inserisci i dati della rete WIFI:</p>";
  
  // Il form HTML con campi di input
  html += "<form action='/submit' method='POST'>";
  html += "Nome rete Wifi (SSID): <input type='text' name='campo1' value='" + defaultCampo1 + "'><br>";
  html += "Password wifi: <input type='text' name='campo2' value='" + defaultCampo2 + "'><br>";
  html += "<input type='submit' value='Invia e reset'>";
  html += "</form>";

  // Fine del body e HTML
  html += "</body></html>";

  // Invia la risposta HTTP con il form e i valori di default
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println();
  client.print(html);
}


// Funzione di utilità per estrarre i valori dei campi POST
String getValue(String data, String fieldName) {
  int start = data.indexOf(fieldName);
  if (start == -1) return "";
  
  start += fieldName.length();
  int end = data.indexOf("&", start);
  if (end == -1) {
    end = data.length();
  }
  return data.substring(start, end);
}


void activate_ap_mode(){
char ap_ssid[] = "GW_Network";    // Set the AP SSID
char ap_pass[] = "GW_Password";        // Set the AP password
char channel[] = "6";               // Set the AP channel
int status = WL_IDLE_STATUS;        // Indicator of Wifi status
int ssid_status = 0;                // Set SSID status, 1 hidden, 0 not hidden


status = WiFi.apbegin(ap_ssid, ap_pass, channel, ssid_status);
  if (status== WL_CONNECTED) {
    printWifiData();printCurrentNet();
    server.begin();
    mini_web_server();
    delay(500);
    server.stop();
    delay(500);
    if (WiFi.disconnect()!=WL_DISCONNECTED) {Serial.print("Errore disconnesione AP...");}
    delay(1000);
  }
}


void printWifiData() {
    // print your WiFi IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print your subnet mask:
    IPAddress subnet = WiFi.subnetMask();
    Serial.print("NetMask: ");
    Serial.println(subnet);

    // print your gateway address:
    IPAddress gateway = WiFi.gatewayIP();
    Serial.print("Gateway: ");
    Serial.println(gateway);
    Serial.println();
}

void printCurrentNet() {
    // print the SSID of the AP:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print the MAC address of AP:
    byte bssid[6];
    WiFi.BSSID(bssid);
    Serial.print("BSSID: ");
    Serial.print(bssid[0], HEX);
    Serial.print(":");
    Serial.print(bssid[1], HEX);
    Serial.print(":");
    Serial.print(bssid[2], HEX);
    Serial.print(":");
    Serial.print(bssid[3], HEX);
    Serial.print(":");
    Serial.print(bssid[4], HEX);
    Serial.print(":");
    Serial.println(bssid[5], HEX);

    // print the encryption type:
    byte encryption = WiFi.encryptionType();
    Serial.print("Encryption Type:");
    Serial.println(encryption, HEX);
    Serial.println();
}

void get_credential_from_flash()

{ uint32_t address = 0, magic=0;
  // uint32_t number;
  Serial.print(F("\nStart FlashStoreAndRetrieve on ")); Serial.println(BOARD_NAME);
  Serial.println(FLASH_STORAGE_RTL8720_VERSION);

  Serial.print("FlashStorage length: ");
  Serial.println(FlashStorage.length());
  FlashStorage.get(address, magic);
  // Read the content of FlashStorage
  
  if (magic==0xABCDEF) {
  address=address+sizeof(magic);
  FlashStorage.get(address, ssid);
  address=address+sizeof(ssid);
  FlashStorage.get(address, pass);
  }
  // Print the current number on the serial monitor
  Serial.print("Get ssid = "); Serial.println(ssid);
  Serial.print("Get pass = "); Serial.println(pass);
  }
void save_credential_to_flash()
  { uint32_t address = 0, magic=0;
  magic=0xABCDEF;
  FlashStorage.put(address, magic);
  address=address+sizeof(magic);
  FlashStorage.put(address, ssid);
  address=address+sizeof(ssid);
  FlashStorage.put(address, pass);
  FlashStorage.commit();
  Serial.println("Done writing to FlashStorage. You can reset now");
  }

  // Intervalli di tempo
#define MAX_WAIT_FOR_BUTTON 8000
#define LONG_PRESS_TIME 2000  // 2 secondi per la pressione lunga
#define CLICK_TIMEOUT 500     // Timeout per determinare clic singoli/multipli
#define DEBOUNCE_CYCLES 5     // Numero di cicli necessari per confermare il cambio di stato

int button_return(uint32_t pin, unsigned long maxwait) {
  int clickCount = 0;            // Numero di clic rilevati
  bool isPressed = false;        // Stato attuale del pulsante
  unsigned long pressTime = 0;   // Tempo della pressione iniziale
  unsigned long releaseTime = 0; // Tempo del rilascio del pulsante
  int pressCounter = 0;          // Contatore per la pressione stabile
  int releaseCounter = 0;        // Contatore per il rilascio stabile

  unsigned long startTime=millis();
  while (millis()-startTime<maxwait) {
  bool reading = digitalRead(pin) == LOW; // Bottone premuto LOW
  unsigned long currentTime = millis();
 
  // Debouncing: conferma il cambio di stato solo se rimane stabile per alcuni cicli
  if (reading) {
    pressCounter++;
    releaseCounter = 0;
  } else {
    releaseCounter++;
    pressCounter = 0;
  }

  // Se il pulsante è premuto stabilmente
  if (pressCounter > DEBOUNCE_CYCLES && !isPressed) {
    isPressed = true;
    pressTime = currentTime;
    Serial.print("Pressed...");
  }
  // Se il pulsante è rilasciato stabilmente
  if (releaseCounter > DEBOUNCE_CYCLES && isPressed) {
    isPressed = false;
      clickCount++;
      releaseTime = currentTime;
      Serial.println("Released...");
  }
   unsigned long pressDuration = currentTime - pressTime;
   if ((pressDuration >= LONG_PRESS_TIME) && isPressed) {
      Serial.println("Premuta lunga rilevata!");
      clickCount = -1;
      break;
    } else {
      // Gestione clic multipli
      if (clickCount > 0 && (currentTime - releaseTime > CLICK_TIMEOUT)) {
        if (clickCount < 4) {
          Serial.print(clickCount);Serial.println(" click(s)");
          break;
        } 
        clickCount = 0; // Reset dopo la rilevazione dei clic
      }
    }
  }  
  return clickCount;
}




OTA ota;
MDNSService service("MyAmeba", "_arduino._tcp", "local", 5000);

void go_ota() {
 int status = WL_IDLE_STATUS;        // Indicator of Wifi status
 
   
    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
        Serial.print("[MAIN] Attempting to connect to SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP
        // network:
        status = WiFi.begin(ssid, pass);
        // wait 10 seconds for connection:
        delay(5000);
    }
    // you're connected now, so print out the status:
    printWifiStatus();

    // setup MDNS service to host OTA Server on the
    // Arduino Network Port
    beginMDNSService();

    // start connecting to OTA server and reboot 
    // with the new image
    ota.beginOTA(OTA_PORT);
}

void printWifiStatus() {
    // print the SSID of the network you're attached to:
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void beginMDNSService() {
    service.addTxtRecord("board", strlen("ameba_rtl8721d"), "ameba_rtl8721d");
    service.addTxtRecord("auth_upload", strlen("no"), "no");
    service.addTxtRecord("tcp_check", strlen("no"), "no");
    service.addTxtRecord("ssh_upload", strlen("no"), "no");

    printf("Start mDNS service\r\n");
    MDNS.begin();

    printf("register mDNS service\r\n");
    MDNS.registerService(service);
}


void cmd_reboot_uartburn()
{
        WDG_InitTypeDef WDG_InitStruct;
        u32 CountProcess;
        u32 DivFacProcess;

        FLASH_ClockSwitch(BIT_SHIFT_FLASH_CLK_XTAL, TRUE);

        UART_RxCmd(UART2, DISABLE);
        RCC_PeriphClockSource_UART(UART2, UART_RX_CLK_XTAL_40M);
        UART_RxMonitorCmd(UART2, DISABLE);
        UART_SetBaud(UART2, 115200);
        UART_RxCmd(UART2, ENABLE);

        FLASH_Write_Lock();
        BKUP_Set(0, BIT_UARTBURN_BOOT);

        Serial.println("\n\rRebooting ...\n\r");
        WDG_Scalar(5, &CountProcess, &DivFacProcess);
        WDG_InitStruct.CountProcess = CountProcess;
        WDG_InitStruct.DivFacProcess = DivFacProcess;
        WDG_Init(&WDG_InitStruct);
        WDG_Cmd(ENABLE);
}
