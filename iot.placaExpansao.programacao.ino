/**
   Biblioteca utilizada
*/
#include <avr/wdt.h>
#include <EEPROM.h>
#include "SPI.h"
#include "mcp_can.h"
#include <Wire.h>
#include "Adafruit_MCP23017.h"
#include "overCAN.h"

#define DEBUG
#define CENTRAL_ID 0x00

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

Adafruit_MCP23017 IO;

/**
   Função para resetar o programa
   Utiliza o WatchDog Timer
*/
void resetSensor() {
#if defined(DEBUG)
  Serial.println("Reiniciando...");
#endif
  wdt_enable(WDTO_15MS);
  while (1);
}

//Estrutura com as sensorConfigurações do módulo
struct {
  uint8_t endereco;
  uint16_t intervaloEnvio; //Intervalo entre o envio das leituras em segundos
} sensorConfig;

boolean entradas[8] = {0};
unsigned char saidas[8] = {0};

// ----------------------
// Contadores
// ----------------------
uint32_t msUltimoEnvio = 0;
uint32_t msUltimaLeitura = 0;
uint32_t intervaloLeituras  = 20;

// ----------------------
// Setup() & Loop()
// ----------------------

void setup()
{
#if defined(DEBUG)
  Serial.begin(115200);
  Serial.println(F(""));
  Serial.println(F("----------------------------------------"));
#endif

  IO.begin();// use default address 0
  for (char i = 8; i <= 15; i++) {
    IO.pinMode(i, INPUT);
  }

  for (char i = 0; i < 8; i++) {
    IO.pinMode(i, OUTPUT);
  }
  atualizaSaidas();

  leSensorConfig();
  //  if (sensorConfig.intervaloLeitura < 0 || sensorConfig.intervaloLeitura > 3600 ) {
  //    sensorConfig.intervaloLeitura = 0;
  //    salvaSensorConfig();
  //  }
  if (sensorConfig.intervaloEnvio < 0 || sensorConfig.intervaloEnvio > 3600 ) {
    sensorConfig.intervaloEnvio = 0;
    salvaSensorConfig();
  }
  if (sensorConfig.endereco == 0) {
    sensorConfig.endereco = 255;
    salvaSensorConfig();
  }

  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
#if defined(DEBUG)
    Serial.println(F("CAN BUS Shield init fail"));
    Serial.println(F(" Init CAN BUS Shield again"));
#endif
    delay(100);
  }
#if defined(DEBUG)
  Serial.println(F("Remota: CAN BUS init ok!"));
#endif


}

void loop() {

  //-------INICIO DA COMUNICAÇÃO------------
  // Verifica se recebeu uma mensagem
  if (!pacoteRecebido()) {
    // Se não recebeu, verifica se está na hora de enviar os dados
    if (millis() > msUltimoEnvio + (sensorConfig.intervaloEnvio * 1000)) {
      enviaDados();
      msUltimoEnvio = millis();
    }
  }
  //-------FIM DA COMUNICAÇÃO------------
  if (millis() > msUltimaLeitura + intervaloLeituras) {
    lerDados();
    msUltimaLeitura = millis();
  }


}

/**
   Le dados dos sensores
*/
void lerDados() {
//#if defined(DEBUG)
//  Serial.println("---------------------------");
//#endif
  for (char i = 0; i <= 7; i++) {
    //entradas
    entradas[i] = IO.digitalRead(i + 8);
//#if defined(DEBUG)
//    Serial.print(entradas[i]); Serial.print(" ");
//#endif
  }
//#if defined(DEBUG)
//  Serial.println();
//#endif
}

/**
   Atualiza estado das saídas
*/
void atualizaSaidas() {
#if defined(DEBUG)
  Serial.println("SAIDAS ");
#endif
  for (char i = 0; i < 8; i++) {
    IO.digitalWrite(i, saidas[i]);
#if defined(DEBUG)
    Serial.print("-->"); Serial.println(saidas[i]);
#endif
  }
#if defined(DEBUG)
  Serial.println("-------------------------------------------");
#endif
}

/**
   Le os sensorConfiguracoes salvas na meméria EEPROM
*/
void leSensorConfig() {
  EEPROM.get(0, sensorConfig);
#if defined(DEBUG)
  Serial.print(F("endereco="));
  Serial.println(sensorConfig.endereco);
  Serial.print(F("intervaloEnvio="));
  Serial.println(sensorConfig.intervaloEnvio);
#endif
}

/**
   Escreve na memória EEPROM a configuração do sensor
*/
void salvaSensorConfig() {
  EEPROM.put(0, sensorConfig);
  leSensorConfig();
}

/**
   Envia dados para a central
*/
void enviaDados() {
  unsigned char msg[2] = {0};

  //Converte os dados binarios em um unico numero de 8bits
  for (char i = 7; i >= 0; i--) {
    //entradas
    msg[0] = (msg[0] << 1);
    msg[0] += entradas[i];
    //saidas
    msg[1] = (msg[1] << 1);
//    Serial.print(saidas[i]);
//    Serial.print(" ");
    msg[1] += saidas[i];
  }

//#if defined(DEBUG)
//  Serial.println(F("\nEnviando..."));
//  Serial.println(msg[1]);
//  Serial.println("<<<<<<<<<<<<<<<<<<<<<");
//#endif

  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);
}

bool pacoteRecebido() {
  if (CAN.checkReceive() != CAN_MSGAVAIL) {
    return false;
  }
  unsigned char buf[8];
  unsigned char len = 0;

  struct {
    unsigned int canId;
    unsigned char comando;
    unsigned char msg[6];
  } canPkt;

  CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
#if defined(DEBUG)
  for (char j = 0; j < 8; j++) {
    Serial.print(buf[j]);
    Serial.print("   ");
  }
  Serial.println();
#endif
  if (CAN.getCanId() != CENTRAL_ID) {
#if defined(DEBUG)
    Serial.println(F("Mensagem nao enviada pela central"));
#endif
    return false;
  }

  canPkt.canId = buf[0];
  if (canPkt.canId != sensorConfig.endereco) {
#if defined(DEBUG)
    Serial.println(F("Esta mensagem nao e para este dispositivo"));
#endif
    return false;
  }

  canPkt.comando = buf[1];

  switch (canPkt.comando) {
    case CHANGE_ID: {
#if defined(DEBUG)
        Serial.println(F("Alterar ID"));
#endif
        if (buf[2] > 0) sensorConfig.endereco = buf[2];
        //salva novas configuracoes
        salvaSensorConfig();
        resetSensor();
      };
      break;

    case CHANGE_OUTPUT_STATE: {
        if (buf[2] > 7) return false;
#if defined(DEBUG)
        Serial.print("Saida["); Serial.print(buf[2]); Serial.print("]:"); Serial.println(buf[3]);
#endif
        if (buf[3] == 0 || buf[3] == 1) {
          saidas[buf[2]] = buf[3] == 1;
          atualizaSaidas();
          //IO.digitalWrite(buf[2], buf[3]);
        }
      };
      break;
#if defined(DEBUG)
    default:
      Serial.println(F("Comando nao reconhecido"));
#endif
  }
  return true;
}

void chartobin ( unsigned char c, unsigned char *s ) {
  char i;
  for ( i = 7; i >= 0; i-- )
  {
    s[i] = (c >> i) % 2;
  }
}
