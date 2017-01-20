/**
   Biblioteca utilizada
*/
#include <avr/wdt.h>
#include <EEPROM.h>
#include "SPI.h"
#include "mcp_can.h"
#include <Wire.h>
#include "ClosedCube_HDC1080.h"
#include "overCAN.h"

#define DEBUG
#define CENTRAL_ID 0x00

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

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
  //  uint16_t intervaloLeitura; //Intervalo entre as leituras em segundos
  uint16_t intervaloEnvio; //Intervalo entre o envio das leituras em segundos
} sensorConfig;

boolean entradas[8];
boolean saidas[8];

// ----------------------
// Contadores
// ----------------------
uint32_t msUltimoEnvio = 0;

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
      lerDados();
    }
  }
  //-------FIM DA COMUNICAÇÃO------------


}

/**
   Le dados dos sensores
*/
void lerDados() {
  for (char i = 7; i >= 0; i--) {
    //entradas
    entradas[i] = digitalRead(i + 3);
    //saidas
    saidas[i] = digitalRead(i);
  }
#if defined(DEBUG)
  Serial.println("ENTRADAS: ");
  for (char i = 0; i < 8; i++) {
    //entradas
    Serial.print("<--"); Serial.println(entradas[i]);
  }
  Serial.println("-------------------------------------------");
#endif
}

/**
   Atualiza estado das saídas
*/
void atualizaSaidas() {
#if defined(DEBUG)
  Serial.println("SAIDAS ");
  for (char i = 0; i < 8; i++) {
    //saidas
    Serial.print("-->"); Serial.println(saidas[i]);
  }
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
  unsigned char msg[2];

  //Converte os dados binarios em um unico numero de 8bits
  for (char i = 7; i >= 0; i--) {
    //entradas
    msg[0] = (msg[0] << 1);
    msg[0] += entradas[i];
    //saidas
    msg[1] = (msg[1] << 1);
    msg[1] += saidas[i];
  }

#if defined(DEBUG)
  Serial.println(F("Enviando..."));
  Serial.println(msg[0]);
  Serial.println("<<<<<<<<<<<<<<<<<<<<<");
#endif

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
        if (buf[3] == 0 || buf[3] == 1) {
          saidas[buf[2]] = buf[3];
          atualizaSaidas();
        }
      };
      break;
    default:
#if defined(DEBUG)
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
