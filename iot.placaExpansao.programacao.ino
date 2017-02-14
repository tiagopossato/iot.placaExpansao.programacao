/**
   Bibliotecas utilizadas
   https://github.com/Seeed-Studio/CAN_BUS_Shield
   https://github.com/tiagopossato/overCAN
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
MCP_CAN CAN(SPI_CS_PIN);// Set CS pin

Adafruit_MCP23017 IO;

const unsigned char resetIO = 15; //pino ligado ao pino de reset do controlador de IO

/**
   Função para resetar o programa
   Utiliza o WatchDog Timer
*/
void reiniciar() {
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

  iniciarIO();

  leSensorConfig();
  //  if (sensorConfig.intervaloLeitura < 0 || sensorConfig.intervaloLeitura > 3600 ) {
  //    sensorConfig.intervaloLeitura = 0;
  //    salvarSensorConfig();
  //  }
  if (sensorConfig.intervaloEnvio < 0 || sensorConfig.intervaloEnvio > 3600 ) {
    sensorConfig.intervaloEnvio = 1;
    salvarSensorConfig();
  }
  if (sensorConfig.endereco == 0) {
    sensorConfig.endereco = 255;
    salvarSensorConfig();
  }

  if (CAN_OK != CAN.begin(CAN_100KBPS))              // init can bus : baudrate = 100k
  {
#if defined(DEBUG)
    Serial.println(F("CAN BUS Shield init fail"));
    Serial.println(F(" Init CAN BUS Shield again"));
#endif
    delay(250);
    reiniciar();
  }
#if defined(DEBUG)
  Serial.println(F("Remota: CAN BUS init ok!"));
#endif

  unsigned char msgCfg[1] = {ONLINE};
  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgCfg), msgCfg);

}

void loop() {
  //-------INICIO DA COMUNICAÇÃO------------
  // Verifica se recebeu uma mensagem
  if (!pacoteRecebido()) {
    // Se não recebeu, verifica se está na hora de enviar os dados
    if (millis() > msUltimoEnvio + (sensorConfig.intervaloEnvio * 1000)) {
      lerEntradas();
      enviarDados();
      //msUltimoEnvio atualizado na funcao enviarDados, para incluir nas contagens quando os dados são enviados por solicitação do mestre e não por tempo
    }
  }
  //-------FIM DA COMUNICAÇÃO------------
  if (millis() > msUltimaLeitura + intervaloLeituras) {
    lerEntradas();
    msUltimaLeitura = millis();
  }
}

void iniciarIO() {

  pinMode(resetIO, OUTPUT);

  digitalWrite(resetIO, LOW);
  delay(10);
  digitalWrite(resetIO, HIGH);

  IO.begin();// use default address 0

  for (char i = 8; i <= 15; i++) {
    IO.pinMode(i, INPUT);
  }

  for (char i = 0; i < 8; i++) {
    IO.pinMode(i, OUTPUT);
  }
  atualizarSaidas(0);
}

/**
   Le dados dos sensores
*/
void lerEntradas() {
  boolean flagEnviar = false;
  for (char i = 0; i <= 7; i++) {
    //entradas
    boolean valor = IO.digitalRead(i + 8);
    if (valor != entradas[i]) {
      entradas[i] = valor;
      flagEnviar = true;
    }
  }
  if (flagEnviar) enviarDados();
}

/**
   Atualiza estado das saídas
*/
void atualizarSaidas(unsigned char estados) {
#if defined(DEBUG)
  Serial.print("SAIDAS: ");
#endif
  for (char i = 0; i < 8; i++) {
    saidas[i] = bitRead(estados, i);
    IO.digitalWrite(i, saidas[i]);
#if defined(DEBUG)
    Serial.print(saidas[i]); Serial.print(" ");
#endif
  }
#if defined(DEBUG)
  Serial.println();
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
void salvarSensorConfig() {
  EEPROM.put(0, sensorConfig);
  leSensorConfig();
}

/**
   Envia configurações para a central
*/
void enviarConfig() {
  unsigned char msgCfg[2] = {0};

  msgCfg[0] = SEND_TIME;
  msgCfg[1] = sensorConfig.intervaloEnvio;

  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgCfg), msgCfg);

}

/**
   Envia dados para a central
*/
void enviarDados() {
  unsigned char msgDados[2];

  //entradas
  msgDados[0] = INPUT_1_STATE;
  msgDados[1] = 0;
  //Converte os dados binarios em um unico numero de 8bits
  for (char i = 7; i >= 0; i--) {
    bitWrite(msgDados[1], i, entradas[i]);
  }
  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgDados), msgDados);
  delay(10);

  //saidas
  msgDados[0] = OUTPUT_1_STATE;
  msgDados[1] = 0;
  //Converte os dados binarios em um unico numero de 8bits
  for (char i = 7; i >= 0; i--) {
    bitWrite(msgDados[1], i, saidas[i]);
  }
  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgDados), msgDados);

  msUltimoEnvio = millis();
}

bool pacoteRecebido() {
  //verifica se foi recebido pacote
  if (CAN.checkReceive() != CAN_MSGAVAIL) {
    return false;
  }

  unsigned char buf[8];
  unsigned char len = 0;

  //Estrutura que vai receber a mensagem
  struct {
    unsigned int canId;
    unsigned char comando;
    unsigned char msg[6];
  } canPkt;

  //Le a mensagem da placa
  CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

  //Pega o ID do transmissor
  if (CAN.getCanId() != CENTRAL_ID) {
#if defined(DEBUG)
    Serial.println(F("Mensagem nao enviada pela central"));
#endif
    return false;
  }

  //Pega o ID do receptor
  canPkt.canId = buf[0];
  //confere se o ID é para este dispositivo
  if (canPkt.canId != sensorConfig.endereco) {
    leSensorConfig();
#if defined(DEBUG)
    Serial.print(F("Esta mensagem nao e para este dispositivo: "));
    Serial.print(canPkt.canId);
    Serial.print(F(" != "));
    Serial.println(sensorConfig.endereco);
#endif
    return false;
  }

  //Pega o comando enviado
  canPkt.comando = buf[1];

  //verifica o comando
  switch (canPkt.comando) {
    case SEND_CONFIG: {
#if defined(DEBUG)
        Serial.println(F("SEND_CONFIG"));
#endif
        enviarConfig();
      };
      break;

    case SEND_DATA: {
#if defined(DEBUG)
        Serial.println(F("SEND_DATA"));
#endif
        enviarDados();
      };
      break;

    case CHANGE_ID: {
#if defined(DEBUG)
        Serial.println(F("CHANGE_ID"));
#endif
        if (buf[2] > 0) sensorConfig.endereco = buf[2];
        //salva novas configuracoes
        salvarSensorConfig();
        reiniciar();
      };
      break;

    case CHANGE_SEND_TIME: {
#if defined(DEBUG)
        Serial.println(F("CHANGE_SEND_TIME"));
#endif
        if (buf[2] >= 0) sensorConfig.intervaloEnvio = buf[2];
        //salva novas configuracoes
        salvarSensorConfig();
        enviarDados();
      };
      break;

    case CHANGE_OUTPUT_STATE: {
#if defined(DEBUG)
        Serial.println(F("CHANGE_OUTPUT_STATE"));
#endif
        atualizarSaidas(buf[2]);
      };
      break;

#if defined(DEBUG)
    default:
      Serial.println(F("Comando nao reconhecido"));
#endif
  }
  return true;
}
