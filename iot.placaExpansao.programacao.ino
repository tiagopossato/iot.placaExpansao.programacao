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
#include "definicoes.h"

#define DEBUG
#define CENTRAL_ID 0x00

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);// Set CS pin

Adafruit_MCP23017 IO;

const unsigned char resetIO = 15; //pino ligado ao pino de reset do controlador de IO

bool triggerWdtState = false;
/**
   Função para resetar o programa
   Utiliza o WatchDog Timer
*/
void reiniciar() {
#if defined(DEBUG)
  Serial.println("Reiniciando...");
#endif
  //Como o Watchdog está habilitado para 15ms, trava o programa e espera o watchdog reiniciar
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
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  delay(2000);
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

  isOnline();
  //Inicializa o Watchdog
  wdt_enable(WDTO_250MS);
  digitalWrite(7, LOW);

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
    if (CAN.checkError() != 0) {
      reiniciar();
    }
  }
  wdt_reset();  //  reseta o watchdog
  digitalWrite(8, triggerWdtState);
  triggerWdtState = !triggerWdtState;

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
    atualizarSaida(i, 0);
  }
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
void atualizarSaida(unsigned char saida, unsigned char estado) {
#if defined(DEBUG)
  Serial.print("Saida ");
  Serial.print(saida);
  Serial.print(": ");
  Serial.println(estado);
#endif
  if (saida < 8) {
    if (estado == 0 || estado == 1) {
      IO.digitalWrite(saida, estado);
      saidas[saida] = estado;
    }
  }
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

void isOnline() {
  unsigned char msgCfg[3] = {ESPECIAL, ONLINE, 1};
  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgCfg), msgCfg);

}

/**
   Envia configurações para a central
*/
void enviarConfig() {
  unsigned char msgCfg[3] = {ESPECIAL, INTERVALO_ENVIO, (uint8_t)sensorConfig.intervaloEnvio};
  CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgCfg), msgCfg);
}


void lerEntradaDigital(int8_t entrada) {
  if (entrada >= 0 && entrada < 8) {
    unsigned char msg[4] = {0};

    msg[0] = (uint8_t)ENTRADA_DIGITAL;
    msg[1] = (uint8_t)entrada;
    msg[2] = entradas[entrada];
    CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);
  }
}

void lerSaidaDigital(int8_t saida) {
  if (saida >= 0 && saida < 8) {
    unsigned char msg[4] = {0};

    msg[0] = (uint8_t)SAIDA_DIGITAL;
    msg[1] = (uint8_t)saida;
    msg[2] = saidas[saida];
    CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msg), msg);
  }
}

void enviarDados() {
  unsigned char msgDados[3];
  msgDados[0] = (uint8_t)ENTRADA_DIGITAL;

  for (uint8_t i = 0; i <= 7; i++) {
    msgDados[1] = i;
    msgDados[2] = (uint8_t)entradas[i];
    CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgDados), msgDados);
    delay(10);
  }

  msgDados[0] = (uint8_t)SAIDA_DIGITAL;

  for (uint8_t i = 0; i <= 7; i++) {
    msgDados[1] = i;
    msgDados[2] = (uint8_t)saidas[i];
    CAN.sendMsgBuf(sensorConfig.endereco, 0, sizeof(msgDados), msgDados);
    delay(10);
  }

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
    uint8_t idRede;
    uint8_t tipoGrandeza;
    uint8_t grandeza;
    int16_t valor;
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

  //Pega o ID do destino
  canPkt.idRede = buf[0];

  //confere se o ID é para este dispositivo
  if (canPkt.idRede != sensorConfig.endereco) {
    leSensorConfig();
#if defined(DEBUG)
    Serial.print(F("Esta mensagem nao e para este dispositivo: "));
    Serial.print(canPkt.idRede);
    Serial.print(F(" != "));
    Serial.println(sensorConfig.endereco);
#endif
    return false;
  }

  //Pega o comando enviado
  canPkt.tipoGrandeza = buf[1];
  canPkt.grandeza = buf[2];
  canPkt.valor = word(buf[3], buf[4]);
#if defined(DEBUG)
  Serial.print(F("Recebido: "));
  Serial.print(canPkt.idRede);
  Serial.print(F("/"));
  Serial.print(canPkt.tipoGrandeza);
  Serial.print(F("/"));
  Serial.print(canPkt.grandeza);
  Serial.print(F("/"));
  Serial.println(canPkt.valor);
#endif

  //verifica o tipo de grandeza
  switch (canPkt.tipoGrandeza) {
    /*--------entradas digitais-----------*/
    case ENTRADA_DIGITAL: {
#if defined(DEBUG)
        Serial.println(F("entradaDigital"));
#endif
        lerEntradaDigital(canPkt.grandeza);
        break;
      };
    /*--------fim das entradas digitais-----------*/

    /*--------saidas digitais-----------*/
    case SAIDA_DIGITAL: {
#if defined(DEBUG)
        Serial.println(F("saidaDigital"));
        Serial.println(canPkt.valor);
#endif
        canPkt.valor = canPkt.valor / 100;
        if (canPkt.valor == -1) {
          lerSaidaDigital(canPkt.grandeza);
        } else {
          atualizarSaida(canPkt.grandeza, canPkt.valor);
        }
        break;
      };
    /*--------fim das saidas digitais-----------*/
    /*--------Especial----------*/
    case ESPECIAL: {
        /*--------Configuracoes----------*/
        if (canPkt.grandeza == ENDERECO) {
#if defined(DEBUG)
          Serial.println(F("Endereco"));
          Serial.println(canPkt.valor);
#endif
          sensorConfig.endereco = canPkt.valor / 100;
          //salva novas configuracoes
          salvarSensorConfig();
          reiniciar();
        }
        if (canPkt.grandeza == INTERVALO_ENVIO) {
#if defined(DEBUG)
          Serial.println(F("Enviar intervaloEnvio"));
#endif
          if (canPkt.valor / 100 == 0) {
            enviarConfig();
            break;
          }
#if defined(DEBUG)
          Serial.println(F("intervaloEnvio"));
          Serial.println(canPkt.valor);
#endif
          sensorConfig.intervaloEnvio = canPkt.valor / 100;
          //salva novas configuracoes
          salvarSensorConfig();
          reiniciar();

        }
        /*--------fim das Configuracoes-----------*/
        if (canPkt.grandeza == ONLINE) {
#if defined(DEBUG)
          Serial.println(F("ONLINE"));
          Serial.println(canPkt.valor);
#endif
          isOnline();
        }

        break;
      };

      /*--------fim do ESPECIAL-----------*/
  };
  return true;
}
