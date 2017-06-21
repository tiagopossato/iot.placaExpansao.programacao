enum TP
{
  ENTRADA_DIGITAL = 00, //3200
  SAIDA_DIGITAL = 01, //3201
  ENTRADA_ANALOGICA = 02, //3202
  ESPECIAL = 99 //9999
} TipoGrandezas;

enum G
{
  TEMPERATURA = 03, //3303
  UMIDADE_AR = 04, //3304
  UMIDADE_SOLO = 20, //3320
} Grandezas;

enum C
{  
  ONLINE = 98,
  ENDERECO = 97, 
  INTERVALO_ENVIO = 96
} Configuracoes;
