#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 256

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <avr/wdt.h>



//—— CONFIG —————————————————————————————————————————————————————
#define DEBUG           1
#define APN             "max4g"
#define MQTT_BROKER     "queue.laundritek.net"
#define MQTT_PORT       1883
#define TOPIC_PREFIX    "DL"
#define TIME_REQ_TOPIC  "time/rx"
#define HB_INTERVAL_MS  300000UL   // 5 min

//—— SERIAL & CLIENTS —————————————————————————————————————————————————
HardwareSerial SerialDebug(PA10, PA9);
HardwareSerial SerialModem(PA3, PA2);
SoftwareSerial SerialTX(PA5, PA4);

TinyGsm        modem(SerialModem);
TinyGsmClient  net(modem);
PubSubClient   mqtt(net);

//—— PINS & STATE —————————————————————————————————————————————————————
constexpr uint8_t RELAY_PIN = PB9, LED_PIN = PC13;
struct {
  uint32_t finishTime = 0;
  uint8_t  trigDelay  = 0;
  uint8_t  numPulses  = 0;
  uint8_t  len        = 0;
  uint8_t  buf[16];
} vend;

//—— IDs & TOPICS —————————————————————————————————————————————————————
char     machineID[13];
uint8_t  mac[6];
char     topicPub[32], topicSub[32];
char     timeRespTopic[32];

//—— TIME SYNC —————————————————————————————————————————————————————
bool     timeSynced    = false;
uint32_t currentEpoch  = 0;
uint32_t epochSyncMs   = 0;
uint32_t lastHbMs      = 0;

//—— DEBUG MACROS —————————————————————————————————————————————————————
#if DEBUG
  #define DBG(x)  SerialDebug.print(x)
  #define DBGL(x) SerialDebug.println(x)
#else
  #define DBG(x)
  #define DBGL(x)
#endif

//—— PROTOCOL CODES ———————————————————————————————————————————————————
enum : uint8_t { CMD_HEARTBEAT=1, CMD_VEND=2, CMD_RESET=3, CMD_BOOT=4 };
enum : uint8_t { DIR_DOWNLINK=1, DIR_UPLINK=2 };

//—— CRC16-CCITT —————————————————————————————————————————————————————
uint16_t crc16_ccitt(const uint8_t *data,size_t len){
  uint16_t crc=0xFFFF;
  while(len--){
    crc ^= (uint16_t)*data++<<8;
    for(int i=0;i<8;i++)
      crc = (crc & 0x8000) ? (crc<<1)^0x1021 : (crc<<1);
  }
  return crc;
}

//—— HEX MAP —————————————————————————————————————————————————————
static const char hexMap[]="0123456789ABCDEF";

//—— ENCODE / DECODE ———————————————————————————————————————————————————
void hexEncode(const uint8_t *in,size_t len,char *out){
  for(size_t i=0;i<len;i++){
    out[2*i]   = hexMap[(in[i]>>4)&0xF];
    out[2*i+1] = hexMap[in[i]&0xF];
  }
  out[2*len]='\0';
}
size_t hexDecode(const char *in,uint8_t *out,size_t maxO){
  size_t p=strlen(in)/2, cnt=min(p,maxO);
  auto cv=[](char c){return (c<='9'?c-'0':c-'A'+10)&0xF;};
  for(size_t i=0;i<cnt;i++)
    out[i]=(cv(in[2*i])<<4)|cv(in[2*i+1]);
  return cnt;
}

//—— GET CURRENT EPOCH —————————————————————————————————————————————
uint32_t getEpoch(){
  if(timeSynced) return currentEpoch + (millis()-epochSyncMs)/1000;
  else           return millis()/1000;
}

//—— PUBLISH HEX FRAME —————————————————————————————————————————————————
void publishFrame(const uint8_t *frame,size_t flen){
  char hexstr[2*64+1];
  hexEncode(frame,flen,hexstr);
  DBG("[PUB] "); DBGL(hexstr);
  mqtt.publish(topicPub,hexstr);
}

//—— BUILD & SEND FRAME —————————————————————————————————————————————————
void sendFrame(uint8_t cmd,uint8_t dir,const uint8_t *pl=nullptr,size_t pln=0){
  uint8_t buf[64]; size_t i=0;
  buf[i++]=0xFF; buf[i++]=cmd; buf[i++]=dir;
  memcpy(buf+i,mac,6); i+=6;
  uint32_t t=getEpoch();
  // 5-byte BE timestamp
  buf[i++]=0x00;
  buf[i++]=(t>>24)&0xFF;
  buf[i++]=(t>>16)&0xFF;
  buf[i++]=(t>> 8)&0xFF;
  buf[i++]= t      &0xFF;
  buf[i++]=vend.trigDelay;
  buf[i++]=vend.numPulses;
  buf[i++]=vend.len;
  if(pln&&pl){ memcpy(buf+i,pl,pln); i+=pln; }
  uint16_t crc=crc16_ccitt(buf+1,i-1);
  buf[i++]=crc>>8; buf[i++]=crc&0xFF;
  buf[i++]=0x0D;
  publishFrame(buf,i);
}
void respBoot()      { sendFrame(CMD_BOOT,     DIR_UPLINK); }
void respHeartbeat() { sendFrame(CMD_HEARTBEAT,DIR_UPLINK); }
void respReset()     { sendFrame(CMD_RESET,    DIR_UPLINK); }
void respVendOK()    { sendFrame(CMD_VEND,     DIR_UPLINK,vend.buf,vend.len); }

void doVend(){
  DBG("[ACT] VEND x"); DBGL(vend.numPulses);
  for(uint8_t k=0;k<vend.numPulses;k++){
    digitalWrite(RELAY_PIN,HIGH);
    delay(vend.trigDelay);
    digitalWrite(RELAY_PIN,LOW);
    delay(vend.trigDelay);
  }
  respVendOK();
}

//—— MQTT CALLBACK —————————————————————————————————————————————————————
void mqttCallback(char* topic, byte* payload, unsigned int len){
  // 1) Time‐response?
  if(strcmp(topic,timeRespTopic)==0){
    char buf[len+1]; memcpy(buf,payload,len); buf[len]=0;
    currentEpoch=strtoul(buf,nullptr,10);
    epochSyncMs=millis();
    timeSynced=true;
    DBGL("[TIME] synced to "+String(currentEpoch));
    // now send BOOT
    respBoot();
    return;
  }

  // 2) Hex‐frame
  if(len>64) return;
  char hexbuf[65]; memcpy(hexbuf,payload,len); hexbuf[len]=0;
  DBG("[IN ] "); DBGL(hexbuf);
  uint8_t frame[64]; size_t fl=hexDecode(hexbuf,frame,sizeof(frame));
  if(fl<1+1+1+6+5+1+1+1+2+1) return;
  if(frame[0]!=0xFF||frame[fl-1]!=0x0D) return;
  uint16_t rc=(frame[fl-3]<<8)|frame[fl-2];
  if(crc16_ccitt(frame+1,fl-4)!=rc){DBGL("[ERR] CRC");return;}
  uint8_t cmd=frame[1], dir=frame[2];
  if(dir!=DIR_DOWNLINK) return;

  // parse vend
  vend.finishTime=(uint32_t)frame[10]<<24
                |(uint32_t)frame[11]<<16
                |(uint32_t)frame[12]<<8
                | frame[13];
  vend.trigDelay=frame[14];
  vend.numPulses=frame[15];
  vend.len      =frame[16];
  memcpy(vend.buf,frame+17,vend.len);

  // forward SerialTX
  if(vend.len){
    DBG("[SERIALTX] ");
    for(uint8_t i=0;i<vend.len;i++){
      SerialTX.write(vend.buf[i]);
      SerialDebug.print(vend.buf[i],HEX);
      SerialDebug.print(" ");
    }
    DBGL("");
  }

  // handle cmd
  switch(cmd){
    case CMD_HEARTBEAT: respHeartbeat(); DBGL("[CMD] HEARTBEAT"); break;
    case CMD_RESET:     respReset();     DBGL("[CMD] RESET");     break;
    case CMD_VEND:      doVend();        DBGL("[CMD] VEND");      break;
    default: break;
  }
}

//—— CONNECT HELPERS —————————————————————————————————————————————————————
bool connectGPRS(){
  DBGL("[NET] waiting…");
  if(!modem.waitForNetwork()) return false;
  DBGL("[NET] attaching GPRS…");
  if(!modem.gprsConnect(APN)) return false;
  DBGL("[NET] GPRS OK");
  return true;
}
void connectMQTT(){
  mqtt.setCallback(mqttCallback);
  mqtt.setServer(MQTT_BROKER,MQTT_PORT);
  while(!mqtt.connected()){
    DBG("[MQTT] connecting…");
    mqtt.connect(topicPub);
    delay(200);
  }
  DBGL("[MQTT] connected");
  mqtt.subscribe(topicSub);
  mqtt.subscribe(timeRespTopic);
}

//—— SETUP & LOOP —————————————————————————————————————————————————————
void setup(){
  pinMode(RELAY_PIN,OUTPUT);digitalWrite(RELAY_PIN,LOW);
  pinMode(LED_PIN,OUTPUT);  digitalWrite(LED_PIN,HIGH);

  SerialDebug.begin(115200);
  SerialModem.begin(9600);
  SerialTX.begin(9600);
  delay(300);

  // derive MAC
  uint32_t u0=*(uint32_t*)0x1FFFF7E8;
  uint16_t u1=*(uint16_t*)0x1FFFF7EC;
  mac[0]=(u0>>24)&0xFF;mac[1]=(u0>>16)&0xFF;
  mac[2]=(u0>>8 )&0xFF;mac[3]=(u0    )&0xFF;
  mac[4]=(u1>>8 )&0xFF;mac[5]=(u1    )&0xFF;
  for(int i=0;i<6;i++){
    machineID[2*i]  =hexMap[(mac[i]>>4)&0xF];
    machineID[2*i+1]=hexMap[ mac[i]    &0xF];
  }
  machineID[12]=0;
  DBG("[ID] ");DBGL(machineID);

  snprintf(topicPub,sizeof(topicPub),"%s/%s/tx",TOPIC_PREFIX,machineID);
  snprintf(topicSub,sizeof(topicSub),"%s/%s/rx",TOPIC_PREFIX,machineID);
  snprintf(timeRespTopic,sizeof(timeRespTopic),"time/%s/tx",machineID);

  connectGPRS();
  connectMQTT();

  // request time
  DBG("[TIME] requesting…"); mqtt.publish(TIME_REQ_TOPIC,machineID,false);

  lastHbMs=millis();
}

void loop(){
  mqtt.loop();
  if(millis()-lastHbMs>HB_INTERVAL_MS){
    DBGL("[EVENT] Auto-HEARTBEAT");
    respHeartbeat();
    lastHbMs=millis();
  }
  if(vend.finishTime&&(millis()/1000)>=vend.finishTime){
    DBGL("[EVENT] Cycle completed");
    respVendOK();
    vend.finishTime=0;
  }
  digitalWrite(LED_PIN,((millis()>>9)&1)?LOW:HIGH);
}
