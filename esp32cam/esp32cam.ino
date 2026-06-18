#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"
#include "esp_camera.h"
#include "webpage.h"

// ESP32-S3 CAMERA NODE - DEBUG NO PANTILT V2
const char *WIFI_SSID = "Trang";
const char *WIFI_PASS = "0908866418";
const char *AP_SSID = "MECANUM_GSA_ROBOT";
const char *AP_PASS = "12345678";
WebServer server(80);

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13
#define CAM_VFLIP 1
#define CAM_HMIRROR 0

#define CAN_TX_PIN GPIO_NUM_47
#define CAN_RX_PIN GPIO_NUM_48

bool cameraOK=false, canOK=false, engineOn=false, emergencyStop=false;
int joyX=0, joyY=0, joyR=0;
uint32_t canTxCounter=0, canTxFailCounter=0, lastStatusPrint=0;
uint8_t motorRxCounter=0;

bool canTransmit(twai_message_t *msg, uint32_t waitMs=10){
  esp_err_t ret=twai_transmit(msg,pdMS_TO_TICKS(waitMs));
  if(ret==ESP_OK){canTxCounter++; return true;}
  canTxFailCounter++; Serial.print("CAN TX FAIL ret="); Serial.println((int)ret); return false;
}

void sendCanJoystick(){
  if(!canOK) return;
  twai_message_t msg={}; msg.identifier=0x100; msg.extd=0; msg.rtr=0; msg.data_length_code=8;
  int16_t x=joyX,y=joyY,r=joyR;
  msg.data[0]=x&0xFF; msg.data[1]=(x>>8)&0xFF;
  msg.data[2]=y&0xFF; msg.data[3]=(y>>8)&0xFF;
  msg.data[4]=r&0xFF; msg.data[5]=(r>>8)&0xFF;
  uint8_t flags=0; if(engineOn)flags|=0x01; if(emergencyStop)flags|=0x02; msg.data[6]=flags; msg.data[7]=0;
  bool ok=canTransmit(&msg,5);
  static uint32_t lastPrint=0;
  if(millis()-lastPrint>350){lastPrint=millis(); Serial.print("CAN JOY TX "); Serial.print(ok?"OK":"FAIL"); Serial.print(" | X="); Serial.print(joyX); Serial.print(" Y="); Serial.print(joyY); Serial.print(" R="); Serial.print(joyR); Serial.print(" engine="); Serial.println(engineOn);}
}

void sendCanSystem(uint8_t cmd){
  if(!canOK){Serial.println("CAN SYSTEM NOT SENT: CAN not OK"); return;}
  twai_message_t msg={}; msg.identifier=0x102; msg.extd=0; msg.rtr=0; msg.data_length_code=1; msg.data[0]=cmd;
  bool ok=canTransmit(&msg,20);
  Serial.print("CAN SYSTEM CMD="); Serial.print(cmd); Serial.print(" TX="); Serial.println(ok?"OK":"FAIL");
}

void pollCanStatus(){
  if(!canOK) return;
  twai_message_t msg;
  while(twai_receive(&msg,0)==ESP_OK){
    if(msg.identifier==0x200 && msg.data_length_code>=3){
      motorRxCounter=msg.data[2];
      if(millis()-lastStatusPrint>1000){lastStatusPrint=millis(); Serial.print("MOTOR STATUS RX | engine="); Serial.print(msg.data[0]); Serial.print(" estop="); Serial.print(msg.data[1]); Serial.print(" rxCounter="); Serial.println(msg.data[2]);}
    }
  }
}

void setupCan(){
  twai_general_config_t g_config=TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN,CAN_RX_PIN,TWAI_MODE_NORMAL);
  g_config.rx_queue_len=20; g_config.tx_queue_len=20;
  twai_timing_config_t t_config=TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config=TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t ret=twai_driver_install(&g_config,&t_config,&f_config);
  if(ret==ESP_OK) Serial.println("TWAI installed"); else {Serial.print("TWAI install failed ret="); Serial.println((int)ret); canOK=false; return;}
  ret=twai_start();
  if(ret==ESP_OK){Serial.println("TWAI started"); canOK=true;} else {Serial.print("TWAI start failed ret="); Serial.println((int)ret); canOK=false;}
}

bool initCamera(){
  camera_config_t config={};
  config.ledc_channel=LEDC_CHANNEL_0; config.ledc_timer=LEDC_TIMER_0;
  config.pin_d0=Y2_GPIO_NUM; config.pin_d1=Y3_GPIO_NUM; config.pin_d2=Y4_GPIO_NUM; config.pin_d3=Y5_GPIO_NUM; config.pin_d4=Y6_GPIO_NUM; config.pin_d5=Y7_GPIO_NUM; config.pin_d6=Y8_GPIO_NUM; config.pin_d7=Y9_GPIO_NUM;
  config.pin_xclk=XCLK_GPIO_NUM; config.pin_pclk=PCLK_GPIO_NUM; config.pin_vsync=VSYNC_GPIO_NUM; config.pin_href=HREF_GPIO_NUM; config.pin_sccb_sda=SIOD_GPIO_NUM; config.pin_sccb_scl=SIOC_GPIO_NUM; config.pin_pwdn=PWDN_GPIO_NUM; config.pin_reset=RESET_GPIO_NUM;
  config.xclk_freq_hz=20000000; config.pixel_format=PIXFORMAT_JPEG;
  if(psramFound()){Serial.println("PSRAM FOUND"); config.frame_size=FRAMESIZE_QVGA; config.jpeg_quality=12; config.fb_count=2; config.fb_location=CAMERA_FB_IN_PSRAM; config.grab_mode=CAMERA_GRAB_LATEST;}
  else{Serial.println("NO PSRAM"); config.frame_size=FRAMESIZE_QVGA; config.jpeg_quality=14; config.fb_count=1; config.fb_location=CAMERA_FB_IN_DRAM; config.grab_mode=CAMERA_GRAB_WHEN_EMPTY;}
  Serial.println("Camera init..."); esp_err_t err=esp_camera_init(&config);
  if(err!=ESP_OK){Serial.printf("Camera init failed 0x%x\n",err); return false;}
  sensor_t *s=esp_camera_sensor_get();
  if(s){Serial.print("Camera PID: 0x"); Serial.println(s->id.PID,HEX); s->set_framesize(s,FRAMESIZE_QVGA); s->set_quality(s,12); s->set_vflip(s,CAM_VFLIP); s->set_hmirror(s,CAM_HMIRROR);}
  Serial.println("Camera OK"); return true;
}

void handleRoot(){server.send_P(200,"text/html",MAIN_page);}
void handleCmd(){
  if(!server.hasArg("c")){server.send(400,"text/plain","Missing command"); return;}
  String cmd=server.arg("c"); cmd.trim(); Serial.print("HTTP CMD RECEIVED: "); Serial.println(cmd);
  if(cmd=="ON"){engineOn=true; emergencyStop=false; sendCanSystem(1);} 
  else if(cmd=="OFF"){engineOn=false; emergencyStop=false; joyX=joyY=joyR=0; sendCanSystem(2); sendCanJoystick();}
  else if(cmd=="S"){joyX=joyY=joyR=0; emergencyStop=true; sendCanSystem(0); sendCanJoystick();}
  else if(cmd.startsWith("J:")){
    int c1=cmd.indexOf(','), c2=cmd.indexOf(',',c1+1);
    if(c1>0 && c2>c1){joyX=cmd.substring(2,c1).toInt(); joyY=cmd.substring(c1+1,c2).toInt(); joyR=cmd.substring(c2+1).toInt(); if(joyX==0&&joyY==0&&joyR==0)emergencyStop=false; sendCanJoystick();}
    else Serial.println("BAD J COMMAND");
  }
  server.send(200,"text/plain","OK");
}
void handleCanTest(){Serial.println("HTTP CANTEST RECEIVED"); engineOn=true; emergencyStop=false; sendCanSystem(1); delay(20); joyX=0; joyY=600; joyR=0; sendCanJoystick(); server.send(200,"text/plain","CANTEST SENT");}
void handleStatus(){String j="{"; j += "\"engine\":" + String(engineOn?1:0); j += ",\"estop\":" + String(emergencyStop?1:0); j += ",\"rx\":" + String(motorRxCounter); j += ",\"tx\":" + String(canTxCounter); j += ",\"fail\":" + String(canTxFailCounter); j += ",\"can\":" + String(canOK?1:0); j += "}"; server.send(200,"application/json",j);}
void handleJpg(){
  if(!cameraOK){server.send(500,"text/plain","Camera not initialized"); return;}
  camera_fb_t *fb=esp_camera_fb_get(); if(!fb){server.send(500,"text/plain","Camera capture failed"); return;}
  WiFiClient client=server.client(); client.println("HTTP/1.1 200 OK"); client.println("Content-Type: image/jpeg"); client.print("Content-Length: "); client.println(fb->len); client.println("Cache-Control: no-store"); client.println("Connection: close"); client.println(); client.write(fb->buf,fb->len); esp_camera_fb_return(fb);
}

bool connectWiFi(){WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID,WIFI_PASS); WiFi.setSleep(false); Serial.print("WiFi connecting"); unsigned long start=millis(); while(WiFi.status()!=WL_CONNECTED && millis()-start<12000){delay(500); Serial.print(".");} Serial.println(); if(WiFi.status()==WL_CONNECTED){Serial.println("WiFi connected"); Serial.print("STA IP: "); Serial.println(WiFi.localIP()); return true;} return false;}
void startAP(){WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID,AP_PASS); Serial.println("WiFi failed. Starting AP mode."); Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());}

void setup(){
  Serial.begin(115200); Serial.setDebugOutput(false); delay(1000);
  Serial.println(); Serial.println("===== ESP32-S3 CAM FINAL LOGO ROTATE NO PANTILT ====="); Serial.println("CAN S3: TX=GPIO47 RX=GPIO48");
  setupCan(); cameraOK=initCamera(); if(!connectWiFi()) startAP();
  server.on("/",HTTP_GET,handleRoot); server.on("/cmd",HTTP_GET,handleCmd); server.on("/cantest",HTTP_GET,handleCanTest); server.on("/status",HTTP_GET,handleStatus); server.on("/jpg",HTTP_GET,handleJpg);
  server.begin(); Serial.println("Server started"); Serial.println("Open browser:"); if(WiFi.getMode()==WIFI_STA && WiFi.status()==WL_CONNECTED) Serial.println(WiFi.localIP()); else Serial.println(WiFi.softAPIP());
  if(!cameraOK) Serial.println("WARNING: Camera init failed, but CAN/web still work.");
}
void loop(){server.handleClient(); pollCanStatus();}
