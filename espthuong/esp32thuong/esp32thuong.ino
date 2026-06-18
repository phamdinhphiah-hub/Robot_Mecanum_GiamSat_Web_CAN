#include <Arduino.h>
#include "driver/twai.h"

// ESP32 THUONG MOTOR NODE - DEBUG NO PANTILT V2
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

#define FL_EN 25
#define FL_IN1 26
#define FL_IN2 27
#define RL_EN 14
#define RL_IN1 12
#define RL_IN2 13
#define FR_EN 33
#define FR_IN1 32
#define FR_IN2 15
#define RR_EN 23
#define RR_IN1 18
#define RR_IN2 19

#define PWM_FREQ 1000
#define PWM_RES 10
#define PWM_MAX 1023
#define FL_REV 0
#define RL_REV 0
#define FR_REV 0
#define RR_REV 0

bool engineOn=false, emergencyStop=false;
int joyX=0, joyY=0, joyR=0;
uint8_t rxCounter=0;
uint32_t lastJoyTime=0,lastStatusSend=0,lastMotorPrint=0,lastAlivePrint=0;
const uint32_t JOY_TIMEOUT=300;

int16_t readInt16LE(const uint8_t *data,int offset){return (int16_t)(data[offset]|(data[offset+1]<<8));}
int limitValue(int v,int mn,int mx){if(v<mn)return mn;if(v>mx)return mx;return v;}
void motorRun(int enPin,int in1,int in2,int speed){speed=limitValue(speed,-PWM_MAX,PWM_MAX);int duty=abs(speed);if(speed>0){digitalWrite(in1,HIGH);digitalWrite(in2,LOW);ledcWrite(enPin,duty);}else if(speed<0){digitalWrite(in1,LOW);digitalWrite(in2,HIGH);ledcWrite(enPin,duty);}else{digitalWrite(in1,LOW);digitalWrite(in2,LOW);ledcWrite(enPin,0);}}
void driveMecanum(int fl,int rl,int fr,int rr){if(FL_REV)fl=-fl;if(RL_REV)rl=-rl;if(FR_REV)fr=-fr;if(RR_REV)rr=-rr;motorRun(FL_EN,FL_IN1,FL_IN2,fl);motorRun(RL_EN,RL_IN1,RL_IN2,rl);motorRun(FR_EN,FR_IN1,FR_IN2,fr);motorRun(RR_EN,RR_IN1,RR_IN2,rr);if(millis()-lastMotorPrint>250){lastMotorPrint=millis();Serial.print("MOTOR FL=");Serial.print(fl);Serial.print(" RL=");Serial.print(rl);Serial.print(" FR=");Serial.print(fr);Serial.print(" RR=");Serial.println(rr);}}
void stopCar(){driveMecanum(0,0,0,0);}
void driveJoystick(int x,int y,int r){x=limitValue(x,-PWM_MAX,PWM_MAX);y=limitValue(y,-PWM_MAX,PWM_MAX);r=limitValue(r,-PWM_MAX,PWM_MAX);int fl=y+x+r,fr=y-x-r,rl=y-x+r,rr=y+x-r;int mx=abs(fl);if(abs(fr)>mx)mx=abs(fr);if(abs(rl)>mx)mx=abs(rl);if(abs(rr)>mx)mx=abs(rr);if(mx>PWM_MAX){fl=fl*PWM_MAX/mx;fr=fr*PWM_MAX/mx;rl=rl*PWM_MAX/mx;rr=rr*PWM_MAX/mx;}driveMecanum(fl,rl,fr,rr);}

void setupCan(){twai_general_config_t g=TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN,CAN_RX_PIN,TWAI_MODE_NORMAL);g.rx_queue_len=30;g.tx_queue_len=10;twai_timing_config_t t=TWAI_TIMING_CONFIG_250KBITS();twai_filter_config_t f=TWAI_FILTER_CONFIG_ACCEPT_ALL();esp_err_t ret=twai_driver_install(&g,&t,&f);if(ret==ESP_OK)Serial.println("TWAI driver installed");else{Serial.print("TWAI install failed ret=");Serial.println((int)ret);while(1)delay(1000);}ret=twai_start();if(ret==ESP_OK)Serial.println("TWAI started");else{Serial.print("TWAI start failed ret=");Serial.println((int)ret);while(1)delay(1000);}}
void sendStatus(){twai_message_t msg={};msg.identifier=0x200;msg.extd=0;msg.rtr=0;msg.data_length_code=3;msg.data[0]=engineOn?1:0;msg.data[1]=emergencyStop?1:0;msg.data[2]=rxCounter;twai_transmit(&msg,pdMS_TO_TICKS(2));}
void handleJoystickMessage(const twai_message_t &msg){if(msg.data_length_code<8)return;joyX=readInt16LE(msg.data,0);joyY=readInt16LE(msg.data,2);joyR=readInt16LE(msg.data,4);uint8_t flags=msg.data[6];engineOn=flags&1;emergencyStop=flags&2;rxCounter++;Serial.print("RX JOY | X=");Serial.print(joyX);Serial.print(" Y=");Serial.print(joyY);Serial.print(" R=");Serial.print(joyR);Serial.print(" engine=");Serial.print(engineOn);Serial.print(" estop=");Serial.println(emergencyStop);if(emergencyStop){stopCar();return;}if(engineOn){driveJoystick(joyX,joyY,joyR);lastJoyTime=millis();}else stopCar();}
void handleSystemMessage(const twai_message_t &msg){if(msg.data_length_code<1)return;uint8_t cmd=msg.data[0];Serial.print("RX SYSTEM CMD=");Serial.println(cmd);if(cmd==1){engineOn=true;emergencyStop=false;lastJoyTime=millis();Serial.println("ENGINE ON");}else if(cmd==2){engineOn=false;emergencyStop=false;stopCar();Serial.println("ENGINE OFF");}else if(cmd==0){emergencyStop=true;stopCar();Serial.println("STOP / ESTOP");}rxCounter++;}
void receiveCan(){twai_message_t msg;bool hasJoy=false,hasSystem=false;twai_message_t latestJoy={},latestSystem={};while(twai_receive(&msg,0)==ESP_OK){Serial.print("CAN RX ID=0x");Serial.print(msg.identifier,HEX);Serial.print(" DLC=");Serial.println(msg.data_length_code);if(msg.identifier==0x100){latestJoy=msg;hasJoy=true;}else if(msg.identifier==0x102){latestSystem=msg;hasSystem=true;}}if(hasSystem)handleSystemMessage(latestSystem);if(hasJoy)handleJoystickMessage(latestJoy);}
void setupMotorPins(){pinMode(FL_IN1,OUTPUT);pinMode(FL_IN2,OUTPUT);pinMode(RL_IN1,OUTPUT);pinMode(RL_IN2,OUTPUT);pinMode(FR_IN1,OUTPUT);pinMode(FR_IN2,OUTPUT);pinMode(RR_IN1,OUTPUT);pinMode(RR_IN2,OUTPUT);ledcAttach(FL_EN,PWM_FREQ,PWM_RES);ledcAttach(RL_EN,PWM_FREQ,PWM_RES);ledcAttach(FR_EN,PWM_FREQ,PWM_RES);ledcAttach(RR_EN,PWM_FREQ,PWM_RES);stopCar();}
void setup(){Serial.begin(115200);delay(800);Serial.println();Serial.println("===== ESP32 MOTOR NODE FINAL DEBUG NO PANTILT V2 =====");setupMotorPins();setupCan();engineOn=false;emergencyStop=false;lastJoyTime=millis();Serial.println("READY FINAL DEBUG NO PANTILT V2");Serial.println("CAN: TX=5 RX=4");Serial.println("FL: EN=25 IN1=26 IN2=27");Serial.println("RL: EN=14 IN1=12 IN2=13");Serial.println("FR: EN=33 IN1=32 IN2=15");Serial.println("RR: EN=23 IN1=18 IN2=19");}
void loop(){receiveCan();if(engineOn&&!emergencyStop&&millis()-lastJoyTime>JOY_TIMEOUT)stopCar();if(millis()-lastStatusSend>500){lastStatusSend=millis();sendStatus();}if(millis()-lastAlivePrint>3000){lastAlivePrint=millis();Serial.print("Alive DEBUG NO PANTILT | engine=");Serial.print(engineOn);Serial.print(" estop=");Serial.print(emergencyStop);Serial.print(" rxCounter=");Serial.println(rxCounter);}}
