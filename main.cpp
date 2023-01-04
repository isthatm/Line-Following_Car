#include "mbed.h"
#include "bbcar.h"
#include "MQTTClient.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"

WiFiInterface *wifi;

Ticker servo_ticker;
Ticker servo_feedback_ticker;

Thread carThread(osPriorityHigh);
EventQueue car_queue;

Thread measureThread;
EventQueue measure_queue;

//Car
PwmIn servo0_f(D12), servo1_f(D11); // servo0 - left ; servo1 - right
PwmOut servo0_c(D9), servo1_c(D10); 
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);

//Laser ping
DigitalInOut pin8(D8);
parallax_ping  ping1(pin8);

// QTI sensors
BusInOut qti_pin(D4,D5,D6,D7);
parallax_qti qti1(qti_pin);

volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;

const char *topic = "Car"; 

int pattern; double objD;
int a; 
int objectDetection = 1; // there is no object ahead by default
int prepTask = 0;
float inita0 = 0; float inita1 = 0;
double d,s;
float dis;

const float PI = 3.13159;
//const float R = 5.1; //distance between wheels -> radius of rotaion
const float rotP = 32.04424507;
float obj[2], w[2];
int i = 0;
int rotatingSpeed = 45; int carSpeed = 32;

int flag = 0;

void carStatus(){
            //distance travelled
            double d0, d1;
            d0 = 6.5*PI*(abs(car.servo0.angle-inita0)/360);
            d1 = 6.5*PI*(abs(car.servo1.angle-inita1)/360);
            s = ( (d0+d1) / 2 ); // speed
            inita0 = car.servo0.angle; //update current wheel angle for next call
            inita1 = car.servo1.angle;
            d = d + s;   
}

void widthCalculation (double d, double currentWheelAngle, double initWheelAngle ){
    float wheelAngle, rotAngle, p, theta;
    wheelAngle =  abs(currentWheelAngle) - abs(initWheelAngle);
    p = 6.5*PI*( abs(rotAngle) / 360 );
    theta = (p / rotP)*360; //
    rotAngle = (theta*PI) / 180; // deg to rad
    w[i] = sin(rotAngle)*d;
    i++;
}

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    //printf(msg);
    ThisThread::sleep_for(1s);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    //printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown> *client1) {
  MQTT::Message message;
  char buff[100];
  sprintf(buff,"Distance travelled: %.2f (cm), Speed: %.2f(cm/s), pattern: %d, ", d, s, pattern);
  //sprintf(buff,"Distance travelled: %.2f (cm), pattern: %d", d, pattern);
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void *)buff;
  message.payloadlen = strlen(buff) + 1;
  int rc = client1->publish(topic, message);

  printf("rc:  %d\r\n", rc);
  printf("[Pubished]: %s\r\n", buff);
}

void close_mqtt() { 
    closed = true; 
}

void obsMeasure(){
    double d1, d2, obsMeasureAngle0;
    obsMeasureAngle0 = car.servo1.angle;
    while(1) {
        d1 = (float)ping1;
        if (d1 > 20){
            car.rotate(rotatingSpeed);
        }
        else{
            car.stop();
            break;
        }
        ThisThread::sleep_for(10ms);
    }
    widthCalculation(d1, car.servo1.angle, obsMeasureAngle0);
    ThisThread::sleep_for(500ms);
    
    // 2nd object
    while(1) {
        pattern = (int)qti1;
        if (pattern != 0b0110){
            car.rotate(-rotatingSpeed);
        }
        else {
            car.stop();
            obsMeasureAngle0 = car.servo1.angle; 
            ThisThread::sleep_for(1s);
            break;
        }
    }
    while(1){
        d2 = (float)ping1;
        if (d2 > 20){
            car.rotate(-rotatingSpeed);
        }
        else{
            car.stop();
            break;
        }
        ThisThread::sleep_for(10ms);
    }
    
    widthCalculation(d2 , car.servo1.angle, obsMeasureAngle0);
    
    while(1){
       pattern = (int)qti1;
       if ( pattern == 0b0110){
           car.stop();
           break;
       }
       else{
           car.rotate(rotatingSpeed);
       }
   }
   if ( w[0] + w[1] > 15){
       objectDetection = 1; // the gap length is wide enough, it's safe to go
       ThisThread::sleep_for(500ms);
    }
}

void carDriving(){
    if(objectDetection){
      pattern = (int)qti1;
          flag = 1;
          switch (pattern) {
            case 0b1000: car.turn(carSpeed, -0.1); a = 0; break;
            case 0b1100: car.turn(carSpeed, -0.5); a = 0; break;
            case 0b0100: car.turn(carSpeed, -0.7); a = 0; break;
            case 0b0110: car.goStraight(25); a = 0; break;
            case 0b0010: car.turn(carSpeed, 0.7); a = 0; break;
            case 0b0011: car.turn(carSpeed, 0.5); a = 0; break;
            case 0b0001: car.turn(carSpeed, 0.1); a = 0; break;
            case 0b0111: { // TASK 1: turning left, pattern 7
                if (a==0){
                    car.stop();
                    a++;
                    ThisThread::sleep_for(1s); 
                    break;
                }
                else{
                    car.goStraight(30);
                    ThisThread::sleep_for(1s);
                    car.turn(30, 0.1);
                    ThisThread::sleep_for(2s); 
                    break;
                }
            }
            case 0b1110: {
                prepTask = 1;
                break;
            }
            case 0b1111: { //TASK 3: FINISHING THE LOOP 
                car.stop();
                a = 2;
                break;
            }
            
            case 0b0000: {// TASK 2:OBJECT DETECTION
                if(a==0 && prepTask == 1){
                    flag = 0;
                    car.goStraight(carSpeed);
                    ThisThread::sleep_for(1500ms);
                    car.stop();
                    ThisThread::sleep_for(1s);
                    objectDetection = 0; a++;
                    ThisThread::sleep_for(1s);
                    measure_queue.call(obsMeasure);
                    prepTask = 0;
                    break;
                }
                else{
                    car.stop();
                    ThisThread::sleep_for(500ms);
                    break;
                }
            } 
            
            default: car.goStraight(carSpeed - 10); break;
          }
      }
}

int main() {

  wifi = WiFiInterface::get_default_instance();
  if (!wifi) {
    printf("ERROR: No WiFiInterface found.\r\n");
    return -1;
  }

  printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
  int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD,
                          NSAPI_SECURITY_WPA_WPA2);
  if (ret != 0) {
    printf("\nConnection error: %d\r\n", ret);
    return -1;
  }

  NetworkInterface *net = wifi;
  MQTTNetwork mqttNetwork(net);
  MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

  // TODO: revise host to your IP
  //const char *host = "192.168.137.1"; //pc
  const char *host = "192.168.43.84"; // phone
  const int port=1883;
  printf("Connecting to TCP network...\r\n");
  printf("address is %s/%d\r\n",host, port); // check setting

  int rc = mqttNetwork.connect(host, port); //(host, 1883);
  if (rc != 0) {
    printf("Connection error.");// -3011: restart mqtt mosquitto broker
    return -1;
  }
  printf("Successfully connected!\r\n");

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.MQTTVersion = 3;
  data.clientID.cstring = "Mbed"; 

  if ((rc = client.connect(data)) != 0) {
    printf("Fail to connect MQTT\r\n");
  }

  if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Failed to subscribe\r\n");
  }   

// *********************

  carThread.start(callback(&car_queue, &EventQueue::dispatch_forever));
  measureThread.start(callback(&measure_queue, &EventQueue::dispatch_forever));
  car_queue.call_every(20ms, &carDriving);
  car_queue.call_every(1s, &carStatus);
  car_queue.call_every(1s, &publish_message, &client);


// **********************
  while (1) {
        if (closed) break;
        client.yield(500);
        ThisThread::sleep_for(1s);
    }
  
    printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic)) != 0) {
        printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
        printf("Failed: rc from disconnect was %d\n", rc);
    }

    mqttNetwork.disconnect();
    printf("Successfully closed!\n");

    return 0;
}





