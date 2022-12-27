#include "mbed.h"
#include "bbcar.h"
#include "MQTTClient.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"

WiFiInterface *wifi;

Ticker servo_ticker;
Ticker servo_feedback_ticker;

Thread carThread;
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

const char *topic = "Car status"; // TODO "Mbed"

int pattern;
int a = 0; int task = 0; int subTask = 0; int taskConfirm = 0; int confirmed;
float d0, d1, inita0, inita1;
float d; float s;
const float PI = 3.13159;
const float R = 10.2; //distance between wheels -> radius of rotaion
const float rotP = 64.08849013;
float obj[2]; float rotAngle[2];
float w1, w2;
float w = 0; 
double objD;


void carStatus(){// task = 1, task = 2, task = 4
            //distance travelled
            d0 = 6.5*PI*(abs(car.servo0.angle-inita0)/360);
            d1 = 6.5*PI*(abs(car.servo1.angle-inita1)/360);
            s = ( (d0+d1) / 2 ); // speed
            //printf("angle0: %d,  angle1: %d", car.servo0.angle, car.servo1.angle);
            //printf("111 Distance travelled from last task: %f\n", d[task]); 
            //printf("Distance travelled: %f, Task executed: %d", d, task);
            inita0 = car.servo0.angle; //update current wheel angle for next call
            inita1 = car.servo1.angle;
            d = d + s;    
}

void obsMeasure2(){
   float p2, theta2, dis2, alpha2, rotAngle2;
   double obsMeasureAngle0;

    while(1) {
        pattern = (int)qti1;
        if (pattern == 0b0110){
            obsMeasureAngle0 = car.servo1.angle; //- rotAngle[0];
        }
        dis2 = (float)ping1;
        if (dis2 > 20){
            //printf("d2: %f\n", dis2);
            car.turn(-45,0.1);
        }
        else{
            obj[1] = dis2;
            car.stop();
            //printf("Distance to object 2: %f\n", obj[1]);
            break;
        }
        ThisThread::sleep_for(10ms);
    }

    rotAngle[1] = abs(car.servo1.angle) - abs(obsMeasureAngle0); 
    p2 = 6.5*PI*( abs( rotAngle[1] ) / 360);
    theta2 = (p2 / rotP)*360; //
    alpha2 = 90 - theta2;
    rotAngle2 =(90 - alpha2)*PI / 180;
    w2 = abs(sin(rotAngle2)*obj[1]);
    w = w1 + w2;
    //printf("Distance to object 2: %f , w2: %f, wheel_angle: : %d, p2: %f, alpha2: %f, rotAngle2: %f\n", obj[1], w2, abs(car.servo1.angle) + 20, p2, alpha2, rotAngle2);
    //printf("Distance to object 2: %f\n", obj[1]);
    //printf("Rotangle: %f, initial angle: %f, current angle: %d\n", rotAngle[1], obsMeasureAngle0, car.servo1.angle);
    //printf("Distance between 2 objects: %f\n", w1+w2);
    
    while(1){
       pattern = (int)qti1;
       if (pattern == 0b0110){
           car.stop();
           break;
       }
       else{
           car.turn(40, 0.1);
       }
   }
   task++; //task = 3 -> not acutually a task, just a mark to indicate that TASK 2 has been done
   //distance_travelled(task, 1); //CALL HERE
   // to run the car again
}

void obsMeasure1(){
    float p1, theta1, dis1, alpha1, rotAngle1;
    double obsMeasureAngle0;
    obsMeasureAngle0 = car.servo1.angle;
    while(1) {
        dis1 = (float)ping1;
        if (dis1 > 20){
            //printf("d1: %f\n", dis1);
            car.turn(40,0.1);
        }
        else{
            obj[0] = dis1;
            car.stop();
            break;
        }
        ThisThread::sleep_for(10ms);
   }
    rotAngle[0] =  abs(car.servo1.angle) - abs(obsMeasureAngle0);
    p1 = 6.5*PI*( abs(rotAngle[0]) / 360);
    theta1 = (p1 / rotP)*360; //
    alpha1 = 90 - theta1;
    rotAngle1 = (90 - alpha1)*PI / 180;
    w1 = sin(rotAngle1)*obj[0];
    //printf("Distance to object 1: %f , w1: %f wheel_angle: %f\n", obj[0], w1, rotAngle[0]);
    //printf("Distance to object 1: %f \n", obj[0]);
    car.turn(-45,0.1);
    ThisThread::sleep_for(2s);
    measure_queue.call(&obsMeasure2);   
}


void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    //printf(msg);
    ThisThread::sleep_for(2000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    //printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown> *client1) {
  MQTT::Message message;
  char buff[100];
  sprintf(buff,"Distance travelled: %f (cm), Speed: %f (cm/s), Task executed: %d,  Object width: %f ", d, s, task, w); // client prints the message if confirmed = 1
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void *)buff;
  message.payloadlen = strlen(buff) + 1;
  int rc = client1->publish(topic, message);

  printf("rc:  %d\r\n", rc);
  printf("Pubished message: %s\r\n", buff);
}

void close_mqtt() { 
    closed = true; 
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
  const char *host = "192.168.43.84";
  const int port=1883;
  printf("Connecting to TCP network...\r\n");
  printf("address is %s/%d\r\n",host, port); // check setting

  int rc = mqttNetwork.connect(host, port); //(host, 1883);
  if (rc != 0) {
    printf("Connection error.");// -3011: open mqtt mosquitto broker
    return -1;
  }
  printf("Successfully connected!\r\n");

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.MQTTVersion = 3;
  data.clientID.cstring = "Car status"; // TODO Mbed

  if ((rc = client.connect(data)) != 0) {
    printf("Fail to connect MQTT\r\n");
  }

  if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
  }

   // ******* CAR DRIVING *******
  carThread.start(callback(&car_queue, &EventQueue::dispatch_forever));
  measureThread.start(callback(&measure_queue, &EventQueue::dispatch_forever));
  car_queue.call_every(5s, &publish_message, &client);
  car_queue.call_every(1s, &carStatus);
    
  inita0 = car.servo0.angle;
  inita1 = car.servo1.angle;

  car.goStraight(40);
    while(1) {

        if (task == 4){
            //distance_travelled(task, 1); // CALL HERE
            break;
        }

       pattern = (int)qti1;
       //printf("pattern: %d\n",pattern);

       if (task==1 && subTask == 1){
           objD = (float)ping1;
           //printf("Distance to object: %f", objD);
           if (objD < 11.5){ 
               //TASK 2: obstacle detetection, prev 13.5
               int rotaionCounter = 0;
               car.stop();
               while(1){
                   car.rotate(40);
                    if (rotaionCounter == 0){
                        ThisThread::sleep_for(1s);
                        rotaionCounter++;
                    }
                    int rotatingPattern;
                    rotatingPattern = (int)qti1;
                    //printf("rotating pattern: %d\n", rotatingPattern);
                    if (rotatingPattern == 6){
                        car.stop();
                        break;
                    }
                    ThisThread::sleep_for(10ms);
                }
                task++; // task = 2
                //distance_travelled(task, 1);
                //subTask++; // subTask = 3
                ThisThread::sleep_for(2s);
                car_queue.call(&obsMeasure1);     
            }
        }
       if (task != 2) { // if (task != 2) -> stop the car when excuting obstacle detection
        switch (pattern) {
            case 0b1000: car.turn(33, -0.1); a = 0; break;
            case 0b1100: car.turn(33, -0.4); a = 0; break;
            case 0b0100: car.turn(30, -0.7); a = 0; break;
            case 0b0110: car.goStraight(30); a = 0; break;
            case 0b0010: car.turn(30, 0.7); a = 0; break;
            case 0b0011: car.turn(33, 0.4); a = 0; break;
            case 0b0001: car.turn(33, 0.1); a = 0; break;
            case 0b0111: car.turn(33,0.3); a = 0; break; // special case for unflattened sections
            case 0b1001: car.turn(33,0.4); a = 0; break;
            case 0b1111: {
                if (a==0 && task == 3){
                    car.stop();
                    task++; //task = 4 (finished)
                    break;
                }
                else if (a==0 && task == 0) { // TODO: change a to 0
                    car.stop(); 
                    a++; task++;// task = 1 
                    ThisThread::sleep_for(1s);
                    break;
                }
                if(a==1 && task == 1){ //TASK 1: turn to the left branch
                    car.turn(31, 0.1);
                    ThisThread::sleep_for(1s);
                    //subTask++; 
                    break;
                }

                if (task == 1 && subTask == 0){
                    car.goStraight(30);
                    subTask++; // subTask = 1: turn on laserPing
                    break;
                }
                /*
                else if(a==1 && task == 4){
                    car.stop();
                    break;
                }
                */
            
                else { // make car go again 
                    car.goStraight(30);
                    break;
                }
            }
            default: car.goStraight(28);
        }
     }
     ThisThread::sleep_for(10ms);
    }
    // ******* END *******

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






