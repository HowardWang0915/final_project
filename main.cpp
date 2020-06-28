#include "mbed.h"
#include "bbcar.h"
#include "arm_math.h"
#include "FXOS8700CQ.h"
#include <math.h>
#include <stdlib.h>

#define bound 0.9
Serial pc(USBTX, USBRX);
Serial uart(D1, D0); // tx, rx for openmv
DigitalOut led1(LED1);
PwmOut pin9(D9), pin8(D8);  // servo
DigitalInOut pin7(D7);      // ultrasoundwave
DigitalIn pin10(D2), pin11(D3); // optical encoder
Serial xbee(D12, D11);              // xbee
Ticker servo_ticker;
Ticker encoder_ticker0;
Ticker encoder_ticker1;
BBCar car(pin8, pin9, servo_ticker);
// ACC
FXOS8700CQ acc(PTD9, PTD8, (0x1D<<1));

Thread thread1;

// openmv function
void recieve_thread();

int main() {
    // turning timer
    Timer t;
    t.start();
    // uart init
    uart.baud(9600);
    // ping, encoder
    parallax_ping  ping1(pin7);
    parallax_encoder encoder0(pin10, encoder_ticker0);
    parallax_encoder encoder1(pin11, encoder_ticker1);
    // the first part going straight
    /**************************************************************/
    xbee.printf("Go straight\r\n");
    xbee.printf("data matrix\r\n");
    car.goStraight(100);
    wait(4);
    while(1){
        if((float)ping1>20)  car.goStraight(100);
        else{
            led1 = 0;
            car.stop();
            break;
        }
        wait(.01);
    }
    // first part going straight end, start to turn left
    /************************************************************/
    wait(1);
    xbee.printf("Turn left\r\n");
    car.turn(-50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    wait(1);
    // turn finished go foward 
    /*************************************************************/
    car.goStraight(100);
    encoder0.reset();
    xbee.printf("Go Foward\r\n");
    while(encoder0.get_cm() < 40 && (float)ping1>20)
        wait_ms(50);

    car.stop();
    // reverse parking
    /****************************************************************/
    wait(1);
    car.reverseParking(-50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    wait(1);
    xbee.printf("%f meters to the wall\r\n", (float)ping1);
    xbee.printf("Reverse Parking\r\n");
    car.goStraight(-100);
    encoder0.reset();
    while(encoder0.get_cm() < 35)
        wait_ms(50);
    car.stop();
    wait(1);
    // get out parking space
    /****************************************************************/
    car.goStraight(100);
    encoder0.reset();
    while(encoder0.get_cm() < 28)
        wait_ms(50);
    car.stop();
    wait(1);
    // turn left and go straight
    car.turn(-50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    wait(1);
    car.goStraight(100);
    encoder0.reset();
    while(encoder0.get_cm() < 55  && (float)ping1>20)
        wait_ms(50);
    car.stop();
    wait(1);
    // turn right and do the detection
    xbee.printf("Turn Right\r\n");
    car.turn(50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    wait(1);
    car.goStraight(-70);
    wait(1);
    car.stop();
    // image detecting
    /*****************************************************************/
    xbee.printf("image classification\r\n");
    char s[21];
    sprintf(s, "MNIST");
    uart.puts(s);
    recieve_thread();
    xbee.printf("send\r\n");
    xbee.printf("Snapshot\r\n");
    wait(3);
    // getting out the fucking parking lot
    /****************************************************************/
    xbee.printf("Leaving the parking lot\r\n"); // turn right and do the detection
    car.goStraight(70);
    car.stop();
    wait(1);
    car.goStraight(100);
    encoder0.reset();
    while(encoder0.get_cm() < 10 && (float)ping1>20)
        wait_ms(50);
    car.stop();
    wait(1);
    car.turn(50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    wait(1);
    car.goStraight(100);
    encoder0.reset();
    while(encoder0.get_cm() < 70 && (float)ping1>20)
        wait_ms(50);
    car.stop();
    // turn 180 degrees
    /***************************************************************/
    xbee.printf("Leaving mission 1 area\r\n");
    wait(1);
    car.turn(50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    // mission two
    // after detecting, the second part of going straight
    /**************************************************/
    wait(1);
    xbee.printf("Go Straight\r\n");
    car.goStraight(100);
    wait(5);
    while(1){
        if((float)ping1>20)  car.goStraight(100);
        else{
            led1 = 0;
            car.stop();
            break;
        }
        wait(.01);
    }
    // turn right
    /*******************************************************/
    wait(1);
    xbee.printf("Turn Right\r\n");
    car.turn(50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    wait(1);
    // turn finished go foward 
    /*************************************************************/
    xbee.printf("Going into mission 2\r\n");
    car.goStraight(100);
    // wait(1);
    encoder0.reset();
    while(encoder0.get_cm() < 20 && (float)ping1>20)
        wait_ms(50);

    car.stop();
    // turn right
    /*******************************************************/
    xbee.printf("Going to the corresponding position\r\n");
    wait(1);
    car.turn(50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    wait(1);
    // detecting the ultrasoundwave
    car.goStraight(100);
    encoder0.reset();
    while(encoder0.get_cm() < 20 && (float)ping1>20)
        wait_ms(50);

    car.stop();
    wait(1);
    xbee.printf("Detecting Objects\r\n");
    car.turn(30, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 0.4 && encoder0.get_cm() + encoder1.get_cm() < 6);
    car.stop();
    wait(1);
   
    switch((int)ping1 % 4){
        case 0:
            xbee.printf("Regular Triangle\r\n");
            break;
        case 1:
            xbee.printf("Square\r\n");
            break;
        case 2:
            xbee.printf("Right Triangle\r\n");
            break;
        case 3:
            xbee.printf("W Shape\r\n");
            break;
    } 
    car.turn(-30, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 0.4 && encoder0.get_cm() + encoder1.get_cm() < 6);
    car.stop();
    wait(1);
    car.goStraight(-100);
    encoder0.reset();
    while(encoder0.get_cm() < 20 && (float)ping1>25)
        wait_ms(50);

    car.stop();
    wait(1);
    // turn left
    /*******************************************************/
    xbee.printf("Leaving mission 2\r\n");
    car.turn(-50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    // leave mission 2
    /**************************************************/
    wait(1);
    car.goStraight(100);
    while(1){
        if((float)ping1>20)  car.goStraight(100);
        else{
            led1 = 0;
            car.stop();
            break;
        }
        wait(.01);
    }
    wait(1);
    // turn right
    /*******************************************************/
    car.turn(50, 1);
    t.reset();
    encoder0.reset();
    encoder1.reset();
    while(t.read() < 1.2 && encoder0.get_cm() + encoder1.get_cm() < 12);
    car.stop();
    wait(1);
     // leave the map
    /**************************************************/
    car.goStraight(100);
    wait(20);
    xbee.printf("finished\r\n");
}

void recieve_thread(){
   while(1) {
      if(uart.readable()){
            char recv = uart.getc();
            xbee.printf("This is: ");
            xbee.putc(recv);
            xbee.printf("\r\n");
            break;
            // pc.putc(recv);
            // pc.printf("\r\n");
      }
   }
}