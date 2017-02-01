/*
 * rc_code.xc
 *
 *  Created on: Nov 16, 2016
 *      Author: harrison/henderson
 */





#include <xs1.h>
#include <print.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <platform.h>
#include <bfs.h>


#include "mpu6050.h"
#include "i2c.h"


#define BAUD_RATE 9600
#define TICKS_PER_US (XS1_TIMER_HZ/1000000)
#define TICKS_PER_SEC (XS1_TIMER_HZ)
#define TICKS_PER_MS (XS1_TIMER_HZ/1000)

#define PWM_FRAME_TICKS (TICKS_PER_MS)

#define MESSAGE_SIZE 128
#define BIN2_ON 0b0010
#define BIN1_ON 0b1000
#define AIN2_ON 0b0100
#define AIN1_ON 0b0001

#define TIMER_MAX (pow(2,31) - 1)
#define M_PI acos(-1.0)



in port iButton = XS1_PORT_1C;
out port oButtonDriver = XS1_PORT_1B;
out port oLED = XS1_PORT_1A;
out port oSTB = XS1_PORT_1O;
out port oWiFiRX = XS1_PORT_1F;
in port iWiFiTX = XS1_PORT_1H;

out port oMotorPWMA = XS1_PORT_1P;
out port oMotorPWMB = XS1_PORT_1I;
out port oMotorControl = XS1_PORT_4D;

in port iEncoders = XS1_PORT_4C;

void encoder_task(in port iEncoder,chanend signal);
void encoder_counter(chanend signalIn, chanend signalOut, chanend touchChan);
void multi_motor_task(out port oLeftPWM, out port oRightPWM, out port oMotorControl, chanend in_motor_cmd_chan);
void uart_transmit_byte(out port oPort, char value, unsigned int baudrate);
char uart_receive_byte(in port iPort, unsigned int baudrate);
void toggle_port(out port oLED, unsigned int hz);
void uart_transmit_bytes(out port oPort, const char values[], unsigned int baudrate);
void uart_to_console_task(chanend trigger_chan, chanend in_motor_cmd_chan, chanend encoderChannel, chanend touchChan, chanend imuChan);
//void uart_to_console_task(chanend trigger_chan, chanend in_motor_cmd_chan);
void line(const char buffer[]);
void send_hello_world_program();
void output_task(chanend trigger_chan);
void send_wifi_setup();
void full_speed(chanend in_motor_cmd_chan);
void half_speed(chanend in_motor_cmd_chan);
void full_reverse(chanend in_motor_cmd_chan);
void half_reverse(chanend in_motor_cmd_chan);
void leftTurn(chanend in_motor_cmd_chan, int percent);
void rightTurn(chanend in_motor_cmd_chan, int percent);
void stop(chanend in_motor_cmd_chan);
void forwardPercent(chanend in_motor_cmd_chan, int percent);
void reversePercent(chanend in_motor_cmd_chan, int percent);
unsigned int computeDelay(unsigned int start, unsigned int stop);
void driveStraight(chanend in_motor_cmd_chan, int percent, float startYaw, float currentYaw);
void calculateMove(char* buffer,int rowStart, int colStart, int rowFinish, int colFinish, char prevDir);
void forwardForTime(chanend in_motor_cmd_chan, unsigned int delay);
void rightForTime(chanend in_motor_cmd_chan, unsigned int delay);
void leftForTime(chanend in_motor_cmd_chan, unsigned int delay);


struct IMU imu = {{
        on tile[0]:XS1_PORT_1L,                         //scl
        on tile[0]:XS1_PORT_4E,                         //sda
        400},};                                         //clockticks (1000=i2c@100kHz)

in  port butP = XS1_PORT_32A;                           //Button is bit 0, used to stop gracefully


typedef struct {
    int leftTicks;
    int rightTicks;
} encoderTicks;

typedef struct {
    float yaw;
    float pitch;
    float roll;
} ypr_t;

typedef struct {
    char data[MESSAGE_SIZE];
} message_t;


typedef struct {
    int left_duty_cycle;
    int right_duty_cycle;
} motor_cmd_t;

void forwardForTime(chanend in_motor_cmd_chan, unsigned int delay){
    timer tmr;
    unsigned int time;
    motor_cmd_t full_speed;
    full_speed.left_duty_cycle = 40;
    full_speed.right_duty_cycle = 40;
    motor_cmd_t stop;
    stop.left_duty_cycle = 0;
    stop.right_duty_cycle = 0;

    in_motor_cmd_chan <: full_speed;
    tmr :> time;
    time += delay;
    tmr when timerafter(time) :> void;
    in_motor_cmd_chan <: stop;

}

void rightForTime(chanend in_motor_cmd_chan, unsigned int delay){
    timer tmr;
    unsigned int time;
    motor_cmd_t full_speed;
    full_speed.left_duty_cycle = 40;
    full_speed.right_duty_cycle = -40;
    motor_cmd_t stop;
    stop.left_duty_cycle = 0;
    stop.right_duty_cycle = 0;

    in_motor_cmd_chan <: full_speed;
    tmr :> time;
    time += delay;
    tmr when timerafter(time) :> void;
    in_motor_cmd_chan <: stop;

}

void leftForTime(chanend in_motor_cmd_chan, unsigned int delay){
    timer tmr;
    unsigned int time;
    motor_cmd_t full_speed;
    full_speed.left_duty_cycle = -40;
    full_speed.right_duty_cycle = 40;
    motor_cmd_t stop;
    stop.left_duty_cycle = 0;
    stop.right_duty_cycle = 0;

    in_motor_cmd_chan <: full_speed;
    tmr :> time;
    time += delay;
    tmr when timerafter(time) :> void;
    in_motor_cmd_chan <: stop;

}



void full_speed(chanend in_motor_cmd_chan)
{
    motor_cmd_t full_speed;
    full_speed.left_duty_cycle = 100;
    full_speed.right_duty_cycle = 100;
    in_motor_cmd_chan <: full_speed;
}

void half_speed(chanend in_motor_cmd_chan)
{
    motor_cmd_t half_speed;
    half_speed.left_duty_cycle = 50;
    half_speed.right_duty_cycle = 50;
    in_motor_cmd_chan <: half_speed;
}

void full_reverse(chanend in_motor_cmd_chan)
{
    motor_cmd_t full_reverse;
    full_reverse.left_duty_cycle = -100;
    full_reverse.right_duty_cycle = -100;
    in_motor_cmd_chan <: full_reverse;
}

void half_reverse(chanend in_motor_cmd_chan)
{
    motor_cmd_t half_reverse;
    half_reverse.left_duty_cycle = -50;
    half_reverse.right_duty_cycle = -50;
    in_motor_cmd_chan <: half_reverse;
}

void stop(chanend in_motor_cmd_chan)
{
    motor_cmd_t stop;
    stop.left_duty_cycle = 0;
    stop.right_duty_cycle = 0;
    in_motor_cmd_chan <: stop;
}

void rightTurn(chanend in_motor_cmd_chan, int percent)
{
    motor_cmd_t rightTurn;
    int speed = 10;
    speed *= percent;
    rightTurn.left_duty_cycle = speed;
    rightTurn.right_duty_cycle = -1 * speed;
    in_motor_cmd_chan <: rightTurn;
}

void leftTurn(chanend in_motor_cmd_chan, int percent)
{
    motor_cmd_t leftTurn;
    int speed = 10;
    speed *= percent;
    leftTurn.left_duty_cycle = -1 * speed;
    leftTurn.right_duty_cycle = speed;
    in_motor_cmd_chan <: leftTurn;
}

void forwardPercent(chanend in_motor_cmd_chan, int percent){
    motor_cmd_t forward;
    forward.left_duty_cycle = percent;
    forward.right_duty_cycle = percent;
    in_motor_cmd_chan <: forward;
}

void reversePercent(chanend in_motor_cmd_chan, int percent){
    motor_cmd_t reverse;
    reverse.left_duty_cycle = -1 * percent;
    reverse.right_duty_cycle = -1 * percent;
    in_motor_cmd_chan <: reverse;
}



void output_task(chanend trigger_chan)
{
    while(1)
    {
        select
        {
        case trigger_chan :> message_t value :
            if(strcmp(value.data, "send_wifi_setup") == 0)
            {
                send_wifi_setup();
            }
            else
            {
                line(value.data);
            }
            break;

        }
    }
}

void send_wifi_setup()
{
    line("wifi.setmode(wifi.SOFTAP)");
    line("cfg={}");
    line("cfg.ssid=\"WifiWorksWHenItWants150\"");
    line("cfg.pwd=\"abc123abc123\"");
    line("cfg.ip=\"192.168.0.1\"");
    line("cfg.netmask=\"255.255.255.0\"");
    line("cfg.gateway=\"192.168.0.1\"");
    line("port = 9876");
    line("wifi.ap.setip(cfg)");
    line("wifi.ap.config(cfg)");
    //line("print(\"ESP8266 TCP to Serial Bridge v1.0 by RoboRemo\")");
    line("tmr.alarm(0,200,0,function()");
    //line("uart.setup(0, 9600, 8, 0, 1, 1)");
    line("srv=net.createServer(net.TCP, 28800)");
    line("srv:listen(port,function(conn)");
    line("uart.on(\"data\", 0, function(data)");
    line("conn:send(data) end, 0)");
    line("conn:on(\"receive\",function(conn,payload)");
    line("uart.write(0, payload)");
    line("end)");
    line("conn:on(\"disconnection\",function(c)");
    line("uart.on(\"data\")");
    line("end)");
    line("end)");
    line("end)");
}

void send_hello_world_program()
{
    line("gpio.mode(3, gpio.OUTPUT)");
    line("while 1 do");
    line("gpio.write(3, gpio.HIGH)");
    line("tmr.delay(1000000)");
    line("gpio.write(3, gpio.LOW)");
    line("tmr.delay(1000000)");
    line("end");
}


void line(const char buffer[])
{
    timer tmr;
    unsigned int time;
    tmr :> time;
    time += (TICKS_PER_SEC/8);
    tmr when timerafter(time) :> void;
    uart_transmit_bytes(oWiFiRX, buffer, BAUD_RATE);
    uart_transmit_bytes(oWiFiRX, "\r\n", BAUD_RATE);
}

//void uart_to_console_task(chanend trigger_chan, chanend in_motor_cmd_chan)
void uart_to_console_task(chanend trigger_chan, chanend in_motor_cmd_chan, chanend encoderChannel, chanend touchChan, chanend imuChan)
{
    char currChar;
    char buffer[64];
    int currElement = 0;
    const float HALFPI = (M_PI/2.0);
    const float PI = (M_PI);
    message_t m;
    strncpy(m.data, "send_wifi_setup", 20);

    while(1)
    {
        currChar = uart_receive_byte(iWiFiTX, BAUD_RATE);
        buffer[currElement] = currChar;
        currElement++;
        if(currElement == 63 || currChar == '\r' || currChar == '\n')
        {
            //printstrln(buffer);
            buffer[currElement] = '\0';
            if(strcmp(buffer, "lua: cannot open init.lua\r") == 0)
            {
                trigger_chan <: m;
            }

            else if(strcmp(buffer, "R\r") == 0)
            {
                //full_reverse(in_motor_cmd_chan);
                motor_cmd_t reverse;
                motor_cmd_t adjust;
                reverse.left_duty_cycle = -90;
                reverse.right_duty_cycle = -88;
                adjust = reverse;
                in_motor_cmd_chan <: reverse;
                float currentYaw;
                float startYaw;
                ypr_t results;
                imuChan :> results;
                startYaw = results.yaw;
                while(1){
                    imuChan :> results;
                    currentYaw = results.yaw;

                    if((currentYaw - startYaw) > 0){
                        adjust.right_duty_cycle -= 1;
                        in_motor_cmd_chan <: adjust;
                    }else{
                        in_motor_cmd_chan <: reverse;
                    }
                }
            }
            else if(strcmp(buffer, "r\r") == 0)
            {
                half_reverse(in_motor_cmd_chan);
            }
            else if(strcmp(buffer, "x\r") == 0)
            {
                stop(in_motor_cmd_chan);
            }
            else if(strcmp(buffer, "<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 1);
            }
            else if(strcmp(buffer, "<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 2);
            }
            else if(strcmp(buffer, "<<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 3);
            }
            else if(strcmp(buffer, "<<<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 4);
            }
            else if(strcmp(buffer, "<<<<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 5);
            }
            else if(strcmp(buffer, "<<<<<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 6);
            }
            else if(strcmp(buffer, "<<<<<<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 7);
            }
            else if(strcmp(buffer, "<<<<<<<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 8);
            }
            else if(strcmp(buffer, "<<<<<<<<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 9);
            }
            else if(strcmp(buffer, "<<<<<<<<<<\r") == 0)
            {
                leftTurn(in_motor_cmd_chan, 10);
            }
            else if(strcmp(buffer, ">\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 1);
            }
            else if(strcmp(buffer, ">>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 2);
            }
            else if(strcmp(buffer, ">>>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 3);
            }
            else if(strcmp(buffer, ">>>>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 4);
            }
            else if(strcmp(buffer, ">>>>>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 5);
            }
            else if(strcmp(buffer, ">>>>>>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 6);
            }
            else if(strcmp(buffer, ">>>>>>>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 7);
            }
            else if(strcmp(buffer, ">>>>>>>>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 8);
            }
            else if(strcmp(buffer, ">>>>>>>>>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 9);
            }
            else if(strcmp(buffer, ">>>>>>>>>>\r") == 0)
            {
                rightTurn(in_motor_cmd_chan, 10);
            }
            else if(strcmp(buffer, "?\r") == 0){
                char ticks[64];
                encoderTicks blah;
                ypr_t results;
                blah.leftTicks = 0;
                blah.rightTicks = 0;
                encoderChannel <: blah;
                encoderChannel :> blah;
                imuChan :> results;
                sprintf(ticks, "Left Ticks: %d  Right Ticks: %d ypr(%0.2f,%0.2f,%0.2f) \n", blah.leftTicks, blah.rightTicks, results.yaw, results.pitch,
                        results.roll);
                printstr(ticks);
            }
            else if(strcmp(buffer, ".\r") == 0){
                //Right Turn
                char ticks[64];
                float endYaw;
                int scenario;
                motor_cmd_t right;
                right.left_duty_cycle = 33;
                right.right_duty_cycle = -33;

                ypr_t results;
                imuChan :> results;
                if((results.yaw + HALFPI) >= (2.0 * PI)){
                    //gets distant past 0 you will need to go
                    float offset = (results.yaw + HALFPI) - (2.0 * PI);
                    endYaw = offset;
                    scenario = 2;
                }else{
                    endYaw = results.yaw + HALFPI;
                    scenario = 1;
                }

                endYaw *= 100.0;
                if(scenario == 1){
                    while((results.yaw * 100.0) < (endYaw - 10.0)){
                        //rightTurn(in_motor_cmd_chan, 4);
                        in_motor_cmd_chan <: right;
                        imuChan :> results;
                    }
                    stop(in_motor_cmd_chan);

                }else if(scenario == 2){
                    //takes you to 0
                    //100 * currentYaw < goal - 10 && 100*(currentYaw -2pi) < goal - 10
                    while((results.yaw + 0.02) < (2.0 * PI)){
                        //rightTurn(in_motor_cmd_chan, 4);
                        in_motor_cmd_chan <: right;
                        imuChan :> results;
                    }

                    imuChan :> results;
                    imuChan :> results;


                    //continues turning until you hit the point past 0
                    while(((results.yaw) * 100.0) < (endYaw - 10.0)){
                        //rightTurn(in_motor_cmd_chan, 4);
                        in_motor_cmd_chan <: right;
                        imuChan :> results;
                    }
                    stop(in_motor_cmd_chan);

                }
                stop(in_motor_cmd_chan);

            }
            else if(strcmp(buffer, ",\r") == 0){
                //left turn
                char ticks[64];
                float endYaw;
                int scenario;
                motor_cmd_t left;
                left.left_duty_cycle = -33;
                left.right_duty_cycle = 33;
                ypr_t results;
                imuChan :> results;
                if((results.yaw - HALFPI) <= 0){
                    //gets distant past 0 you will need to go
                    float offset = (results.yaw - HALFPI) + (2.0 * PI);
                    endYaw = offset;
                    scenario = 2;
                }else{
                    endYaw = results.yaw - HALFPI;
                    scenario = 1;
                }

                endYaw *= 100.0;
                if(scenario == 1){
                    while((results.yaw * 100.0) > (endYaw + 10.0)){
                        //leftTurn(in_motor_cmd_chan, 4);
                        in_motor_cmd_chan <: left;

                        imuChan :> results;
                    }
                    stop(in_motor_cmd_chan);

                }else if(scenario == 2){
                    //takes you to 0
                    //100 * currentYaw < goal - 10 && 100*(currentYaw -2pi) < goal - 10
                    while((results.yaw + 0.02) > 0){
                        //leftTurn(in_motor_cmd_chan, 4);
                        in_motor_cmd_chan <: left;
                        imuChan :> results;
                    }

                    imuChan :> results;
                    imuChan :> results;


                    //continues turning until you hit the point past 0
                    while(((results.yaw) * 100.0) > (endYaw + 10.0)){
                        //leftTurn(in_motor_cmd_chan, 4);
                        in_motor_cmd_chan <: left;
                        imuChan :> results;
                    }
                    stop(in_motor_cmd_chan);

                }
                stop(in_motor_cmd_chan);
            }
            else if(buffer[0] == 'F'){
                int counter = 0;
                for(int i = 1; buffer[i] != '\r'; i++){
                    counter ++;
                }

                if(counter == 0){
                    //full_speed(in_motor_cmd_chan);
                    motor_cmd_t straight;
                    motor_cmd_t adjust;
                    straight.left_duty_cycle = 90;
                    straight.right_duty_cycle = 88;
                    adjust = straight;
                    in_motor_cmd_chan <: straight;
                    float currentYaw;
                    float startYaw;
                    ypr_t results;
                    imuChan :> results;
                    startYaw = results.yaw;
                    while(1){
                        imuChan :> results;
                        currentYaw = results.yaw;

                        if((currentYaw - startYaw) > 0){
                            adjust.right_duty_cycle += 1;
                            in_motor_cmd_chan <: adjust;
                        }else{
                            in_motor_cmd_chan <: straight;
                        }
                    }
                }
                else if(counter == 1){
                    int mySpeed = buffer[1] - '0';
                    forwardPercent(in_motor_cmd_chan, mySpeed);
                }else if(counter == 2){
                    int tens = buffer[1] - '0';
                    int ones = buffer[2] - '0';
                    tens *= 10;
                    forwardPercent(in_motor_cmd_chan, tens + ones);
                }
            }
            else if(buffer[0] == 'f'){
                int counter = 0;
                for(int i = 1; buffer[i] != '\r'; i++){
                    counter ++;
                }

                if(counter == 0){
                    half_speed(in_motor_cmd_chan);

                }
                else if(counter == 1){
                    int mySpeed = buffer[1] - '0';
                    forwardPercent(in_motor_cmd_chan, mySpeed);
                }else if(counter == 2){
                    int tens = buffer[1] - '0';
                    int ones = buffer[2] - '0';
                    tens *= 10;
                    forwardPercent(in_motor_cmd_chan, tens + ones);
                }


            }
            else if(buffer[0] == 'T'){
                int counter = 0;
                for(int i = 1; buffer[i] != '\r'; i++){
                    counter ++;
                }

                if(counter == 0){
                    full_speed(in_motor_cmd_chan);

                }
                else if(counter == 1){
                    int mySpeed = buffer[1] - '0';
                    forwardPercent(in_motor_cmd_chan, mySpeed);

                }else if(counter == 2){
                    int tens = buffer[1] - '0';
                    int ones = buffer[2] - '0';
                    tens *= 10;
                    forwardPercent(in_motor_cmd_chan, tens + ones);

                }
                touchChan <: 1;

            }
            else if(buffer[0] == 't'){
                int counter = 0;
                for(int i = 1; buffer[i] != '\r'; i++){
                    counter ++;
                }

                if(counter == 0){
                    full_reverse(in_motor_cmd_chan);

                }
                else if(counter == 1){
                    int mySpeed = buffer[1] - '0';
                    reversePercent(in_motor_cmd_chan, mySpeed);

                }else if(counter == 2){
                    int tens = buffer[1] - '0';
                    int ones = buffer[2] - '0';
                    tens *= 10;
                    reversePercent(in_motor_cmd_chan, tens + ones);

                }
                touchChan <: 1;

            }
            else if(buffer[0] == '@'){
                forwardForTime(in_motor_cmd_chan, 0.75 * TICKS_PER_SEC);
            }
            else if(buffer[0] == '$'){
                forwardForTime(in_motor_cmd_chan, 1.5 * TICKS_PER_SEC);
            }
            else if(buffer[0] == '&'){
                forwardForTime(in_motor_cmd_chan, 3.0 * TICKS_PER_SEC);
            }
            else if(buffer[0] == '*'){
                leftForTime(in_motor_cmd_chan, 0.5 * TICKS_PER_SEC);
            }
            else if(buffer[0] == ';'){
                rightForTime(in_motor_cmd_chan, 0.5 * TICKS_PER_SEC);
            }
            else if(buffer[0] == '#'){
                //forwardForTime(in_motor_cmd_chan, 2.2 * TICKS_PER_SEC);
                timer tmr;
                unsigned int time;
                unsigned int curTime;
                unsigned int delay = 1.9 * TICKS_PER_SEC;
                motor_cmd_t straight;
                motor_cmd_t adjust;
                straight.left_duty_cycle = 40;
                straight.right_duty_cycle = 38;
                adjust = straight;
                in_motor_cmd_chan <: straight;
                float currentYaw;
                float startYaw;
                ypr_t results;
                imuChan :> results;
                startYaw = results.yaw;
                tmr :> time;
                tmr :> curTime;
                time += delay;
                while(curTime < time){
                    imuChan :> results;
                    currentYaw = results.yaw;
                    tmr :> curTime;

                    if((currentYaw - startYaw) > 0){
                        adjust.right_duty_cycle += 1;
                        in_motor_cmd_chan <: adjust;
                    }else{
                        in_motor_cmd_chan <: straight;
                    }
                }
                stop(in_motor_cmd_chan);
            }
            else{

            }


            currElement = 0;
        }

    }
}

void toggle_port(out port oLED, unsigned int hz)
{
    timer tmr;
    unsigned int t;
    unsigned pattern = 0b1;
    unsigned int delay = XS1_TIMER_HZ / hz;

    oLED <: pattern;
    tmr :> t;
    while (1)
    {
        oLED <: pattern;
        t += delay;
        tmr when timerafter(t) :> void;
        pattern = ~pattern;
    }
}

void uart_transmit_bytes(out port oPort, const char values[], unsigned int baudrate)
{
    int i = 0;
    while(values[i] != '\0')
    {

        uart_transmit_byte(oPort, values[i], baudrate);
        i++;
    }


}



void uart_transmit_byte(out port oPort, char value, unsigned int baudrate)
{

    timer tmr;
    unsigned int time;
    const unsigned int bitTime = XS1_TIMER_HZ / baudrate;
    unsigned int val = value;


    tmr :> time;

    oPort <: 0;
    time += bitTime;
    tmr when timerafter(time) :> void;

    for(int i = 0; i < 8; i++)
    {
        oPort <: (val & 0x1);
        val >>= 1;
        time += bitTime;
        tmr when timerafter(time) :> void;
    }

    oPort <: 1;
    time += bitTime;
    tmr when timerafter(time) :> void;

}

char uart_receive_byte(in port iPort, unsigned int baudrate)
{
    timer tmr;
    unsigned int time, value;
    const unsigned int bitTime = XS1_TIMER_HZ / baudrate;

    iPort when pinseq(1) :> void;

    iPort when pinseq(0) :> void;
    tmr :> time;
    time += bitTime/2;

    //input data bits
    value = 0;
    for(int i = 0; i < 8; i++)
    {
        time += bitTime;
        tmr when timerafter(time) :> void;
        iPort :> >> value;

    }

    // input stop bit //
    time += bitTime;
    tmr when timerafter(time) :> void;

    return (char) (value >> 24);

}


void multi_motor_task(out port oLeftPWM, out port oRightPWM, out port oMotorControl, chanend in_motor_cmd_chan)
{
    int lastLeft = 0;
    int lastRight = 0;
    int timeout = 0;
    unsigned int leftTicks;
    unsigned int rightTicks;
    unsigned int delayLeft;
    unsigned int delayRight;
    unsigned int delayPWM;
    unsigned int mask;
    timer timerLeft;
    timer timerRight;
    timer timerPWM;

    while(1){

        leftTicks = lastLeft * (PWM_FRAME_TICKS / 100);
        rightTicks = lastRight * (PWM_FRAME_TICKS / 100);
        oMotorControl <: mask;

        timeout = 0;

        oLeftPWM <: 1;
        oRightPWM <: 1;

        timerLeft :> delayLeft;
        timerRight :> delayRight;
        timerPWM :> delayPWM;
        delayLeft += leftTicks;
        delayRight += rightTicks;
        delayPWM += PWM_FRAME_TICKS;

        while(timeout == 0){
            select{

            case timerLeft when timerafter(delayLeft) :> void:
                oLeftPWM <: 0;
                delayLeft += XS1_TIMER_HZ;
                break;
            case timerRight when timerafter(delayRight) :> void:
                oRightPWM <: 0;
                delayRight += XS1_TIMER_HZ;
                break;
            case timerPWM when timerafter(delayPWM) :> void:
                timeout = 1;
                break;
            case in_motor_cmd_chan :> motor_cmd_t currentDuty:

                lastLeft = currentDuty.left_duty_cycle;
                lastRight = currentDuty.right_duty_cycle;

                if(currentDuty.left_duty_cycle < 0){
                    lastLeft = currentDuty.left_duty_cycle * -1;
                    mask = AIN1_ON;
                }else{
                    mask = AIN2_ON;
                }

                if(currentDuty.right_duty_cycle < 0){
                    lastRight = currentDuty.right_duty_cycle * -1;
                    mask = (BIN2_ON | mask);
                }else{
                    mask = (BIN1_ON | mask);
                }
                break;
            }
        }
    }
}

void encoder_task(in port iEncoder,chanend signal)
{
    unsigned int lastEncoder;
    unsigned int portValue;

    while(1)
    {
        iEncoder :> portValue;
        if((portValue & 0b0001) != (lastEncoder & 0b0001))
        {
            signal <: 1;
        }

        if((portValue & 0b0010) != (lastEncoder & 0b0010))
        {
            signal <: 2;
        }
        lastEncoder = portValue;
    }
}

void encoder_counter(chanend signalIn, chanend signalOut, chanend touchChan){
    encoderTicks counter;
    unsigned int total;
    unsigned int average;
    int counterOfTimes;
    timer tmr;
    unsigned int leftInitTime;
    unsigned int leftEndTime;
    unsigned int rightInitTime;
    unsigned int rightEndTime;
    unsigned int delay;
    unsigned int timeout;
    int breakout = 0;
    int leftBool;
    int rightBool;
    while(1){
        select{
        case signalIn :> int value:
            if(value == 1){
                counter.leftTicks ++;

            }else if(value == 2){
                counter.rightTicks ++;
            }
            break;
        case signalOut :> encoderTicks foo:
            signalOut <: counter;
            counter.leftTicks = 0;
            counter.rightTicks = 0;
            break;
        }

    }
}

unsigned int computeDelay(unsigned int start, unsigned int stop){
    unsigned int difference;
    if(stop < start){
        difference = (TIMER_MAX + 2) - ((stop - start) * -1);
        return difference;
    }else{
        difference = stop - start;
        return difference;
    }
}


void imu_task(chanend dmp_out){                        //does all MPU6050 actions
    int packetsize,mpuIntStatus,fifoCount;
    int address;
    unsigned char result[64];                           //holds dmp packet of data
    float qtest;
    float q[4]={0,0,0,0},g[3]={0,0,0},euler[3]={0,0,0},ypr[3]={0,0,0};
    int but_state;
    int fifooverflowcount=0,fifocorrupt=0;
    int GO_FLAG=1;

    //printf("Starting MPU6050...\n");
    mpu_init_i2c(imu);
    //printf("I2C Initialized...\n");
    address=mpu_read_byte(imu.i2c, MPU6050_RA_WHO_AM_I);
    //printf("MPU6050 at i2c address: %.2x\n",address);
    mpu_dmpInitialize(imu);
    mpu_enableDMP(imu,1);   //enable DMP

    mpuIntStatus=mpu_read_byte(imu.i2c,MPU6050_RA_INT_STATUS);
    //printf("MPU Interrupt Status:%d\n",mpuIntStatus);
    packetsize=42;                  //size of the fifo buffer
    delay_milliseconds(250);

    //The hardware interrupt line is not used, the FIFO buffer is polled
    while (GO_FLAG){
        mpuIntStatus=mpu_read_byte(imu.i2c,MPU6050_RA_INT_STATUS);
        if (mpuIntStatus >= 2) {
            fifoCount = mpu_read_short(imu.i2c,MPU6050_RA_FIFO_COUNTH);
            if (fifoCount>=1024) {              //fifo overflow
                mpu_resetFifo(imu);
                fifooverflowcount+=1;           //keep track of how often this happens to tweak parameters
                //printf("FIFO Overflow!\n");
            }
            while (fifoCount < packetsize) {    //wait for a full packet in FIFO buffer
                fifoCount = mpu_read_short(imu.i2c,MPU6050_RA_FIFO_COUNTH);
            }
            //printf("fifoCount:%d\n",fifoCount);
            mpu_getFIFOBytes(imu,packetsize,result);    //retrieve the packet from FIFO buffer

            mpu_getQuaternion(result,q);
            qtest=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
            if (fabs(qtest-1.0)<0.001){                             //check for fifo corruption - quat should be unit quat


                mpu_getGravity(q,g);


                ypr_t results;
                mpu_getYawPitchRoll(q,g,ypr);
                results.yaw = ypr[0];
                results.pitch = ypr[1];
                results.roll = ypr[2];
                if(results.yaw < 0){
                    results.yaw = (M_PI + results.yaw) + M_PI + 0.01;
                }
                dmp_out <: results;

            } else {
                mpu_resetFifo(imu);     //if a unit quat is not received, assume fifo corruption
                fifocorrupt+=1;
            }
        }
        //butP :> but_state;               //check to see if button is pushed to end program, low means pushed
        //but_state &=0x1;
        if (but_state==0){
            printf("Exiting...\n");
            GO_FLAG=0;
        }
    }
    mpu_Stop(imu);      //reset hardware gracefully and put into sleep mode
    //printf("Fifo Overflows:%d Fifo Corruptions:%d\n",fifooverflowcount,fifocorrupt);
}


void calculateMove(char* buffer,int rowStart, int colStart, int rowFinish, int colFinish, char prevDir){
    //calculate how to get from A -> B
    //   E
    // N   S
    //   W

    if(colFinish < colStart){
        //go north
        //go FORWARD for x ticks
        sprintf(buffer, "u\n");

    }else if(colFinish > colStart){
        //go south
        //go REVERSE for x ticks


    }else if(rowFinish > rowStart){
        //go west
        //turn LEFT
        //go FORWARD for x ticks
        if(prevDir == 'w'){
            sprintf(buffer, "u\n");
        }else{
            sprintf(buffer, ",|u\n");
        }

    }else if(rowFinish < rowStart){
        //go east
        //turn RIGHT
        //go FORWARD for x ticks
        if(prevDir == 'e'){
            sprintf(buffer, "u\n");
        }else{
            sprintf(buffer, ".|u\n");
        }
    }
}



int main()
{
    chan trigger;
    chan motor_cmd_chan;
    chan signal;
    chan encoderCount;
    chan touchChan;
    chan imuChan;
    oSTB <: 1;

    oWiFiRX <: 1;
    par
    {
        uart_to_console_task(trigger, motor_cmd_chan, encoderCount, touchChan, imuChan);
        output_task(trigger);
        multi_motor_task(oMotorPWMA, oMotorPWMB, oMotorControl, motor_cmd_chan);
        encoder_task(iEncoders, signal);
        encoder_counter(signal, encoderCount, touchChan);
        imu_task(imuChan);
    }




    return 0;
}


