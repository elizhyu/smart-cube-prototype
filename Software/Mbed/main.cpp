#include "mbed.h"
#include "rtos.h"
#include "uLCD_4DGL.h"
#include "LSM9DS1.h"
#define PI 3.14159

Serial pc(USBTX, USBRX);
Serial ESP(p9, p10);
uLCD_4DGL uLCD(p13,p14,p30); // serial tx, serial rx, reset pin;
LSM9DS1 IMU(p28, p27, 0xD6, 0x3C);

Mutex usage_LCD;
Mutex usage_UART;
int prev_min;
int min;
int year;
int month;
int date;
char day[3];
bool shake;
int orientation;

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);

void pose_update(void const *argument)
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float roll;
    float pitch;
    bool shake_count;
    int orientation_count;  // -1 for LEFT, 1 for RIGHT, 0 for UP
    while(1)
    {
        myled2 = 1;
        // Read from sensor
        while(!IMU.accelAvailable());
        IMU.readAccel();
        while(!IMU.gyroAvailable());
        IMU.readGyro();
        // Calculate
        ax = IMU.calcAccel(IMU.ax);
        ay = IMU.calcAccel(IMU.ay);
        az = IMU.calcAccel(IMU.az);
        gx = IMU.calcGyro(IMU.gx);
        gy = IMU.calcGyro(IMU.gy);
        gz = IMU.calcGyro(IMU.gz);
        // Calculate roll and pitch
        roll = atan2(ay, az);
        pitch = atan2(-ax, sqrt(ay * ay + az * az));
        // Convert from radians to degrees
        pitch *= 180.0 / PI;
        roll  *= 180.0 / PI;
        // Shake Detection
        if(abs(gx) > 10 || abs(gy) > 10 || abs(gz) > 10)
        {
            if(shake_count && !shake)
            {
                shake = true;
                usage_UART.lock();
                ESP.printf("\r*VI=Y#\r");
                usage_UART.unlock();
            }
            else    shake_count = true;
        }
        else
        {
            shake_count = false;
            if(shake)
            {
                shake = false;
                usage_UART.lock();
                ESP.printf("\r*VI=N#\r");
                usage_UART.unlock();
            }            
        }
        // Orientation Detection
        if(pitch > -120 && pitch < -60)
        {
            if(orientation_count == -1 && orientation != -1)
            {
                orientation = -1;
                usage_UART.lock();
                ESP.printf("\r*ORI=L#\r");
                usage_UART.unlock();
            }
            else    orientation_count = -1;
        }
        else if(pitch > 60 && pitch < 120)
        {
            if(orientation_count == 1 && orientation != 1)
            {
                orientation = 1;
                usage_UART.lock();
                ESP.printf("\r*ORI=R#\r");
                usage_UART.unlock();
            }
            else    orientation_count = 1;
        }
        else
        {
            if(orientation_count == 0 && orientation != 0)
            {
                orientation = 0;
                usage_UART.lock();
                ESP.printf("\r*ORI=U#\r");
                usage_UART.unlock();
            }
            else    orientation_count = 0;
        }
        myled2 = 0;
        Thread::wait(1000);
    }
}

void time_update(void const *argument)
{
    while(1)
    {
        myled1 = 1;
        usage_UART.lock();
        ESP.printf("\r*TI=?#\r");
        usage_UART.unlock();
        myled1 = 0;
        Thread::wait(10000);
    }
}

void execute(char *head, char *load)
{
    if(strcmp(head, "ti") == 0)
    {
        ESP.printf("\r*TI GOT#\r");
        prev_min = min;
        min = (load[2] - 0x30) * 10 + (load[3] - 0x30);
        if(min != prev_min)
        {
            usage_LCD.lock();
            uLCD.cls();
            uLCD.locate(4, 5);
            uLCD.printf("%04d-%02d-%02d", year, month, date);
            uLCD.locate(8, 7);
            uLCD.printf("%c%c%c", day[0], day[1], day[2]);
            uLCD.locate(6, 10);
            uLCD.printf("%c%c : %c%c", load[0], load[1], load[2], load[3]);
            usage_LCD.unlock();
        }
    }
    else if(strcmp(head, "da") == 0)
    {
        year = (((load[0] - 0x30) * 10 + (load[1] - 0x30)) * 10 + (load[2] - 0x30)) * 10 + (load[3] - 0x30);
        month = (load[4] - 0x30) * 10 + (load[5] - 0x30);
        date = (load[6] - 0x30) * 10 + (load[7] - 0x30);
        switch(load[8])
        {
            case '0':
                day[0] = 'S';
                day[1] = 'U';
                day[2] = 'N';
                break;
            case '1':
                day[0] = 'M';
                day[1] = 'O';
                day[2] = 'N';
                break;
            case '2':
                day[0] = 'T';
                day[1] = 'U';
                day[2] = 'E';
                break;    
            case '3':
                day[0] = 'W';
                day[1] = 'E';
                day[2] = 'D';
                break;
            case '4':
                day[0] = 'T';
                day[1] = 'H';
                day[2] = 'U';
                break;
            case '5':
                day[0] = 'F';
                day[1] = 'R';
                day[2] = 'I';
                break;
            case '6':
                day[0] = 'S';
                day[1] = 'A';
                day[2] = 'T';
                break;
        }
    }
}

int main()
{       
    ESP.baud(9600);
    
    
    IMU.begin();
    myled1 = 1;
    myled4 = 1;
    IMU.calibrate(1);
    myled1 = 0;
    myled4 = 0;
    
    usage_UART.lock();
    ESP.printf("\r*INIT#\r");
    usage_UART.unlock();
    
    orientation = 2;
    shake = false;
    //Thread thread1(time_update);
    Thread thread2(pose_update);
    
    char read_char;
    char msg_head[3];
    msg_head[2] = '\0';
    char msg[10];
    int index = 0;
    
    while(1)
    {
        usage_UART.lock();
        if(ESP.readable())
        {
            read_char = ESP.getc();
            switch(read_char)
            {
                case '*':
                    index = 0;
                    break;
                case '=':
                    index = 0;
                    msg_head[0] = msg[0];
                    msg_head[1] = msg[1];
                    break;
                case '\r':
                    msg[index] = '\0';
                    index = 0;
                    execute(msg_head, msg);
                    break;
                default:
                    msg[index] = read_char;
                    index++;
                    break;
            }
        }
        usage_UART.unlock();
        Thread::wait(500); 
    }
}

