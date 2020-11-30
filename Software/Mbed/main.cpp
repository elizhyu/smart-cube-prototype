#include "mbed.h"
#include "rtos.h"
#include "uLCD_4DGL.h"
#include "LSM9DS1.h"
#define PI 3.14159

// ========================================================== 
// =========== Control board devices declaration ============
// ==========================================================
// USB UART interface
Serial pc(USBTX, USBRX);
// UART interface to ESP32 board
Serial ESP(p9, p10); // TX, RX
// I2C interface to LSM9DS1 pose sensor
LSM9DS1 IMU(p28, p27, 0xD6, 0x3C); // SDA, SCL

// ========================================================== 
// ============ FRONT panel devices declaration =============
// ==========================================================
// uLCD screen
uLCD_4DGL uLCD(p13,p14,p8); // TX, RX, RESET;
// RGB Notification LED
DigitalOut LED_R(p7); // red
DigitalOut LED_G(p5); // green
DigitalOut LED_B(p6); // blue

// ========================================================== 
// ========== RIGHT panel IO devices declaration ============
// ==========================================================
// Green status LEDs
DigitalOut LED_TL(p21); // top-left
DigitalOut LED_TR(p23); // top-right
DigitalOut LED_BL(p25); // bottom-left
DigitalOut LED_BR(p26); // bottom-right
// Pushbuttons
DigitalIn PB_TL(p22); // top-left
DigitalIn PB_TR(p24); // top-right
DigitalIn PB_BL(p29); // bottom-left
DigitalIn PB_BR(p30); // bottom-right

// ========================================================== 
// ================== On-board debug LEDs ===================
// ==========================================================
//DigitalOut myled1(LED1);
//DigitalOut myled2(LED2);
//DigitalOut myled3(LED3);
//DigitalOut myled4(LED4);

// ========================================================== 
// ============ RTOS multi-thread mutex locks ===============
// ==========================================================
Mutex usage_LCD;
Mutex usage_UART;

// ========================================================== 
// ============ Global variables declaration ================
// ==========================================================
// local variables storing pulled SNTP time  
int prev_min;
int min;
int year;
int month;
int date;
char day[3];
// status indication variables for shake and orientation
bool shake;
int orientation;
// notification flag
bool new_notification;

// ========================================================== 
// ========== Pose & Vibration Processing Thread ============
// ==========================================================
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
        // debug only
        //myled2 = 1;
        
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
                if(new_notification)
                {
                    new_notification = false;
                    LED_R = 0;
                    LED_G = 0;
                    LED_B = 0;
                    usage_UART.lock();
                    ESP.printf("\r*NOT DIS#\r");
                    usage_UART.unlock();
                }
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
        
        // debug only
        //myled2 = 0;
        
        // Thread execution frequency
        Thread::wait(1000);
    }
}

/* OBSELETE -- DEBUG ONLY
// ========================================================== 
// ============== Manual Time Pulling Thread ================
// ==========================================================
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
}*/

// ========================================================== 
// ================= IO Interaction Thread ==================
// ==========================================================
void io_function(void const *argument)
{
    while(1)   
    {
        // pose activate LEFT panel
        if(orientation == -1)
        {
            usage_UART.lock();
            ESP.printf("\r*DE=?#\r");
            usage_UART.unlock();
            while(orientation == -1)
            {
                if(PB_TL)
                {
                    LED_TL = !LED_TL;
                    usage_UART.lock();
                    if(LED_TL)  ESP.printf("\r*L1=1#\r");
                    else    ESP.printf("\r*L1=0#\r");
                    usage_UART.unlock();
                }
                else if(PB_TR)
                {
                    LED_TR = !LED_TR;
                    usage_UART.lock();
                    if(LED_TR)  ESP.printf("\r*L2=1#\r");
                    else    ESP.printf("\r*L2=0#\r");
                    usage_UART.unlock();
                }
                else if(PB_BL)
                {
                    LED_BL = !LED_BL;
                    usage_UART.lock();
                    if(LED_BL)  ESP.printf("\r*L3=1#\r");
                    else    ESP.printf("\r*L3=0#\r");
                    usage_UART.unlock();
                }
                else if(PB_BR)
                {
                    LED_BR = !LED_BR;
                    usage_UART.lock();
                    if(LED_BR)  ESP.printf("\r*L4=1#\r");
                    else    ESP.printf("\r*L4=0#\r");
                    usage_UART.unlock();
                }
            }
        }
        // pose activate RIGHT panel
        else if(orientation == 1)   
        {
            
        }
        // deactivate LEFT and RIGHT panels
        else    
        {
            LED_TL = 0;
            LED_TR = 0;
            LED_BL = 0;
            LED_BR = 0;
        }
        
        // Thread execution frequency
        Thread::wait(1000);
    }
}

// ========================================================== 
// =========== UART Command Execution Function ==============
// ==========================================================
void execute(char *head, char *load)
{
    // Transcode and store pulled SNTP time to local variables
    // and update LCD screen time display
    if(strcmp(head, "ti") == 0)
    {
        // debug only
        //ESP.printf("\r*TI GOT#\r");
        // store previous minute value
        prev_min = min;
        // transcode minute value from (char) to (int)
        min = (load[2] - 0x30) * 10 + (load[3] - 0x30);
        // update LCD time display only if next minute
        if(min != prev_min)
        {
            // lock LCD mutex
            usage_LCD.lock();
            // refresh time and date display
            uLCD.cls();
            uLCD.locate(4, 5);
            uLCD.printf("%04d-%02d-%02d", year, month, date);
            uLCD.locate(8, 7);
            uLCD.printf("%c%c%c", day[0], day[1], day[2]);
            uLCD.locate(6, 10);
            uLCD.printf("%c%c : %c%c", load[0], load[1], load[2], load[3]);
            // unlock LCD mutex
            usage_LCD.unlock();
        }
    }
    // Transcode and store pulled SNTP date to local variables
    else if(strcmp(head, "da") == 0)
    {
        // transcode date from (char) to (int)
        year = (((load[0] - 0x30) * 10 + (load[1] - 0x30)) * 10 + (load[2] - 0x30)) * 10 + (load[3] - 0x30);
        month = (load[4] - 0x30) * 10 + (load[5] - 0x30);
        date = (load[6] - 0x30) * 10 + (load[7] - 0x30);
        // transcode day of week from (int) to (char)
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
    // 
    else if(strcmp(head, "no") == 0)
    {
        // raise new notification flag
        new_notification = true;
        
        // process notification color
        switch(load[0])
        {
            case 'r':   // red
                LED_R = 1;
                LED_G = 0;
                LED_B = 0;
                break;
            case 'g':   // green
                LED_R = 0;
                LED_G = 1;
                LED_B = 0;
                break;
            case 'b':   // blue
                LED_R = 0;
                LED_G = 0;
                LED_B = 1;
                break;    
            case 'p':   // purple
                LED_R = 1;
                LED_G = 0;
                LED_B = 1;
                break;
        }
    }
    else if(strcmp(head, "de") == 0)
    {
        ESP.printf("\r*DE GOT#\r");
        // update device indicator LEDs
        LED_TL = load[0] - 0x30;
        LED_TR = load[1] - 0x30;
        LED_BL = load[2] - 0x30;
        LED_BR = load[3] - 0x30;
    }
}

int main()
{
    
    // ========================================================== 
    // ============== Bootup Initiation Operations ==============
    // ==========================================================
    uLCD.cls();
    // set up UART interface BAUD rate to ESP32 board
    ESP.baud(9600);
    // debug only
    LED_R = 0;
    LED_G = 0;
    LED_B = 0;
    // LSM9DS1 pose sensor startup and calibration
    IMU.begin();
    Thread::wait(1000);
    // debug only
    //LED_G = 1;
    IMU.calibrate(1);
    // debug only
    //LED_B = 1;
    
    // debug only
    //usage_UART.lock();
    //ESP.printf("\r*INIT#\r");
    //usage_UART.unlock();
    
    // preset values for global variables
    // pose and vibration status indicator variables
    orientation = 2;
    shake = false;
    // new notification flag
    new_notification = false;
    
    // RTOS multi-thread initialization
    //Thread thread1(time_update);  // obselete, dubug only
    Thread thread2(pose_update);    // pose and vibration detection
    Thread thread3(io_function);    // IO interaction
    
    char read_char;
    char msg_head[3];
    msg_head[2] = '\0';
    char msg[10];
    int index = 0;
    
    
    // ========================================================== 
    // ========= ESP32 UART Interface Decoding Thread ===========
    // ==========================================================
    while(1)
    {
        // lock UART mutex
        usage_UART.lock();
        
        // new UART data received
        if(ESP.readable())
        {
            // read and store received byte
            read_char = ESP.getc();
            
            // process received byte
            switch(read_char)
            {
                // start of message
                // clear buffer
                case '*':
                    index = 0;
                    break;
                // mid-point of message
                // store command from buffer
                case '=':
                    index = 0;
                    msg_head[0] = msg[0];
                    msg_head[1] = msg[1];
                    break;
                // end of message
                // store payload from buffer
                case '\r':
                    msg[index] = '\0';
                    index = 0;
                    execute(msg_head, msg);
                    break;
                // middle of command or payload
                // proceed buffer index
                default:
                    msg[index] = read_char;
                    index++;
                    break;
            }
        }
        
        // unlock UART mutex
        usage_UART.unlock();
        
        // Thread execution frequency
        Thread::wait(500); 
    }
}

