/*
The contents of this file is under the LICENSE file
*/

#include <Wire.h>
#include <EEPROM.h>

//////////////////////////////////////////////////////////////
// PID GAIN AND LIMIT SETTINGS
//////////////////

float pid_p_gain_roll = 1.4;               //Gain roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain roll I-controller (0.05)
float pid_d_gain_roll = 15;                //Gain roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3;                  //Gain yaw P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain yaw I-controller. //0.02
float pid_d_gain_yaw = 0;                  //Gain yaw D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

/////////////////////////////////////////////////////////////////
// GLOBAL VARIABLED
/////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;

int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];

unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis[4], gyro_axis_cal[4];

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;


/////////////////////////////////////////////////////////////////////
// SETUP Procedure
///////////////////

void setup(){
  //Read EEPROM for fast access data.
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  gyro_address = eeprom_data[32]; 

  Wire.begin();                         //Start the I2C as master.
   
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                    //Digital port 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                    //Digital port 12 and 13 as output.
    
  //Use the led on the Arduino for startup indication.
  PORTB |= _BV(PB4)                     //Set digital pin 12 high

  //Check the EEPROM signature to make sure that the setup program is executed
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);


  set_gyro_registers();                 //Set the MPU6050 gyro registers.
  
  // 1000us puls to prevent beeping while we wait for the 5sec delay
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){  //Wait 5 seconds before continuing.
    PORTD |= B11110000;                            //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                       //Wait 1000us.
    PORTD &= B00001111;                            //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                       //Wait 3000us.
  }
  
}

void set_gyro_registers() {
  Wire.beginTransmission(gyro_address);  //Start communication with 6050 address
    Wire.write(0x6B);                    //Write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                    //Set the register bits as 00000000 to activate gyro
    Wire.endTransmission();              //End the transmission with the gyro.


    /*
      GYRO_FS_SEL - 1B register - B[4:3] (Gyro Full Scale Select)
      00 = ±250dps
      01= ±500dps <--
      10 = ±1000dps
      11 = ±2000dps

      FCHOICE_B - 1B register B[1:0] - 
      00 - Enable DLPF (Lowpass filter)
    */
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);                     //Write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                     //Set the register bits as 00001000 (500dps)
    Wire.endTransmission();               
    
    //check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                        
    Wire.write(0x1B);                     //Read @ register 0x1B
    Wire.endTransmission();                                     
    Wire.requestFrom(gyro_address, 1);    //Request 1 bytes from the gyro
    while(Wire.available() < 1);          //Wait until the 1 byte is received
    if(Wire.read() != 0x08){              //Check value is 0x08
      while(1)delay(10);                  //Stay in this loop for ever
    }

    /* Low pass filter settings
      DLPF_CFG - 1A Register B(2:0)
      CFG Hz  Delay
      0   250 0.97
      1   184 2.9 
      2   92  3.9
      3   41  5.9 <--
      4   20  9.9
      5   10  17.85
      6   5   33.48
      
    */
    
    Wire.beginTransmission(gyro_address); 
    Wire.write(0x1A);                      //Write to the CONFIG register (1A hex)
    Wire.write(0x03);                      //Set the register bits as 00000011 DLPF-3
    Wire.endTransmission();     
}











