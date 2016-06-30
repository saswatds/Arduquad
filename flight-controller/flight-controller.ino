/*
 * The contents of this file is under the LICENSE file
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

int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis[4], gyro_axis_cal[4];

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float bttry_compens;
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;

boolean auto_stabilize;
boolean set_gyro_angles;

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
  PORTB |= _BV(PB4);                     //Set digital pin 12 high

  //Check the EEPROM signature to make sure that the setup program is executed
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);


  imu_setup();                 //Set the MPU6050 gyro registers.
  
  // 1000us puls to prevent beeping while we wait for the 5sec delay
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){  //Wait 5 seconds before continuing.
    PORTD |= B11110000;                            //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                       //Wait 1000us.
    PORTD &= B00001111;                            //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                       //Wait 3000us.
  }

  //Calculate the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){   //Take 2000 readings
    if(cal_int % 15 == 0) 
      digitalWrite(12, !digitalRead(12));           // LED toggle
    imu_read();                                    //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];               //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];               //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];               //Ad yaw value to gyro_yaw_cal.

    // Prevent Beeping
    PORTD |= B11110000;                                       
    delayMicroseconds(1000);                                   
    PORTD &= B00001111;                                        
    delay(3);                                                 
  }

  // Find the gyro mean offset
  gyro_axis_cal[1] /= 2000;
  gyro_axis_cal[2] /= 2000;           
  gyro_axis_cal[3] /= 2000;

  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiver_input_channel_3 < 990 
        || receiver_input_channel_3 > 1020 
        || receiver_input_channel_4 < 1400){
    receiver_input_channel_3 = get_receiver_channel(3);  // convert standard 1000 - 2000us
    receiver_input_channel_4 = get_receiver_channel(4);  // convert standard 1000 - 2000us  
    // Prevent beeping of ESC
    PORTD |= B11110000;                                        
    delayMicroseconds(1000);                                   
    PORTD &= B00001111;                                       
    delayMicroseconds(1000);                                   
    
  }
  start = 0;       //Set start back to 0.
  auto_stabilize = true; //Set auto_strabilization
  
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;
  
  //When everything is done, turn off the led.
  PORTB &= ~_BV(PB4);
}

void imu_setup() {
  Wire.beginTransmission(gyro_address);  //Start communication with 6050 address
    Wire.write(0x6B);                    //Write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                    //Set the register bits as 00000000 to activate gyro
    Wire.endTransmission();              //End the transmission with the gyro.

    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68);         
    Wire.write(0x1C);                     //Send the requested starting register
    Wire.write(0x10);                     //Set the requested starting register
    Wire.endTransmission();               
    
    /*
     * GYRO_FS_SEL - 1B register - B[4:3] (Gyro Full Scale Select)
     * 00 = ±250dps
     * 01= ±500dps <--
     * 10 = ±1000dps
     * 11 = ±2000dps
     * 
     * FCHOICE_B - 1B register B[1:0] - 
     * 00 - Enable DLPF (Lowpass filter)
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
     *  DLPF_CFG - 1A Register B(2:0)
     *  CFG Hz  Delay
     *  0   250 0.97 
     *  1   184 2.9  
     *  2   92  3.9
     *  3   41  5.9 <-- 
     *  4   20  9.9 
     *  5   10  17.85  
     *  6   5   33.48   
    */
    
    Wire.beginTransmission(gyro_address); 
    Wire.write(0x1A);                      //Write to the CONFIG register (1A hex)
    Wire.write(0x03);                      //Set the register bits as 00000011 DLPF-3
    Wire.endTransmission();     
}

void imu_read(){
  Wire.beginTransmission(gyro_address);                        
  Wire.write(0x3B);                                   //Start reading @ register 43h and auto increment with every read
  Wire.endTransmission();
  Wire.requestFrom(gyro_address,14);                   //Request 14 bytes from the gyro
  while(Wire.available() < 14);                        //Wait until the 6 bytes are received
  // Read the data
  acc_x = Wire.read()<<8|Wire.read();                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();            //Add the low and high byte to the temperature variable
  gyro_axis[1] = Wire.read()<<8|Wire.read();
  gyro_axis[2] = Wire.read()<<8|Wire.read();
  gyro_axis[3] = Wire.read()<<8|Wire.read();
  

  // If calibrated then compensate
  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;
}

int get_receiver_channel(byte function){
  byte channel, reverse;
  int low, center, high, actual;
  int difference;
  
  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse
  
  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel
  
  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                    //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void loop() {

  receiver_input_channel_1 = get_receiver_channel(1);      //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = get_receiver_channel(2);      //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = get_receiver_channel(3);      //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = get_receiver_channel(4);      //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

  // Read the data from the gyroscope
  imu_read();

  if(auto_stabilize == true){
      stabilize();
      //To dampen the pitch and roll angles a complementary filter is used
      gyro_pitch_input = gyro_pitch_input * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
      gyro_roll_input = gyro_roll_input * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  }
  else {
    /*
     * For MPU5060 With FS_SEL= 1 - 500dps
     * 1 degree/s = 65.5 unit/sec
     * To filter the gyro data we use a complementary filter of 20%/80%
    */
    gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 65.5) * 0.2);      
    gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 65.5) * 0.2);         
  }
  // The yaw is not required for stabilization
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 65.5) * 0.2); 
  // GOOD IDEA TO PRINT THE ABOVE DATA AND DO AN EXCEL SPREADSHEET(W&W/O PROPS)

  //STEP 1. Arm Motors: Throttle LOW, Yaw LEFT
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;

  //STEP 2: Motor start: Throttle LOW, Yaw CENTER
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  //STOP: Unarm Motors: Throttle LOW, Yaw RIGHT 
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;


  /////////////////
  ///SET POINT CALCULATION
  //////

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  //We need a little dead band of 16us for better results.
  pid_roll_setpoint = 0;
  if(receiver_input_channel_1 > 1508) pid_roll_setpoint = (receiver_input_channel_1 - 1508)/3.0;
  else if(receiver_input_channel_1 < 1492) pid_roll_setpoint = (receiver_input_channel_1 - 1492)/3.0;

  pid_pitch_setpoint = 0;
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = (receiver_input_channel_2 - 1508)/3.0;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = (receiver_input_channel_2 - 1492)/3.0;

  pid_yaw_setpoint = 0;
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508) pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492) pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }


  /////////////////////////
  /// CALCULATE PID
  ////////////////
  calculate_pid();


  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //(andlogRead(0)+65)* 1.2317* 0.08 = a(x)*0.09853
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1030 && battery_voltage > 600) PORTB |= _BV(PB4);

  ///////////// THROTTLE ///////////
  throttle = receiver_input_channel_3;

  //////////////////////////////////////////////////////
  // STATE 2 - MOTOR START
  /////////////////////////
  
  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //pulse for esc 4 (front-left - CW)

    //Compensate the esc's pulse for voltage drop.
    bttry_compens = ((1240 - battery_voltage)/(float)3500);
    if (battery_voltage < 1240 && battery_voltage > 800){  //Is the battery connected?
      esc_1 += esc_1 * bttry_compens ;  
      esc_2 += esc_2 * bttry_compens;  
      esc_3 += esc_3 * bttry_compens;   
      esc_4 += esc_4 * bttry_compens;   
    } 

    //Keep the motors running.
    if (esc_1 < 1200) esc_1 = 1200;             
    if (esc_2 < 1200) esc_2 = 1200;                                         
    if (esc_3 < 1200) esc_3 = 1200;     
    if (esc_4 < 1200) esc_4 = 1200;                                         

     //Limit the esc-1 pulse to 2000us.
    if(esc_1 > 2000)esc_1 = 2000;  
    if(esc_2 > 2000)esc_2 = 2000;  
    if(esc_3 > 2000)esc_3 = 2000;
    if(esc_4 > 2000)esc_4 = 2000;    
  }

  //If start is not 2 keep a 1000us pulse
  else{
    esc_1 = 1000;
    esc_2 = 1000;  
    esc_3 = 1000;   
    esc_4 = 1000;
  }

  ////////////////////////////////////
  // EXECUTING THE ESC SIGNAL
  //////////////////////

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
//ISR for getting radio input ( Might not be required later if implement software controller)
////////////////////
ISR(PCINT0_vect){
  current_time = micros();
  
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input[1] = current_time - timer_1;                //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input[2] = current_time - timer_2;                //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input[3] = current_time - timer_3;                //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input[4] = current_time - timer_4;                //Channel 4 is current_time - timer_4
  }
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

void stabilize() {
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_roll * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_yaw * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_yaw * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle


   //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
}



