/*
[2025-03-05] Ivan Lopez-Sanchez
This code was made for the Catheter Robot V2.
The Catheter robot uses MyActuator motors RMD-L-4010 & H-50-15.
Until now, the API does not work for the H-50-15 motors.
This code must be used along the Python GUI: 2024_12_13_Catheter_Robot_GUI_V2.py
https://github.com/biomechatronics001/6-DoF_Catheter_Robot_GCI/blob/main/2024-08-10%20Catheter%20GUI%20Code/2024_12_13_Catheter_Robot_GUI_V2.py
*/

#include "H50_motor_lib.h"
#include <FlexCAN_T4.h>
#include <SD.h>
#include <cmath> // Include cmath for fmod
#include <stdbool.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;

uint32_t ID_offset = 0x140;
uint32_t motor_ID1 = 1; // Motor Can Bus ID 1
uint32_t motor_ID2 = 2; // Motor Can Bus ID 2
uint32_t motor_ID3 = 3; // Motor Can Bus ID 3 
uint32_t motor_ID4 = 4; // Motor Can Bus ID 4 
uint32_t motor_ID5 = 5; // Motor Can Bus ID 5 
uint32_t motor_ID6 = 6; // Motor Can Bus ID 6

int CAN_ID = 3;  // CAN port from Teensy
double Gear_ratio = 1;
int Control_Mode_Flag = 1;
uint8_t motor_extra_cmd = 0x00;
// Collision detection variables
// Consider that k_t = 0.1. Therefore, max_current = 0.5 A -> max_tau = 0.05 Nm
static double motor_rated_current = 3.2; // [A] From the datasheet (max instant current) https://www.myactuator.com/l-4010-details
static double percentage_max_current = 75;
static double max_current_collision = double ((percentage_max_current/100)*motor_rated_current);
static uint8_t max_current_hybrid_mode = (percentage_max_current/100)*255; // Maximum current as a percentage of rated current (1% per LSB, range: 0–255)
// =============================

H50_motor_lib m1(motor_ID1, CAN_ID, Gear_ratio, 0); // index 0: used in the for loops to go through all the motors
H50_motor_lib m2(motor_ID2, CAN_ID, Gear_ratio, 1);
H50_motor_lib m3(motor_ID3, CAN_ID, Gear_ratio, 2);
H50_motor_lib m4(motor_ID4, CAN_ID, Gear_ratio, 3);
H50_motor_lib m5(motor_ID5, CAN_ID, Gear_ratio, 4);
H50_motor_lib m6(motor_ID6, CAN_ID, Gear_ratio, 5);

// Create array of motor pointers
H50_motor_lib* motors[] = {&m1, &m2, &m3, &m4, &m5, &m6};
float motors_angles[std::size(motors)]              = {0.0};
float motors_currents[std::size(motors)]            = {0.0};
float motors_cmds[std::size(motors)]                = {0.0};
int   motors_temperatures[std::size(motors)]        = {0};
int   motors_speeds[std::size(motors)]              = {0};
int   motors_signals_to_transmit[std::size(motors)] = {0};

double freq_ctrl = 100;
double freq_ble  = 100;
unsigned long t_0 = 0;
unsigned long t   = 0;
unsigned long prev_t_ctrl = 0;
unsigned long prev_t_ble  = 0;                                      
unsigned long T_ctrl_micros = (unsigned long)(1000000.0 / freq_ctrl);
unsigned long T_ble_micros  = (unsigned long)(1000000.0 / freq_ble);

double currentTime  = 0;
double previousTime = 0;


// Data sent via bluetooth
char datalength_ble    = 32;
char data_ble[60]      = {0};
char data_rs232_rx[60] = {0};
/****************************/

int t_teensy = 0;

float m1_received_cmd = 0;
float m2_received_cmd = 0;
float m3_received_cmd = 0;
float m4_received_cmd = 0;
float m5_received_cmd = 0;
float m6_received_cmd = 0;

// float pos_kd_gain = 0.95;
// float pos_kp_gain = 0.01*pos_kd_gain;

float pos_kp_gain = 0.095;
float pos_kd_gain = 0.001*pos_kp_gain;
float pos_ki_gain = 0.0*pos_kd_gain; // When using hybrid position/force control, set the integral gain to 0 for a better performance

uint32_t pos_accel_decel = 0;
uint32_t spd_accel_decel = pos_accel_decel;


void setup()
{
  Serial.println("* * * * * * * * * * * * * * * * * *");
  Serial.println("...Starting System Initialization...");
  Serial.begin(115200); //used for communication with computer.
  Serial5.begin(115200);  //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  
  initial_CAN();

  motors_initialization();

  Serial.println("- - - - - - - - - - - - - - - - - -");
  Serial.print("Controller executed at ");
  Serial.print(String(freq_ctrl));
  Serial.println(" Hz");
  Serial.print("BT com executed at ");
  Serial.print(String(freq_ble));
  Serial.println(" Hz");
  Serial.println("- - - - - - - - - - - - - - - - - -");
  delay(500);
  t_0 = micros();
}

void loop()
{  
  t = micros() - t_0;
  currentTime = micros();

  if (t - prev_t_ctrl > T_ctrl_micros)
  {
    
    if (t - prev_t_ble > T_ble_micros)
    {
      Receive_ble_Data();
    }
    
    cmd_assignment();
    // trajectory_cmds();

    send_cmd_to_motors();
    
    read_motor_data();

    collision_detection();

    if (t - prev_t_ble > T_ble_micros)
    {
      Transmit_ble_Data();
      // Serial.print(m1_pos_cmd);
      // Serial.print(" | ");
      // Serial.print(m1_angle);
      // Serial.print(" | ");
      // Serial.println(m1_current);

      prev_t_ble = t;
    }

    prev_t_ctrl = t;
  }
}

void initial_CAN()
{
  Serial.println("...Starting CAN Communication Setup...");
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(500);
  Serial.println("...CAN Communication Setup DONE...");
  // delay(500);
}

void motors_initialization()
{
  for (H50_motor_lib* motor : motors) {
    motor->init_motor();
  }

  set_PID_pos_gains();

  set_accel_decel_param();

  Serial.println("...Motors Setup Complete...");
  // delay(50000); // To check the initialization info
}

void Receive_ble_Data()
{
  if (Serial5.available() >= 20) // Read the incoming byte:
  {
    Serial5.readBytes(&data_rs232_rx[0], 20);

    if (data_rs232_rx[0] == 165 && data_rs232_rx[1] == 90 && data_rs232_rx[2] == 20) // Check the number of elemnts in the package
    {
      m1_received_cmd = (float)((int16_t)((data_rs232_rx[4] << 8) | data_rs232_rx[3])) / 10.0;
      m2_received_cmd = (float)((int16_t)((data_rs232_rx[6] << 8) | data_rs232_rx[5])) / 10.0;
      m3_received_cmd = (float)((int16_t)((data_rs232_rx[8] << 8) | data_rs232_rx[7])) / 10.0;
      m4_received_cmd = (float)((int16_t)((data_rs232_rx[10] << 8) | data_rs232_rx[9])) / 10.0;
      m5_received_cmd = (float)((int16_t)((data_rs232_rx[12] << 8) | data_rs232_rx[11])) / 10.0;
      m6_received_cmd = (float)((int16_t)((data_rs232_rx[14] << 8) | data_rs232_rx[13])) / 10.0;
      Control_Mode_Flag = data_rs232_rx[15];
      motor_extra_cmd   = data_rs232_rx[16];           
    }
  }
}

void Transmit_ble_Data()
{  
  t_teensy = (t / 1000000.0)*10.0; // t is in microseconds
  switch (Control_Mode_Flag)
  {
    case 1:
      for (H50_motor_lib* motor : motors) {
        motors_signals_to_transmit[motor->index] = (int) (motors_angles[motor->index] *10);
      }
      break;
    
    case 2:
      for (H50_motor_lib* motor : motors) {
        motors_signals_to_transmit[motor->index] = (int) (motors_speeds[motor->index]);
      }
      break;
    
    case 3:
      for (H50_motor_lib* motor : motors) {
        motors_signals_to_transmit[motor->index] = (int) (motors_currents[motor->index] *100);
      }
      break;
    
    case 4:
      for (H50_motor_lib* motor : motors) {
        motors_signals_to_transmit[motor->index] = (int) (motors_angles[motor->index] *10);
      }
      break;

    default:  
      break;
  }

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = t_teensy;
  data_ble[4]  = t_teensy >> 8; 
  data_ble[5]  = motors_signals_to_transmit[0];
  data_ble[6]  = motors_signals_to_transmit[0] >> 8;
  data_ble[7]  = motors_signals_to_transmit[1];
  data_ble[8]  = motors_signals_to_transmit[1] >> 8;
  data_ble[9]  = motors_signals_to_transmit[2];
  data_ble[10] = motors_signals_to_transmit[2] >> 8;
  data_ble[11] = motors_signals_to_transmit[3];
  data_ble[12] = motors_signals_to_transmit[3] >> 8;
  data_ble[13] = motors_signals_to_transmit[4];
  data_ble[14] = motors_signals_to_transmit[4] >> 8;
  data_ble[15] = motors_signals_to_transmit[5];
  data_ble[16] = motors_signals_to_transmit[5] >> 8;
  data_ble[17] = 0;
  data_ble[18] = 0 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0 >> 8;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 123;

  Serial5.write(data_ble, datalength_ble);
}

void cmd_assignment()
{
  motors_cmds[0] = m1_received_cmd;
  motors_cmds[1] = m2_received_cmd;
  motors_cmds[2] = m3_received_cmd;
  motors_cmds[3] = m4_received_cmd;
  motors_cmds[4] = m5_received_cmd;
  motors_cmds[5] = m6_received_cmd;

  switch (motor_extra_cmd){
        case 0x76: // Motor Reset
          for (H50_motor_lib* motor : motors) {
            motor->motor_shutdown();
            motor->write_multi_turn_encoder_pos_to_ROM_as_zero();
            motor->motor_reset();
            motor_extra_cmd = 0;
          }
          break;
        
        case 0x81: // Motor Stop
          for (H50_motor_lib* motor : motors) {
            motor->motor_stop();
          }
          break;
      } 
}

void trajectory_cmds()
{
  motors_cmds[0] = 0 + 135 * sin((2 * PI * 0.25) * (t / 1000000.0));
  motors_cmds[1] = 0 + 45 * sin((2 * PI * 0.5) * (t / 1000000.0));
  motors_cmds[2] = 0 + 90 * sin((2 * PI * 0.25) * (t / 1000000.0));
  motors_cmds[3] = 0 + 90 * sin((2 * PI * 0.5) * (t / 1000000.0));
  motors_cmds[4] = 0 + 135 * sin((2 * PI * 0.25) * (t / 1000000.0));
  motors_cmds[5] = 0 + 135 * sin((2 * PI * 0.5) * (t / 1000000.0));
}

void send_cmd_to_motors()
{
  switch (Control_Mode_Flag)
  {
    case 1:
      send_position_cmd_to_motors();
      break;
    
    case 2:        
      send_speed_cmd_to_motors();
      break;
    
    case 3:
      send_torque_cmd_to_motors();
      break;
    
    case 4:
      send_hybrid_posforce_cmd_to_motors();
      break;

    default:  
      break;
  }
}

void read_motor_data()
{
  // Loop through each motor
  for (H50_motor_lib* motor : motors) {
    motor->read_multi_turn_angle();
    motors_angles[motor->index] = motor->motorAngle_new;
    motor->read_motor_status_2();
    motors_temperatures[motor->index] = motor->temperature;
    motors_currents[motor->index] = motor->iq_A;
    motors_speeds[motor->index] = motor->speed_value;
    // Serial.println((String) motors_temperatures[motor->index] + " | " + motors_angles[motor->index] + " | " + motors_speeds[motor->index] + " | " + motors_currents[motor->index]);
    // Serial.println((String) motors_temperatures[0] + " | " + motors_angles[0] + " | " + motors_speeds[0] + " | " + motors_currents[0]);
  }
}

void set_PID_pos_gains()
{
  Serial.println("...Overwriting PID Gains...");
  for (H50_motor_lib* motor : motors) {
    motor->write_PID_RAM(0x07, pos_kp_gain);
    motor->write_PID_RAM(0x08, pos_ki_gain);
    motor->write_PID_RAM(0x09, pos_kd_gain);
  }
}

void set_accel_decel_param()
{
  for (H50_motor_lib* motor : motors) {
    motor->write_acceleration(0x00, pos_accel_decel);
    motor->write_acceleration(0x01, pos_accel_decel);
  }
}

void send_position_cmd_to_motors()
{
  int max_spd = 0; // 2000 RPM = 12000 deg/s
  for (H50_motor_lib* motor : motors) {
    motor->send_position_command_2(motors_cmds[motor->index], max_spd);
  }
}

void send_speed_cmd_to_motors()
{
  for (H50_motor_lib* motor : motors) {
    motor->send_speed_command(motors_cmds[motor->index]);
  }
}

void send_torque_cmd_to_motors()
{
  for (H50_motor_lib* motor : motors) {
    motor->send_current_command(motors_cmds[motor->index]);
  }
}

void send_hybrid_posforce_cmd_to_motors()
{
  /* Based on the block diagram and description of the Absolute Position Closed-Loop Control Command (0xA4) [page 47 in the manual] which has
  the same structure as this one (with the difference that this one has current limit) using accelration 0 and velocity 0 will make the
  controller to perform direct tracking mode, which showed better results.
  */
  int max_spd = 0; // 2000 RPM = 12000 deg/s
  // uint8_t max_current = 0.25*255; // Maximum torque as a percentage of rated current (1% per LSB, range: 0–255)
  for (H50_motor_lib* motor : motors) {
    motor->send_position_force_command(motors_cmds[motor->index], max_spd, max_current_hybrid_mode);
  }
}

void collision_detection()
{
  for (H50_motor_lib* motor : motors)
  {
    Serial.println((String) max_current_collision + " | " + fabs(motors_currents[motor->index]));
    if (fabs(motors_currents[motor->index]) > max_current_collision)
    {
      double value = fabs(motors_currents[motor->index]);
      Serial.println((String) "Collision detected" + " | " + value);
      collision_avoidance(motor->index);
      delay(5000);
    }    
  }
}

void collision_avoidance(int motor_index)
{
  int value = motor_index + 1;
  Serial.println((String) "Motor " + value + " in collision");
}