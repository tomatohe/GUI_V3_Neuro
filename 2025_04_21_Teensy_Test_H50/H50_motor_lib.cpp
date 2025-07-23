/*
[2025-02-20] Ivan Lopez-Sanchez
This library was made for MyActuator motors RMD-L-4010 & H-50-15
|Driver: V3 | Version: V4.2 | Date: 2024.05|
Check the manuals for the motors:
RMD-L-4010 ("Motor Motion Protocol V4.2-250208.pdf"): https://www.myactuator.com/downloads-lseries
H-50-15    ("Motor Motion Protocol V4.2-241219.pdf"): https://www.myactuator.com/downloads-hseries
*/

#include "H50_motor_lib.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

float decode_uint32_to_float(const uint8_t Data[8]) {
    union {
        uint32_t u32;
        float f;
    } converter;
    
    // Combine bytes in little-endian format (Data[4] is LSB, Data[7] is MSB)
    converter.u32 = ((uint32_t)Data[7] << 24) |  // Most significant byte
                    ((uint32_t)Data[6] << 16) |
                    ((uint32_t)Data[5] << 8)  |
                    (uint32_t)Data[4];         // Least significant byte

    return converter.f;
}

float decode_uint32_to_int(const uint8_t Data[8]) {
    union {
        uint32_t u32;
        int i;
    } converter;
    
    // Combine bytes in little-endian format (Data[4] is LSB, Data[7] is MSB)
    converter.u32 = ((uint32_t)Data[7] << 24) |  // Most significant byte
                    ((uint32_t)Data[6] << 16) |
                    ((uint32_t)Data[5] << 8)  |
                    (uint32_t)Data[4];         // Least significant byte

    return converter.i;
}

uint8_t indexes[] = {0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x09};

H50_motor_lib::H50_motor_lib(uint32_t id, int Can_id, double Gear_ratio, int INDEX)
{
  ID = id;
  gear_ratio = Gear_ratio;
  index = INDEX;
}

H50_motor_lib::~H50_motor_lib()
{}

void H50_motor_lib::init_motor()
{
  Serial.println((String)"...Starting motor initialization for Motor ID: " + ID + "...");
  motor_shutdown();
  read_motor_status_1();
  read_motor_status_2();
  read_motor_status_3();
  comm_baud_rate_set();
  motor_stop();
  write_multi_turn_encoder_pos_to_ROM_as_zero();
  delay(500);  
  read_PID_gains();
  // read_single_turn_encoder();
  // read_single_turn_angle();
  // read_acceleration();

  read_motor_status_1();
  read_motor_status_2();
  read_motor_status_3();
  motor_brake_release(); // Needed for torque (current) commands
}


////////////////Received CAN Message Decoding////////////////////////////////
void H50_motor_lib::DataExplanation(CAN_message_t msgR2)
{
  int len = msgR2.len;
  if (len == 8)
  {
    switch (msgR2.buf[0])
    {
      case 0x30: // Read PID Parameter Command
        switch (msgR2.buf[1])
        {
          case 0x01:
            cur_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)" Current Control kp: " + cur_kp);
            break;
          case 0x02:
            cur_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)" Current Control ki: " + cur_ki);
            break;
          case 0x04:
            spd_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"   Speed Control kp: " + spd_kp);
            break;
          case 0x05:
            spd_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"   Speed Control ki: " + spd_ki);
            break;
          case 0x07:
            pos_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control kp: " + pos_kp);
            break;
          case 0x08:
            pos_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control ki: " + pos_ki);
            break;
          case 0x09:
            pos_kd = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control kd: " + pos_kd);
            break;
          default:
            break;
        }
        break;

      case 0x31: // Write PID Parameters to RAM Command
        Serial.println("Writing Controller Gains to RAM...");
        switch (msgR2.buf[1])
        {
          case 0x01:
            cur_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Current Control kp: " + cur_kp);
            break;
          case 0x02:
            cur_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Current Control ki: " + cur_ki);
            break;
          case 0x04:
            spd_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Speed Control kp: " + spd_kp);
            break;
          case 0x05:
            spd_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Speed Control ki: " + spd_ki);
            break;
          case 0x07:
            pos_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control kp: " + pos_kp);
            break;
          case 0x08:
            pos_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control ki: " + pos_ki);
            break;
          case 0x09:
            pos_kd = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control kd: " + pos_kd);
            break;
          default:
            break;
        }
        break;

      case 0x32: // Write PID Parameters to ROM Command
        Serial.println("Writing Controller Gains to ROM...");
        switch (msgR2.buf[1])
        {
          case 0x01:
            cur_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Current Control kp: " + cur_kp);
            break;
          case 0x02:
            cur_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Current Control ki: " + cur_ki);
            break;
          case 0x04:
            spd_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Speed Control kp: " + spd_kp);
            break;
          case 0x05:
            spd_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Speed Control ki: " + spd_ki);
            break;
          case 0x07:
            pos_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control kp: " + pos_kp);
            break;
          case 0x08:
            pos_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control ki: " + pos_ki);
            break;
          case 0x09:
            pos_kd = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control kd: " + pos_kd);
            break;
          default:
            break;
        }
        break;

      case 0x42: // Read Acceleration Command
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
        Serial.print("Success to read parameter Accel: ");
        Serial.println(Accel);
        break;

      case 0x43: // Write Acceleration to RAM and ROM Command
        Serial.println("Writing Acceleration Parameters to RAM and ROM...");
        switch (msgR2.buf[1])
        {
          case 0x00:
            pos_accel = decode_uint32_to_int(msgR2.buf);
            Serial.println((String)"Position Planning Acceleration: " + pos_accel);
            break;
          case 0x01:
            pos_decel = decode_uint32_to_int(msgR2.buf);
            Serial.println((String)"Position Planning Deceleration: " + pos_decel);
            break;
          case 0x02:
            spd_accel = decode_uint32_to_int(msgR2.buf);
            Serial.println((String)"Speed Planning Acceleration: " + spd_accel);
            break;
          case 0x03:
            spd_decel = decode_uint32_to_int(msgR2.buf);
            Serial.println((String)"Speed Planning Deceleration: " + spd_decel);
            break;
          default:
            break;
        }
        break;

      case 0x60: // Read Multi-Turn Encoder Position Data Command
        Serial.println("Read Multi-Turn Encoder Position Data Command():");
        encoder_multi_turn_pos = decode_uint32_to_int(msgR2.buf);
        Serial.println(encoder_multi_turn_pos);
        break;

      case 0x64: // Write the Current Multi-Turn Position of the Encoder to the ROM as the Motor Zero Command
        Serial.println("Write the Current Multi-Turn Position of the Encoder to the ROM as the Motor Zero Command():");
        encoder_zero_bias = decode_uint32_to_int(msgR2.buf);
        Serial.println(encoder_zero_bias);
        break;

      case 0x76: // System Reset Command
        Serial.println("System Reset Command():");
        break;

      case 0x77: // System Brake Release Command
        Serial.println("System Brake Release Command():");
        break;

      case 0x78: // System Brake Lock Command
        Serial.println("System Brake Lock Command():");
        break;

      case 0x80: // Motor Shutdown Command
        Serial.println("Successful: Motor Shutdown Command()");
        break;

      case 0x81: // Motor Stop Command
        Serial.println("Successful: Motor Stop Command()");
        break;

      case 0x90: // Read Single-Turn Encoder Command
        encoder       = (uint16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        encoderRaw    = (uint16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoderOffset = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        Serial.println("Success to read encoder parameters:");
        Serial.print("encoder:");
        Serial.print(encoder);
        Serial.print("| encoderRaw:");
        Serial.print(encoderRaw);
        Serial.print("| encoderOffset:");
        Serial.println(encoderOffset);
        break;

      case 0x92: // Read Multi-Turn Angle Command
        motorAngle_new = decode_uint32_to_int(msgR2.buf) * 0.01;
        break;

      case 0x94: // Read Single-Turn Angle Command
        Serial.println("Read Single-Turn Angle Command()");
        circleAngle = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        Serial.println(circleAngle);
        break;

      case 0x9A: // Read Motor Status 1 and Error Flag Command
        temperature = (int8_t)msgR2.buf[1]; // Motor Temperature
        RlyCtrlRslt = (int8_t)msgR2.buf[3]; // Brake release command
        voltage     = (uint16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4])); // Voltage
        errorState  = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6])); // Error Status
        Serial.print((String) " | " + "Temp: " + temperature + " | " + "Voltage: " + (voltage * 0.1) + " | " + "Error State: " + errorState + " = ");
        
        switch (errorState) {
          case 0x0000:
            Serial.println("No Errors Found");
            break;
          case 0x0002:
            Serial.println("Motor Stall");
            break;
          case 0x0004:
            Serial.println("Low Voltage");
            break;
          case 0x0008:
            Serial.println("Over Voltage");
            break;
          case 0x0010:
            Serial.println("Over Current");
            break;
          case 0x0040:
            Serial.println("Power Overrun");
            break;
          case 0x0080:
            Serial.println("Calibration Parameter Writing Error");
            break;
          case 0x0100:
            Serial.println("Speeding");
            break;
          case 0x1000:
            Serial.println("Motor Temperature Over Temperature");
            break;
          case 0x2000:
            Serial.println("Encoder Calibration Error");
            break;
        }
        break;
    
      case 0x9C: // Read Motor Status 2 Command: return temperature, iq current(-2048~2048 mapping-32A to 32A), speed (1dps/LSB), encoder (0~16383)
        temperature       = (int8_t)msgR2.buf[1];
        iq                = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A              = ((double)iq) * 0.01;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value       = (double)speed_value_int16;
        speed_value       = speed_value / gear_ratio;
        angle             = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        angle_val_float   = (float) angle;
        // Serial.println((String) " | " + "Temp: " + temperature + " | " + "Current: " + iq_A + " | " + "Speed: " + speed_value);
        break;

      case 0x9D: //15: read motor status 3 return temperature, phase A B C current
        temperature = (int8_t)msgR2.buf[1];
        iA          = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iA_A        = ((double)iA) * 0.01;
        iB          = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        iB_A        = ((double)iB) * 0.01;
        iC          = (int16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        iC_A        = ((double)iC) * 0.01;
        Serial.println((String) " | " + "Temp: " + temperature + " | " + "Phase A current: " + iA_A + " | " + "Phase B current: " + iB_A + " | " + "Phase A current: " + iC_A);
        break;

      case 0xA1: // Torque Closed-Loop Control Command
        temperature       = (int8_t) msgR2.buf[1];
        iq                = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A              = ((double)iq) * 0.01;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value       = (double) speed_value_int16;
        speed_value       = speed_value / gear_ratio;
        angle             = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        angle_val_float   = (float) angle;
        // Serial.println((String) "[0xA1] Temp: " + temperature + " | " + "Curent: " + iq_A + " | " + "Speed: " + speed_value + " | " + "Angle: " + angle_val_float);
        break;
      
      case 0xA2: // Speed Closed-Loop Control Command
        temperature       = (int8_t) msgR2.buf[1];
        iq                = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A              = ((double)iq) * 0.01;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value       = (double) speed_value_int16;
        speed_value       = speed_value / gear_ratio;
        angle             = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        angle_val_float   = (float) angle;
        // Serial.println((String) "[0xA2] Temp: " + temperature + " | " + "Curent: " + iq_A + " | " + "Speed: " + speed_value + " | " + "Angle: " + angle_val_float);
        break;
            
      case 0xA4: // Absolute Position Closed-Loop Control Command
        temperature       = (int8_t)msgR2.buf[1];
        iq                = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A              = ((double)iq) * 32 / 2048;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value       = (double)speed_value_int16;
        speed_value       = speed_value / gear_ratio;
        angle             = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        angle_val_float   = (float) angle;
        // Serial.println((String) "[0xA4] Temp: " + temperature + " | " + "Curent: " + iq_A + " | " + "Speed: " + speed_value + " | " + "Angle: " + angle_val_float);
        break;

      case 0xA9: //Position Closed-Loop Force Control Command
        temperature       = (int8_t) msgR2.buf[1];
        iq                = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A              = ((double)iq) * 0.01;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value       = (double)speed_value_int16;
        speed_value       = speed_value / gear_ratio;
        angle             = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        angle_val_float   = (float) angle;
        // Serial.println((String) "[0xA9] Temp: " + temperature + " | " + "Curent: " + iq_A + " | " + "Speed: " + speed_value + " | " + "Angle: " + angle_val_float);
        break;
      
      default:
        break;
    }
  }
}

/////////////////////////////////////////////////////////////////////
void H50_motor_lib::receive_CAN_data()
{
  if (Can3.read(msgR))
  {
    Can3.read(msgR);
    DataExplanation(msgR);
  }
}

/* [0x30] Read PID Parameter Command.
This command can read the PID parameters of the current,speed and position, the data type is Float, determined by the index value.
0x01 Current loop KP parameter.
0x02 Current loop KI parameter.
0x04 Speed loop KP parameter.
0x05 Speed loop KI parameter.
0x07 Position loop KP parameter.
0x08 Position loop KI parameter.
0x09 Position loop KD parameter.
*/
void H50_motor_lib::read_PID_gains()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x30;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Serial.println("...Reading Controller Gains...");
  for (int i = 0; i < sizeof(indexes)/sizeof(indexes[0]); i++) {
    msgW.buf[1] = indexes[i];
    Can3.write(msgW);
    delay(10); // less than 3 ms does not work
    receive_CAN_data();
  }
}

/* [0x31] Write PID Parameters to RAM Command.
This command can write the parameters of current, speed, position loop KP and KI to RAM at one time, and it will not be saved after power off.
The data type is Float, and it is determined by the index value.
Be careful to avoid writing parameters when the motor has just started and is in motion.
*/
void H50_motor_lib::write_PID_RAM(uint8_t index, float gain)
{
  union {
    float f;
    uint8_t bytes[4];
  } converter;
  converter.f = gain;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x31;
  msgW.buf[1] = index;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = converter.bytes[0];
  msgW.buf[5] = converter.bytes[1];
  msgW.buf[6] = converter.bytes[2];
  msgW.buf[7] = converter.bytes[3];
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

//******3.Write PID gain to ROM******//
void H50_motor_lib::write_PID_ROM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x32;
  msgW.buf[1] = 0;
  msgW.buf[2] = anglePidKp;
  msgW.buf[3] = anglePidKi;
  msgW.buf[4] = speedPidKp;
  msgW.buf[5] = speedPidKi;
  msgW.buf[6] = iqPidKp;
  msgW.buf[7] = iqPidKi;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******5.Write Acceleration RAM******//
void H50_motor_lib::write_acceleration_RAM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x34;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&Accel);
  msgW.buf[5] = *((uint8_t*)(&Accel) + 1);
  msgW.buf[6] = *((uint8_t*)(&Accel) + 2);
  msgW.buf[7] = *((uint8_t*)(&Accel) + 3);
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

/*[0x42] Read Acceleration Command.
Read the acceleration parameters of the current motor.
*/
void H50_motor_lib::read_acceleration()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x42;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

/*[0x43] Write Acceleration to RAM and ROM Command.
The host sends this command to write the acceleration into the RAM and ROM, which can be saved after power off.
Acceleration data Accel is uint32_t type, the unit is 1dps/s,and the parameter range is 100-60000.
The command contains the acceleration and deceleration values in the position and velocity planning, which are determined by the index value.
0x00: position planning acceleration.
0x01: position planning deceleration.
0x02: speed planning acceleration.
0x03: speed planning deceleration.
*/
void H50_motor_lib::write_acceleration(uint8_t index, uint32_t accel_value)
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x43;
  msgW.buf[1] = index;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = (uint8_t)(accel_value);
  msgW.buf[5] = (uint8_t)(accel_value >> 8);
  msgW.buf[6] = (uint8_t)(accel_value >> 16);
  msgW.buf[7] = (uint8_t)(accel_value >> 24);
  Can3.write(msgW);
  delay(50);
  receive_CAN_data();
}

//******2.11. Read Single-Turn Encoder Command******//
void H50_motor_lib::read_single_turn_encoder()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x90;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

//******7.Write ENCODER OFFSET ROM******//
void H50_motor_lib::write_encoder_offset_RAM(uint16_t encoder_Offset )
{
  encoderOffset = encoder_Offset;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x91;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = *(uint8_t*)(&encoderOffset);
  msgW.buf[7] = *((uint8_t*)(&encoderOffset) + 1);
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******8.Write current postioton as Zero degree******//
void H50_motor_lib::write_current_position_as_zero_position()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x19;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

void H50_motor_lib::read_multi_turn_encoder_pos() // Read Multi-Turn Encoder Position Data Command (0x60)
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x60;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

/* [0x64] Write the Current Multi-Turn Position of the Encoder to the ROM as the Motor Zero Command.
Write the current encoder position of the motor as the multi-turn encoder zero offset (initial position) into the ROM.
Note: After writing the new zero point position,you need to send 0x76 (system reset
command) to restart the system to be effective. Because of the change of the zero
offset,the new zero offset (initial position) should be used as a reference when setting
the target position.*/
void H50_motor_lib::write_multi_turn_encoder_pos_to_ROM_as_zero()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x64;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(100);
  receive_CAN_data();
  motor_reset();
}

/*[0x76] This command is used to reset the system program.*/
void H50_motor_lib::motor_reset()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x76;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(100);
  receive_CAN_data();
}

/*[0x77] System Brake Release Command.
This command is used to open the system brake.
The system will release the holding brake,and the motor will be in a movable state without being restricted by the holding
brake.*/
void H50_motor_lib::motor_brake_release()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x77;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

/*[0x78] System Brake Lock Command.
This command is used to close the system holding brake.
The holding brake locks the motor and the motor can no longer run. The holding brake is also in this state after the system is powered off.*/
void H50_motor_lib::motor_brake_lock()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x78;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

/*[0x80] Motor Shutdown Command.
Turns off the motor output and also clears the motor running state, not in any closed loop mode.
*/
void H50_motor_lib::motor_shutdown()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x80;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(50);
  receive_CAN_data();
}

/* [0x81] Motor Stop Command.
Stop the motor, the closed-loop mode where the motor is still running, just stop the motor speed.
*/
void H50_motor_lib::motor_stop()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x81;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

//******18.start motor******//
void H50_motor_lib::start_motor()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0X88;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  if (Can3.write(msgW)) {
    char buffer[50];
    sprintf(buffer, "start_motor(): Successful - Motor ID: %d", ID);
    Serial.println(buffer);
  }
  else {
    Serial.println("Fail to send start motor command");
  }
   delay(10);
  receive_CAN_data();
}

/*[0x92] Read Multi-Turn Angle Command.
read the current multi-turn absolute angle value of the motor.
Motor angle motorAngle, (int32_t type, value range, valid data 4 bytes), unit 0.01°/LSB.*/
void H50_motor_lib::read_multi_turn_angle()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x92;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(3);
  receive_CAN_data();
}

/*[0x94] Read Single-Turn Angle Command.
Read the current single-turn angle of the motor.
The single circle angle of the motor, circleAngle is int16_t type data, starting
from the zero point of the encoder, increasing clockwise, and returning to 0 when
it reaches the zero point again, the unit is 0.01°/LSB, and the value range is 0~35999.
*/
void H50_motor_lib::read_single_turn_angle()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x94;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  // delay(10);
  // receive_CAN_data();
}

//(current cannot use it)******11.clear all angle command and offset currnet position as zero command (unit 0.01deg/LSB)******//
void H50_motor_lib::clear_motor_angle_command()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x95;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******13.clear motot error and reset motor******//
void H50_motor_lib::clear_motor_error()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9B;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
}

/*[0x9A] Read Motor Status 1 and Error Flag Command.
This command reads the current motor temperature, voltage and error status flags.
1. Motor temperature temperature (int8_t type,unit 1℃/LSB)
2. Brake control command: Indicates the state of the brake control command, 1 represents the brake release command, and 0 represents the brake lock command
3. Voltage (uint16_t type,unit 0.1V/LSB)
4. Error flag errorState (of type uint16_t,each bit represents a different motor state)*/
void H50_motor_lib::read_motor_status_1()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9A;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Serial.println("...Reading Motor Status 1...");
  Can3.write(msgW);
  delay(5);
  receive_CAN_data();
}

//******14.read motor status 2 (temperature 1degreeC/LSB, iq(-2048~2048 mapping to -32A ~32A), speed(1dps/LSB), 14 bit encoder value(0~16383))******//
void H50_motor_lib::read_motor_status_2()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9C;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  // Serial.println("...Reading Motor Status 2...");
  Can3.write(msgW);
  delay(3);
  receive_CAN_data();
}

//******15.read motor status 3 (temperature 1degreeC/LSB,A phase current(1A/64LSB),B phase current(1A/64LSB),C phase current(1A/64LSB) )******//
void H50_motor_lib::read_motor_status_3()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9D;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Serial.println("...Reading Motor Status 3...");
  Can3.write(msgW);
  delay(50);
  receive_CAN_data();
}

/* [0xA1] Torque Closed-Loop Control Command.
Control the torque and current output of the motor.
The control value iqControl is of type int16_t and the unit is 0.01A/LSB.
For safety reasons, this command cannot open the brake directly.
But, you can use the 0x77 command to open the brake first, then you can use A1 command.*/
void H50_motor_lib::send_current_command(double current)
{
  iqControl = (int16_t) (current * 100);
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA1;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = (uint8_t) (iqControl);
  msgW.buf[5] = (uint8_t) (iqControl >> 8);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(5);
  receive_CAN_data();
}

/* [0xA2] Speed Closed-Loop Control Command.
The host sends this command to control the speed of the motor output shaft.
The control value speedControl is int32_t type,and the corresponding actual speed is 0.01dps/LSB. */
void H50_motor_lib::send_speed_command(double speed)
{
  speed = speed * 100 * gear_ratio;
  speedControl = (int32_t) speed;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA2;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = (uint8_t) speedControl;
  msgW.buf[5] = (uint8_t) (speedControl >> 8);
  msgW.buf[6] = (uint8_t) (speedControl >> 16);
  msgW.buf[7] = (uint8_t) (speedControl >> 24);
  Can3.write(msgW);
  delay(5);
  receive_CAN_data();
}

/* [0xA4] Absolute Position Closed-Loop Control Command
The control value angleControl is int32_t type, and the corresponding actual position is 0.01 degree/LSB, that is, 36000 represents 360°, and the rotation direction of the motor is determined by the difference between the target position and the current position.
The control value maxSpeed limits the maximum speed of the motor output shaft rotation, which is of type uint16_t, corresponding to the actual speed of 1 dps/LSB.
According to the position planning acceleration value set by the system, different operating modes will be different:
1. If the position loop acceleration is 0, then the position loop will enter the direct tracking mode,and directly track the target position through the PI controller.
Among them, maxSpeed limits the maximum speed during the position operation process.
If the maxSpeed value is 0, then it is completely output by the calculation result of the PI controller.
2. If the position loop acceleration is not 0, then the motion mode with speed planning will be run,and the motor will complete the acceleration and deceleration process.
The maximum operating speed is determined by maxSpeed, and the acceleration is determined by the acceleration set by the position loop.
*/
void H50_motor_lib::send_position_command_2(double angle, double max_speed) // max_spd = 2000 RPM = 12000 deg/s.
{
  angle = angle * 100;
  angleControl = (int32_t) angle;
  maxiSpeed = (int16_t) max_speed;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA4;
  msgW.buf[1] = 0x00;
  msgW.buf[2] = (uint8_t)(maxiSpeed);
  msgW.buf[3] = (uint8_t)(maxiSpeed >> 8);
  msgW.buf[4] = (uint8_t)(angleControl);
  msgW.buf[5] = (uint8_t)(angleControl >> 8);
  msgW.buf[6] = (uint8_t)(angleControl >> 16);
  msgW.buf[7] = (uint8_t)(angleControl >> 24);
  Can3.write(msgW);
  delay(1);
}

/* [0xA9] Position Closed-Loop Force Control Command.
This command controls the motor’s position (multi-turn angle) when no motor faults are present.
The host sends this command with the following parameters:
•	angleControl (int32_t): Target position in 0.01°/LSB (e.g., 36000 = 360°). Direction is determined by the difference between the target and current positions.
•	maxSpeed (uint16_t): Maximum output shaft speed in 1°/s per LSB.
•	maxTorque (uint8_t): Maximum torque as a percentage of rated current (1% per LSB, range: 0–255). If the current exceeds the stall current, force control mode is disabled, and torque is limited by the motor’s stall current value.
*/
void H50_motor_lib::send_position_force_command(float angle, uint16_t max_speed, uint8_t max_current)
{
  angleControl = (int32_t) (angle * 100.0);
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA9;
  msgW.buf[1] = max_current;
  msgW.buf[2] = (uint8_t)(max_speed);
  msgW.buf[3] = (uint8_t)(max_speed >> 8);
  msgW.buf[4] = (uint8_t)(angleControl);
  msgW.buf[5] = (uint8_t)(angleControl >> 8);
  msgW.buf[6] = (uint8_t)(angleControl >> 16);
  msgW.buf[7] = (uint8_t)(angleControl >> 24);
  Can3.write(msgW);
  delay(5);
  receive_CAN_data();
}

void H50_motor_lib::read_multi_turn_angle_for36()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x92;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //delay(1);
  //receive_CAN_data();
}

/*[0xB4] Communication Baud Rate Setting Command.
This instruction can set the communication baud rate of CAN and RS485 bus.
The parameters will be saved in ROM after setting,and will be saved after power off, and will run at the modified baud rate when powered on again.
CAN: 0 stands for 500Kbps baud rate, 1 stands for 1Mbps baud rate.
*/
void H50_motor_lib::comm_baud_rate_set()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xB4;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = (uint8_t)(1); // 0 = 500Kbps | 1 = 1Mbps
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
  if (msgW.buf[7] == 0){
    Serial.println("Baud rate = 500Kbps");
  }
  else {
    Serial.println("Baud rate = 1Mbps");
  }
}
