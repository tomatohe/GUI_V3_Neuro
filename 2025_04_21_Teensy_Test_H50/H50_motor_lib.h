#include <FlexCAN_T4.h>
#include <Arduino.h>

#define xbox_control_mode 7
#define profile_position_control_mode 6
#define position_control_mode 3
#define velocity_control_mode 2
#define current_control_mode 1

class H50_motor_lib
{
  public:
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
//    uint8_t anglePidKp = 250;
//    uint8_t anglePidKi = 25;
//    uint8_t speedPidKp = 150;
//    uint8_t speedPidKi = 5;
//    uint8_t iqPidKp = 100;
//    uint8_t iqPidKi = 5;
    uint8_t anglePidKp = 50;
    uint8_t anglePidKi = 50;
    uint8_t speedPidKp = 100;
    uint8_t speedPidKi = 5;
    uint8_t iqPidKp = 100;
    uint8_t iqPidKi = 5;

    float cur_kp = 0;
    float cur_ki = 0;
    float spd_kp = 0;
    float spd_ki = 0;
    float pos_kp = 0;
    float pos_ki = 0;
    float pos_kd = 0;
    float gain   = 0;
    float angle_val_float = 0;

    uint pos_accel = 0;
    uint pos_decel = 0;
    uint spd_accel = 0;
    uint spd_decel = 0;

    int encoder_multi_turn_pos = 0;
    int encoder_zero_bias = 0;


    int32_t Accel = 0;
    uint8_t iqControli1 = 0;
    uint8_t iqControli2 = 0;
    int16_t iqControl = 0;
    int16_t iqControl_1 = 0;
    int16_t iqControl_2 = 0;
    int16_t iqControl_3 = 0;
    int16_t iqControl_4 = 0;
    int32_t speedControl = 0;
    int32_t angleControl = 0;
    int16_t maxiSpeed = 0;
    int16_t iq = 0;
    double iq_A = 0;
    int16_t iA = 0;
    double iA_A = 0;
    int16_t iB = 0;
    double iB_A = 0;
    int16_t iC = 0;
    double iC_A = 0;
    int16_t speed_value_int16 = 0;
    double speed_value = 0;
    uint16_t encoder = 0;
    int8_t temperature = 0;
    uint16_t encoderRaw = 0;
    uint16_t encoderOffset = 0;
    double motorAngle = 0;
    float motorAngle_new = 0;
    double motorAngle_offset = 0;
    double motorAngle_raw = 0;
    int64_t motorAngle_int64 = 0;
    int16_t angle = 0;

    int32_t motorAngle_int32 = 0;
    int32_t speed_value_int32 = 0;
    uint8_t circleAngle = 0;
    uint16_t voltage = 0;
    uint8_t RlyCtrlRslt = 0;
    uint16_t errorState = 0;

    H50_motor_lib(uint32_t id, int Can_id, double Gear_ratio, int index);
    ~H50_motor_lib();
    void init_motor();
    void DataExplanation(CAN_message_t msg2);
    void receive_CAN_data();
    void read_PID_gains();
    void write_PID_RAM(uint8_t index, float gain);
    void write_PID_ROM();
    void read_acceleration();
    void write_acceleration(uint8_t accel_index, uint32_t accel_value = 60000);
    void write_acceleration_RAM();
    void read_single_turn_encoder();
    void write_encoder_offset_RAM(uint16_t encoder_Offset);
    void write_current_position_as_zero_position();
    void write_multi_turn_encoder_pos_to_ROM_as_zero();
    void motor_reset();
    void motor_brake_release();
    void motor_brake_lock();
    void read_multi_turn_angle();
    void read_single_turn_angle();
    void read_multi_turn_encoder_pos();
    void clear_motor_angle_command();    
    void clear_motor_error();
    void read_motor_status_1();
    void read_motor_status_2();
    void read_motor_status_3();
    void motor_stop();
    void motor_shutdown();
    void start_motor();
    void comm_baud_rate_set();
    void send_current_command(double current);
    void send_speed_command(double speedvalue);
    void send_position_command(double angle);
    void send_position_command_2(double angle, double max_speed);
    void send_position_command_3(double angle, uint8_t spinDirection);
    void send_position_command_4(double angle, double max_speed, uint8_t spinDirection);
    void send_position_force_command(float angle, uint16_t max_speed, uint8_t max_current);
    void send_multi_motor_current_command(double Motor1_current, double Motor2_current, double Motor3_current, double Motor4_current);
    void send_current_command_for36(double current);
    void read_multi_turn_angle_for36();
    void read_motor_current_for36();
    int index;
  private:
    CAN_message_t msgW;
    CAN_message_t msgR;
    uint32_t ID;
    double gear_ratio;
    uint32_t acceleration_uint32 = 0;
    int32_t acceleration_int32 = 0;

};
//#endif
