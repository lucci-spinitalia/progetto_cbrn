#ifndef _SEGWAY_H
#define _SEGWAY_H
 
/* Definition of bitmap1 parameters */
#define NONE                       0x00000000
#define FAULT_STATUS_WORD_1        0x00000001
#define FAULT_STATUS_WORD_2        0x00000002
#define FAULT_STATUS_WORD_3        0x00000004
#define FAULT_STATUS_WORD_4        0x00000008
#define MCU_0_FAULT_STATUS         0x00000010
#define MCU_1_FAULT_STATUS         0x00000020
#define MCU_2_FAULT_STATUS         0x00000040
#define MCU_3_FAULT_STATUS         0x00000080
#define FRAME_COUNT                0x00000100
#define OPERATIONAL_STATE          0x00000200
#define DYNAMIC_RESPONSE           0x00000400
#define MIN_PROPULSION_BATT_SOC    0x00000800
#define AUX_BATT_SOC               0x00001000
#define INERTIAL_X_ACC_G           0x00002000
#define INERTIAL_Y_ACC_G           0x00004000
#define INERTIAL_X_RATE_RPS        0x00008000
#define INERTIAL_Y_RATE_RPS        0x00010000
#define INERTIAL_Z_RATE_RPS        0x00020000
#define PSE_PITCH_DEG              0x00040000
#define PSE_PITCH_RATE_DPS         0x00080000
#define PSE_ROLL_DEG               0x00100000
#define PSE_ROLL_RATE_DPS          0x00200000
#define PSE_YAW_RATE_DPS           0x00400000
#define PSE_DATA_IS_VALID          0x00800000
#define YAW_RATE_LIMIT_RPS         0x01000000
#define VEL_LIMIT_MPS              0x02000000
#define LINEAR_ACCEL_MPS2          0x04000000
#define LINEAR_VEL_MPS             0x08000000
#define DIFFERENTIAL_WHEEL_VEL_RPS 0x10000000
#define RIGHT_FRONT_VEL_MPS        0x20000000
#define LEFT_FRONT_VEL_MPS         0x40000000
#define RIGHT_REAR_VEL_MPS         0x80000000

/* Definition of bitmap2 parameters */
#define LEFT_REAR_VEL_MPS          0x00000001
#define RIGHT_FRONT_POS_M          0x00000002
#define LEFT_FRONT_POS_M           0x00000004
#define RIGHT_REAR_POS_M           0x00000008
#define LEFT_REAR_POS_M            0x00000010
#define LINEAR_POS_M               0x00000020
#define RIGHT_FRONT_CURRENT_A0PK   0x00000040
#define LEFT_FRONT_CURRENT_A0PK    0x00000080
#define RIGHT_REAR_CURRENT_A0PK    0x00000100
#define LEFT_REAR_CURRENT_A0PK     0x00000200
#define MAX_MOTOR_CURRENT_A0PK     0x00000400
#define RIGHT_FRONT_CURRENT_LIMIT_A0PK  0x00000800 //!
#define LEFT_FRONT_CURRENT_LIMIT_A0PK   0x00001000 //!
#define RIGHT_REAR_CURRENT_LIMIT_A0PK   0x00002000 //!
#define LEFT_REAR_CURRENT_LIMIT_A0PK    0x00004000 //!
#define MIN_MOTOR_CURRENT_LIMIT_A0PK    0x00008000 //!
#define FRONT_BASE_BATT_1_SOC      0x00010000
#define FRONT_BASE_BATT_2_SOC      0x00020000
#define REAR_BASE_BATT_1_SOC       0x00040000
#define REAR_BASE_BATT_2_SOC       0x00080000
#define FRONT_BASE_BATT_1_TEMP_DEGC  0x00100000 //!
#define FRONT_BASE_BATT_2_TEMP_DEGC  0x00200000 //!
#define REAR_BASE_BATT_1_TEMP_DEGC   0x00400000
#define REAR_BASE_BATT_2_TEMP_DEGC   0x00800000
#define VEL_TARGET_MPS             0x01000000
#define YAW_RATE_TARGET_RPS        0x02000000
#define FRAM_VEL_LIMIT_MPS         0x04000000
#define FRAM_ACCEL_LIMIT_MPS2      0x08000000
#define FRAM_DECEL_LIMIT_MPS2      0x10000000
#define FRAM_DTZ_DECEL_LIMIT_MPS2  0x20000000
#define FRAM_COASTDOWN_DECEL_MPS2  0x40000000
#define FRAM_YAW_RATE_LIMIT_RPS    0x80000000

/* Definition of bitmap3 parameters */
#define FRAM_YAW_ACCEL_LIMIT_RPS2  0x00000001
#define FRAM_TIRE_DIAMETER_M       0x00000002
#define FRAM_WHEEL_TRACK_WIDTH_M   0x00000004
#define FRAM_TRANSMISSION_RATIO    0x00000008
#define FRAM_CONFIG_BITMAP         0x00000010
#define FRAM_ETH_IP_ADDRESS        0x00000020
#define FRAM_ETH_PORT_NUMBER       0x00000040
#define FRAM_ETH_SUBNET_MASK       0x00000080
#define FRAM_ETH_GATEWAY           0x00000100
#define USER_FEEDBACK_BITMAP_1     0x00000200
#define USER_FEEDBACK_BITMAP_2     0x00000400
#define USER_FEEDBACK_BITMAP_3     0x00000800
#define AUX_BATT_VOLTAGE_V         0x00001000
#define AUX_BATT_CURRENT_A         0x00002000
#define AUX_BATT_TEMP_DEGC         0x00004000
#define ABB_SYSTEM_STATUS          0x00008000
#define AUX_BATT_STATUS            0x00010000
#define AUX_BATT_FAULTS            0x00020000
#define CCU7P2_BATTERY_VOLTAGE     0x00040000
#define SP_SW_BUILD_ID             0x00080000
#define UIP_SW_BUILD_ID            0x00100000

/* Fault status word and mcu internal fault status definitions */
// Fault status word 0
#define FSW_ARCH_FAULTS_INDEX     0  
#define FSW_ARCH_FAULTS_SHIFT     0
#define FSW_ARCH_FAULTS_MASK      0x00000FFF
#define FSW_CRITICAL_FAULTS_INDEX 0
#define FSW_CRITICAL_FAULTS_SHIFT 12
#define FSW_CRITICAL_FAULTS_MASK  0xFFFFF000

// Fault status word 1
#define FSW_COMM_FAULTS_INDEX     1
#define FSW_COMM_FAULTS_SHIFT     0
#define FSW_COMM_FAULTS_MASK      0x0000FFFF
#define FSW_INTERNAL_FAULTS_INDEX 1
#define FSW_INTERNAL_FAULTS_SHIFT 16
#define FSW_INTERNAL_FAULTS_MASK  0x000F0000

// Fault status word 2
#define FSW_SENSORS_FAULTS_INDEX  2
#define FSW_SENSORS_FAULTS_SHIFT  0
#define FSW_SENSORS_FAULTS_MASK   0x0000FFFF
#define FSW_BSA_FAULTS_INDEX      2
#define FSW_BSA_FAULTS_SHIFT      16
#define FSW_BSA_FAULTS_MASK       0xFFFF0000 

// Fault status word 3
#define FSW_MCU_FAULTS_INDEX      3
#define FSW_MCU_FAULTS_SHIFT      0
#define FSW_MCU_FAULTS_MASK       0xFFFFFFFF

/* Segway limit value */
#define MAX_VELOCITY_LIMIT         8.047
#define MAX_ACCELERATION_LIMIT     7.848
#define MAX_DECELERATION_LIMIT     7.848
#define MAX_DTZ_DECEL_RATE_LIMIT   7.848
#define COASTDOWN_ACCEL_LIMIT      1.961
#define MAX_TURN_RATE_LIMIT        4.5
#define MAX_TURN_ACCEL_LIMIT       28.274
#define TIRE_DIAMETER_LIMIT        1.0
#define WHEEL_TRACK_WIDTH_LIMIT    1.0
#define TRANSMISSION_RATIO_LIMIT   200.0

/* Segway default value */
#define MAX_VELOCITY_DEF           2.2352
#define MAX_ACCELERATION_DEF       3.923
#define MAX_DECELERATION_DEF       3.923
#define MAX_DTZ_DECEL_RATE_DEF     3.923
#define COASTDOWN_ACCEL_DEF        1.961
#define MAX_TURN_RATE_DEF          3.0
#define MAX_TURN_ACCEL_DEF         28.274
#define TIRE_DIAMETER_DEF          0.483616
#define WHEEL_TRACK_WIDTH_DEF      0.7112
#define TRANSMISSION_RATIO_DEF     24.2667   

/* Segway configuration */
#define MAX_VELOCITY               2
#define MAX_ACCELERATION           2
#define MAX_DECELERATION           2
#define MAX_DTZ_DECEL_RATE         MAX_DTZ_DECEL_RATE_DEF
#define COASTDOWN_ACCEL            COASTDOWN_ACCEL_DEF
#define MAX_TURN_RATE              MAX_TURN_RATE_DEF
#define MAX_TURN_ACCEL             MAX_TURN_ACCEL_DEF
#define TIRE_DIAMETER              0.53
#define WHEEL_TRACK_WIDTH          WHEEL_TRACK_WIDTH_DEF
#define TRANSMISSION_RATIO         TRANSMISSION_RATIO_DEF

/* Operational mode */
#define CCU_INIT           0x00
#define PROPULSION_INIT    0x01
#define CHECK_STARTUP      0x02
#define SEGWAY_STANDBY     0x03
#define SEGWAY_TRACTOR     0x04
#define DISABLE_POWER      0x05

#define SEGWAY_STANDBY_REQ 0x04
#define SEGWAY_TRACTOR_REQ 0x05

/* MACRO */
#define segway_configure_none(socket, message_param) (segway_configure_load(socket, 0x00, message_param))
#define segway_configure_max_vel(socket, message_param) (segway_configure_load(socket, 0x01, convert_to_ieee754(message_param)))
#define segway_configure_max_acc(socket, message_param) (segway_configure_load(socket, 0x02, convert_to_ieee754(message_param)))
#define segway_configure_max_decel(socket, message_param) (segway_configure_load(socket, 0x03, convert_to_ieee754(message_param)))
#define segway_configure_max_dtz(socket, message_param) (segway_configure_load(socket, 0x04, convert_to_ieee754(message_param)))
#define segway_configure_coastdown(socket, message_param) (segway_configure_load(socket, 0x05, convert_to_ieee754(message_param)))
#define segway_configure_max_turn_rate(socket, message_param) (segway_configure_load(socket, 0x06, convert_to_ieee754(message_param)))
#define segway_configure_max_turn_accel(socket, message_param) (segway_configure_load(socket, 0x07, convert_to_ieee754(message_param)))
#define segway_configure_tire_diameter(socket, message_param) (segway_configure_load(socket, 0x08, convert_to_ieee754(message_param)))
#define segway_configure_wheel_track_width(socket, message_param) (segway_configure_load(socket, 0x09, convert_to_ieee754(message_param)))
#define segway_configure_transmission_ratio(socket, message_param) (segway_configure_load(socket, 0x0a, convert_to_ieee754(message_param)))
#define segway_configure_input_config_bitmap(socket, message_param) (segway_configure_load(socket, 0x0b, message_param))
#define segway_configure_eth_ip(socket, message_param) (segway_configure_load(socket, 0x0c, message_param))
#define segway_configure_eth_port(socket, message_param) (segway_configure_load(socket, 0x0d, message_param))
#define segway_configure_eth_subnet(socket, message_param) (segway_configure_load(socket, 0x0e, message_param))
#define segway_configure_eth_gatway(socket, message_param) (segway_configure_load(socket, 0x0f, message_param))
#define segway_configure_feedback_bitmap1(socket, message_param) (segway_configure_load(socket, 0x10, message_param))
#define segway_configure_feedback_bitmap2(socket, message_param) (segway_configure_load(socket, 0x11, message_param))
#define segway_configure_feedback_bitmap3(socket, message_param) (segway_configure_load(socket, 0x12, message_param))
#define segway_configure_force_config_feedback_bitmap(socket, message_param) (segway_configure_load(socket, 0x13, message_param))
#define segway_configure_audio_command(socket, message_param) (segway_configure_load(socket, 0x14, message_param))
#define segway_configure_operational_mode(socket, message_param) (segway_configure_load(socket, 0x15, message_param))
#define segway_configure_sp_faultlog(socket, message_param) (segway_configure_load(socket, 0x16, message_param))
#define segway_configure_reset_integrator(socket, message_param) (segway_configure_load(socket, 0x17, message_param))
#define segway_configure_reset_params_to_default(socket, message_param) (segway_configure_load(socket, 0x18, message_param))

struct segway_struct
{
  __u32 fault_status_word1;
  __u32 fault_status_word2;
  __u32 fault_status_word3;
  __u32 fault_status_word4;
  __u32 mcu_0_fault_status;
  __u32 mcu_1_fault_status;
  __u32 mcu_2_fault_status;
  __u32 mcu_3_fault_status;
  __u32 frame_count;
  __u32 operational_state;
  __u32 dynamic_response;
  __u32 min_propulsion_batt_soc;
  __u32 aux_batt_soc;
  __u32 inertial_x_acc_g;
  __u32 inertial_y_acc_g;
  __u32 inertial_x_rate_rps;
  __u32 inertial_y_rate_rps;
  __u32 inertial_z_rate_rps;
  __u32 pse_pitch_deg;
  __u32 pse_pitch_rate_dps;
  __u32 pse_roll_deg;
  __u32 pse_roll_rate_dps;
  __u32 pse_yaw_rate_dps;
  __u32 pse_data_is_valid;
  __u32 yaw_rate_limit_rps;
  __u32 vel_limit_mps;
  __u32 linear_accel_mps2;
  __u32 linear_vel_mps;
  __u32 differential_wheel_vel_rps;
  __u32 right_front_vel_mps;
  __u32 left_front_vel_mps;
  __u32 right_rear_vel_mps;

  __u32 left_rear_vel_mps;
  __u32 right_front_pos_m;
  __u32 left_front_pos_m;
  __u32 right_rear_pos_m;
  __u32 left_rear_pos_m;
  __u32 linear_pos_m;
  __u32 right_front_current_A0pk;
  __u32 left_front_current_A0pk;
  __u32 right_rear_current_A0pk;
  __u32 left_rear_current_A0pk;
  __u32 max_motor_current_A0pk;
  __u32 right_front_current_limit_A0pk;
  __u32 left_front_current_limit_A0pk;
  __u32 right_rear_current_limit_A0pk;
  __u32 left_rear_current_limit_A0pk;
  __u32 min_motor_current_limit_A0pk;
  __u32 front_base_batt_1_soc;
  __u32 front_base_batt_2_soc;
  __u32 rear_base_batt_1_soc;
  __u32 rear_base_batt_2_soc;
  __u32 front_base_batt_1_temp_degC;
  __u32 front_base_batt_2_temp_degC;
  __u32 rear_base_batt_1_temp_degC;
  __u32 rear_base_batt_2_temp_degC;
  __u32 vel_target_mps;
  __u32 yaw_rate_target_rps;
  __u32 fram_vel_limit_mps;
  __u32 fram_accel_limit_mps2;
  __u32 fram_decel_limit_mps2;
  __u32 fram_dtz_decel_limit_mps2;
  __u32 fram_coastdown_decel_mps2;
  __u32 fram_yaw_rate_limit_rps;

  __u32 fram_yaw_accel_limit_rps2;
  __u32 fram_tire_diameter_m;
  __u32 fram_wheel_track_width_m;
  __u32 fram_transmission_ratio;
  __u32 fram_config_bitmap;
  __u32 fram_eth_ip_address;
  __u32 fram_eth_port_number;
  __u32 fram_eth_subnet_mask;
  __u32 fram_eth_gateway;
  __u32 user_feedback_bitmap_1;
  __u32 user_feedback_bitmap_2;
  __u32 user_feedback_bitmap_3;
  __u32 aux_batt_voltage_V;
  __u32 aux_batt_current_A;
  __u32 aux_batt_temp_degC;
  __u32 abb_system_status;
  __u32 aux_batt_status;
  __u32 aux_batt_faults;
  __u32 battery_voltage_7p2V;
  __u32 sp_sw_build_id;
  __u32 uip_sw_build_id;
};

extern int segway_motion_set(int can_socket, float velocity, float yaw, int scale_value);
extern int segway_configure_load(int socket, __u32 message_id, __u32 message_param);
extern int segway_init(int can_socket, struct segway_struct *segway_status);
extern int segway_config_update(struct can_frame *frame);
extern int segway_config_decode_arch_fault(__u32 fault_status_word1, char *fault_message);
extern int segway_config_decode_critical_fault(__u32 fault_status_word1, char *fault_message);
extern int segway_config_decode_comm_fault(__u32 fault_status_word2, char *fault_message);
extern int segway_config_decode_internal_fault(__u32 fault_status_word2, char *fault_message);
extern int segway_config_decode_sensors_fault(__u32 fault_status_word3, char *fault_message);
extern int segway_config_decode_bsa_fault(__u32 fault_status_word3, char *fault_message);
extern int segway_config_decode_mcu_fault(__u32 fault_status_word4, char *fault_message);
extern int segway_config_decode_mcu_message(__u32 mcu_fault_status, char *fault_message);

#endif
