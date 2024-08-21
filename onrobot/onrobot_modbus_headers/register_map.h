/* 
 * modbus registers
 *
 * Use 'Read Holding Registers' to read multiple consecutive registers
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */


// Gecko
// preload force is in units of 1/100N
// ultrasonic range is in units of 1/100mm (datasheet typo?)

#define ONROBOT_GECKO_REG_PAD_CONTROL_RW            ((uint16_t) 0)
#define ONROBOT_GECKO_REG_PRELOAD_THRESH_RW         ((uint16_t) 4)
#define ONROBOT_GECKO_REG_PART_DETECT_R             ((uint16_t) 256) // bool 0,1
#define ONROBOT_GECKO_REG_PADS_WORN_R               ((uint16_t) 257) // bool 0,1
#define ONROBOT_GECKO_REG_BUSY_R                    ((uint16_t) 260) // bool 0,1
#define ONROBOT_GECKO_REG_PRELOAD_FORCE_R           ((uint16_t) 261)
#define ONROBOT_GECKO_REG_ULTRASONIC_RANGE_R        ((uint16_t) 262)
#define ONROBOT_GECKO_REG_PAD_POS_R                 ((uint16_t) 263)

#define ONROBOT_GECKO_VAL_PAD_CONTROL_IN            ((uint16_t) 0)
#define ONROBOT_GECKO_VAL_PAD_CONTROL_OUT           ((uint16_t) 1)

#define ONROBOT_GECKO_VAL_PAD_POS_IN                ((uint16_t) 0)
#define ONROBOT_GECKO_VAL_PAD_POS_OUT               ((uint16_t) 1)

#define ONROBOT_GECKO_VAL_PRELOAD_THRESH_50N        ((uint16_t) 0)
#define ONROBOT_GECKO_VAL_PRELOAD_THRESH_90N        ((uint16_t) 1)
#define ONROBOT_GECKO_VAL_PRELOAD_THRESH_120N       ((uint16_t) 2)
#define ONROBOT_GECKO_VAL_PRELOAD_THRESH_DEFAULT ONROBOT_GECKO_REG_PRELOAD_THRESH_120N

// HEX-E/H QC
// cast the force and torque to signed integer
// force is in units of 1/10N
// torque is in units of 1/100 Nm
#define ONROBOT_HEX_REG_ZERO_RW                     ((uint16_t) 0)
#define ONROBOT_HEX_REG_STATUS_R                    ((uint16_t) 257)
#define ONROBOT_HEX_REG_FX_R                        ((uint16_t) 259)
#define ONROBOT_HEX_REG_FY_R                        ((uint16_t) 260)
#define ONROBOT_HEX_REG_FZ_R                        ((uint16_t) 261)
#define ONROBOT_HEX_REG_TX_R                        ((uint16_t) 262)
#define ONROBOT_HEX_REG_TY_R                        ((uint16_t) 263)
#define ONROBOT_HEX_REG_TZ_R                        ((uint16_t) 264)


// RG2 / RG6
// distances are signed integers
// distances are in units of 1/10mm

#define ONROBOT_RG2_REG_TARGET_FORCE_W              ((uint16_t) 0)
#define ONROBOT_RG2_REG_TARGET_WIDTH_W              ((uint16_t) 1)
#define ONROBOT_RG2_REG_CONTROL_W                   ((uint16_t) 2)
#define ONROBOT_RG2_REG_FINGERTIP_OFFSET_R          ((uint16_t) 258)
#define ONROBOT_RG2_REG_DEPTH_R                     ((uint16_t) 263)
#define ONROBOT_RG2_REG_RELATIVE_DEPTH_R            ((uint16_t) 264)
#define ONROBOT_RG2_REG_WIDTH_R                     ((uint16_t) 267)
#define ONROBOT_RG2_REG_STATUS_R                    ((uint16_t) 268)
#define ONROBOT_RG2_REG_WIDTH_WITH_OFFSET_R         ((uint16_t) 275)
#define ONROBOT_RG2_REG_SET_FINGERTIP_OFFSET_W      ((uint16_t) 1031)


#define ONROBOT_RG2_VAL_CONTROL_GRIP                ((uint16_t) 1)
#define ONROBOT_RG2_VAL_CONTROL_STOP                ((uint16_t) 8)
#define ONROBOT_RG2_VAL_CONTROL_GRIP_WITH_OFFSET    ((uint16_t) 16)




