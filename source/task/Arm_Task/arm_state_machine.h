#ifndef ARM_STATE_MACHINE_H
#define ARM_STATE_MACHINE_H

typedef enum{
    #define Arm_State(name ,func) name,
    #include "arm_control_state.def"
    #undef Arm_State
} arm_control_state_t;
#endif
