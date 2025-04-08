#ifndef KeyBoard_H
#define KeyBoard_H

#include "main.h"

typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    union {
        uint16_t key_code;
        struct 
        {
          uint16_t W : 1;
          uint16_t S : 1;
          uint16_t A : 1;
          uint16_t D : 1;
          uint16_t SHIFT : 1;
          uint16_t CTRL : 1;
          uint16_t Q : 1;
          uint16_t E : 1;
          uint16_t R : 1;
          uint16_t F : 1;
          uint16_t G : 1;
          uint16_t Z : 1;
          uint16_t X : 1;
          uint16_t C : 1;
          uint16_t V : 1;
          uint16_t B : 1;
        } bit;
    } key_code;
    uint16_t reserved;
}keyboard_t;

#endif