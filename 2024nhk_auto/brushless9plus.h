// from brushless8
// brushless9: to memorize absolute coordination
// made by Morinaga

#ifndef BRUSH_LESS_H_
#define BRUSH_LESS_H_
#include "mbed.h"
#include "nomutexcan.h"

// srart change by Morinaga
constexpr short DEGREE_RECOGNITION_REVOLUTION = 4000;
constexpr short DEGREE_MAX = 8191;
constexpr short DEGREE_MIN = 0;
// end   change by Morinaga
            

//#define CAN_Send_ID 0x200
const int CAN_SEND_ID[] = {0x200, 0x1FF};
const int CAN_RECEIVE_ID[] = {
    0x201, 0x202, 0x203, 0x204,
    0x205, 0x206, 0x207, 0x208
};


struct SendData {
    short speed[8];    
};

struct ReceiveData {
    short degree;
    short rpm;
    short TC;
    char temp;
    char none;
    int abs_degree = 0;
};

class BrushLess {
private:
    SendData S;
    NoMutexCAN can;
    void CanIntr();
public:
    ReceiveData R[8]; 

    BrushLess(PinName CAN_RX, PinName CAN_TX);
    void Init();
    void SetSpeed(int, float); //id:0~7
    bool Write();
};
#endif