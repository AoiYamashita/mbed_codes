// from brushless8
// brushless9: to memorize absolute coordination
// made by Morinaga

#include "brushless9plus.h"

BrushLess::BrushLess(PinName CAN_RX, PinName CAN_TX) : can(CAN_RX, CAN_TX, 1000000){
}

void BrushLess::CanIntr() {
    CANMessage msg;
    can.read(msg);
    char data[8] = {msg.data[1],msg.data[0],msg.data[3],msg.data[2],msg.data[5],msg.data[4],msg.data[6], msg.data[7]};
    for (int i = 0; i < 8; i++) {
        if (msg.id == CAN_RECEIVE_ID[i]) {
            
            short degree_prev = R[i].degree;

            memcpy((ReceiveData*)&(R[i]), data, 8);

            short deffer_0 = R[i].degree - degree_prev;
            short deffer_1 = (((int)DEGREE_MAX/2 + R[i].degree) % DEGREE_MAX-((int)DEGREE_MAX/2 + degree_prev)%DEGREE_MAX);

            //R[i].abs_degree = deffer_1;
            
            if(abs(deffer_0) < abs(deffer_1)) R[i].abs_degree += deffer_0;
            else R[i].abs_degree += deffer_1;

            /*if(abs(R[i].abs_degree) < DEGREE_MAX*2/3){
                if(R[i].abs_degree > 0){
                    R[i].abs_degree = R[i].degree;
                }else{
                    R[i].abs_degree = (R[i].degree - DEGREE_MAX);
                }
                
            }*/
            
        }
    }
}

void BrushLess::Init() {
    can.attach(callback(this, &BrushLess::CanIntr));
}

void BrushLess::SetSpeed(int id, float speed) {
    float sp = speed;
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    S.speed[id] = 10000 * sp;
}

bool BrushLess::Write() {
    for (int i = 0; i < 2; i++) {
        char b[8];
        memcpy((char*)b, (&S)->speed + (4 * i), 8);
        char send[8] = {b[1], b[0], b[3],b[2],b[5],b[4],b[7],b[6]};
        if (can.write(CANMessage(CAN_SEND_ID[i], send, 8)) == 0) {
        return false;
        }
    }
    return true;
}