#include "mbed.h"
#include "brushless9plus.h"
#include <cstdio>

//UnbufferedSerial PC(USBTX,USBRX,9600);
I2C Controll(D4,D5);

#define PI 3.14159265359
#define I2cDataSize 8
#define I2cSlaveId 8

class UkebaRobo{
    public:
    BrushLess Bl;
    Timer T;
    double Pgain,Igain,Dgain;
    double Pro[4];
    double Inte[4];
    double targets[4];
    double Position[3];//x,y,rotate
    double ArmPgain,ArmIgain,ArmDgain;
    int Id[4];
    UkebaRobo(int forward,int rightback,int leftBack,double P,double I,double D):
    Bl(D10,D2),
    T(),
    Pgain(P),
    Igain(I),
    Dgain(D),
    Pro{0,0,0,0},
    Inte{0,0,0,0},
    targets{0,0,0,0},
    Position{0,0,0},
    Id{forward,rightback,leftBack}
    {
        Bl.Init();
        T.start();
    }
    void setArmGains(int ID,double pGain,double iGain,double dGain){
        ArmPgain = pGain;
        ArmDgain = dGain;
        ArmIgain = iGain;
        Id[3] = ID;
    }
    void pidCallBack(){
        T.stop();
        double dt = T.read_us();

        Bl.SetSpeed(Id[3],0);
        Bl.Write();
        double ArmD = (Pro[3] - (targets[3] - Bl.R[Id[3]].rpm))/dt;
        Inte[3] += (Pro[3] + (targets[3] - Bl.R[Id[3]].rpm))*dt/2;
        Pro[3] = targets[3] - Bl.R[Id[3]].rpm;
        Bl.SetSpeed(Id[3],ArmPgain*Pro[3]+ArmDgain*ArmD+ArmIgain*Inte[3]);
        if((PI*Bl.abs_degree[Id[3]]/(8191.0*19.0*3.0) < -1.3 && ArmPgain*Pro[3]+ArmDgain*ArmD+ArmIgain*Inte[3] < 0) || 
            (PI*Bl.abs_degree[Id[3]]/(8191.0*19.0*3.0) >= 0 && ArmPgain*Pro[3]+ArmDgain*ArmD+ArmIgain*Inte[3] > 0)){
            targets[3] = 0;
            ArmD = (Pro[3] - (targets[3] - Bl.R[Id[3]].rpm))/dt;
            Inte[3] += (Pro[3] + (targets[3] - Bl.R[Id[3]].rpm))*dt/2;
            Pro[3] = targets[3] - Bl.R[Id[3]].rpm;
            Bl.SetSpeed(Id[3],ArmPgain*Pro[3]+ArmDgain*ArmD+ArmIgain*Inte[3]);
        }
        
        for(int i = 0;i < 3;i++){
            Bl.SetSpeed(Id[i],0);
            Bl.Write();
            double Derivative = (Pro[i] - (targets[i] - Bl.R[Id[i]].rpm))/dt;
            Inte[i] += (Pro[i] + (targets[i] - Bl.R[Id[i]].rpm))*dt/2;
            Pro[i] = targets[i] - Bl.R[Id[i]].rpm;
            Bl.SetSpeed(Id[i],Pgain*Pro[i]+Dgain*Derivative+Igain*Inte[i]);
        }
        Bl.Write();

        double VecX,VecY,VecW;
        getVec(&VecX, &VecY, &VecW);
        // dt/1000000.0 : us -> s
        Position[0] += sin(Position[2])*VecY*dt/1000000.0 + cos(Position[2])*VecX*dt/1000000.0;
        Position[1] += cos(Position[2])*VecY*dt/1000000.0 - sin(Position[2])*VecX*dt/1000000.0;;
        Position[2] += VecW*dt/1000000.0;
        
        // char ArmBuff[127];
        // int size = sprintf(ArmBuff,"%lf\n", PI*Bl.abs_degree[Id[3]]/(8191.0*19.0*3.0));
        // PC.write(ArmBuff,size);
        T.reset();
        T.start();
    }

    void getVec(double* vx,double* vy,double* w){
        *vx = (Bl.R[0].rpm*1.0 + Bl.R[1].rpm*cos(-PI*2.0/3.0) + Bl.R[2].rpm*cos(PI*2.0/3.0))/3.0;
        *vy = (Bl.R[0].rpm*0.0 + Bl.R[1].rpm*sin(-PI*2.0/3.0) + Bl.R[2].rpm*sin(PI*2.0/3.0))/2.0;
        *w = (Bl.R[0].rpm + Bl.R[1].rpm + Bl.R[2].rpm)/3.0;

        double GearRate = 2.0;
        *vx /= 18*60*GearRate;
        *vy /= 18*60*GearRate;
        *w /= 18*60*GearRate;

        double wheelSize = 60*PI;
        double RobotSize = 340*PI;

        *vx *= wheelSize;
        *vy *= wheelSize;
        *w *= PI*wheelSize/RobotSize;
    }
    void SetArmSpeed(double Speed){
        targets[3] = Speed;
    }
    void setMoveAndRotate(double Rad,int Speed,int RSpeed){//forward : 0,| back : PI
        for(int i = 0;i < 3;i++)Inte[i] = 0.0;
        double yaxis = Speed*sin(Rad);
        double xaxis = Speed*cos(Rad);
        targets[0] = xaxis*cos(0.0)         + yaxis*0.0              + RSpeed;
        targets[1] = xaxis*cos(-PI*2.0/3.0) + yaxis*sin(-PI*2.0/3.0) + RSpeed;
        targets[2] = xaxis*cos(PI*2.0/3.0)  + yaxis*sin(PI*2.0/3.0)  + RSpeed;
    }
};

UkebaRobo Robo(0,1,2,0.001, 0.000000000000001, 0.01);

/*void cb(){
    int speed = 5000;
    int Rspeed = 0;
    char c;
    PC.read(&c,1);
    switch(c){
        case 'w':
        Robo.setMoveAndRotate(PI/2, speed, Rspeed);
        break;
        case 'a':
        Robo.setMoveAndRotate(PI, speed, Rspeed);
        break;
        case 'd':
        Robo.setMoveAndRotate(0.0, speed, Rspeed);
        break;
        case 's':
        Robo.setMoveAndRotate(-PI/2, speed, Rspeed);
        break;
        case 'r':
        Robo.setMoveAndRotate(-PI/2, 0, 1000);
        break;
        case 'e':
        Robo.setMoveAndRotate(-PI/2, 0, -1000);
        break;
        case 'q':
        Robo.setMoveAndRotate(-PI/2, 0, 0);
        Robo.SetArmSpeed(0);
        break;
        case 'l':
        Robo.SetArmSpeed(700);
        break;
        case 'h':
        Robo.SetArmSpeed(-700);
        break;

    }
}*/

DigitalOut LED(LED1);

void ControllCb(void){
    char I2Cdata[I2cDataSize];
    Controll.read((I2cSlaveId << 1),I2Cdata,I2cDataSize);
    wait_us(100);
    int x = (I2Cdata[1]<<8)|I2Cdata[2];
    int y = (I2Cdata[3]<<8)|I2Cdata[4];
    if(I2Cdata[0]%2 == 1) x *= -1.0;
    if(I2Cdata[0]/2 >= 1) y *= -1.0;

    if(I2Cdata[7] == 0)
    {
        LED = 0;
        return;
    }
    LED = 1;
    int Rspeed = 3000;
    if(I2Cdata[5] == 1)Rspeed *= -1;
    if(I2Cdata[5] == 2)Rspeed *= 1;
    Robo.setMoveAndRotate((double)(atan2(x,y)+PI/2),sqrt(x*x+y*y)*8.0, Rspeed);

    int ArmSpeed = 400;
    if(I2Cdata[6] == 1)ArmSpeed *= 1;
    if(I2Cdata[6] == 2)ArmSpeed *= -1;
    Robo.SetArmSpeed(ArmSpeed);
}

// main() runs in its own thread in the OS
int main()
{
    Controll.frequency(400000);
    //PC.attach(&cb);
    Robo.setArmGains(3,0.0012,0.0000000001,0.000006);
    while (true) {
        Robo.pidCallBack();
        wait_us(500);
        ControllCb();
        wait_us(500);
    }
}

