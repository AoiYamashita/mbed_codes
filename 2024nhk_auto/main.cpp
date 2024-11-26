#include "mbed.h"
#include "brushless9plus.h"

RawSerial RasPi(USBTX,USBRX);

#define PI 3.14159265359

class OutoRobotLeg{
    public:
    BrushLess Bl;
    Timer T;
    int id[4];
    double rpms[4], Proportional[4], Integral[4];
    double gainP, gainI, gainD;
    OutoRobotLeg(int right,int left,int forward,int back):
    Bl(PA_11,PA_12),
    T(),
    id{right,left,forward,back},
    rpms{0,0,0,0},
    Proportional{0,0,0,0},
    Integral{0,0,0,0}
    {//bl can ids
        /*  forward
                _
        left |==|==| right
                |
                - back
        */
        Bl.Init();
    }
    void pid_gains(double p,double i,double d){
        gainP = p;
        gainI = i;
        gainD = d;
    }
    void pidCallBack(){
        T.stop();
        double dt = T.read_us();
        double Derivative = 0;
        for(int i = 0;i < 4;i++){
            Bl.SetSpeed(id[i],0);
            Bl.Write();
            Derivative = (Proportional[i] - (rpms[i] - Bl.R[id[i]].rpm))/dt;
            Integral[i] += (Proportional[i] + (rpms[i] - Bl.R[id[i]].rpm))*dt/2;
            Proportional[i] = rpms[i] - Bl.R[id[i]].rpm;
            Bl.SetSpeed(id[i],gainP*Proportional[i]+gainD*Derivative+gainI*Integral[i]);
        }
        bool a = Bl.Write();

        RasPi.printf("%lf\n",gainP*Proportional[1]+gainD*Derivative+gainI*Integral[1]);
        T.reset();
        T.start();
    }
    void setMove(double Rad,int Speed){//forward is 0, back is PI
        double yaxis = sin(Rad);
        double xaxis = cos(Rad);
        double weight[4] = {1,-1,1,-1};
        for(int i = 0;i < 4;i++){
            if(i <= 1){
                rpms[i] = yaxis*weight[i]*Speed;
                continue;
            }
            rpms[i] = xaxis*weight[i]*Speed;
        }
    }
    void Rotate(int Speed){
        double weight[4] = {1,1,0.25,1.375};
        for(int i = 0;i < 4;i++){
            rpms[i] = weight[i]*Speed;
        }
    }
};

OutoRobotLeg Leg(0,2,3,1);

int cont = 0;

void cb(){
    cont = 0;
    char c = RasPi.getc();
    int speed = 100;
    int Rspeed = 600;
    switch (c) {
    case 'p':
        Leg.setMove(3.0/4.0*PI, speed);
        break;
    case 'd':
        Leg.setMove(0, speed);
        break;
    case 's':
        Leg.setMove(PI/2, speed);
        break;
    case 'a':
        Leg.setMove(PI, speed);
        break;
    case 'w':
        Leg.setMove(-PI/2, speed);
        break;
    case 'r':
        Leg.Rotate(Rspeed);
        break;
    case 'e':
        Leg.Rotate(-Rspeed);
        break;
    case 'q':
        Leg.setMove(0, 0);
        break;
    }
    
}

// main() runs in its own thread in the OS
int main()
{
    RasPi.baud(115200);
    RasPi.attach(&cb);
    Leg.pid_gains(0.001, 0., 0);
    while (true) {
        Leg.pidCallBack();
        /*if(cont > 50){
            Leg.setMove(0, 0);
        }*/
        wait_us(1000);
        cont++;
    }
}

