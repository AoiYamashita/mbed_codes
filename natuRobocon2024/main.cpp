#include "mbed.h"
#include "brushless9plus.h"

RawSerial RasPi(USBTX,USBRX);

#define PI 3.14159265359

int argToPlse(int arg){
    return int(arg*200/18+500);
}

class AutoRobot{
    public:
    Timer T;
    PwmOut PWM_No0;
    PwmOut PWM_No1;
    AnalogIn SlidePin;
    BrushLess Bl;
    double slideFlag;
    int slideCount;
    int armFlag;
    int arg;
    int id[5];
    int isopen[2];
    double targets[5], Proportional[5], Integral[5];
    double gainP, gainI, gainD;
    double gainPs, gainIs, gainDs;
    AutoRobot(int leftforward,int rightforward,int rightback,int leftback,int slide):
    PWM_No1(PC_3),
    PWM_No0(PC_0),
    SlidePin(A0),
    Bl(PA_11,PA_12),
    T(),
    //bl can ids
    id{leftforward,rightforward,rightback,leftback,slide},
    targets{0,0,0,0,0},
    Proportional{0,0,0,0,0},
    Integral{0,0,0,0,0},
    isopen{0,0}
    {
        PWM_No1.period_us(20000);
        PWM_No0.period_us(20000);
        Bl.Init();
        slideFlag = -1;
        slideCount = 0;
        armFlag = 1;
        
        /*  
        leftforward  rightforward
            \----|----/
            |    |    |
            |    |    |
            /----|----\
        leftback     rightback
        */
    }
    void callbackInterPin(){
        double value = SlidePin.read();
        if(slideCount > 1000 && value < 0.5*slideFlag && targets[4] != 0){
            targets[4] = 0.0;
            openArm(-armFlag);
            armFlag *= -1;
        }
        slideFlag = value;
        slideCount ++;
        //RasPi.printf("%lf\n",SlidePin.read());
        return;
    }
    void pid_gains(double p,double i,double d){
        gainP = p;
        gainI = i;
        gainD = d;
    }
    void setGain(double pgain,double igain,double dgain,int Armrotate){
        gainPs = pgain;
        gainIs = igain;
        gainDs = dgain;
        arg = Armrotate;
    }
    void openArm(int num){
        if(num < 0){
            isopen[0] = 1-isopen[0];
        }
        else{
            
            isopen[1] = 1-isopen[1];
        }
    }
    void closeAllArm(){
        if(isopen[0] ){
            isopen[0] = 1-isopen[0];
        }
        if(isopen[1]){
            
            isopen[1] = 1-isopen[1];
        }
    }
    void pidCallBack(){
        callbackInterPin();
        PWM_No0.pulsewidth_us(argToPlse(isopen[0]*arg));
        PWM_No1.pulsewidth_us(argToPlse(isopen[1]*arg));
        T.stop();
        double dt = T.read_us();
        double Derivative = 0;
        Bl.SetSpeed(id[4],0);
        int a = Bl.Write();
        Derivative = (Proportional[4] - (targets[4] - Bl.R[id[4]].rpm))/dt;
        Integral[4] += (Proportional[4] + (targets[4] - Bl.R[id[4]].rpm))*dt/2;
        Proportional[4] = targets[4] - Bl.R[id[4]].rpm;
        Bl.SetSpeed(id[4],gainPs*Proportional[4]+gainDs*Derivative+gainIs*Integral[4]);
        wait_us(500);
        Bl.Write();
        //RasPi.printf("%d:%f\n",Bl.R[id[4]].rpm,gainP*Proportional[4]+gainD*Derivative+gainI*Integral[4]);

        for(int i = 0;i < 4;i++){
            Bl.SetSpeed(id[i],0);
            Bl.Write();
            Derivative = (Proportional[i] - (targets[i] - Bl.R[id[i]].rpm))/dt;
            Integral[i] += (Proportional[i] + (targets[i] - Bl.R[id[i]].rpm))*dt/2;
            Proportional[i] = targets[i] - Bl.R[id[i]].rpm;
            Bl.SetSpeed(id[i],gainP*Proportional[i]+gainD*Derivative+gainI*Integral[i]);
        }
        Bl.Write();

        T.reset();
        T.start();
    }
    void getVec(double* vx,double* vy,double* w){
        //平均化 and par min -> par sec
        *vx = (Bl.R[id[0]].rpm - Bl.R[id[1]].rpm - Bl.R[id[2]].rpm + Bl.R[id[3]].rpm)/4.0;
        *vy = (Bl.R[id[0]].rpm + Bl.R[id[1]].rpm - Bl.R[id[2]].rpm - Bl.R[id[3]].rpm)/4.0;
        *w = (Bl.R[id[0]].rpm + Bl.R[id[1]].rpm + Bl.R[id[2]].rpm + Bl.R[id[3]].rpm)/4.0;
        *vx /= 60.0*18*sqrt(2);
        *vy /= 60.0*18*sqrt(2);
        *w /= 60.0*18;
        //mmに直す
        *vx *= 195.0;
        *vy *= 195.0;
        *w *= 195.0;
        *w /= 1414.0;
        //digに直す
        //*w *= 180.0;
        //argに直す
        *w *= PI;
    }
    void setMove(double Rad,int Speed){//forward : 0,| back : PI
        double yaxis = Speed*sin(Rad)*1.0/sqrt(2);
        double xaxis = Speed*cos(Rad)*1.0/sqrt(2);
        double weight[4] = {1,1,1,1};
        targets[0] = xaxis*weight[0] + yaxis*weight[0];
        targets[1] = -xaxis*weight[1] + yaxis*weight[1];
        targets[2] = -xaxis*weight[2] - yaxis*weight[2];
        targets[3] = xaxis*weight[3] - yaxis*weight[3];
    }
    void Rotate(int Speed){
        double weight[4] = {1,1,1,1};
        for(int i = 0;i < 4;i++){
            targets[i] = weight[i]*Speed;
        }
    }
    void setMoveRotate(double Rad,int Speed,int RSpeed){
        double yaxis = Speed*sin(Rad)*1.0/sqrt(2);
        double xaxis = Speed*cos(Rad)*1.0/sqrt(2);
        double weight[4] = {1,1,1,1};
        targets[0] =  xaxis*weight[0] + yaxis*weight[0] + weight[0]*RSpeed;
        targets[1] = -xaxis*weight[1] + yaxis*weight[1] + weight[1]*RSpeed;
        targets[2] = -xaxis*weight[2] - yaxis*weight[2] + weight[2]*RSpeed;
        targets[3] =  xaxis*weight[3] - yaxis*weight[3] + weight[3]*RSpeed;
    }
    void slide(int pose){
        closeAllArm();
        targets[4] = armFlag*pose;
    }
};

bool isArm(char p[]){
    char k[] = "Arm";
    int i = 0;
    for(i = 0;i < 3;i ++){
        if(k[i] != p[i])return false;
    }
    return true;
}

bool isSlide(char p[]){
    char k[] = "Slide";
    int i = 0;
    for(i = 0;i < 5;i ++){
        if(k[i] != p[i])return false;
    }
    return true;
}

bool isMove(char p[]){
    char k[] = "move";
    int i = 0;
    for(i = 0;i < 4;i ++){
        if(k[i] != p[i])return false;
    }
    return true;
}

AutoRobot Auto(0,1,2,3,4);


double pose_g = 0;

int cont = 0;

char buf[124];

void cb(){
    //RawSerial of Raspi
    
    int speed = 0;
    int Rspeed = 0;
    double dig = 0.0;
    char *a = buf;
    while(RasPi.readable()){
        *a = RasPi.getc();
        wait_us(10000000/115200);
        //RasPi.printf("a:%c",*a);
        a++;
    }
    *a = '\0';
    // if(isArm(buf)){
    //     int Id = 0;
    //     sscanf(buf,"Arm:%d",&Id);
    //     Auto.openArm(Id);
    //     //RasPi.printf("change %d",Id);
    //     return;
    // }
    if(isSlide(buf)){
        //sscanf(buf,"Slide:%d",&pose);
        Auto.slideCount = 0;

        Auto.slide(2000);
        //RasPi.printf("change %d",pose);
        return;
    }
    if(isMove(buf)){
        sscanf(buf,"move:%lf,%d,%d",&dig,&speed,&Rspeed);
        Auto.setMoveRotate(dig,speed,Rspeed);
        //RasPi.printf("change %lf,%d,%d\n",dig,speed,Rspeed);
    }
    /*
    int speed = 3000;
    int Rspeed = 1000;

    char c = RasPi.getc();
    switch (c) {
    case 'w':
        Auto.setMove(0, speed);
        break;
    case 's':
        Auto.setMove(PI, speed);
        break;
    case 'd':
        Auto.setMove(PI/2, speed);
        break;
    case 'a':
        Auto.setMove(-PI/2, speed);
        break;
    case 'r':
        Auto.Rotate(-Rspeed);
        break;
    case 'e':
        Auto.Rotate(Rspeed);
        break;
    case '3':
        Auto.openArm(1);
        break;
    case '0':
        Auto.openArm(-1);
        break;
    case 'p':
        //pose_g += int(19*8191/8);
        Auto.slide(1000);
        break;
    case 'm':
        //pose_g -= int(19*8191/8);
        Auto.slide(-1000);
        break;
    case 'o':
        if(int(cont/500)%2 == 0){
            Auto.setMove(2*PI*cont/500.0, speed);
        }else Auto.setMove(-2*PI*cont/500.0, speed);
        
        Auto.setMoveRotate(1.57, 100, 200);
        //Auto.Rotate(3000);
        break;
    default:
        Auto.setMove(0, 0);
        Auto.Rotate(0);
        break;
    }*/
    
}

// main() runs in its own thread in the OS
int main()
{
    RasPi.baud(115200);
    RasPi.attach(&cb);
    Auto.pid_gains(0.001    , 0.000000000000001, 0.01);//leg
    Auto.setGain(  0.003, 0.00, 0.000003,54);//arm slide and open digree
    double vx,vy,w;
    while (true) {
        Auto.pidCallBack();
        Auto.getVec(&vx, &vy, &w);
        if(cont % 100 == 0)
            RasPi.printf("%.3lf,%.3lf,%.3lf\n",vx,vy,w);
        wait_us(1000);
        cont++;
    }
}

