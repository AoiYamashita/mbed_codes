#include "mbed.h"
#include "brushless9plus.h"
#include <cstdio>

RawSerial raspi(USBTX,USBRX);
//初期
DigitalOut In0(PB_15);//in2
DigitalOut In1(PB_14);//in3
DigitalOut In2(PB_13);//in4
//追加分
DigitalOut In3(PA_7);//in2
DigitalOut In4(PB_6);//in3
DigitalOut In5(PA_6);//in4

class SwerveDrive{
    public:
    int GearRatios = 3;
    double gain[6];
    int canId[2],Move[2],Proportional[2],Integral[2];
    BrushLess Bl;
    Timer T;
    SwerveDrive(PinName Rx,PinName Tx):
    T(),
    canId{0,0},
    Move{0,0},
    Proportional{0,0},
    Integral{0,0},
    Bl(Rx,Tx)
    {
        Bl.Init();
        T.start();
    }
    void Init(int SteerId,int DriveId){
        canId[0] = SteerId;
        canId[1] = DriveId;
    }
    void SetGain(double SP,double SI,double SD,double DP,double DI,double DD){
        gain[0] = SP;
        gain[1] = SI;
        gain[2] = SD;
        gain[3] = DP;
        gain[4] = DI;
        gain[5] = DD;
    }
    void SetMove(int Deg,int Speed){
        int sflag = 1;
        if((3600+Deg)%360 >= 180)sflag = -1;
        Move[0] = (3600+Deg)%180;
        Move[1] = Speed*sflag;
    }
    void CallBack(){
        Bl.SetSpeed(canId[0],0);
        Bl.SetSpeed(canId[1],0);
        Bl.Write();
        T.stop();
        double dt = T.read_us();
        double Derivative = 0;
        double DegParValue = 10.0/8191.0;
        double Pv[2] = {(Bl.R[canId[0]].abs_degree*DegParValue/GearRatios),
                        double(Bl.R[canId[1]].rpm)};
        for(int i = 0;i < 2;i++){
            Derivative = (Proportional[i] - (Move[i] - Pv[i]))/dt;
            Integral[i] += (Proportional[i] + (Move[i] - Pv[i]))*dt/2;
            Proportional[i] = Move[i] - Pv[i];
            Bl.SetSpeed(canId[i],gain[0+3*i]*Proportional[i]+gain[1+3*i]*Integral[i]+gain[2+3*i]*Derivative);
            //pc.printf("%lf|",gain[0+3*i]*Proportional[i]+gain[1+3*i]*Integral[i]+gain[2+3*i]*Derivative);
        }
        bool a = Bl.Write();
        //pc.printf("%lf|%lf\n",Pv[0],Pv[1]);
        //raspi.printf("%d\n",Bl.R[canId[1]].rpm);
        //raspi.printf("%d\n",int(Bl.R[canId[0]].abs_degree*DegParValue/GearRatios));
        if(int(Bl.R[canId[0]].abs_degree*DegParValue/GearRatios) > 360 || int(Bl.R[canId[0]].abs_degree*DegParValue/GearRatios) < -360){
            Bl.SetSpeed(canId[0], 0);
            Bl.Write();
        }
        T.reset();
        T.start();
    }
};

bool isName(char p[]){
    char k[] = "name";
    int i = 0;
    for(i = 0;i < 4;i ++){
        if(k[i] != p[i])return false;
    }
    return true;
}

bool isAir(char p[]){
    char k[] = "Air";
    int i = 0;
    for(i = 0;i < 3;i ++){
        if(k[i] != p[i])return false;
    }
    return true;
}

double DegParValue = 360/(19.0*8191.0);

class Sling{
    public:
    int id0,id1,reverseFlag,SlingSt,stackCnt;
    double releasePoint,speed,Pro,Int,initdeg;
    BrushLess Bl;
    Timer T;
    Sling(PinName Rx,PinName Tx,int id0_,int id1_):
    Bl(Rx,Tx),
    T()
    {
        id0 = id0_;
        id1 = id1_;
        releasePoint = 90.0;
        Bl.Init();
        reverseFlag = 2;
        speed = -1;
        SlingSt = 0;//0:待機 1:発射 2:リセット
        Pro = 0;
        Int = 0;

    };

    void cb(){
        if(initdeg == 0)initdeg = Bl.R[id0].abs_degree*DegParValue;
        T.stop();
        if(reverseFlag == 2){
            Bl.SetSpeed(id0,0);
            Bl.SetSpeed(id1,0);
            Bl.Write();
            return;
        }
        if(reverseFlag == 1){
            resetCb();
            if(Bl.R[id0].abs_degree*DegParValue-initdeg <= 10){
                reverseFlag = 2;
                return;
            }
            Bl.SetSpeed(id0, speed);
            Bl.SetSpeed(id1, -speed);
            Bl.Write();
        }
        if(Bl.R[id0].abs_degree*DegParValue < releasePoint && reverseFlag == 0){
            Bl.SetSpeed(id0, speed);
            Bl.SetSpeed(id1, -speed);
        }else{
            Bl.SetSpeed(id0,0);
            Bl.SetSpeed(id1,0);
            reverseFlag = 1;
        }
        if(initdeg > Bl.R[id0].abs_degree*DegParValue){
            speed = 1;
        }
        //raspi.printf("%d|%lf,%lf\n",reverseFlag ,initdeg,Bl.R[id0].abs_degree*DegParValue);
        Bl.Write();
        T.reset();
        T.start();
    }
    void resetCb(){
        Bl.SetSpeed(id0, 0);
        Bl.SetSpeed(id1, 0);
        Bl.Write();
        T.stop();
        double targetRpm = -500;
        double dt = T.read_us();
        double Der = (Pro-(targetRpm - Bl.R[id0].rpm))/dt;
        Int += (Pro+(targetRpm - Bl.R[id0].rpm))*dt/2;
        Pro = targetRpm - Bl.R[id0].rpm;
        //0.0005,0.0000000001,0.005
        speed =  0.00135*Pro+0.000000001*Int+0.005*Der;
        T.reset();
        T.start();
    }
};

Sling sling(PA_11,PA_12,2,3);
void Air(int num){
    switch(num){
        case 0:
        In0 = !In0;
        break;
        case 1:
        In1 = !In1;
        break;
        case 2:
        In2 = !In2;
        break;
        case 3:
        In3 = !In3;
        break;
        case 4:
        In4 = !In4;
        break;
        case 5:
        In5 = !In5;
        break;
        case 8:
        sling.initdeg = 0;
        sling.reverseFlag = 0;
        break;
    }
}


SwerveDrive SD0(PA_11,PA_12);

char buf[124];
int speed = 0;
int deg = 0;

int id = 2;

void cb(){
    char *a = buf;
    while(raspi.readable()){
        *a = raspi.getc();
        //wait_us(1000000/115200);
        wait_us(10000000/115200);
        a++;
    }
    *a = '\0';
    if(isName(buf)){
        raspi.printf("%d",id);
    }
    if(isAir(buf)){
        int airId = 0;
        sscanf(buf,"Air:%d",&airId);
        Air(airId);
        raspi.printf("change %d",airId);
    }
    //raspi.printf("%s\n",&buf[0]);
    sscanf(buf,"speed:%d:deg:%d",&speed,&deg);
    //raspi.printf("%d::%d",speed,deg);
    SD0.SetMove(deg, speed);
}

// main() runs in its own thread in the OS
int main()
{
    raspi.baud(115200);
    SD0.Init(0,1);
    //SD0.SetGain(0.0073,0.000000005,0.0,0.0005,0.0000000001,0.005);
    SD0.SetGain(0.018,0.000000005,0.0,0.0005,0.0000000001,0.005);//id2
    raspi.attach(&cb);
    while (true) {
        SD0.CallBack();
        sling.cb();
        wait_us(1000);
        //Bl.Write();
    }
}

