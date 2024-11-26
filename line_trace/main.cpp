#include "mbed.h"

const int CYCLE_MS = 100;

Serial pc(USBTX,USBRX);

class PID
{
    private:

    double Proportional;
    double Integral;
    double Derivative;

    public:

    double P_gain;
    double I_gain;
    double D_gain;

    double set_point;

    double cycle;

    PID(double p,double i,double d){
        P_gain = p;
        I_gain = i;
        D_gain = d;
        cycle = CYCLE_MS;
        cycle /= 1000;
    }

    void update_set_point(double new_setpoint){
        set_point = new_setpoint;
    }

    void update_input_value(double process_value,double* input)
    {
        if(Proportional * (set_point-process_value) < 0){//モーター加速方向の逆転
            Integral = 0;
        }
        Derivative = ((set_point-process_value) - Proportional)/cycle;
        Integral += ((set_point-process_value) - Proportional)*cycle/2;
        Proportional = set_point-process_value;

        *input = P_gain*Proportional + I_gain*Integral + D_gain * Derivative;
    }
};

class Encoder{
    public:

    InterruptIn phaseA;
    Timer timer;
    double time;
    int rpp;
    double rpm;
    int number;

    Encoder(PinName pin,int this_rpp)
    :phaseA(pin)
    {
        timer.start();
        rpp = this_rpp;
        time = 0;
        rpm = 0;
        number = 0;
        phaseA.rise(this,&Encoder::update);
    }

    void update(){
        number ++;
    }

    double get_RPM()
    {
        rpm = (double) number*360/rpp/(timer.read_ms()-time);
        time = timer.read_ms();
        number = 0;
        return rpm;
    }
};

class PhotoRef{
    private:
    
    AnalogIn ref0;
    AnalogIn ref1;
    AnalogIn ref2;
    AnalogIn ref3;
    
    public :

    double values[4];
    
    struct range{
        double max;
        double min;
    }Line_Value_range;

    int last_cencer_value;

    PhotoRef(PinName pin[4])
    :ref0(pin[0]),ref1(pin[1]),ref2(pin[2]),ref3(pin[3])
    {
        double values[] = {0,0,0,0};

        Line_Value_range.max = 0.5;
        Line_Value_range.min = 0;

        last_cencer_value = 0;
    }
    //フォトリフレクタの値を取得。
    void update_Values_location(){
        values[0] = ref0.read();
        values[1] = ref1.read();
        values[2] = ref2.read();
        values[3] = ref3.read();

        for(int i = 0;i < 4;i++){
            if(Line_Value_range.min <= values[i] && values[i] <= Line_Value_range.max){
                last_cencer_value = i+1;
            }
        }
    }
    
    void show_Value(){
        for(int i = 0;i < 4;i++){
            pc.printf("| %d : %lf ",i,values[i]);
        }
        pc.printf("\n");
    }
};

class Motor{
    public:

    DigitalOut In0;
    DigitalOut In1;
    PwmOut Vref;

    PID pid;

    bool pid_;

    double Out_vref;

    Motor(PinName pin[3],bool pid_use)
    :
    In0(pin[0]),In1(pin[1]),Vref(pin[2]),
    pid(0.001,0.001,0.001)
    {
        pid_ = pid_use;
        Out_vref = 0;
    }

    #if pid_
    //pidを使う場合

    void move(double rpm)
    {
        if(rpm != pid.set_point) pid.update_set_point(rpm);
    }

    void loop(double process_value){
        double output = 0;
        pid.update_input_value(process_value, &output);

        if(output >= 0){
            In0 = 1;
            In1 = 0;
            Vref.write(output);
        }
        else{
            In0 = 0;
            In1 = 1;
            Vref.write(-output);
        }
    }

    #else
    //pidを使わない場合

    void move(double vref)
    {
        Out_vref = vref;
    }

    void loop(){
        double output = Out_vref;
        if(output >= 0){
            In0 = 1;
            In1 = 0;
            Vref.write(output);
        }
        else{
            In0 = 0;
            In1 = 1;
            Vref.write(-output);
        }
    }

    #endif

    void Off(){
        In0 = 0;
        In1 = 0;
        Vref = 0;
    }
    
    void stop(){
        In0 = 1;
        In1 = 1;
        Vref = 0;
    }

};

int main() {

    Timer timer;

    PinName PhotoRef_pins[4] = {PA_0,PA_1,PA_4,PC_1};
    PinName Motor_pins0[3] = {PB_1,PB_2,PB_15};
    PinName Motor_pins1[3] = {PC_15,PC_14,PC_13};

    bool pid_use = false;
    
    Motor motor0(Motor_pins0,pid_use);
    Motor motor1(Motor_pins1,pid_use);

    PhotoRef photoref(PhotoRef_pins);
    
    Encoder encoder0(PA_6,200);
    Encoder encoder1(PA_7,200);


    double max_rpm = 0.5;
    double differences_rate = 1.0;
    double curve_rpm_rate = 0.7;

    while(true) {
        photoref.update_Values_location();
        
        switch(photoref.last_cencer_value){
            case 0:
            motor0.move(max_rpm);
            motor1.move(max_rpm*differences_rate);
            case 1:
            motor0.move(max_rpm);
            motor1.stop();
            break;
            case 2:
            motor0.move(max_rpm);
            motor1.move(max_rpm*curve_rpm_rate*differences_rate);
            break;
            case 3:
            motor0.move(max_rpm*curve_rpm_rate);
            motor1.move(max_rpm*differences_rate);
            break;
            case 4:
            motor0.stop();
            motor1.move(max_rpm*differences_rate);
            break;
        }

        motor0.loop();
        motor1.loop();
        
        pc.printf("%lf | ",encoder0.get_RPM());
        pc.printf("%lf\n",encoder1.get_RPM());

        wait_ms(CYCLE_MS);
    }
}