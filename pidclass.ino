typedef void (*ActFunc) (float ctrl);
typedef void (*MeasFunc) ();

void setup() {


}

void loop() {


}


class PID {
  public:
    float kp;
    float ki;
    float kd;

    float preverror;
    long prevtime;
    float integral;

    float target;
    ActFunc act;
    MeasFunc meas;

    PID(float kp_, float ki_, float kd_, ActFunc actf, MeasFunc measf) : kp(kp_), ki(ki_), kd(kd_), act(actf), meas(measf){}

    void setControl(ActFunc func) {
      act = func;
    }

    void setMeasurement(MeasFunc func) {
      meas = func;
    }

    void initialize() {
      preverror = meas() - target;
      prevtime = millis();
      integral = 0;
    }

    float PIDstep() {
      long newtime = millis();
      long dt = newtime - prevtime;
      
      float error = meas() - target;
      float derivative = (error - preverror) / dt;
      integral += error * dt;

      float ctrlsignal = kp * error + ki * integral + kd * derivative;
      act(ctrlsignal);

      prevtime = newtime;
      preverror = error;

      return abs(error);
    }

    void moveTo(float targ, float eps) {
      target = targ;
      initialize();
      while (PIDstep() > eps) {}
    }
};
