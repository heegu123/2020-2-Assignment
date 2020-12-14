#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_SERVO 10         //SERVO를 10번 핀에 연결
#define PIN_IR A0            //적외선 센서를 A0에 연결

// Framework setting
#define _DIST_TARGET 255     //레일플레이트의 중간지점(목표지점=25.5cm)
#define _DIST_MIN 100        //측정 가능한 최소 거리(mm)
#define _DIST_MAX 410        //측정 가능한 최대 거리(mm)

// Distance sensor
#define _DIST_ALPHA 0.35      //EMA 가중치

// Servo range 
#define _DUTY_MIN 1120       //[3028] 서보 각도조작 최소값
#define _DUTY_NEU 1380       //[3038] 레일 수평 서보 펄스폭
#define _DUTY_MAX 1700       //[3031] 서보 각도조작 최대값

// Servo speed control
#define _SERVO_ANGLE 50       //서보모터의 작동 각도범위(단위 : degree)
#define _SERVO_SPEED 100       //[3040] 서보의 각속도(초당 각도 변화량)

// Event periods
#define _INTERVAL_DIST 30     //적외선 센서 측정 간격
#define _INTERVAL_SERVO 30    //서보갱신간격
#define _INTERVAL_SERIAL 100  //시리얼 플로터 갱신간격

// PID parameters
#define _KP 2             //비례 제어의 상수 값
#define _KD 78
#define _KI 0.002

//filter
#define DELAY_MICROS 1500

//////////////////////
// global variables //
//////////////////////


// Servo instance     
Servo myservo;         

// Distance sensor
float dist_target = _DIST_TARGET;            //location to send the ball
float dist_raw, dist_ema;     //적외선센서로 측정한 보정하기 전 거리값과 ema필터를 적용한 거리값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;  //dist, servo, serial의 last sampling time
bool event_dist, event_servo, event_serial;    //적외선센서의 거리측정, 서보모터, 시리얼의 이벤트 발생 여부

// Servo speed control
int duty_chg_per_interval;    //interval당 servo가 돌아가는 최대 정도
int duty_target; 
int duty_curr ;   //servo 목표 위치, servo 현재 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;   //비례 제어를 위한 전에 측정한 오차, 새로 측정한 오차 값, 비례항, 적분항, 미분항 변수

//filter
float dist_filtered;
float samples_num = 3;
int a, b;

void setup() { 
  
  duty_target = duty_curr = _DUTY_NEU;
  myservo.attach(PIN_SERVO);    //servo를 연결
  
  // initialize global variables
  int dist_min = _DIST_MIN;       //측정가능 거리의 최소값
  int dist_max= _DIST_MAX;        //측정값의 최대값
  int dist_target = _DIST_TARGET; //목표위치

  a = 70;
  b = 300;
  iterm = 0;

  // initialize serial port
  Serial.begin(57600);    //시리얼 모니터 속도 지정 

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); //서보를 레일이 수평이 되는 값으로 초기화

  

  // convert angle speed into duty change per interval.
  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0); 
  /*
  _SERVO_ANGLE/_SERVO_SPEED=
  (_DUTY_MAX - _DUTY_MIN_)* INTERVAL_SERVO/duty_chg_per_interval = 
  interval 반복횟수*INTERVAL_SERVO=
  _SERVO_ANGLE만큼 돌아가는데 걸리는 시간 
  */
}

void loop() {
  /////////////////////
  // Event generator // [3155] 설정된 주기마다 이벤트 생성
  /////////////////////
  unsigned long time_curr = millis();
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

  ////////////////////
  // Event handlers //
  ////////////////////

    if(event_dist) {
      event_dist = false;
      //get a distance reading from the distance sensor
      dist_raw = filtered_ir_distance();  //적외선 선서로 측정한 값에 필터를 적용한 값
      dist_raw = 100 + 300.0 / (300 - 70) * (dist_raw - 70);
      //PID control logic
      error_curr = _DIST_TARGET - dist_raw; // 목표값 에서 현재값을 뺀 값이 오차값
       
      pterm = _KP*error_curr;
      pterm = -(_DIST_TARGET-(pterm+dist_raw))/5.5*8;
       
      dterm = _KD * (error_curr - error_prev); 
      
      iterm += _KI * error_curr;
      
      control = pterm + dterm+ iterm;;
      
      //duty_target = f(duty_neutral, control)
      duty_target = _DUTY_NEU + control;

      //keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
      if (duty_target < _DUTY_MIN) {
        duty_target = _DUTY_MIN;  // 
      } 
      else if (duty_target > _DUTY_MAX) {
        duty_target = _DUTY_MAX; //
      }  
       
      error_prev = error_curr;
    }

    if(event_servo) {
      event_servo = false;
      //adjust duty_curr toward duty_target by duty_chg_per_interval
      if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target) duty_curr = duty_target;
      }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr);   //위에서 바뀐 현재위치 값을 갱신
    }

    if(event_serial) {
      event_serial = false; // serial EventHandler Ticket -> false
      Serial.print("IR:");
      Serial.print(dist_raw);
      Serial.print(",T:");
      Serial.print(dist_target);
      Serial.print(",P:");
      Serial.print(map(pterm,-1000,1000,510,610));
      Serial.print(",D:");
      Serial.print(map(dterm,-1000,1000,510,610));
      Serial.print(",I:");
      Serial.print(map(iterm,-1000,1000,510,610));
      Serial.print(",DTT:");
      Serial.print(map(duty_target,1000,2000,410,510));
      Serial.print(",DTC:");
      Serial.print(map(duty_curr,1000,2000,410,510));
      Serial.println(",-G:245,+G:265,m:0,M:800");
    }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;                    
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  dist_ema = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*dist_ema;
  return dist_ema;
}
