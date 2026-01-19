#include <Servo.h>
enum StepDir : uint8_t {
  STEP_FWD = 1,
  STEP_REV = 0
};

/* =========================================================
   PINMAP (Motor Shield Rev3 固定占用)
   ========================================================= */
static const uint8_t PWM_A   = 3;   // Motor A speed (PWM)
static const uint8_t DIR_A   = 12;  // Motor A direction
static const uint8_t BRAKE_A = 9;   // Motor A brake
static const uint8_t PWM_B   = 11;  // Motor B speed (PWM)
static const uint8_t DIR_B   = 13;  // Motor B direction
static const uint8_t BRAKE_B = 8;   // Motor B brake
// +*****************************Servo pin ***************************/
static const uint8_t SERVO_PIN = 5;
bool Shutter_Open = 0;
/*******************************CAPTEURS*****************************/
static const uint8_t cap_REV  = 2;
static const uint8_t cap_FWD  = 4;
static const uint8_t cap_Shutter = A0;
/****************************bouton***************************/
static const uint8_t Btn_Small_FWD  = A3;
static const uint8_t Btn_Small_REV  = A2;
static const uint8_t Btn_Big_FWD  = 10;
static const uint8_t Btn_Big_REV  = 6;
static const uint8_t Btn_Shutter  = 7;
static const uint8_t Shutter_Led  = A1;
/*****************************************************************/
/*=====================================================
                  capteur control                
=====================================================*/
void cap_init(){
  pinMode(cap_REV,INPUT_PULLUP);
  pinMode(cap_FWD,INPUT_PULLUP);
}
bool is_limit_capREV_pressed() {
  return digitalRead(cap_REV)==LOW;
}
bool is_limit_capFWD_pressed() {
  return digitalRead(cap_FWD)==LOW;
}
bool is_limit_capShutter_pressed() {
  return digitalRead(cap_Shutter)==LOW;
}
/* =========================================================
   bouton CONTROL
   ========================================================= */
void Init_Btn(){
  pinMode(Btn_Big_FWD,   INPUT_PULLUP);
  pinMode(Btn_Big_REV,   INPUT_PULLUP);
  pinMode(Btn_Small_FWD, INPUT_PULLUP);
  pinMode(Btn_Small_REV, INPUT_PULLUP);
  pinMode(Btn_Shutter,   INPUT_PULLUP);
  pinMode(Shutter_Led, OUTPUT);
}
bool btn_pressed(int btn){
  return (digitalRead(btn)==LOW);
}
/* =========================================================
   SERVO CONTROL
 ========================================================= */
static Servo shutterServo;
static const int SERVO_OPEN_DEG = 30;
static const int SERVO_CLOSE_DEG  = 112;
void servo_init() {
  shutterServo.attach(SERVO_PIN);
  servo_set_deg(SERVO_CLOSE_DEG);
}
void servo_set_deg(int deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  shutterServo.write(deg);
}
void servo_close() {
  servo_set_deg(SERVO_CLOSE_DEG);}
void servo_open() {
  servo_set_deg(SERVO_OPEN_DEG);}
/* =========================================================
   STEPPER CONTROL (via Motor Shield L298P)
   ========================================================= */
void stepper_init() {
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);
  // stop both channels
  digitalWrite(BRAKE_A, HIGH);
  digitalWrite(BRAKE_B, HIGH);
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}
void stepper_phase_A(bool dir, uint8_t pwm) {
  digitalWrite(BRAKE_A, LOW);   // enable A
  digitalWrite(BRAKE_B, HIGH);  // disable B
  digitalWrite(DIR_A, dir ? HIGH : LOW);
  analogWrite(PWM_A, pwm);
}
void stepper_phase_B(bool dir, uint8_t pwm) {
  digitalWrite(BRAKE_A, HIGH);  // disable A
  digitalWrite(BRAKE_B, LOW);   // enable B
  digitalWrite(DIR_B, dir ? HIGH : LOW);
  analogWrite(PWM_B, pwm);
}
void stepper_stop() {
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  digitalWrite(BRAKE_A, HIGH);
  digitalWrite(BRAKE_B, HIGH);
}
static uint8_t phase = 0; // 0..3
void stepper_one_step(StepDir dir, uint8_t pwm, uint16_t dt_ms){
  // dir=FWD: 0->1->2->3
  // dir=REV: 0->3->2->1
  if (dir == STEP_FWD) phase = (phase + 1) & 0x03;
  else                phase = (phase + 3) & 0x03;
  switch(phase){
    case 0: stepper_phase_A(true,  pwm); break; // A+
    case 1: stepper_phase_B(true,  pwm); break; // B+
    case 2: stepper_phase_A(false, pwm); break; // A-
    case 3: stepper_phase_B(false, pwm); break; // B-
  }
  delay(dt_ms);
}
void stepper_motor_control_pas(StepDir dir, uint8_t pwm, uint16_t dt_ms,int pas){
  if(dir==STEP_FWD){
    for (int i = 0; i < pas; i++){
     //stepper_step_forward(200, 1000);
      if(is_limit_capFWD_pressed()==0){
        stepper_one_step(dir, pwm, dt_ms);
        //delay(3000);
       }else{
        stepper_stop();
        break;}}
  }else{
    for (int i = 0; i < pas; i++){
     //stepper_step_forward(200, 1000);
      if(is_limit_capREV_pressed()==0){
        stepper_one_step(dir, pwm, dt_ms);
        //delay(3000);
       }else{
        stepper_stop();
        break;}}
  }
  stepper_stop();
}
/*==================================================================
    logique
====================================================================*/ 
static const uint8_t PWM = 200;
static const uint8_t vitesse = 30;
static const uint8_t big_step = 17;
int counter_pas =0;
float position_act=0;
bool pressed_edge(uint8_t pin) {
  static uint8_t last[30]; 
  uint8_t cur = digitalRead(pin);
  bool edge = (last[pin] == HIGH && cur == LOW);
  last[pin] = cur;
  delay(20); 
  return edge;
}  
void button_control_stepper_motor(){
  if(pressed_edge(Btn_Big_FWD)){
    stepper_motor_control_pas(STEP_FWD,PWM,vitesse,big_step);
    counter_pas+=17;
  }else if(pressed_edge(Btn_Big_REV)){
    stepper_motor_control_pas(STEP_REV,PWM,vitesse,big_step);
    counter_pas-=17;
  }else if(pressed_edge(Btn_Small_FWD)){
    stepper_motor_control_pas(STEP_FWD,PWM,vitesse,1);
    counter_pas+=1;
    }else if(pressed_edge(Btn_Small_REV)){
    stepper_motor_control_pas(STEP_REV,PWM,vitesse,1);
    counter_pas-=1;
  }else{
    stepper_stop();
  }
} 
void button_control_servo_motor(){
  if(pressed_edge(Btn_Shutter)){
    if(Shutter_Open & (is_limit_capShutter_pressed()==0)){
      servo_close();
      Shutter_Open= 0;
      digitalWrite(Shutter_Led, HIGH);
    }else{
      servo_open();
      Shutter_Open= 1;
      digitalWrite(Shutter_Led, LOW);
    }
  }
} 
void initialisation_position(){
  stepper_motor_control_pas(STEP_REV,200,50,340);
}
void reset_position(){
  if(pressed_edge(Btn_Big_FWD) & pressed_edge(Btn_Big_REV)){
    initialisation_position();
    counter_pas=0;
    position_act=0;
  }
}
float calcul_position(int pas,int position_act){
  return float(pas) * 0.56+float(position_act);
}
/* =========================================================
   MAIN TEST 
   =========================================================*/
void setup() {
  servo_init();
  stepper_init();
  cap_init();
  Init_Btn();
  initialisation_position();
}
void loop() {
  // ---- Test 1: shutter open/close
  reset_position();
  button_control_stepper_motor();
  position_act=calcul_position(counter_pas,position_act);
  button_control_servo_motor(); 
}
