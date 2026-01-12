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
/*******************************CAPTEURS*****************************/
static const uint8_t cap_1  = 2;
static const uint8_t cap_2  = 4;
/****************************bouton***************************/
static const uint8_t Btn_Small_FWD  = A3;
static const uint8_t Btn_Small_REV  = D6;
static const uint8_t Btn_Big_FWD  = A0;
static const uint8_t Btn_Big_REV  = A2;
static const uint8_t Btn_Ok  = 7;

/*****************************************************************/
/*=====================================================
                  capteur control                
=====================================================*/
void cap_init(){
  pinMode(cap_1,INPUT_PULLUP);
  pinMode(cap_2,INPUT_PULLUP);
}
bool is_limit_cap1_pressed() {
  return digitalRead(cap_1);
}
bool is_limit_cap2_pressed() {
  return digitalRead(cap_2);
}
/* =========================================================
   bouton CONTROL
   ========================================================= */
void Init_Btn(){
  pinMode(BTN_BIG_FWD,   INPUT_PULLUP);
  pinMode(BTN_BIG_REV,   INPUT_PULLUP);
  pinMode(BTN_SMALL_FWD, INPUT_PULLUP);
  pinMode(BTN_SMALL_REV, INPUT_PULLUP);
  pinMode(BTN_CONFIRM,   INPUT_PULLUP);
}
bool btn_pressed(int btn){
  return digitalRead(btn)== LOW;
}
int counter pas(){
  int pas =0;
  if btn_pressed(Btn_Small_FWD){
    pas +=1;}
  if btn_pressed(Btn_Small_REV){
    pas -=1;}
  if btn_pressed(Btn_Big_FWD){
    pas +=17;}
  if btn_pressed(Btn_Big_REV){
    pas -=17;}
  return pas ;    
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
  for (int i = 0; i < pas; i++){
   //stepper_step_forward(200, 1000);
   if(is_limit_cap1_pressed()){
    stepper_one_step(dir, pwm, dt_ms);
    //delay(3000);
   }else{
    stepper_stop();
   }
  }
}
/* =========================================================
   MAIN TEST 
   ========================================================= */
void setup() {
  servo_init();
  stepper_init();
  cap_init();
}

void loop() {
  // ---- Test 1: shutter open/close
  /*servo_close();
  delay(3000);
  servo_open();
  delay(3000);*/

  // ---- Test 2: stepper forward/back
  bool limit1 = false;
  limit1=is_limit_cap1_pressed();
  stepper_motor_control_pas(STEP_REV,200,100,200);//un cercle
  //delay(2000);
  /*stepper_stop();
  delay(5000);
  stepper_motor_control_pas(STEP_FWD,200,100,200);
  delay(5000);*/
  // reverse
 /*for (int i = 0; i < 20; i++) stepper_step_reverse(400, 300);
  stepper_stop();
  delay(2000);*/
}
