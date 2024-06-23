#include "PinChangeInterrupt.h"

#define Encoder_R_output_B 7
#define Encoder_L_output_B 8

#define Encoder_R_output_A 3 
#define Encoder_L_output_A 2 
int right_count = 0;
int left_count = 0;
void setup() {
Serial.begin(9600); 
pinMode(Encoder_R_output_A,INPUT); 
pinMode(Encoder_L_output_A,INPUT); 

pinMode(Encoder_R_output_B, INPUT_PULLUP);
pinMode(Encoder_L_output_B, INPUT_PULLUP);

attachPCINT(digitalPinToPCINT(Encoder_R_output_B), do_right_motor, CHANGE);
attachPCINT(digitalPinToPCINT(Encoder_L_output_B), do_left_motor, CHANGE);

attachInterrupt(digitalPinToInterrupt(Encoder_R_output_A),do_right_motor,RISING);
attachInterrupt(digitalPinToInterrupt(Encoder_L_output_A),do_left_motor,RISING);
}

void loop() {
  Serial.print("R: ");
  Serial.print(right_count);
  Serial.print("    ");
  Serial.print("L: ");
  Serial.println(left_count);
  delay(100);
}

void do_right_motor(){
  int b = digitalRead(Encoder_R_output_B);
  if(b > 0){
    right_count--;
  }
  else{
    right_count++;
  }
}

void do_left_motor(){
  int b = digitalRead(Encoder_L_output_B);
  if(b > 0){
    left_count++;
  }
  else{
    left_count--;
  }
}