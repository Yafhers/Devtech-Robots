#include <SoftwareSerial.h>
SoftwareSerial miBT(0, 1);
char DATO=0; // Crea el objeto mi BT de la clase Software Serial
//Minisumo de Daniel
/*
#define Trigger 2   //Pin digital 2 para el Trigger del sensor
#define Echo 3   //Pin digital 3 para el Echo del sensor
#define S_IZQ A0  //Sensor de la derecha
#define S_DER A1  //Sensor de la izquierda
#define MOT_2A 9  
#define MOT_2B 10
#define MOT_1A 5
#define MOT_1B 6
*/
//Fierrobot

#define Trigger 4   //Pin digital 2 para el Trigger del sensor
#define Echo 5   //Pin digital 3 para el Echo del sensor
#define S_IZQ A0  //Sensor de la derecha
#define S_DER A1  //Sensor de la izquierda
#define MOT_2A 7  
#define MOT_2B 6
#define MOT_1A 9
#define MOT_1B 8


void setup() {
  Serial.begin(9600);
  pinMode(MOT_1A,OUTPUT);
  pinMode(MOT_2A,OUTPUT);
  pinMode(MOT_1B,OUTPUT);
  pinMode(MOT_2B,OUTPUT);
  pinMode(S_IZQ,INPUT);
  pinMode(S_DER,INPUT);
  pinMode(Echo,INPUT);  //pin como entrada
  pinMode(Trigger,OUTPUT); //pin como salida
  miBT.begin(38400);
}

void loop() {
  //LECTURA DEL HCSR04
  long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros
  digitalWrite(Trigger,HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger,LOW); 
  t = pulseIn(Echo,HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm

  //motores(-1,-1);
  
 /* int sens1 = analogRead(S_IZQ);
  int sens2 = analogRead(S_DER);

  Serial.print(d);
  Serial.print(" ");
  Serial.print(sens1);
  Serial.print(" ");
  Serial.println(sens2);

  delay(200);*/
    if (miBT.available()){ //Si hay datos disponibles en el dispositivo controlado por Bt
    DATO = miBT.read(); //Se leen los datos y se guardan en la variable dato
    
    if(DATO == 'F'){
      motores(1,1);
    }

    if(DATO == 'B'){
      motores(-1,-1);
    }

    if(DATO == 'L'){
      motores(0,1);

    }


    if(DATO == 'R'){
    motores(1,0);
    }


    if(DATO == 'S'){
      motores(0,0);
    }

  }


  
}


void motores(int motor1, int motor2){
  if(motor1 == 1 && motor2 == 1){
    digitalWrite(MOT_1A,1);
    digitalWrite(MOT_1B,0);
    digitalWrite(MOT_2A,1);
    digitalWrite(MOT_2B,0);
  }

  else if(motor1 == 1 && motor2 == 0){
    digitalWrite(MOT_1A,1);
    digitalWrite(MOT_1B,0);
    digitalWrite(MOT_2A,0);
    digitalWrite(MOT_2B,0);
  }
  else if(motor1 == 0 && motor2 == 1){
    digitalWrite(MOT_1A,0);
    digitalWrite(MOT_1B,0);
    digitalWrite(MOT_2A,1);
    digitalWrite(MOT_2B,0);
  }
  else if(motor1 == 0 && motor2 == 0){
    digitalWrite(MOT_1A,0);
    digitalWrite(MOT_1B,0);
    digitalWrite(MOT_2A,0);
    digitalWrite(MOT_2B,0);
  }
  else if(motor1 == -1 && motor2 == 0){
    digitalWrite(MOT_1A,0);
    digitalWrite(MOT_1B,1);
    digitalWrite(MOT_2A,0);
    digitalWrite(MOT_2B,0);
  }
  else if(motor1 == 0 && motor2 == -1){
    digitalWrite(MOT_1A,0);
    digitalWrite(MOT_1B,0);
    digitalWrite(MOT_2A,0);
    digitalWrite(MOT_2B,1);
  }
  else if(motor1 == -1 && motor2 == -1){
    digitalWrite(MOT_1A,0);
    digitalWrite(MOT_1B,1);
    digitalWrite(MOT_2A,0);
    digitalWrite(MOT_2B,1);
  }
else if(motor1 == 1 && motor2 == -1){
    digitalWrite(MOT_1A,1);
    digitalWrite(MOT_1B,0);
    digitalWrite(MOT_2A,0);
    digitalWrite(MOT_2B,1);
  }
  else if(motor1 == -1 && motor2 == 1){
    digitalWrite(MOT_1A,0);
    digitalWrite(MOT_1B,1);
    digitalWrite(MOT_2A,1);
    digitalWrite(MOT_2B,0);
  }

}
