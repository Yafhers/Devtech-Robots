//PINES DEL MOTOR
#define motor1A 9
#define motor1B 8
#define motor2A 7
#define motor2B 6
int pwmA = 10;
int pwmB = 11;

//PINES DEL HCSR04
#define triggerPin 4
#define echoPin 5

//PINES DEL SENSOR 
#define irSensor1 A1
#define irSensor2 A0


//Variables globales
int distancia; //Para el HCSR04
int infR1;  //Para el sensor infrarrojo 
int infR2;
int flag=0;

void setup() {
  //PINES DEL MOTOR SALIDA
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);

  //PINES DEL ULTRASÓNICO 
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);


  //PINES SENSOR IR ENTRADA
  pinMode(irSensor1, INPUT);
  pinMode(irSensor2, INPUT);

  Serial.begin(9600);
  analogWrite(pwmA, 20);
  analogWrite(pwmB, 20);
}

void loop() {
  //Leer la distancia del HCSR04
  leer_distancia();

  //Lectura de los valores de los sensores infrarrojos
  infR1= analogRead(irSensor1);
  infR2= analogRead(irSensor2);
  
  Serial.print("Distancia: "); Serial.print(distancia);
  Serial.print(" Sensor derecho: "); Serial.print(infR1);
  Serial.print(" Sensor izquierdo: "); Serial.print(infR2);

//Evaluación de las condiciones para el movimiento del robot
  if(distancia < 20){
    motores (1,1);
    Serial.println(" Ataco");
    

      while((infR1<250||infR2<250)&&flag==0){
        infR1= analogRead(irSensor1);
        infR2= analogRead(irSensor2);

        motores(-1,-1);
        Serial.print("Retrocedo");
        delay(2000);
        flag = 1;
      }
    }
  else{
    motores(-1,1);
    flag = 0;
    Serial.println(" Buscando oponente");
  }
    
  
  ;  
}

void motores(int motor1_value, int motor2_value){
  if(motor1_value == 1){
    digitalWrite(motor1A, HIGH);
    digitalWrite(motor1B, LOW);
  }
  else if(motor1_value == 0){
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, LOW);
  }
  else if(motor1_value == -1){
    digitalWrite(motor1A, LOW);
    digitalWrite(motor1B, HIGH);
  }

  if(motor2_value == 1){
    digitalWrite(motor2A, HIGH);
    digitalWrite(motor2B, LOW);
  }
  else if(motor2_value == 0){
    digitalWrite(motor2A, LOW);
    digitalWrite(motor2B, LOW);
  }
  else if(motor2_value == -1){
    digitalWrite(motor2A, LOW);
    digitalWrite(motor2B, HIGH);
  }
}

void leer_distancia(){
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  distancia = pulseIn(echoPin, HIGH)/58; //ida y vuelta 
}