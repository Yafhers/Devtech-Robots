//Definicion de pines de los motores
int izq1 = 4;                                 //AIN1 Motor Izquierdo
int izq2 = 5;                                 //AIN2 Motor Izquierdo
int der1 = 6;                                 //BIN1 Motor Derecho
int der2 = 7;                                 //BIN2 Motor Derecho
int pwmA=3;                                   //PWM Motor Izquierdo
int pwmB=11;                                  //PWM Motor Derecho

//Definicion de pines de los sensores de línea
const int boton = 2;                          //Boton para calibrar los sensores
const int NUM_SENSOR = 5 ;                    //Numero de sensores a usar
int sensors[NUM_SENSOR]={A0,A1,A2,A3,A4};     //Array de los pines de los sensores

//Definicion de variables de posicion
int position;                                 
int last_position;

//Definicion de arreglos de lecturas
int lecturas[NUM_SENSOR];
int digital[NUM_SENSOR];
int lecturas_lineas[NUM_SENSOR];
int lecturas_fondos[NUM_SENSOR];
int umbral[NUM_SENSOR];

//Definicion de variables para el PID
float KP = 0.35;
float KD = 6.5;
float KI = 0.0025;
int vel = 100;
int velAdelante =   180;
int velAtras    =   -100;

//Definicion de variables para la integral
int error1=0;
int error2=0;
int error3=0;
int error4=0;
int error5=0;
int error6=0;

//Definicion de variables de error 
int error=0;
int errorIntegral = 0;
int errorDerivativo = 0;
int correccion = 0;
int last_error = 0;
int setpoint = 200;    //Posicion de la linea a seguir

void setup() {
  //TCCR2B = TCCR2B & B11111000 | B00000001;
  //set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz

  //TCCR2B = TCCR2B & B11111000 | B00000010; 
  //set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz

  //TCCR2B = TCCR2B & B11111000 | B00000011;   
  //set timer 2 divisor to    32 for PWM frequency of   980.39 Hz

  TCCR2B = TCCR2B & B11111000 | B00000100;   
  //set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)

  //TCCR2B = TCCR2B & B11111000 | B00000101;   
  //set timer 2 divisor to   128 for PWM frequency of   245.10 Hz

  //TCCR2B = TCCR2B & B11111000 | B00000110;   
  //set timer 2 divisor to   256 for PWM frequency of   122.55 Hz

  //TCCR2B = TCCR2B & B11111000 | B00000111;   
  //set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

  //Declaracion de pines del motor como salidas
  pinMode(boton,INPUT);
  pinMode(izq1,OUTPUT);
  pinMode(izq2,OUTPUT);
  pinMode(der1,OUTPUT);
  pinMode(der2,OUTPUT);
  pinMode(pwmA,OUTPUT);
  pinMode(pwmB,OUTPUT);
  
  //Monitor Serial
  Serial.begin(9600);
  Serial.println("Listo");

  //Calibracion 
  while(!digitalRead(boton)); 
  lineas();                   //Presiono el boton sobre la linea
  delay(500);
  while(!digitalRead(boton));
  fondos();                   //Presiono el boton sobre el fondo
  delay(500);
  while(!digitalRead(boton));
  promedios();                //Realiza un promedio de los valores obtenidos
  delay(500);
}

void loop() {
 int go  = digitalRead(boton);
  while(true){
    int go  = digitalRead(boton);
    posicion();
    frenos();
    PID();
    if(go==1){
      motores(-20,-20);
      break;
    }
  }
  while(true){
    motores(0,0);
  }
}

/////////////////////////////////////////////////////////////////
//////////Funciones para calcular la posicion////////////////////
/////////////////////////////////////////////////////////////////

void posicion(){
  for(int i=0;i<NUM_SENSOR;i++){
    lecturas[i]=analogRead(sensors[i]);
    if(lecturas[i]>=umbral[i]){
      digital[i]=1;
    }
    else{
      digital[i]=0;
    }
  }

  int suma1=digital[0]*0+digital[1]*100+digital[2]*200+digital[3]*300+digital[4]*400;
  int suma2=digital[0]+digital[1]+digital[2]+digital[3]+digital[4];
  
  position=suma1/suma2;
  if(last_position<=50 && position==-1)
    position=0;
  if(last_position>350 && position==-1)
    position=400;
  last_position=position;
  //imprimeLecturasAnalog();
  imprimeLecturasDig();
}


void imprimeLecturasAnalog(){
  for(int i=0;i<NUM_SENSOR;i++){
    Serial.print(lecturas[i]);
    Serial.print("\t");
  }
  Serial.print("\t");
  Serial.println(position);
  Serial.println("");
}

void imprimeLecturasDig(){
  for(int i=0;i<NUM_SENSOR;i++){
    Serial.print(digital[i]);
    Serial.print("\t");
  }
  Serial.print("\t");
  Serial.println(position);
  Serial.println("");
}

/////////////////////////////////////////////////////////////////
//////Funciones para calcular la lectura de las lineas///////////
/////////////////////////////////////////////////////////////////

void lineas(){
  Serial.println("Leyendo lineas: ");
   for(int i=0;i<NUM_SENSOR;i++){
     lecturas_lineas[i]=analogRead(sensors[i]);
     Serial.print(lecturas_lineas[i]);
     Serial.print("\t");
  }
  Serial.println("");
}

/////////////////////////////////////////////////////////////////
//////Funciones para calcular la lectura del fondo///////////////
/////////////////////////////////////////////////////////////////
void fondos(){
  Serial.println("Leyendo fondos: ");
  
   for(int i=0;i<NUM_SENSOR;i++){
     lecturas_fondos[i]=analogRead(sensors[i]); 
     Serial.print(lecturas_fondos[i]);
     Serial.print("\t");
   }
   Serial.println("");
  
}

/////////////////////////////////////////////////////////////////
//////Funciones para calcular el promedio umbral/////////////////
/////////////////////////////////////////////////////////////////
void promedios(){
  Serial.println("Leyendo umbrales: ");
  for(int i=0;i<NUM_SENSOR;i++){
    umbral[i]=(lecturas_lineas[i]+lecturas_fondos[i])/2;
    Serial.print(umbral[i]);
    Serial.print("\t");
  }
  Serial.println("");
}

/////////////////////////////////////////////////////////////////
///////////Funcion controladora de motores///////////////////////
/////////////////////////////////////////////////////////////////
void motores(int izq, int der) {
  
  //CONTROL DE MOTOR IZQUIERDO
  if(izq>=0){
    digitalWrite(izq1,HIGH);
    digitalWrite(izq2,LOW);
    analogWrite(pwmA,izq);
  }
  else{
    digitalWrite(izq1,LOW);
    digitalWrite(izq2,HIGH);
    analogWrite(pwmA,-izq);
  }


  //CONTROL DE MOTOR DERECHO
  if(der >= 0){
    digitalWrite(der1,LOW);
    digitalWrite(der2,HIGH);
    analogWrite(pwmB,der);
  }
  else{
    digitalWrite(der1,HIGH);
    digitalWrite(der2,LOW);
    analogWrite(pwmB,-der);
  }
}

/////////////////////////////////////////////////////////////////
///////////////////////////Funcio PID////////////////////////////
/////////////////////////////////////////////////////////////////
void PID(){
  error = position-setpoint;
  errorDerivativo = error-last_error;
  errorIntegral = error1 + error2 + error3 + error4 + error5 + error6 ;
  last_error = error;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = error;
  int correccion = (error*KP)+(errorDerivativo*KD)+(errorIntegral*KI);

  //if(correccion>vel);
  //correccion=vel;
  //if(correccion<-vel);
  //correccion=-vel;

  if (correccion<0){
    motores(vel,vel-correccion);
    Serial.print(vel);Serial.print(" ");Serial.println(vel-correccion);
  }
  else{
    motores(vel+correccion,vel);
    Serial.print(vel+correccion);Serial.print(" ");Serial.println(vel);
  }
}


/////////////////////////////////////////////////////////////////
/////////////////////////Funcio Frenos///////////////////////////
/////////////////////////////////////////////////////////////////
void frenos(){
  while(position==0 || position==400){
    if(position==0){
      motores(velAtras,velAdelante);
    }
    

    if(position==400){
      motores(velAdelante,velAtras);
    }
    posicion();
  }
  
}