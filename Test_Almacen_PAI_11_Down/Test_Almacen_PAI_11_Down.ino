/*
   ros//Serial::std_msgs::Float64 Test
   Receives a Float64 input, subtracts 1.0, and publishes it
*/
#define F_CPU 8000000UL
#include <ros.h>
#include <std_msgs/Int16.h>
//*******************************************************************
//* Librerias para el teclado    Matriz 4x4                         *
//*******************************************************************

#include <Key.h>
#include <Keypad.h>

//*******************************************************************
//* Librerias para pantalla ////lcd 16x2 I2C                            *
//*******************************************************************



//*******************************************************************
//* Librerias para Motor paso a paso                                *
//*******************************************************************

#include <Stepper.h>
///////////////////////////Variables para control de los motores de la tijera///
///Motor Nema23///////
int PUL = 12; //Pin para la señal de pulso
int DIR = 11; //define Direction pin
int EN = 10; //define Enable Pin
///Motor Nema17////////
int PUL1 = 9; //Pin para la señal de pulso
int DIR1 = 38; //define Direction pin
int EN1 = 40; //define Enable Pin
//////////////////////////////////////////////////////////////////////////////////
//*******************************************************************
//*           Inicializacion de variables para el teclado           *
//*                                                                 *
//* https://www.prometec.net/teclados-matriciales/                  *
//* https://www.luisllamas.es/arduino-teclado-matricial/            *
//* Columnas a pines   36, 37, 38, 39                               *
//* Filas a pines      42, 43, 44, 45                               *
//* Tecla Enter     '#'                                             *
//* Tecla Delete    '*';                                            *
//*******************************************************************

const byte rowsCount = 4;
const byte columsCount = 4;
int estado = 0;
char keys[rowsCount][columsCount] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

const byte rowPins[rowsCount] = { 36, 37, 38, 39  };
const byte columnPins[columsCount] = { 42, 43, 44, 45};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, columnPins, rowsCount, columsCount);

String KeyString = "";
char Enter = '#';
char Delete = '*';
bool almacenar = 0;
bool done = 0;
char KeyIn;

//*******************************************************************
//*           Inicializacion de variables para ////lcd                  *
//*                                                                 *
//* https://www.geekfactory.mx/tutoriales/tutoriales-arduino/       *
//* ////lcd-16x2-por-i2c-con-arduino/                                   *
//* https://naylampmechatronics.com/blog/35_Tutorial--////lcd-con-I2C-  *
//* controla-un-////lcd-con-so.html                                     *
//*******************************************************************

// Constructor de la librería de ////lcd 16x2
// Aqui se configuran los pines asignados a la pantalla del PCF8574


//*******************************************************************
//*           Inicializacion de variables para Motores eje Z        *
//*                                                                 *
//* L = Left                                                        *
//* R = Right                                                       *
//* U = Up                                                          *
//* D = Down                                                        *
//*******************************************************************


const int Vel = 255;
const int Encoder_L = 2; // Pin Encoder_L optico
const int Encoder_R = 3; // Pin Encoder_R optico
int RPWM_Z_UP = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Z_UP = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
int RPWM_Z_DOWN = 7; // Arduino PWM output pin 7; connect to IBT-2 pin 1 (RPWM)
int LPWM_Z_DOWN = 8; // Arduino PWM output pin 8; connect to IBT-2 pin 2 (LPWM)
int FCZLU = A3;
int FCZRU = 28;
int FCZLD = A5;
int FCZRD = 30;
bool FC_Z_L_U = 0;
bool FC_Z_R_U = 0;
bool FC_Z_L_D = 0;
bool FC_Z_R_D = 0;
volatile int ISRCounter_L = 0; // Contador de la interrupcion
volatile int ISRCounter_R = 0; // Contador de la interrupcion



const float consR=30.42;//pulsos/cm
const float consL=28.48;//pulsos/cm

int Pasos_pisoL;
int Pasos_pisoR;

const int DisPiso2=39.5;//cm
const int DisPiso1=60;//cm
const int Pasos_a_tijera=2100;
int subiendo = 0;
bool Z_done = 0;

//*******************************************************************
//*        Inicializacion de variables para Motor paso a paso       *
//*                                                                 *
//* A1 = 22                                                         *
//* A2 = 23                                                         *
//* B1 = 24                                                         *
//* B2 = 25                                                         *
//*******************************************************************

//int FCYA = 50;
//int FCYL = 27;
//bool FC_Y_L = 0;
bool FC_Y_A = 0;
//int revs = 0;
int Pos_Y = 0;
int LPos_Y = 0;
const int stepsPerRevolution = 200;
bool Y_done = 0;
Stepper myStepper(stepsPerRevolution, 42, 44, 46, 48);


//*******************************************************************
//*           Inicializacion de variables para Motor eje X          *
//* Pos 0 = Izquierda                                               *
//* Pos 1 = Derecha                                                 *
//*******************************************************************

int R_X = 32;
int L_X = 34;
int FCXL = A1;
int FCXR = A0;
bool FC_X_L = 0;
bool FC_X_R = 0;
int Pos_X = 0;
bool X_done = 0;
bool fin = 0;
//*******************************************************************
//*                   Interrupciones Encoder eje Z                  *
//* Finales de carrera normalmente en 1                             *
//*******************************************************************

void interruptCount_R()
{
  ISRCounter_R++;
  //////Serial.println(ISRCounter_R);
}
void interruptCount_L()
{
  ISRCounter_L++;
  //////Serial.println(ISRCounter_R);
}

//*******************************************************************
//* Funcion para leer los finales de carrera                        *
//*******************************************************************

void fc(void) {
  FC_Z_L_D = digitalRead(FCZLD);
  FC_Z_L_U = digitalRead(FCZLU);
  FC_Z_R_D = digitalRead(FCZRD);
  FC_Z_R_U = digitalRead(FCZRU);
  FC_X_L = digitalRead(FCXL);
  FC_X_R = digitalRead(FCXR);

}

void test_fc(void) {
  ////Serial.print("ZLD: ");
  ////Serial.print(FC_Z_L_D);
  //
  ////Serial.print("  ZLU: ");
  ////Serial.print(FC_Z_L_U);
  //
  ////Serial.print("  ZRD: ");
  ////Serial.print(FC_Z_R_D);
  //
  ////Serial.print(" ZRU: ");
  ////Serial.print(FC_Z_R_U);
  //
  ////Serial.print("  XL: ");
  ////Serial.print(FC_X_L);
  //
  ////Serial.print("  XR: ");
  ////Serial.println(FC_X_R);

}

//*******************************************************************
//* Funciones de movimiento del eje Z                               *
//*******************************************************************

void Up_R() {
  analogWrite(LPWM_Z_DOWN, 252 * 0.6);
  analogWrite(RPWM_Z_DOWN, 0);

}
void Up_L() {
  analogWrite(LPWM_Z_UP, 255 * 0.6);
  analogWrite(RPWM_Z_UP, 0);

}

void Down_R() {
  analogWrite(LPWM_Z_DOWN, 0);
  analogWrite(RPWM_Z_DOWN, 243 * 0.6);

}

void Down_L() {
  analogWrite(LPWM_Z_UP, 0);
  analogWrite(RPWM_Z_UP, 255 * 0.6);

}

void TwoUp(void) {
  int subiendo_L = 0;
  int subiendo_R = 0;
  fc();
  if (FC_Z_R_U) {
    analogWrite(RPWM_Z_UP, 255);
    analogWrite(RPWM_Z_DOWN, 0);
  } else {
    analogWrite(RPWM_Z_UP, 0);
    analogWrite(RPWM_Z_DOWN, 0);
    subiendo_R = 1;
  }
  if (FC_Z_L_U) {
    analogWrite(LPWM_Z_UP, 255);
    analogWrite(LPWM_Z_DOWN, 0);
  } else {
    analogWrite(LPWM_Z_UP, 0);
    analogWrite(LPWM_Z_DOWN, 0);
    subiendo_L = 1;
  }
  subiendo = subiendo_L && subiendo_R;
  if (subiendo) {
    ISRCounter_L = 0;
    ISRCounter_R = 0;
    Stop_L();
    Stop_R();
    Z_done = 1;
  }
}

void TwoDown(void) {
  int subiendo_L = 1;
  int subiendo_R = 1;
  fc();
  if (Pasos_pisoR != 0) {
    if (FC_Z_L_D && ISRCounter_L < Pasos_pisoL) {
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 240);
    } else {
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);
      subiendo_L = 0;
    }
    if (FC_Z_R_D && ISRCounter_R < Pasos_pisoR) {
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 250);
    } else {
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      subiendo_R = 0;
    }
  }
  subiendo = subiendo_L || subiendo_R;
  if (ISRCounter_R >= Pasos_pisoR && ISRCounter_L >= Pasos_pisoL) {
    subiendo = 0;
    Z_done = 1;
  }
  if (!subiendo) {
    ISRCounter_L = 0;
    ISRCounter_R = 0;
    Stop_L();
    Stop_R();
    Z_done = 1;
  }
}

/*
   Detiene el movimiento del motor izquierdo
*/

void Stop_L(void) {
  analogWrite(LPWM_Z_UP, 0);
  analogWrite(LPWM_Z_DOWN, 0);
}

/*
   Detiene el movimiento del motor derecho
*/

void Stop_R(void) {
  analogWrite(RPWM_Z_UP, 0);
  analogWrite(RPWM_Z_DOWN, 0);
}


//*******************************************************************
//* Funciones de movimiento del Motor paso a paso                   *
//*******************************************************************

void Left_Y() {

  myStepper.step(-990);

  myStepper.step(0);
  Y_done = 1;
  digitalWrite(42, LOW);
  digitalWrite(44, LOW);
  digitalWrite(46, LOW);
  digitalWrite(48, LOW);
}


void Right_Y() {

  myStepper.step(990);

  myStepper.step(0);
  Y_done = 1;
  digitalWrite(42, LOW);
  digitalWrite(44, LOW);
  digitalWrite(46, LOW);
  digitalWrite(48, LOW);
}

//*******************************************************************
//* Funciones de movimiento del eje X                               *
//* 0 Posicion Home                                                 *
//*******************************************************************



void X_Axis(bool Posx) {
  FC_X_L = digitalRead(FCXL);
  FC_X_R = digitalRead(FCXR);

  if (Posx) {
    if (FC_X_R) {
      FC_X_R = digitalRead(FCXR);
      digitalWrite(R_X, HIGH);
      digitalWrite(L_X, LOW);
    } else {
      digitalWrite(R_X, LOW);
      digitalWrite(L_X, LOW);
      X_done = 1;
    }
  }
  if (!Posx) {
    if (FC_X_L) {
      FC_X_L = digitalRead(FCXL);
      digitalWrite(R_X, LOW);
      digitalWrite(L_X, HIGH);
    } else {
      digitalWrite(R_X, LOW);
      digitalWrite(L_X, LOW);
      X_done = 1;

    }
  }
}

//*******************************************************************
//* Rutina Set Up                                                   *
//* Eje Z parte superior                                            *
//* Conteo pasos Stepper                                            *
//*******************************************************************

void Rutinas_almacenar() {

  X_done = 0;
  Y_done = 0;
  Z_done = 0;
  switch (estado) {

    case 1: //Yendo a celda 1
      // c1,p1,x1,R
      Pasos_pisoL=0;
      Pasos_pisoR=0;
      Pos_X = 1;

      while (!Z_done || !X_done) {
        if (!Z_done) { //Bajando Z
          TwoDown();
        }
        if (!X_done) {//Moviendo X
          X_Axis(Pos_X);
        }
      }
      while (!Y_done) {//Moviendo Y
        Right_Y();// define a que lado sale primero
      }

      Y_done = 0;
      while (!Y_done) {//Moviendo Y2
        Left_Y();
      }

      //Subiendo Z
      //Moviendo X
      Home();

      estado = 0;
      break;

    case 2://Yendo a celda 2
      // c2,p1,x1,L
      Pasos_pisoL=0;
      Pasos_pisoR=0;
      Pos_X = 1;

      //Bajando Z
      //Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y

      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }


      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      //volver a home
      Home();

      estado = 0;
      break;

    case 3://Yendo a celda 3
      // c3,p1,x0,L
      Pasos_pisoL=0;
      Pasos_pisoR=0;
      Pos_X = 0;

      //Bajando Z
      //Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y

      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }

      //Moviendo Y2
      while (!Y_done) {
        Right_Y();
      }

      //volver a home
      X_done = 0;
      Z_done = 0;

      Home();

      estado = 0;
      break;
    ///////////////////////////// Piso 2 ALMACENAR.......
    case 4://Yendo a celda 4
      // c4,p2,x1,R
      Pasos_pisoL=DisPiso2*consL;
      Pasos_pisoR=DisPiso2*consR;
      Pos_X = 1;

      //Bajando Z
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //moviendo x
      while (!X_done) {
        delay(10);
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }


      //moviendo Y
      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }
      delay(500);
      Down_L();
      Down_R();
      delay(2000);
      Stop_R();
      Stop_L();
      delay(200);

      //moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }


      X_done = 0;
      Z_done = 0;
      //moviendo x
      while (!X_done) {
        delay(10);
        if (!X_done) {
          X_Axis(0);
        }
      }
      //moviendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }

      estado = 0;
      break;

    // CELDA 5 ALMACENAR
    case 5://Yendo a celda 5
      // c5,p2,x1,L
      Pasos_pisoL=DisPiso2*consL;
      Pasos_pisoR=DisPiso2*consR;
      Pos_X = 1;


      delay(1000);
      //Bajando Z
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //Moviendo X
      while (!X_done) {
        delay(10);
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }
      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);
      Down_L();
      Down_R();
      delay(2000);
      Stop_R();
      Stop_L();
      delay(200);

      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      X_done = 0;
      Z_done = 0;
      //Moviendo X
      while (!X_done) {
        delay(10);
        if (!X_done) {
          X_Axis(0);
        }
      }
      //Subiendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }

      estado = 0;
      break;

    case 6://Yendo a celda 6
      // c6,p2,x0,R
      Pasos_pisoL=DisPiso2*consL;
      Pasos_pisoR=DisPiso2*consR;
      Pos_X = 0;
      delay(1000);

      //Bajando Z  Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }
      delay(500);
      Down_L();
      Down_R();
      delay(2000);
      Stop_R();
      Stop_L();
      delay(200);

      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }

      X_done = 0;
      Z_done = 0;

      //Moviendo X
      while (!X_done) {
        delay(10);
        if (!X_done) {
          X_Axis(0);
        }
      }
      //Subiendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }

      estado = 0;
      break;

    case 7://Yendo a celda 7
      // c5,p2,x0,L
      Pasos_pisoL=DisPiso2*consL;
      Pasos_pisoR=DisPiso2*consR;
      Pos_X = 0;

      //Bajando Z  Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y

      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);
      Down_L();
      Down_R();
      delay(2000);
      Stop_R();
      Stop_L();
      delay(200);

      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      X_done = 0;
      Z_done = 0;
      //Moviendo X
      while (!X_done) {
        delay(10);
        if (!X_done) {
          X_Axis(0);
        }
      }
      //Subiendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }

      estado = 0;
      break;

    //////////////////////////////////////////////

    case 8://Yendo a celda 8
      // c8,p3,x1,R
      Pasos_pisoL=DisPiso1*consL;
      Pasos_pisoR=DisPiso1*consR;
      Pos_X = 1;

      //Bajando Z
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //Moviendo X
      while (!X_done) {
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }
      delay(500);
      Down_L();
      Down_R();
      delay(2000);
      Stop_R();
      Stop_L();
      delay(200);
      Z_done = 0;
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }

      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }


      X_done = 0;
      Z_done = 0;
      //Moviendo X
      while (!X_done) {
        delay(10);

        if (!X_done) {
          X_Axis(0);
        }
      }
      //Subiendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }
      estado = 0;
      break;

    case 9://Yendo a celda 9
      // c9,p3,x1,L
      Pasos_pisoL=DisPiso1*consL;
      Pasos_pisoR=DisPiso1*consR;
      Pos_X = 1;



      //Bajando Z
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //Moviendo X
      while (!X_done) {
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }


      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);
      Down_L();
      Down_R();
      delay(2000);
      Stop_R();
      Stop_L();
      delay(200);
      Z_done = 0;
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }


      X_done = 0;
      Z_done = 0;
      //Moviendo X
      while (!X_done) {
        delay(10);

        if (!X_done) {
          X_Axis(0);
        }
      }
      //Subiendo Z

      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }

      estado = 0;
      break;

    case 10://Yendo a celda 10
      // c10,p3,x0,R
      Pasos_pisoL=DisPiso1*consL;
      Pasos_pisoR=DisPiso1*consR;
      Pos_X = 0;

      //Bajando Z Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }
      delay(500);
      Down_L();
      Down_R();
      delay(2000);
      Stop_R();
      Stop_L();
      delay(200);
      Z_done = 0;
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }

      delay(200);

      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }

      X_done = 0;
      Z_done = 0;
      //Moviendo X
      while (!X_done) {
        delay(10);
        if (!X_done) {
          X_Axis(0);
        }
      }

      //Subiendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }

      estado = 0;
      break;

    case 11://Yendo a celda 11
      // c11,p3,x0,L
      Pasos_pisoL=DisPiso1*consL;
      Pasos_pisoR=DisPiso1*consR;
      Pos_X = 0;

      //Bajando Z //Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);
      delay(500);
      Down_L();
      Down_R();
      delay(2000);
      Stop_R();
      Stop_L();
      delay(200);
      Z_done = 0;
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      X_done = 0;
      Z_done = 0;
      //Moviendo X
      while (!X_done) {
        delay(10);
        if (!X_done) {
          X_Axis(0);
        }
      }
      //Subiendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }

      estado = 0;
      break;

    default://Celda no programada
      estado = 0;
  }
}


void Home() {

  X_done = 0;
  Z_done = 0;

  while (!X_done) {
    delay(10);//
    if (!X_done) {

      X_Axis(0);
    }
  }
  while (!Z_done) {
    if (!Z_done) {
      TwoUp();
    }
  }




  X_done = 0;
  Z_done = 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Inicialización de las variables del nodo de ROS                                                                                                                                       /
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float x;
void rutina_retirar();
ros::NodeHandle nh;
std_msgs::Int16 test;
ros::Publisher p("response", &test);
void rutina_retirar() {
  X_done = 0;
  Y_done = 0;
  Z_done = 0;
  switch (estado) {

    case 1://Yendo a celda 1
      // c1,p1,x1,R
     Pasos_pisoL=0;
      Pasos_pisoR=0;
      Pos_X = 1;

      //Bajando Z Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }


      delay(500);
      //mover Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }

      //volver a home
      Home();

      estado = 0;
      break;

    case 2://Yendo a celda 2
      // c2,p1,x1,L
      Pasos_pisoL=0;
      Pasos_pisoR=0;
      Pos_X = 1;


      //Bajando Z Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }

      //Moviendo Y2
      delay(500);

      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      //volver home
      Home();

      estado = 0;
      break;

    case 3://Yendo a celda 3
      // c3,p1,x0,L
      Pasos_pisoL=0;
      Pasos_pisoR=0;
      Pos_X = 0;

      //Bajando Z Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);


      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      //volver a home
      X_done = 0;
      Z_done = 0;

      Home();

      estado = 0;
      break;

    case 4: //Yendo a celda 4
      // c4,p2,x1,R
      Pasos_pisoL=DisPiso2*consL;
      Pasos_pisoR=DisPiso2*consR;
      Pos_X = 1;

      //Bajando Z

      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //Moviendo X
      while (!X_done) {
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }




      //Moviendo Y
      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }
      delay(500);
      analogWrite(RPWM_Z_UP, 255);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 255);
      analogWrite(LPWM_Z_DOWN, 0);
      delay(2000);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);

      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }

      X_done = 0;
      Z_done = 0;
      //Moviendo X
      while (!X_done) {
        delay(10);

        if (!X_done) {
          X_Axis(0);
        }
      }
      //Subiendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }
      estado = 0;
      break;

    case 5://Yendo a celda 5
      // c5,p2,x1,L
      Pasos_pisoL=DisPiso2*consL;
      Pasos_pisoR=DisPiso2*consR;
      Pos_X = 1;

      //Bajando Z
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //Moviendo X
      while (!X_done) {
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }


      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);
      analogWrite(RPWM_Z_UP, 255);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 255);
      analogWrite(LPWM_Z_DOWN, 0);
      delay(2000);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }


      X_done = 0;
      Z_done = 0;
      //Moviendo X
      while (!X_done) {
        delay(10);

        if (!X_done) {
          X_Axis(0);
        }
      }
      //Subiendo Z
      while (!Z_done) {
        if (!Z_done) {
          TwoUp();
        }
      }


      estado = 0;
      break;

    case 6://Yendo a celda 6
      // c6,p2,x0,R
      Pasos_pisoL=DisPiso2*consL;
      Pasos_pisoR=DisPiso2*consR;
      Pos_X = 0;

      //Bajando Z Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }
      delay(500);
      analogWrite(RPWM_Z_UP, 255);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 255);
      analogWrite(LPWM_Z_DOWN, 0);
      delay(2000);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }

      //regresar home

      X_done = 0;
      Z_done = 0;

      Home();
      estado = 0;
      break;

    case 7://Yendo a celda 7
      // c5,p2,x0,L
      Pasos_pisoL=DisPiso2*consL;
      Pasos_pisoR=DisPiso2*consR;
      Pos_X = 0;

      //Bajando Z Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);
      analogWrite(RPWM_Z_UP, 255);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 255);
      analogWrite(LPWM_Z_DOWN, 0);
      delay(2000);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      //regreso home
      X_done = 0;
      Z_done = 0;

      Home();
      estado = 0;
      break;


    //////////////////////////////////////////////

    case 8://Yendo a celda 8
      // c8,p3,x1,R
      Pasos_pisoL=DisPiso1*consL;
      Pasos_pisoR=DisPiso1*consR;
      Pos_X = 1;

      //Bajando Z
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //Moviendo X
      while (!X_done) {
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }
      delay(500);
      analogWrite(RPWM_Z_UP, 255);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 255);
      analogWrite(LPWM_Z_DOWN, 0);
      delay(2000);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }

      //regresar a home
      X_done = 0;
      Z_done = 0;
      Home();
      estado = 0;
      break;

    case 9://Yendo a celda 9
      // c9,p3,x1,L
      Pasos_pisoL=DisPiso1*consL;
      Pasos_pisoR=DisPiso1*consR;
      Pos_X = 1;

      //Bajando Z
      while (!Z_done) {
        if (!Z_done) {
          TwoDown();
        }
      }
      //mover X
      while (!X_done) {
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }


      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);
      analogWrite(RPWM_Z_UP, 255);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 255);
      analogWrite(LPWM_Z_DOWN, 0);
      delay(2000);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      //volver home
      X_done = 0;
      Z_done = 0;
      Home();

      estado = 0;
      break;

    case 10://Yendo a celda 10
      // c10,p3,x0,R
      Pasos_pisoL=DisPiso1*consL;
      Pasos_pisoR=DisPiso1*consR;
      Pos_X = 0;

      //Bajando Z Moviendo X

      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y

      while (!Y_done) {
        Right_Y();// define a que lado sale primero
      }
      delay(500);
      analogWrite(RPWM_Z_UP, 255);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 255);
      analogWrite(LPWM_Z_DOWN, 0);
      delay(2000);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Left_Y();
      }

      //volver Home
      X_done = 0;
      Z_done = 0;
      Home();
      
      estado = 0;
      break;

    case 11://Yendo a celda 11
      // c11,p3,x0,L
      ////////////Aquí se varía la altura a la cual empieza el despliegue de la caja ///////////////////////////////
      Pasos_pisoL=DisPiso1*consL;
      Pasos_pisoR=DisPiso1*consR; //////Entre menor sea este valor, mayor será la altura a la cual empieza///////////////////
      Pos_X = 0;

      //Bajando Z Moviendo X
      while (!Z_done || !X_done) {
        if (!Z_done) {
          TwoDown();
        }
        if (!X_done) {
          X_Axis(Pos_X);
        }
      }

      //Moviendo Y
      while (!Y_done) {
        Left_Y();// define a que lado sale primero
      }
      delay(500);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 255);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 255);
      delay(2250);
      analogWrite(RPWM_Z_UP, 0);
      analogWrite(RPWM_Z_DOWN, 0);
      analogWrite(LPWM_Z_UP, 0);
      analogWrite(LPWM_Z_DOWN, 0);
      //Moviendo Y2
      Y_done = 0;
      while (!Y_done) {
        Right_Y();
      }

      //volver a home
      X_done = 0;
      Z_done = 0;

      Home();

      estado = 0;
      break;

    default://celda no programada
      estado = 0;
  }

  ///////Final de la rutina de retirar /////////////////
};

void toSDV() {
  /////Se mueve el motor de la tijera //////////////////
  digitalWrite(DIR, HIGH);
  for (int i = 0; i < 200; i++) //Forward 1600 steps
  {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(1000);
    digitalWrite(PUL, LOW);
    delayMicroseconds(1000);
  }

  delay(1000);

  ///////////Despliegue y retracción de la compúerta////////////
  digitalWrite(DIR1, LOW);
  for (int i = 0; i < 800; i++) //Forward 1600 steps
  {
    digitalWrite(PUL1, HIGH);
    delayMicroseconds(15000);   //Este valor de periodo NO SE CAMBIA
    digitalWrite(PUL1, LOW);
    delayMicroseconds(15000);
  }
  delay(1000);

  //Hacia abajo/

  digitalWrite(DIR1, HIGH);
  for (int i = 0; i < 800; i++) //Backward 1600 steps
  {
    digitalWrite(PUL1, HIGH);
    delayMicroseconds(150000);
    digitalWrite(PUL1, LOW);
    delayMicroseconds(150000);
  }
  delay(1000);
  //////////////////////////////////////////////////////////////

  digitalWrite(DIR, LOW);

  for (int i = 0; i < 200; i++) //Backward 1600 steps
  {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(8000);
    digitalWrite(PUL, LOW);
    delayMicroseconds(8000);
  }
  delay(1000);
  ///////////////////////////////////////////////////////////////////////
}
void ROS_CALLBACK(std_msgs::Int16 msg) {
  estado = msg.data;
  X_done = 0;
  Y_done = 0;
  Z_done = 0;
  rutina_retirar();
  estado = 11;
  X_done = 0;
  Y_done = 0;
  Z_done = 0;
  rutina_retirar();
  toSDV();
  test.data = x;
  p.publish( &test);
}


ros::Subscriber<std_msgs::Int16> s("chatter", ROS_CALLBACK);


void setup()
{

  //*****************Variables de la tijera***********
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (EN, OUTPUT);
  digitalWrite(EN, HIGH);
  //***************Variables de la compuerta**********
  pinMode (PUL1, OUTPUT);
  pinMode (DIR1, OUTPUT);
  pinMode (EN1, OUTPUT);
  digitalWrite(EN1, LOW);
  ////////////////////////////////////////////////////

  // ////Serial.begin(9600);
  //delay(2000);

  //*******************************************************************
  //*                       Inicializacion ////lcd                        *
  //*******************************************************************
  // Indicar a la libreria que tenemos conectada una pantalla de 16x2
  ////lcd.begin(16, 2);
  // Mover el cursor a la primera posición de la pantalla (0, 0)
  ////lcd.home ();
  // Esperar un segundo
  delay(1000);
  //*******************************************************************
  //*                       Inicializacion Motores eje Z              *
  //* Finales de carrera normalmente en 1                             *
  //*******************************************************************

  pinMode(Encoder_R, INPUT);

  pinMode(FCZLU, INPUT_PULLUP);
  pinMode(FCZRU, INPUT_PULLUP);
  pinMode(FCZLD, INPUT_PULLUP);
  pinMode(FCZRD, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Encoder_R), interruptCount_R, CHANGE );
  attachInterrupt(digitalPinToInterrupt(Encoder_L), interruptCount_L, CHANGE );

  //*******************************************************************
  //*                 Inicializacion Motor paso a paso                *
  //* Finales de carrera normalmente en 1                             *
  //*******************************************************************

  myStepper.setSpeed(60);
  //pinMode(FCYR, INPUT_PULLUP);
  //  pinMode(FCYA, INPUT_PULLUP);

  //*******************************************************************
  //*                 Inicializacion Motor Eje X                      *
  //*******************************************************************

  pinMode(FCXR, INPUT_PULLUP);
  pinMode(FCXL, INPUT_PULLUP);

  ////lcd.home ();
  ////lcd.clear ();
  // ////lcd.print("Inicio");



  estado = 0;
  ////lcd.home ();
  ////lcd.clear ();
  ////lcd.print("Home");
  //Right_Y();
  Home();

  almacenar = 0;
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{
  nh.spinOnce();

}
