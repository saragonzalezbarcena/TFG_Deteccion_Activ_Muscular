// Librerias I2C necesarias para controlar MPU6050
// libreria MPU6050.h requiere I2Cdev.h
// libreria I2Cdev.h requiere Wire.h

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


// La dirección del MPU6050 puede ser 0x68 o 0x69 
// en función del estado de AD0. (0x68 por defecto)

MPU6050 mpu;
// en caso de ser 0x69: MPU6050 mpu(0x69);

// Valores sin procesar (valores RAW) del acelerómetro y giroscopio (ejes x, y, z)
int ax, ay, az; // rango por defecto: -2g a +2g
int gz, gy, gz; // rango por defecto: -250°/sec a +250°/sec


long tiempo_prev
float dt;
float girosc_ang_x, girosc_ang_y;
float girosc_ang_x_prev, girosc_ang_y_prev;

// variables para filtro complemento
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;


// Factores de conversion (valores SI)
const float accScale = 2.0 * 9.81 / 32768.0;
const float gyroScale = 250.0 / 32768.0;

void printMedidas()
{
	// Mostrar medidas en el Sistema Internacional
	Serial.print(F("a[x y z](m/s2) g[x y z](deg/s):\t"));
  	Serial.print(ax * accScale); Serial.print("\t");
  	Serial.print(ay * accScale); Serial.print("\t");
  	Serial.print(az * accScale); Serial.print("\t");
  	Serial.print(gx * gyroScale); Serial.print("\t");
  	Serial.print(gy * gyroScale); Serial.print("\t");
  	Serial.println(gz * gyroScale);
}


void getGiro()
{
	// Calcular el ángulo de rotación
	dt = millis() - tiempo_prev;
   	tiempo_prev = millis();

   	girosc_ang_x = (gx / 131)*dt / 1000.0 + girosc_ang_x_prev;
   	girosc_ang_y = (gy / 131)*dt / 1000.0 + girosc_ang_y_prev;

   	girosc_ang_x_prev = girosc_ang_x;
   	girosc_ang_y_prev = girosc_ang_y;
}


void filtroComplemento()
{
	// ecuación para calcular el ángulo usando el filtro de complemento
	// angulo = 0.98(angulo + girosc_ang dt)+0.02(accel_ang)
	// se pueden probar valores distintos a 0.98 y 0.02, pero siempre tienen que dar 1
	
	dt = (millis() - tiempo_prev) / 1000.0;
   	tiempo_prev = millis();

   	//Calcular los ángulos con acelerometro
   	float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   	float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
   	
	//Calcular angulo de rotación con giroscopio y filtro complementario
   	ang_x = 0.98*(ang_x_prev + (gx / 131)*dt) + 0.02*accel_ang_x;
   	ang_y = 0.98*(ang_y_prev + (gy / 131)*dt) + 0.02*accel_ang_y;

   	ang_x_prev = ang_x;
   	ang_y_prev = ang_y;
}

void setup()
{
	// Configuración de pines
	// GND - GND
	// VCC - VCC
	// SDA - Pin A4
	// SCL - Pin A5
	pinMode(A4, INPUT);
	pinMode(A5, INPUT);

	Serial.begin(9600); // Iniciar puerto serial
	Wire.begin(); // Iniciar I2C
	mpu.initialize(); // Iniciar sensor

	if (mpu.testConnection()) Serial.println("Sensor iniciado correctamente");
  	else Serial.println("Error al iniciar el sensor");
}

// La resolución de las lecturas es de 16 bits (rango de lectura es de -32768 a 32767)

void loop()
{
	// Leer las aceleraciones y velocidades angulares
	mpu.getAcceleration(&ax, &ay, &az);
	mpu.getRotation(&gx, &gy, &gz);

	printMedidas();
	
	//Calcular los angulos de inclinacion:
 	float accel_ang_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  	float accel_ang_y=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);

  	//Mostrar los angulos de inclinacion del acelerometro
  	Serial.print("Inclinacion en X: ");
  	Serial.print(accel_ang_x); 
  	Serial.print("tInclinacion en Y:");
  	Serial.println(accel_ang_y);


	// Calcular los angulos de rotacion del giroscopio 
	/getGiro();
	// Mostrar los angulos de rotacion
   	Serial.print(F("Rotacion en X:  "));
   	Serial.print(girosc_ang_x);
   	Serial.print(F("\tRotacion en Y: "));
   	Serial.println(girosc_ang_y);


	// Calcular los angulos de rotacion del giroscopio y filtro complemento
	filtroComplemento();
	Serial.print(F("Rotacion en X:  "));
   	Serial.print(ang_x);
   	Serial.print(F("\t Rotacion en Y: "));
   	Serial.println(ang_y);
  	
	//delay(100);

}
