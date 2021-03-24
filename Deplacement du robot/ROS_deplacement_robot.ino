#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

// Inclusion des librairies
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//Création d'un node ROS pour la communication
ros::NodeHandle  nh;
std_msgs::Float32 str_msg;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Le node ROS crée affichera sur la Jetson les valeurs du capteur à ultrason et de l'IMU
ros::Publisher chatter("Ultrason", &str_msg);
ros::Publisher chatter2("Euler", &str_msg);
// Création des variables vitesse gauche et vitesse droite qui serviront à ajuster la trajectoire du robot
int vitesse_G = 0;
int vitesse_D = 0;

int i = 0;
int a = 0;
//Pins du capteur à ultrason
int trig = 10;
int echo = 9;
long lecture_echo;
long cm;

//Avant gauche 
int AvG1 = 37;
int AvG2 = 35;
int enaAvG1 = 6;

//Arrière gauche 
int ArG1 = 29;
int ArG2 = 31;
int enaArG1 = 3;

//Avant droit
int AvD1 =51;
int AvD2 =49;
int enaAvD1 =5;

//Arrière droit
int ArD1 = 50;
int ArD2 = 48;
int enaArD1 = 12;

//Création de la fonction d'avance du robot
void avancer(int vitesse) {
    
  digitalWrite(AvG1, LOW);
  digitalWrite(AvG2, HIGH);

  digitalWrite(AvD1, HIGH);
  digitalWrite(AvD2, LOW);

  digitalWrite(ArG1, LOW);
  digitalWrite(ArG2, HIGH);
  
  digitalWrite(ArD1, LOW);
  digitalWrite(ArD2, HIGH);
  
  //Lecture des valeurs de l'IMU
  sensors_event_t event; 
  bno.getEvent(&event);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //On publie les valeurs de l'IMU (angle)
  str_msg.data = euler.x();
  chatter2.publish( &str_msg );
  
  //Si on devie vers la gauche on stop les roues de droite
  if (euler.x()>=180) {
    vitesse_G = vitesse;
    vitesse_D = 0;
  }
  
  //Si on devie vers la droite on stop les roues de gauche
  if (euler.x()<=180) {
    vitesse_G = 0;
    vitesse_D = vitesse;
  }
  //Si le robot devie vers la gauche on compense en le faisant tourner a droite et inversement
  analogWrite(enaAvG1, vitesse_G);
  analogWrite(enaAvD1, vitesse_D);
  analogWrite(enaArG1, vitesse_G);
  analogWrite(enaArD1, vitesse_D);
}
//Cretion de la fonction qui permet au robot de reculer droit
void reculer() {
    
  digitalWrite(AvG1, HIGH);
  digitalWrite(AvG2, LOW);

  digitalWrite(AvD1, LOW);
  digitalWrite(AvD2, HIGH);

  digitalWrite(ArG1, HIGH);
  digitalWrite(ArG2, LOW);
  
  digitalWrite(ArD1, HIGH);
  digitalWrite(ArD2, LOW);

  analogWrite(enaAvG1, 255);
  analogWrite(enaAvD1, 255);
  analogWrite(enaArG1, 255);
  analogWrite(enaArD1, 255);
}
//Creation de la fonction qui permet au robot de freiner
void stopMoteur() {
  digitalWrite(AvG1, LOW);
  digitalWrite(AvG2, LOW);

  digitalWrite(AvD1, LOW);
  digitalWrite(AvD2, LOW);


  digitalWrite(ArG1, LOW);
  digitalWrite(ArG2, LOW);

  digitalWrite(ArD1, LOW);
  digitalWrite(ArD2, LOW);
}
//La arduino recoit des indications de vitesse par la jetson nano grace au node de ROS
void rot_moteur( const std_msgs::UInt16& cmd_msg){
  
  a=cmd_msg.data;
}

ros::Subscriber<std_msgs::UInt16> sub("speed", rot_moteur);

void setup() {
  //On definis le type de chaque pins
  analogWrite(enaAvG1, 0);
  analogWrite(enaAvD1, 0);
  analogWrite(enaArG1, 0);
  analogWrite(enaArD1, 0);

  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);

  //Initialisation des nodes de ROS 
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  nh.advertise(chatter2);

  //Initialisation de l'IMU
  if(!bno.begin())
  {
    /* Si nous ne detectons pas le BN0055 */
    Serial.print("BNO055 non detecte");
    while(1);
  }
  
  bno.setExtCrystalUse(true);

  while (i != 10){
    sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.println(event.orientation.x, 4);
  delay(100);

  i=i+1;
}
}
//Fonction de deplacement du robot
void loop() {
  
  nh.spinOnce();
  delay(1);
  //On avance en fonction de la vitesse donne par la Jetson Nano
  avancer(a);
  //On lit la donne du capteur a ultrason
  digitalWrite(trig, HIGH);
  delayMicroseconds(100);
  digitalWrite(trig, LOW);
  lecture_echo = pulseIn(echo,HIGH);
  cm = lecture_echo /58;
  str_msg.data = cm;
  chatter.publish( &str_msg );
  //Tant qu'il n'y a pas d'obstacles on avance
  //On prevoit une grande distance a cause de l'inertie des roues
  //Pour contrer l'inertie on recule un bref instant
  while (cm<70){
    digitalWrite(trig, HIGH);
    delayMicroseconds(100);
    digitalWrite(trig, LOW);
    lecture_echo = pulseIn(echo,HIGH);
    cm = lecture_echo /58;
    reculer();
    delay(300);
    //Tant que qu'il n'y a pas d'obstacle en vue on avance
    //Une fois un obstacle vu, le robot recule un peu et sera a l'arret lorsque l'obstacles sera entre 70 et 85 cm
    while (cm<85){
      stopMoteur();
      delay(100);
      digitalWrite(trig, HIGH);
      delayMicroseconds(100);
      digitalWrite(trig, LOW);
      lecture_echo = pulseIn(echo,HIGH);
      cm = lecture_echo /58;
    }
  }
}
