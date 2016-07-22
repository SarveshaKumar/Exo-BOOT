#include <unistd.h>
#include <sys/stat.h>
#include "PWM.h"
#include "util.h"
#include <cstdlib>
#include<iostream>
#include<fstream>
#include<sstream>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include<math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <sys/time.h>

#define OE_ADDR 0x134
#define GPIO_DATAOUT 0x13C
#define GPIO_DATAIN 0x138
#define GPIO0_ADDR 0x44E07000
#define GPIO1_ADDR 0x4804C000
#define GPIO2_ADDR 0x481AC000
#define GPIO3_ADDR 0x481AF000

#define HIGH 1
#define LOW 0
using namespace std;
double rPose=0;
int r_last = LOW;
int inr = 0;
int inrB = 0;

namespace exploringBB {
	
int write(string path, string filename, string value){
   ofstream fs;
   fs.open((path + filename).c_str());
   if (!fs.is_open()){
	   perror("GPIO: write failed to open file ");
	   return -1;
   }
   fs << value;
   fs.close();
   return 0;
}

string read(string path, string filename){
   ifstream fs;
   fs.open((path + filename).c_str());
   if (!fs.is_open()){
	   perror("GPIO: read failed to open file ");
    }
   string input;
   getline(fs,input);
   fs.close();
   return input;
}
int write(string path, string filename, int value){
   stringstream s;
   s << value;
   return write(path,filename,s.str());
}

PWM::PWM(string pinName) {
	this->name = pinName;
	this->path = PWM_PATH + this->name + "/";
	this->analogFrequency = 100000;
	this->analogMax = 3.3;
}

int PWM::setPeriod(unsigned int period_ns){
	return write(this->path, PWM_PERIOD, period_ns);
}

unsigned int PWM::getPeriod(){
	return atoi(read(this->path, PWM_PERIOD).c_str());
}

float PWM::period_nsToFrequency(unsigned int period_ns){
	float period_s = (float)period_ns/1000000000;
	return 1.0f/period_s;
}

unsigned int PWM::frequencyToPeriod_ns(float frequency_hz){
	float period_s = 1.0f/frequency_hz;
	return (unsigned int)(period_s*1000000000);
}

int PWM::setFrequency(float frequency_hz){
	return this->setPeriod(this->frequencyToPeriod_ns(frequency_hz));
}

float PWM::getFrequency(){
	return this->period_nsToFrequency(this->getPeriod());
}

int PWM::setDutyCycle(unsigned int duty_ns){
	return write(this->path, PWM_DUTY, duty_ns);
}

int PWM::setDutyCycle(double percentage){
	if ((percentage>100.0f)||(percentage<0.0f)) return -1;
	unsigned int period_ns = this->getPeriod();
	float duty_ns = period_ns * (percentage/100.0f);
	this->setDutyCycle((unsigned int) duty_ns );
	return 0;
}

unsigned int PWM::getDutyCycle(){
	return atoi(read(this->path, PWM_DUTY).c_str());
}

float PWM::getDutyCyclePercent(){
	unsigned int period_ns = this->getPeriod();
	unsigned int duty_ns = this->getDutyCycle();
	return 100.0f * (float)duty_ns/(float)period_ns;
}

int PWM::setPolarity(PWM::POLARITY polarity){
	return write(this->path, PWM_POLARITY, polarity);
}

void PWM::invertPolarity(){
	if (this->getPolarity()==PWM::ACTIVE_LOW) this->setPolarity(PWM::ACTIVE_HIGH);
	else this->setPolarity(PWM::ACTIVE_LOW);
}

PWM::POLARITY PWM::getPolarity(){
	if (atoi(read(this->path, PWM_POLARITY).c_str())==0) return PWM::ACTIVE_LOW;
	else return PWM::ACTIVE_HIGH;
}

int PWM::calibrateAnalogMax(float analogMax){ //must be between 3.2 and 3.4
	if((analogMax<3.2f) || (analogMax>3.4f)) return -1;
	else this->analogMax = analogMax;
	return 0;
}

int PWM::analogWrite(float voltage){
	if ((voltage<0.0f)||(voltage>3.3f)) return -1;
	this->setFrequency(this->analogFrequency);
	this->setPolarity(PWM::ACTIVE_LOW);
	this->setDutyCycle((100.0f*voltage)/this->analogMax);
	return this->run();
}

int PWM::run(){
	return write(this->path, PWM_RUN, 1);
}

bool PWM::isRunning(){
	string running = read(this->path, PWM_RUN);
	return (running=="1");
}

int PWM::stop(){
	return write(this->path, PWM_RUN, 0);
}

PWM::~PWM() {}

} /* namespace exploringBB */

double encoder(){

    
    char logicA = '1';
    char logicB = '2';
    int fd = open("/dev/mem",O_RDWR | O_SYNC);
    ulong* pinconf1 =  (ulong*) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO1_ADDR);
    //configure encoder pins as input
    pinconf1[OE_ADDR/4]  |= ((1<<13)|(1 << 12)|(1<<15)|(1<<14)); //P8_11, P8_12, P8_15, P8_16   

   //Read
	{
		//inr = read_rEncoder(*pinconf1, logicA); //logic A
		if(pinconf1[GPIO_DATAIN/4] & (1 << 13)){
//                        cout << "A is HIGH" <<endl;
                        inr = HIGH;
                }else{
//                        cout << "A is LOW" << endl;
                        inr = LOW;
                }


		if((r_last == LOW)&&(inr == HIGH)){
			if(pinconf1[GPIO_DATAIN/4] & (1 << 12)) {
//	                        cout << "B is HIGH" << endl;
        	                inrB = HIGH;
                	}else{
//                        	cout << "B is low" << endl;
                        	inrB = LOW;
                	}

			if(inrB == LOW){
		//		cout << "increasing" << endl;
				rPose--;
			}else{
		//		cout << "decreasing" << endl;
				rPose++;
			}
		}
		r_last = inr;
		cout <<"\t rpose="<<rPose<<"\t";
//		sleep(.01);
	}
	
	return(rPose);
}


double fxdes(double k)
{  
double g;
if(k<250)

g=10;

else if(k>=250 && k<=500)
g=15;

else
g=10;

return g;}


    int main(){
    
    using namespace exploringBB;
   PWM pwm_m1("pwm_test_P9_42.15");  // example alternative pin
   double duty; 
 struct timeval t1, t2;
    double elapsedTime=0,t=0,e2=0,prev=0,de,dt;
double xdes,xact,e,kp=0.04,kd=0.004;
   double te,derr;      
 gettimeofday(&t1, NULL);
 while(t!=1000)

  { 
  
  xdes=fxdes(elapsedTime);
  xact=encoder();
       e=xdes-xact;  
  
  gettimeofday(&t2, NULL);

    // compute and print the elapsed time in millisec
    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
    de=e-e2;
    dt=elapsedTime-prev;
    derr=de/dt;
    te=kp*e+kd*1000*(derr);
    cout << elapsedTime << " ms.\n";t=elapsedTime; 
    e2=e;
    prev=elapsedTime;
    cout<<"te ="<<te<<"\t" ;
    //ofstream outputFile;
    //outputFile.open("xact.txt");
    //outputFile<<xact<<"\t \t"<<elapsedTime<<"\n";
    //outputFile.close();
    

  if(te<0) 
   {duty=40;}
   else if(te>0)
   {duty=90;}
   else
   {duty=51;}
  
   pwm_m1.setFrequency(400);         // Set the period in ns
   pwm_m1.setDutyCycle(duty*1.0f);       // Set the duty cycle as a percentage
   pwm_m1.setPolarity(PWM::ACTIVE_LOW);  // using active low PWM
   pwm_m1.run();  
      //xdes-=50; 
     
  }
 
 
   }     // start the PWM output
