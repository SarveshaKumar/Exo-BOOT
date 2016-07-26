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

int encoder(){ using namespace std; int f,enc;
	const char *fname = "/sys/devices/ocp.3/48304000.epwmss/48304180.eqep/position";
        int count = 0, fd, len;
        char enco[32] = {0};
               
         
                fd = open(fname, O_RDONLY);
                if(fd == -1){
                       // printf("error: %s %d\n", strerror(errno), errno);
                        exit(1);
	                }
 
              //  if(count % 10 == 0 && count != 0)
                        cout<<"\n";
 
	                len = read(fd, enco, sizeof(enco));
 
	                if(len == -1){
	                        close(fd);
	                                   
	                }
	                else if(len == 0){
	                        cout<<"\n"<<"buffer is empty";
	                }
	                else{  cout<<"\n";
	                        enco[len] ='\0';
	                        
	                        enc= atoi(enco);
 
	                        cout<<enc<<"\t\t";
	               
 
	                close(fd);
 
	              //  count++;
	        }
 
	        return  enc;
	}

int fxdes(int k)
{  
int g;
if(k<10000)

g=10000;

else if(k>=10000 && k<=20000)
g=50000;

else
g=10000;

return g;}


    int main(){
    
    using namespace exploringBB;
   PWM pwm_m1("pwm_test_P9_42.15");  // example alternative pin
   double duty; 
 struct timeval t1, t2;
    int elapsedTime=0,t=0,e2=0,prev=0,de,dt;
int xdes,xact,e;
float kp=0.006,kd=1;

   float te,derr;      
 gettimeofday(&t1, NULL);
 
 FILE *fp = fopen(("/sys/devices/ocp.3/48304000.epwmss/48304180.eqep/position"), "w");
     /* if(fp == NULL)
    {
        // Error, break out
        //std::cerr << "[eQEP " << this->path << "] Unable to open position for write" << std::endl;
        return;
    }*/

    // Write the desired value to the file
    fprintf(fp, "%d\n", 0);
    
    // Commit changes
    fclose(fp); 
    
 while(t<50000)

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
    te=kp*e+kd*(derr);
    cout<<te<<"\t\t";
    cout << elapsedTime << " ms.\t\t";
    t=elapsedTime; 
    e2=e;
    prev=elapsedTime;
    cout<<"e ="<<e<<"\t" ;
    //ofstream outputFile;
    //outputFile.open("xact.txt");
    //outputFile<<xact<<"\t \t"<<elapsedTime<<"\n";
    //outputFile.close();
    

  
  if(te>50.0)
   {duty=1.0;
   }
   else if(te>0.0 && te<50.0)
   {duty=99.0-(te+50.0);
      cout<<"hello"; 
   }
   else if(te==50.0)
   {duty=99.0-(te+50.0);
       
   }
   else if(te<0.0 && te>-50.0 )
    {  cout<<"hey";
        duty=99.0-(te+50.0);}
   else if(te==-50.0)
    { 
        duty=(te+50.0);}
    else
    {cout<<"i m here";
        duty=99.0;}
    
   
   
  
   pwm_m1.setFrequency(400);         // Set the period in ns
   pwm_m1.setDutyCycle(duty*1.0f);       // Set the duty cycle as a percentage
   pwm_m1.setPolarity(PWM::ACTIVE_LOW);  // using active low PWM
   pwm_m1.run();  
      //xdes-=50; 
     
  }
 
 
   }     // start the PWM output
