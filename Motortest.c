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
using namespace std;

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

int PWM::setDutyCycle(float percentage){
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

int adc(){
	const char *fname = "/sys/bus/iio/devices/iio:device0/in_voltage2_raw";
        int count = 0, fd, len,i=0,j=0,h=0;
        int val; int array[10];
        char adc[5] = {0};
        
         while(count < 10){ val=0;
                fd = open(fname, O_RDONLY);
                if(fd == -1){ 
                        cout << "error:";
                        exit(1);
	                }
 
                if(count % 10 == 0 && count != 0)
                        cout << "count % 10\n";
 
	                len = read(fd, adc, sizeof(adc - 1));
 
	                if(len == -1){
	                        close(fd);
	                        continue;               
	                }
	                else if(len == 0){
	                        cout << "buffer is empty";
	                }
	                else{   
	                        adc[len] ='\0';
	                        
	                       for(i=0;i<len;i++)
	                       {for(j=0;j<i;j++)
	                        val+=(adc[i]<<3)+(adc[i]<<1);
	                       } 
	                    array[h]=val;
	                    h++;
	                } 
 
	                close(fd);
 
	                count++;
	        }
	        
    int n, c, d, swap;
 
  n=10;
 
  for (c = 0 ; c < ( n - 1 ); c++)
  {
    for (d = 0 ; d < n - c - 1; d++)
    {
      if (array[d] > array[d+1]) /* For decreasing order use < */
      {
        swap       = array[d];
        array[d]   = array[d+1];
        array[d+1] = swap;
      }
    }
  }

   return(array[5]);
 
}  

int main(){
    
    using namespace exploringBB;
   PWM pwm_m1("pwm_test_P9_42.16");  // example alternative pin
  int min; float d;
 int voltage; 
 float mean; 
 min=adc();
 count=5;
  //cout<<min;
 voltage=(min-3094);
 while(voltage>0 && count!=0) 
 {mean=(voltage*1.8)/4095;
   
 d=(mean*200)/3.3;
 cout<<"\n"<<mean<<"\n"<<d;


   pwm_m1.setPeriod(5000);         // Set the period in ns
   pwm_m1.setDutyCycle(d*1.0f);       // Set the duty cycle as a percentage
   pwm_m1.setPolarity(PWM::ACTIVE_LOW);  // using active low PWM
   pwm_m1.run();
       count--;
   }    
  //if(voltage<=0)
  {pwm_m1.stop();}
 
   }     // start the PWM output
