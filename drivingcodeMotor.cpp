/*
 * PIDMotor.cpp
 *
 *  Created on: Oct 16, 2018
 *      Author: hephaestus
 */

#include "PIDMotor.h"
#include <Arduino.h>
#include <PID_v1.h>
#include "lab3/RBEPID.h"


PIDMotor::PIDMotor() :
		myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT) {

	rbePid.setpid(0.1, 0.01, 0.07);
	rbePid.setgyropid(1.0, 0.01, 0.01);
	//Setpoint = 12000;
	maxValue = 90;
}

PIDMotor::~PIDMotor() {
	// TODO Auto-generated destructor stub
}
void PIDMotor::pidinit() {
	myPID.SetMode(MANUAL);
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(-PID_OUTPUT_COMPUTE_RANGE, PID_OUTPUT_COMPUTE_RANGE);
	myPID.SetTunings(Kp, Ki, Kd, P_ON_E);
	myPID.SetSampleTime(5);
}

void PIDMotor::loop() {
	Input = (float) getPosition();
	bool thisErrPositive = Input > 0;
	if (thisErrPositive != lastErrPositive) {
		// strobe mode to trigger zeroing of the Integral buffer
		// In case of passing zero clear the integral sum
		Output = 0;
		myPID.SetMode(MANUAL);
		myPID.SetMode(AUTOMATIC);
	}
	lastErrPositive = thisErrPositive;
	if (myPID.Compute()) {
		int out = map(Output, -PID_OUTPUT_COMPUTE_RANGE,
				PID_OUTPUT_COMPUTE_RANGE, getOutputMin(), getOutputMax());
		setOutput(out);
	}

}
void PIDMotor::overrideCurrentPosition(int64_t val) {
	overrideCurrentPositionHardware(val);

	setSetpoint(val);
	myPID.SetTunings(Kp, Ki, Kd, P_ON_E);
	// strobe mode to trigger zeroing of the Integral buffer
	Output = 0;
	myPID.SetMode(MANUAL);
	myPID.SetMode(AUTOMATIC);
}

void PIDMotor::setSetpoint(int64_t val) {
	Setpoint = (float) val;
	//Serial.println(Setpoint);
}

void PIDMotor::setTurningDegrees(int64_t val) {
	TurningSetpoint = (float) val;
	//Serial.println(TurningSetpoint);
}

float PIDMotor::getSetPoint(){
	return Setpoint;
}
void PIDMotor::SetTunings(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	overrideCurrentPosition(getPosition());
}

float PIDMotor::getTurnSetPoint(){
	return TurningSetpoint;
}
// Returns Vel in degress/second
double PIDMotor::calcVel(){
  //current positions
  double curPos=getPosition();
  //current time
  curTime=millis();
  //time change in ms from last call
  timeInterval=curTime-prevTime;
  //encoder ticks since last call
  movement=curPos-prevPos;
  //encoder ticks to degrees
  movement= movement * ticksToDeg;
  //timeInterval in seconds
  timeInterval=timeInterval/1000;
  //Velocity in degrees per milliseconds
  Vel=movement/timeInterval;
  //sets curent vals to previous
  prevPos=curPos;
  prevTime=curTime;
  return Vel;
}

void PIDMotor::velocityLoop() {
	int64_t time = esp_timer_get_time();
	if ((time/1000) - prevTime > 10) {
		double vel = this->calcVel();
		vel = vel / 6;
		Output = rbePid.calc(Setpoint, vel, maxValue);
		this->setOutput(Output);
		//this->setOutput(Output);
		//float current = this->calcCur();
		//float torque = table.torque(current, vel); // used for lab3

		//Serial.print("Current: ");
		//Serial.print(current);
		Serial.print("PWM duty cycle");
		Serial.println(Output);
		//Serial.print(" | RPM: ");
		//Serial.print(vel);
		//Serial.print(" | Torque: ");
		//Serial.println(torque);
		//*/

	}
}


void PIDMotor::positionLoop() {
	int64_t time = esp_timer_get_time();
	if ((time/1000) - prevTime > 10) {
		double curPos=getPosition();
		Output = this->rbePid.calc(Setpoint, curPos, maxValue);
		this->setOutput(90+Output);
	}
}

void PIDMotor::lookDown(int dir) {
	int64_t time = esp_timer_get_time();
		if ((time/1000) - prevTime > 10) {
			double curPos=getPosition();
			double P = 0.33;
			Output = P * (Setpoint - curPos);
			this->setOutput(90+(Output)*dir);

		}
}

void PIDMotor::positionLoopOffset(double offset) {
	int64_t time = esp_timer_get_time();
		if ((time/1000) - prevTime > 10) {
			double curPos=getPosition();
			Output = this->rbePid.calc(Setpoint, curPos, maxValue);
			this->setOutput(90+Output+offset);
			//Serial.print(" PWM :");
			//Serial.println(90+Output);

		}
}

void PIDMotor::turnLoopGyro(float heading, int dirFlag) {
	//dirFlag is set to -1 if turning Left as the gyro thinks right rotation is positive
	int64_t time = esp_timer_get_time();
	if ((time/1000) - prevTime > 10) {
		Output = this->rbePid.calcGyro(TurningSetpoint, heading);
		this->setOutput(90+(Output*dirFlag));
	}
}

float PIDMotor::calcCur() {
	float inp = analogRead(33);
	return ((inp)/525 * 1000); //525 mV per Amp, * 1000 mA / Amp
}

void PIDMotor::resetBounds() {
	maxValue = 90;
}
