#include "PID.hpp"


PID::PID(float _p, float _i, float _d, uint16_t _rate, uint32_t _out_limit) :
	p(_p),
	i(_i),
	d(_d),
	rate(_rate),
	out_lower_limit(-_out_limit),
	out_upper_limit(_out_limit)
{
}


PID::PID(float _p, float _i, float _d, uint16_t _rate, int32_t _out_lower_limit, int32_t _out_upper_limit) :
	p(_p),
	i(_i),
	d(_d),
	rate(_rate),
	out_lower_limit(_out_lower_limit),
	out_upper_limit(_out_upper_limit)
{
}


int32_t PID::update(int32_t target, int32_t in)
{
	error = target - in;

	if(((error > 0) && (out < out_upper_limit)) || ((error < 0) && (out > out_lower_limit))) error_integral += error;

	int32_t error_diff = error - error_last;
	error_last = error;

	out = error * p + error_integral * (i / rate) + error_diff * d * rate;

	if(out > out_upper_limit) out = out_upper_limit;
	if(out < out_lower_limit) out = out_lower_limit;

	return out;
}


void PID::reset()
{
	error_integral = 0;
	out = 0;
}

void PID::setP(float _p)
{
	this->p = _p;
}

void PID::setI(float _i)
{
	this->i = _i;
}

void PID::setD(float _d)
{
	this->d = _d;
}

float PID::getP() const
{
	return this->p;
}

float PID::getI() const
{
	return this->i;
}

float PID::getD() const
{
	return this->d;
}