#pragma once

#include <stdint.h>


class PID
{
	public:
		PID(float _p, float _i, float _d, uint16_t _rate, uint32_t _out_limit);
		PID(float _p, float _i, float _d, uint16_t _rate, int32_t _out_lower_limit, int32_t _out_upper_limit);

		int32_t update(int32_t target, int32_t in);

		void reset();

		void setP(float _p);
		void setI(float _i);
		void setD(float _d);

		float getP() const;
		float getI() const;
		float getD() const;

	private:
		float p;
		float i;
		float d;

		uint16_t rate;

		int32_t out_lower_limit;
		int32_t out_upper_limit;

		int64_t error = 0;
		int64_t error_integral = 0;
		int64_t error_last = 0;
		int32_t out = 0;
};
