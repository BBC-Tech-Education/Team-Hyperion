#include "Vect.h"

Vect::Vect() {
	setStandard(0, 0);
}

Vect::Vect(float val1, float val2, bool polar){
	polar ? setPolar(val1, val2) : setStandard(val1, val2);
}

void Vect::setStandard(float _i, float _j){
	i = _i;
	j = _j;

	mag = calcMag(_i, _j);
	arg = calcArg(_i, _j);
	if(mag < 0) {
		arg = float_mod(arg + 180, 360);
		mag = mag * -1;
	}
}

void Vect::setPolar(float _mag, float _arg){
	i = calcI(_mag, _arg);
	j = calcJ(_mag, _arg);

	mag = _mag;
	arg = float_mod(_arg, 360);
}

Vect Vect::operator+(Vect vector2){
	return Vect(i + vector2.i, j + vector2.j, false);
}

Vect Vect::operator-(Vect vector2){
	return Vect(i - vector2.i, j - vector2.j, false);
}

Vect Vect::operator*(float scalar){
	return Vect(mag * scalar, arg);
}

Vect Vect::operator/(float scalar){
	return Vect(mag / scalar, arg);
}

void Vect::operator+=(Vect vector2){
	setStandard(i + vector2.i, j + vector2.j);
}

void Vect::operator-=(Vect vector2){
	setStandard(i - vector2.i, j - vector2.j);
}

Vect Vect::operator*=(float scalar){
	return Vect(mag * scalar, arg);
}

Vect Vect::operator/=(float scalar){
	return Vect(mag / scalar, arg);
}

bool Vect::operator==(Vect vector2){
	return mag == vector2.mag;
}

bool Vect::operator!=(Vect vector2){
	return mag != vector2.mag;
}

bool Vect::operator<(Vect vector2){
	return mag < vector2.mag;
}

bool Vect::operator<=(Vect vector2){
	return mag <= vector2.mag;
}

bool Vect::operator>(Vect vector2){
	return mag > vector2.mag;
}

bool Vect::operator>=(Vect vector2){
	return mag >= vector2.mag;
}

bool Vect::exists(){
	return mag != 0;
}

bool Vect::isBetween(float leftAngle, float rightAngle){
	return angleIsInside(leftAngle, rightAngle, arg);
}

float Vect::calcI(float _mag, float _arg){
	return _mag * cosf(DEG_TO_RAD * _arg);
}

float Vect::calcJ(float _mag, float _arg){
	return _mag * sinf(DEG_TO_RAD * _arg);
}

float Vect::calcMag(float _i, float _j){
	return sqrtf(_i * _i + _j * _j);
}

float Vect::calcArg(float _i, float _j){
	return float_mod(RAD_TO_DEG * atan2f(_j, _i), 360.0);
}

Vect Vect::to_bearing() const {
    if (mag != 0) {
        float bearing_arg = float_mod(270.0f - arg, 360.0f);
        return Vect(mag, bearing_arg, true);
    }
    return *this;
}