#pragma once

typedef signed long long int64;

#define grid_size 65536
#define grid_epsilon 256

class grd {
	int64 n;
	enum unchecked_t { unchecked };

	grd(int64 num, unchecked_t);
public:

	grd();
	grd(int64 num);

	grd(grd &&target);
	grd(grd const &target);

	grd & operator=(int64 const & target);
	grd & operator=(grd const & target);
	
	grd operator-() const;
	grd operator+(int64 factor) const;
	grd operator+(const grd &add) const;
	grd operator-(int64 factor) const;
	grd operator-(const grd &sub) const;
	grd operator*(int64 factor) const;
	grd operator*(const grd &target) const;
	grd operator/(int64 factor) const;
	grd operator/(const grd &target) const;

	grd& operator+=(int64 factor);
	grd& operator+=(const grd &target);
	grd& operator-=(int64 factor);
	grd& operator-=(const grd &target);
	grd& operator*=(int64 factor);
	grd& operator*=(const grd &target);
	grd& operator/=(int64 factor);
	grd& operator/=(const grd &target);

	bool operator==(const int64 &test) const;
	bool operator==(const grd &test) const;
	bool operator!=(const int64 &test) const;
	bool operator!=(const grd &test) const;
	bool operator>(const int64 &test) const;
	bool operator>(const grd &test) const;
	bool operator<(const int64 &test) const;
	bool operator<(const grd &test) const;
	bool operator>=(const int64 &test) const;
	bool operator>=(const grd &test) const;
	bool operator<=(const int64 &test) const;
	bool operator<=(const grd &test) const;

	grd round() const;

	float toFloat() const;
};