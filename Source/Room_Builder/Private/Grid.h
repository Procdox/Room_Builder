#pragma once

#define grid_epsilon .000001

class grd {
	

public:
	double n;

	grd();
	grd(double num);

	grd(grd &&target);
	grd(grd const &target);

	grd & operator=(double const & target);
	grd & operator=(grd const & target);
	
	grd operator-() const;
	grd operator+(double factor) const;
	grd operator+(const grd &add) const;
	grd operator-(double factor) const;
	grd operator-(const grd &sub) const;
	grd operator*(double factor) const;
	grd operator*(const grd &target) const;
	grd operator/(double factor) const;
	grd operator/(const grd &target) const;

	grd& operator+=(double factor);
	grd& operator+=(const grd &target);
	grd& operator-=(double factor);
	grd& operator-=(const grd &target);
	grd& operator*=(double factor);
	grd& operator*=(const grd &target);
	grd& operator/=(double factor);
	grd& operator/=(const grd &target);

	bool operator==(const double &test) const;
	bool operator==(const grd &test) const;
	bool operator!=(const double &test) const;
	bool operator!=(const grd &test) const;
	bool operator>(const double &test) const;
	bool operator>(const grd &test) const;
	bool operator<(const double &test) const;
	bool operator<(const grd &test) const;
	bool operator>=(const double &test) const;
	bool operator>=(const grd &test) const;
	bool operator<=(const double &test) const;
	bool operator<=(const grd &test) const;
};