#pragma once

class rto {
	int d;

	//to prevent division by 0, we forbid direct access to the denominator
	rto(int num, int den);

public:
	int n;

	rto();
	rto(int num);
	

	rto(rto &&target);
	rto(rto const &target);

	rto & operator=(int const & target);
	rto & operator=(rto const & target);
	
	rto operator-() const;
	rto operator+(int factor) const;
	rto operator+(const rto &add) const;
	rto operator-(int factor) const;
	rto operator-(const rto &sub) const;
	rto operator*(int factor) const;
	rto operator*(const rto &target) const;
	rto operator/(int factor) const;
	rto operator/(const rto &target) const;

	rto& operator+=(int factor);
	rto& operator+=(const rto &target);
	rto& operator-=(int factor);
	rto& operator-=(const rto &target);
	rto& operator*=(int factor);
	rto& operator*=(const rto &target);
	rto& operator/=(int factor);
	rto& operator/=(const rto &target);

	bool operator==(const int &test) const;
	bool operator==(const rto &test) const;
	bool operator!=(const int &test) const;
	bool operator!=(const rto &test) const;
	bool operator>(const int &test) const;
	bool operator>(const rto &test) const;
	bool operator<(const int &test) const;
	bool operator<(const rto &test) const;
	bool operator>=(const int &test) const;
	bool operator>=(const rto &test) const;
	bool operator<=(const int &test) const;
	bool operator<=(const rto &test) const;

	float toFloat() const;
};