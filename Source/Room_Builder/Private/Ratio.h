#pragma once

typedef signed long long int64;

class rto {
	int64 d;

	//to prevent division by 0, we forbid direct access to the denominator
	rto(int64 num, int64 den);

public:
	int64 n;

	rto();
	rto(int64 num);
	

	rto(rto &&target);
	rto(rto const &target);

	rto & operator=(int64 const & target);
	rto & operator=(rto const & target);
	
	rto operator-() const;
	rto operator+(int64 factor) const;
	rto operator+(const rto &add) const;
	rto operator-(int64 factor) const;
	rto operator-(const rto &sub) const;
	rto operator*(int64 factor) const;
	rto operator*(const rto &target) const;
	rto operator/(int64 factor) const;
	rto operator/(const rto &target) const;

	rto& operator+=(int64 factor);
	rto& operator+=(const rto &target);
	rto& operator-=(int64 factor);
	rto& operator-=(const rto &target);
	rto& operator*=(int64 factor);
	rto& operator*=(const rto &target);
	rto& operator/=(int64 factor);
	rto& operator/=(const rto &target);

	bool operator==(const int64 &test) const;
	bool operator==(const rto &test) const;
	bool operator!=(const int64 &test) const;
	bool operator!=(const rto &test) const;
	bool operator>(const int64 &test) const;
	bool operator>(const rto &test) const;
	bool operator<(const int64 &test) const;
	bool operator<(const rto &test) const;
	bool operator>=(const int64 &test) const;
	bool operator>=(const rto &test) const;
	bool operator<=(const int64 &test) const;
	bool operator<=(const rto &test) const;

	float toFloat() const;
};