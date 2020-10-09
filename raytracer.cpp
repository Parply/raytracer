/*** header ***/

#include <stdio.h>
#include <math.h>
#include <gd.h>

/*** maths ***/

inline double dmin(double a,double b) { return a<b ? a:b; }

struct XYZ
{
	double d[3];
	inline void Set(double a,double b, double c) { d[0]=a; d[1]=b; d[2]=c; }
	#define do_op(o) \
	inline void operator o##= (const XYZ&b) { for(unsigned n=0;n<3;++n) d[n] o##= b.d[n]; } \
	inline void operator o##= (double b) { for(unsigned n=0;n<3;++n) d[n] o##= b; } \
	XYZ operator o (const XYZ &b) const { XYZ tmp(*this); tmp o##= b; return tmp; }	\
	XYZ operator o (double b) const { XYZ tmp(*this); tmp o##= b; return tmp; }
	do_op(*)
	do_op(+)
	do_op(-)
	#undef do_op
	XYZ operator- () const { XYZ tmp={{-d[0],-d[1],-d[2]}}; return tmp; }
	XYZ Pow(double b) const {XYZ tmp { {pow(d[0],b),pow(d[1],b),pow(d[2],b)}}; return tmp; }


}


/*** main ***/

int main(){
	return 0;
}
