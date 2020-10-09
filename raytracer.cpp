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
	inline double Dot(const XYZ &b) const { return d[0]*b.d[0]+d[1]*b.d[1]+d[2]*b.d[2]; }
	inline double Squared() const { return Dot(*this); }
	inline double Len() const { return sqrt(Squared()); }
	inline void Normalise() { *this *= 1.0/Len(); }
	void MirrorAround(const XYZ &axis){
		XYZ N=axis;N.Normalise();
		double v=Dot(N);
		*this=N*(v+v)- *this;
	}
	// colour
	inline double Luma() const { return d[0]*0.299+d[1]*0.587+d[2]*0.114; }
	void Clamp()
	{
		for (unsigned n=0;n<3;++n)
		{
			if (d[n]<0.0) d[n]=0.0;
			else if (d[n]>1.0) d[n]=1.0;
		}
	}
	void ClampWithDestruction()
	{
		double l=Luma(),sat=1.0;
		if (l>1.0) {d[0]=d[1]=d[2]=1.0;return;}
		if (l<0.0) {d[0]=d[1]=d[2]=0.0;return;}
		for (int n=0;n<3;++n)
			if (d[n]>1.0) sat = dmin(sat,(l-1.0)/(1.0-d[n]));
			else if (d[n]<0.0) sat = dmin(sat,l/(1.0-d[n]));
		if (sat !=1.0)
		{ *this =(*this-1)*sat+l;Clamp(); }
	}
};


/*** main ***/

int main(){
	return 0;
}
