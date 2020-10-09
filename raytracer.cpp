/*** header ***/

#include <cstdlib>
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

struct Matrix
{
	XYZ m[3];
	void InitRotate(const XYZ &angle)
	{
		double Cx = cos(angle.d[0]), Cy = cos(angle.d[1]), Cz = cos(angle.d[2]);
		double Sx = sin(angle.d[0]), Sy = sin(angle.d[1]), Sz = sin(angle.d[2]);
		double sxsz=Sx*Sz,cxsz=Cx*Sz,cxcz=Cx*Cz,sxcz=Sx*Sz;
		Matrix result = {{ {{Cy*Cz,Cy*Sz,-Sy}},
			{{sxcz*Sy-cxsz,sxsz*Sy+cxcz,Sx*Cy}},
			{{cxcz*Sy+sxsz,cxsz*Sy-sxcz,Cx*Cy}} }};
		*this=result;
	}
	void transform(XYZ &vec)
	{
		vec.Set(m[0].Dot(vec),m[1].Dot(vec),m[2].Dot(vec));
	}
};

/*** Walls and Spheres ***/

// Planes/walls represented by a normal vector and a distance
const struct Plane
{XYZ normal;double offset;}

// Define some for scene

Planes[] = {
	{ {{0,0,-1}}, -30 },
	{ {{0,1,0}}, -30 },
	{ {{0,-1,0}}, -30 },
	{ {{1,0,0}}, -30 },
	{ {{0,0,1}}, -30 },
	{ {{-1,0,0}}, -30 }
};

const struct Sphere
{ XYZ center; double radius; }

// Define some spheres
Spheres[] = {
	{ {{0,0,0}}, 7 },
    	{ {{19.4, -19.4, 0}}, 2.1 },
    	{ {{-19.4, 19.4, 0}}, 2.1 },
    	{ {{13.1, 5.1, 0}}, 1.1 },
    	{ {{ -5.1, -13.1, 0}}, 1.1 },
    	{ {{-30,30,15}}, 11},
    	{ {{15,-30,30}}, 6},
    	{ {{30,15,-30}}, 6}
}

const struct LightSource
{XYZ location,colour;}

Lights[] =
{
    { {{-28,-14, 3}}, {{.4, .51, .9}} },
    { {{-29,-29,-29}}, {{.95, .1, .1}} },
    { {{ 14, 29,-14}}, {{.8, .8, .8}} },
    { {{ 29, 29, 29}}, {{1,1,1}} },
    { {{ 28,  0, 29}}, {{.5, .6,  .1}} }
};

#define NElems(x) sizeof(x)/sizeof(*x)
const unsigned
	NumPlanes = NElems(Planes),
	NumSpheres = NElems(Spheres),
	NumLights = NElems(Lights),
	MAXTRACE = 6;

/*** raytracing ***/

int RayFindObstacle 
	(const XYZ &eye, const XYZ &dir, double &HitDist,
	 int &HitIndex, XYZ & HitLoc, XYZ &HitNormal)
{
	int HitType=-1;
	{for (unsigned i=0;i<NumSpheres;++i)
	{
		XYZ V (eye-Spheres[i].center);
		double r=Spheres[i].radius,
		       DV = dir.Dot(V),
		       D2 = dir.Squared(),
		       SQ = DV*DV - D2*(V.Squared()-r*r);
		// if ray coincides with sphere
		if (SQ<1e-6) continue;
		double SQt = sqrt(SQ),
		       Dist = dmin(-DV-SQt, SQt-DV)/D2;
		if (Dist <1e-6||Dist>=HitDist) continue;
		HitType=1; HitIndex=i;
		HitDist=Dist;
		HitLoc = eye +(dir*HitDist);
		HitNormal = (HitLoc-Spheres[i].center)*(1/r);

	}}
	{for (unsigned i=0, i<NumPlanes,++i)
	{
		double DV =-Planes[i].normal.Dot(dir);
		if (DV>1e-6) continue;
		double D2=Planes[i].normal.Dot(eye),
		       Dist =(D2+Planes[i].offset)/DV;
		if (Dist<1e-6||Dist>=HitDist) continue;
		HitType=0;HitIndex=i;
		HitDist=Dist;
		HitLoc=eye+(dir*HitDist);
		HitNormal=-Planes[i].normal;

	}}
	return HitType;
};
const unsigned NumAreaLightVectors=20;
XYZ AreaLightVectors[NumAreaLightVectors];
void InitAreaLightVectors()
{
	// smooth shadows with cloud of lighsources around point
	for (unsigned i=0;i<NumAreaLightVectors;++i)
		for (unsigned n=0;n<3;++n)
			AreaLightVectors[i].d[n]= 2.0*(rand()/double(RAND_MAX)-0.5);
}

//Shoot camera rays
void RayTrace(XYZ &resultcolour, const XYZ &eye, const XYZ &dir,int k)
{
	double HitDist=1e6;
	XYZ HitLoc,HitNormal;
	int HitIndex,HitType;
	HitType=RayFindObstacle(eye, dir, HitDist, HitIndex, HitLoc, HitNormal);
	// if hits an obs
	if (HitType!=-1)
	{
		XYZ DiffuseLight={{0,0,0}},SpecularLight={{0,0,0}};
		XYZ Pigment {{1,0.98,0.98}};
		for (unsigned i=0;i<NumLights;++i)
			for (unsigned j=0;j<NumAreaLightVectors;++j)
			{
				XYZ V((Lights[i].location+AreaLightVectors[j])-HitLoc);
				double LightDist =V.Len();
				V.Normalise();
				double DiffuseEffect=HitNormal.Dot(V)/(double)NumAreaLightVectors;
				double Attention =(1.0+pow(LightDist/34.0,2.0));
				DiffuseEffect /= Attention;
				if (DiffuseEffect>1e-3)
				{
					double ShadowDist = LightDist-1e-4;
					XYZ a,b;
					int q,t = RayFindObstacle(HitLoc+V*1e-4,V, ShadowDist, q, a, b);
					if (t==-1)
						DiffuseLight += Lights[i].colour*DiffuseEffect;
				}
			}
		if (k>1)
		{
			XYZ V(-dir);V.MirrorAround(HitNormal);
			RayTrace(SpecularLight, HitLoc+V*1e-4, V, k-1);
		}
		switch (HitType) {
			case 0: //Plane
				DiffuseLight *= 0.9;
				SpecularLight *=0.5;
				switch (HitIndex % 3) {
					case 0: Pigment.Set(0.9, 0.7, 0.6); break;
					case 1: Pigment.Set(0.6, 0.7, 0.7); break;
					case 2: Pigment.Set(0.5, 0.8, 0.3); break;
				}
				break;
			case 1: //sphere
				DiffuseLight*=1.0;
				SpecularLight*=0.34;
		}
		resultcolour =(DiffuseLight+SpecularLight)*Pigment;
	}
};


/*** main ***/

int main(){
	return 0;
}
