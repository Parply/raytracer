/*** header ***/

#include <cstdlib>
#include <stdio.h>
#include <math.h>
#include <gd.h>
#include <iostream>
#include <mpi.h>


/*** maths ***/

inline double dmin(double a,double b) { return a<b ? a:b; }
inline int imin(int a,int b) { return a<b ? a:b; }
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
	void ClampWithDesaturation()
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
	void Transform(XYZ &vec)
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
};

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
	{for (unsigned i=0; i<NumPlanes;++i)
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

/*** Colour ***/

const unsigned CandCount = 64;
const double Gamma = 2.2, Ungamma = 1.0 / Gamma;
unsigned Dither8x8[8][8];
XYZ Pal[16], PalG[16];
double luma[16];
void InitDither()
{
    // We will use the default 16-colour EGA/VGA palette.
 
    for(unsigned i=0; i<16; ++i)
    {
        static const char s[16*3] =
            {0,0,0, 0,0,42, 0,42,0, 0,42,42, 42,0,0, 42,0,42, 42,21,0, 21,21,21,
             42,42,42, 21,21,63, 21,63,21, 21,63,63, 63,21,21, 63,21,63, 63,63,21, 63,63,63};
        Pal[i].Set(s[i*3+0],s[i*3+1],s[i*3+2]);

        Pal[i] *= 1/63.0;
        PalG[i] = Pal[i].Pow(Gamma);
        luma[i] = Pal[i].Luma();
    }
    // Create bayer dithering matrix, adjusted for candidate count
    for(unsigned y=0; y<8; ++y)
        for(unsigned x=0; x<8; ++x)
        {
            unsigned i = x ^ y, j;
            j = (x & 4)/4u + (x & 2)*2u + (x & 1)*16u;
            i = (i & 4)/2u + (i & 2)*4u + (i & 1)*32u;
            Dither8x8[y][x] = (j+i)*CandCount/64u;
        }
}

/*** MPI helper functions ***/

void MPE_DECOMP1D(int n, int numprocs,
		int myid, unsigned *s,unsigned *e)
{
	unsigned nlocal = n/numprocs;
	unsigned st = myid*nlocal;
	unsigned deficit = n % numprocs;
	st += imin(myid,deficit);
	if (myid < deficit)
		nlocal++;
	unsigned et = st+nlocal;
	if (et>n || myid ==(numprocs-1))
		et=n;
	*e=et;
	*s=st;
	

}

/*** main ***/

int main(int argc, char *argv[])
{
    int id;
    int ierr;
    int p;
    unsigned start,end;
    ierr = MPI_Init (&argc, &argv);
 
    if ( ierr != 0 )
    {
        std::cout << "\n";
    	std::cout << "Fatal error!\n";
    	std::cout << "  MPI_Init returned nonzero ierr.\n";
	exit ( 1 );
    }
    ierr = MPI_Comm_size ( MPI_COMM_WORLD, &p );
    ierr = MPI_Comm_rank ( MPI_COMM_WORLD, &id );
    
    MPE_DECOMP1D(2048, p, id, &start, &end);
    
    InitDither();
    InitAreaLightVectors();
    XYZ camangle      = { {0,0,0} };
    XYZ camangledelta = { {-.005, -.011, -.017} };
    XYZ camlook       = { {0,0,0} };
    XYZ camlookdelta  = { {-.001, .005, .004} };

    double zoom = 46.0, zoomdelta = 0.99;
    double contrast = 32, contrast_offset = -0.17;

    const unsigned W = 680, H = 480;
    
    
    for(unsigned frameno=start; frameno<end; ++frameno)
    {
        fprintf(stderr, "Begins frame %u; contrast %g, contrast offset %g\n",
            frameno,contrast,contrast_offset);

        gdImagePtr im = gdImageCreate(W,H);
	
        for(unsigned p=0; p<16; ++p)
            gdImageColorAllocate(im, (int)(Pal[p].d[0]*255+0.5),
                                     (int)(Pal[p].d[1]*255+0.5),
                                     (int)(Pal[p].d[2]*255+0.5));
				     

        // Put camera between the central sphere and the walls
        XYZ campos = { { 0.0, 0.0, 16.0} };
        // Rotate it around the center
        Matrix camrotatematrix, camlookmatrix;
        camrotatematrix.InitRotate(camangle);
        camrotatematrix.Transform(campos);
        camlookmatrix.InitRotate(camlook);

        // Determine the contrast ratio for this frame's pixels
        double thisframe_min = 100;
        double thisframe_max = -100;

      #pragma omp parallel for collapse(2)
        for(unsigned y=0; y<H; ++y)
            for(unsigned x=0; x<W; ++x)
            {
                XYZ camray = { { x / double(W) - 0.5,
                                 y / double(H) - 0.5,
                                 zoom } };
                camray.d[0] *= 4.0/3; // Aspect ratio correction
                camray.Normalise();
                camlookmatrix.Transform(camray);
                XYZ campix;
                RayTrace(campix, campos, camray, MAXTRACE);
                campix *= 0.5;
              #pragma omp critical
              {
                // Update frame luminosity info for automatic contrast adjuster
                double lum = campix.Luma();
                #pragma omp flush(thisframe_min,thisframe_max)
                if(lum < thisframe_min) thisframe_min = lum;
                if(lum > thisframe_max) thisframe_max = lum;
                #pragma omp flush(thisframe_min,thisframe_max)
              }
                // Exaggerate the colours to bring contrast better forth
                campix = (campix + contrast_offset) * contrast;
                // Clamp, and compensate for display gamma (for dithering)
                campix.ClampWithDesaturation();
                XYZ campixG = campix.Pow(Gamma);
                XYZ qtryG = campixG;
                // Create candidate for dithering
                unsigned candlist[CandCount];
                for(unsigned i=0; i<CandCount; ++i)
                {
                    unsigned k = 0;
                    double b = 1e6;
                    // Find closest match from palette
                    for(unsigned j=0; j<16; ++j)
                    {
                        double a = (qtryG - PalG[j]).Squared();
                        if(a < b) { b = a; k = j; }
                    }
                    candlist[i] = k;
                    if(i+1 >= CandCount) break;
                    // Compensate for error
                    qtryG += (campixG - PalG[k]);
                    qtryG.Clamp();
                }
                // Order candidates by luminosity
                // using insertion sort.
                for(unsigned j=1; j<CandCount; ++j)
                {
                    unsigned k = candlist[j], i;
                    for(i=j; i>=1 && luma[candlist[i-1]] > luma[k]; --i)
                        candlist[i] = candlist[i-1];
                    candlist[i] = k;
                }
                unsigned colour = candlist[Dither8x8[x & 7][y & 7]];
		//int colour = gdTrueColor((int) campix.d[0]*255,(int) campix.d[1]*255, (int) campix.d[2]*255);
                gdImageSetPixel(im, x,y, colour);
            }

        char Buf[64]; sprintf(Buf, "trace%d.png", frameno);
        fprintf(stderr, "Writing %s...\n", Buf);
        FILE* fp = fopen(Buf, "wb");
        gdImagePng(im, fp);
        gdImageDestroy(im);
        fclose(fp);


        // Tweak coordinates / camera parameters for the next frame
        double much = 1.0;

        // In the beginning, do some camera action (play with zoom)
        if(zoom <= 1.1)
            zoom = 1.1;
        else
        {
            if(zoom > 40) { if(zoomdelta > 0.95) zoomdelta -= 0.001; }
            else if(zoom < 3) { if(zoomdelta < 0.99) zoomdelta += 0.001; }
            zoom *= zoomdelta;
            much = 1.1 / pow(zoom/1.1, 3);
        }

        // Update the rotation angle
        camlook  += camlookdelta * much;
        camangle += camangledelta * much;

        // Dynamically readjust the contrast based on the contents
        // of the last frame
        double middle = (thisframe_min + thisframe_max) * 0.5;
        double span   = (thisframe_max - thisframe_min);
        thisframe_min = middle - span*0.60; // Avoid dark tones
        thisframe_max = middle + span*0.37; // Emphasize bright tones
        double new_contrast_offset = -thisframe_min;
        double new_contrast        = 1 / (thisframe_max - thisframe_min);
        // Avoid too abrupt changes, though
        double l = 0.85;
        if(frameno == 0) l = 0.7;
        contrast_offset = (contrast_offset*l + new_contrast_offset*(1.0-l));
        contrast        = (contrast*l + new_contrast*(1.0-l));
    }
    MPI_Finalize();
}
