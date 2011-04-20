/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#include <stdio.h>
#include "spherical_harmonics.h"
#include "data_WMM.h"


vec3 spherical_harmonic_expansion_mag(const struct spherical_harmonic_coefficient coef[][MAX_DEGREE + 1], const geodetic coord, const int degree);

vec3 magnetic_field(geodetic position)
{
    vec3 mag_field;
    double xp, zp;
    //convert to spherical
    double phi = position.latitude;
    double A = 6378137;
    double f = 1/298.257223563;
    double e2 = f*(2-f);
    double Rc = A/sqrt(1-e2*sin(phi)*sin(phi));
    double p = (Rc + position.altitude)*cos(phi);
    double z = (Rc*(1-e2) + position.altitude)*sin(phi);
    double r = sqrt(p*p + z*z);
    double phi_prime = asin(z/r);
    position.latitude = phi_prime;
    position.altitude = r;

    //make magfield static, and only update when there is a certain change in position/time?
    
    mag_field = spherical_harmonic_expansion_mag(magnetic_coefficients, position, MAX_DEGREE); //goes upt to 404
    //convert magfield into geodetic NED
    xp = mag_field.x*cos(phi_prime-phi)-mag_field.z*sin(phi_prime-phi);
    zp = mag_field.x*sin(phi_prime-phi)+mag_field.z*cos(phi_prime-phi);
    mag_field.x = xp;
    mag_field.z = zp;
    //convert to LTP
    //mag_field.z *= -1;
    return mag_field;
}

/*The spherical harmonic formula is:

    N             n    _                   _                  _
  sigma (a/r)^n sigma (g{n,m}*cos(m*lon) + h{n,m}*sin(m*lon))*P{n,m}(sin(lat))
  n = 0         m = 0

Where:
	N is the maximum degree of the expansion
	n and m are iterators for the degree and order respectivly
	a is the model's refference radius for the earth
	r is the required radius from the center of the earth
	lon and lat are longitude and latitude in a spherical earth centered refference frame
	g{n,m} and h{n,m} are the given model coefficients of degree n and order m. The overbar
		signifies that they are Schmidt semi-normalized. See below.
	P{n,m}(x) is the Associated Legendre Function (of the first type) of degree n and order m:
			((1-x^2)^(m/2))/(n!*2^n) * (d^(n+m)(x^2-1)^n)/dx^(n+m)
		This equation does not include the Codon-Shortly phase (a factor of (-1)^m) as is used in
		quantum physics applications. Geophysics does not use the Codon-Shortly phase.
	_	As with the coefficients, the overbar signifies Schmit semi-normalizaiton. Again, see below.
	 (overbar indicating Schmidt Semi-normalization) is the normalization factor:
			Sqrt((2-KroneckerDelta(0,m)) * (n-m)!/(n+m)!)
		Alternatively it can be expressed:
			{1,                     if m == 0
			{Sqrt(2*(n-m)!/(n+m)!), if m != 0
		The coefficeints already come normalized so nothing needs to be done to them, but the Associated
		Legendre Funcions do need to be multiplied by the normalization factor.
*/
/*
rewrite so that it:
1.doesn't use as many sqrt (I think cant be written without them, without extensive modification to SHE formula)
2.handles +-PI/2 latitude gracefully
3.sequential array access?
*/
//apply to multiple applications. Do it by renormalizing leg ftns, or coeffs? I could add a flag that swaps between
//full and schmidt semi, but I don't like flags. I could pass a normalizing function in?
vec3 spherical_harmonic_expansion_mag(const struct spherical_harmonic_coefficient coef[][MAX_DEGREE + 1], const geodetic coord, const int degree)
{//should fail at the poles (latitude = +-90 degree) But I've yet to get it to.
    int n, m = 0;
    double x, y, z;
    const double cos_lon  = cos(coord.longitude);//get sin from cos? pull out? not here,
    const double sin_lon  = sin(coord.longitude);//even when hard code vals is same speed. why?
    const double cos_lat  = cos(coord.latitude);
    const double sin_lat  = sin(coord.latitude);
    const double a_over_r =  6371200/coord.altitude; //thats radius in m
    double aoverr_const   = (a_over_r)*(a_over_r);//pow(a/radius_km,2); //find proper name in MIT pdf.
    double leg, leg_n1 = sqrt(2), leg_m1 = 0, leg_m2;
    double cos_lon_arr[degree];
    double sin_lon_arr[degree];
    double sqrt_val1;
    vec3 vector   = {0,0,coef[0][0].g};//degree 0
    cos_lon_arr[0] = 1; //cos(0)
    sin_lon_arr[0] = 0; //sin(0)

    for(n = 1; n <= degree; ++n)
    {
           
        x = y = z = 0;
        cos_lon_arr[n] = cos_lon_arr[n-1]*cos_lon - sin_lon_arr[n-1]*sin_lon;
        sin_lon_arr[n] = cos_lon_arr[n-1]*sin_lon + sin_lon_arr[n-1]*cos_lon;
        /*m==n*/
        sqrt_val1 = sqrt((2*n-1)/(2.0*n));
        leg = cos_lat*leg_n1*sqrt_val1;
        x  +=   (coef[n][n].g*cos_lon_arr[n] + coef[n][n].h*sin_lon_arr[n]) * -leg_n1*sin_lat*sqrt_val1*n;
        y  += n*(coef[n][n].g*sin_lon_arr[n] - coef[n][n].h*cos_lon_arr[n]) * leg;
        z  +=   (coef[n][n].g*cos_lon_arr[n] + coef[n][n].h*sin_lon_arr[n]) * leg;
        leg_n1 = leg;
        /*end m==n*/
        for(m = n-1; m > 0; --m)
        {
            
            leg_m2 = leg_m1;
            leg_m1 = leg;
            leg = (sin_lat/cos_lat*(2*m+2)*leg_m1 - sqrt((n-m-1)*(n+m+2))*leg_m2) / sqrt((n-m)*(n+m+1));        
            x  +=   (coef[n][m].g*cos_lon_arr[m] + coef[n][m].h*sin_lon_arr[m]) * (leg_m1*sqrt((n-m)*(n+m+1)) - m*sin_lat/cos_lat*leg);
            y  += m*(coef[n][m].g*sin_lon_arr[m] - coef[n][m].h*cos_lon_arr[m]) * leg;
            //printf("Z: %e, g: %e, leg: %e\n", z, coef[n][m].g, leg);
            z  +=   (coef[n][m].g*cos_lon_arr[m] + coef[n][m].h*sin_lon_arr[m]) * leg;
            //printf("%d\n",m);
            

        }
        /*m == 0*/
        
        sqrt_val1 = sqrt(n*(n+1)*2);
        leg_m2 = leg_m1;
        leg_m1 = leg;
        leg = (sin_lat/cos_lat*2*leg_m1 - sqrt((n-1)*(n+2))*leg_m2) / sqrt_val1;
        x  += coef[n][m].g * leg_m1*sqrt_val1/2;
        z  += coef[n][m].g * leg;
        /*end m == 0*/
        aoverr_const  *= a_over_r;
        vector.x  += aoverr_const * x;
        vector.y  += aoverr_const * y;
        vector.z  += aoverr_const * z * (n+1);
    }
    vector.x  *= -1;
    vector.y  /=  cos(coord.latitude);
    vector.z  *= -1;
    return vector;
}



/*
vec3 spherical_harmonic_epansion_grav(const struct spherical_harmonic_coefficient **coef, point3 coord, int degree)
{//Yes, this is ugly.
    //start_counter();
    if(coord.latitude == 0)
        coord.latitude = 1e-13; //get better way of fixing.
    int n, m;
    double theta, phi, radius; //x, y, z
    double cos_m_lon, cos_n_lon, sin_m_lon, sin_n_lon, cos_temp;
    double cos_lon = cos(coord.longitude);
    double sin_lon = sin(coord.longitude);
    double cos_lat = cos(coord.latitude);
    double sin_lat = sin(coord.latitude);
    double a_over_r =  6378137/coord.radius; //thats radius in m
    double aoverr_const = 1;//(a_over_r);//\*(a_over_r);//pow(a/radius_km,2); //find proper name in MIT pdf.
    double leg, leg_n1, leg_m1 = 0, leg_m2;
    vec3 vector   = {0,0,0};
    vector.radius = coef[0][0].g; //degree 0
    cos_n_lon = 1; //cos(0)
    sin_n_lon = 0; //sin(0)
    leg_n1 = sqrt(2);
    //double timer = get_counter();
    for(n = 1; n <= degree; ++n)//maybe change to foward row for the trig?
    {
        theta  = 0;
        phi    = 0;
        radius = 0;
        /\*m==n*\/
        cos_temp  = cos_n_lon;
        cos_n_lon = cos_n_lon*cos_lon - sin_n_lon*sin_lon;
        sin_n_lon = cos_temp *sin_lon + sin_n_lon*cos_lon;
        cos_m_lon = cos_n_lon;
        sin_m_lon = sin_n_lon;
        leg = cos_lat*leg_n1*sqrt((2*n+1)/(2.0*n));
        theta  +=   (coef[n][n].g*cos_n_lon + coef[n][n].h*sin_n_lon) * -leg_n1*sin_lat*sqrt(((2*n+1)*n)/2.0);//is calc'd to be * -1, but doesn't work. figure out why. has something to do with below?
        phi    += n*(coef[n][n].g*sin_n_lon - coef[n][n].h*cos_n_lon) * leg;
        radius +=   (coef[n][n].g*cos_n_lon + coef[n][n].h*sin_n_lon) * leg;
        leg_n1 = leg;
        /\*end m==n*\/
        for(m = n-1; m > 0; --m)
        {
            cos_temp  = cos_m_lon;
            cos_m_lon = cos_m_lon*cos_lon + sin_m_lon*sin_lon;
            sin_m_lon = sin_m_lon*cos_lon - cos_temp *sin_lon;
            leg_m2 = leg_m1;
            leg_m1 = leg;
            leg    = (sin_lat/cos_lat*(2*m+2)*leg_m1 - sqrt((n-m-1)*(n+m+2))*leg_m2) / sqrt((n-m)*(n+m+1));
            theta  +=   (coef[n][m].g*cos_m_lon + coef[n][m].h*sin_m_lon) * (m*sin_lat/cos_lat*leg - leg_m1*sqrt((n-m)*(n+m+1)));
            phi    += m*(coef[n][m].g*sin_m_lon - coef[n][m].h*cos_m_lon) * leg;
            radius +=   (coef[n][m].g*cos_m_lon + coef[n][m].h*sin_m_lon) * leg;
        }
        /\*m == 0*\/
        leg_m2 = leg_m1;
        leg_m1 = leg;
        leg = (sin_lat/cos_lat*2*leg_m1 - sqrt((n-1)*(n+2))*leg_m2) / sqrt(n*(n+1)*2);
        theta  += coef[n][0].g * -leg_m1*sqrt(n*(n+1)/2.0);
        radius += coef[n][0].g * leg;
        /\*end m == 0*\/
        aoverr_const  *= a_over_r;// pow(a/radius_km, n+2);
        vector.theta  += aoverr_const * theta;
        vector.phi    += aoverr_const * phi;
        vector.radius += aoverr_const * radius * (n+1);
    }
    double GM_r2 = 1;//3986004.418e08/((coord.radius*coord.radius));
    //printf("%e\n",GM_r2);
    vector.theta  *=  GM_r2;
    vector.phi    *=  GM_r2/cos(coord.latitude);
    vector.radius *= -GM_r2;
    //printf("Cycles: %.0f\n", timer);
    return vector;
}
*/
