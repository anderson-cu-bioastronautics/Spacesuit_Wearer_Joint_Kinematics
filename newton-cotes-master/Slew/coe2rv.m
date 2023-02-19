function [ ri, vi ] = coe2rv( a, e, i, Om, om, nu, mu )
%COE2RV Given classical orbital elements in primary-centered inertial 
%frame, computes position and velocity vectors in primary-centered inertial
%frame.
%   Detailed explanation goes here

p = a*(1 - e^2);
r = p/(1 + e*cosd(nu));

ro = [r*cosd(nu); r*sind(nu); 0];
vo = [-sqrt(mu/p)*sind(nu); sqrt(mu/p)*(e + cosd(nu)); 0];

Com = [cosd(-om) sind(-om) 0; -sind(-om) cosd(-om) 0; 0 0 1];
Ci = [1 0 0; 0 cosd(-i) sind(-i); 0 -sind(-i) cosd(-i)];
COm = [cosd(-Om) sind(-Om) 0; -sind(-Om) cosd(-Om) 0; 0 0 1];

ri = COm*Ci*Com*ro;
vi = COm*Ci*Com*vo;

end

