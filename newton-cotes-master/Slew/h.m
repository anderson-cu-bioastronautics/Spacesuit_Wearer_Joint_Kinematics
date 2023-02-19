function [ xkp1 ] = h( x, u, dt )
%h Nonlinear discrete-time motion model for satellite attitude toy
%problem
%   Detailed explanation goes here
phi = norm(u)*dt;
a = u/norm(u);
dq = [a*sin(phi/2); cos(phi/2)];

xkp1 = (dq(4)*eye(4) + [-crossm(dq(1:3)) dq(1:3); -dq(1:3)' 0])*x;

end

