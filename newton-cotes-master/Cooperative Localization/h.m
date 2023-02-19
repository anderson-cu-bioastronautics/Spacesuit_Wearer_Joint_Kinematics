function [ xdot ] = hcoop( x, u, L )
%hcoop Continuous-time nonlinear motion model for cooperative localization 
%problem
%   Detailed explanation goes here

% Unpack state
thg = x(3);
tha = x(6);

% Unpack input
vg = u(1);
phig = u(2);
va = u(3);
oma = u(4);

% Compute xdot
xdot = [vg*cos(thg);
    vg*sin(thg);
    vg/L*tan(phig);
    va*cos(tha);
    va*sin(tha);
    oma];

end

