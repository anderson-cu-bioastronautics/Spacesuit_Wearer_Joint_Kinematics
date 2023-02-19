function [ H ] = Hxcoop( x, u )
%Hx Linearized continuous-time system dynamics matrix
%   Detailed explanation goes here

% Unpack state and input
thg = x(3);
tha = x(6);
vg = u(1);
va = u(3);

% Compute Hx
H = [zeros(6, 2) [-vg*sin(thg); vg*cos(thg); zeros(4, 1)] zeros(6, 2) [zeros(3, 1); -vg*sin(tha); va*cos(tha); 0]];

end

