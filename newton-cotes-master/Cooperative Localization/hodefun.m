function [ xdot ] = hodefun( t, x, u, L )
%hcoop Continuous-time nonlinear motion model for cooperative localization 
%problem
%   Detailed explanation goes here

% Compute xdot
xdot = h(x, u(t), L);

end

