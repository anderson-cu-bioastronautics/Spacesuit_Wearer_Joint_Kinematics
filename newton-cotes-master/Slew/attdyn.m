function [ xd ] = attdyn( t, x, Iinv, I, u )
%ATTDYN Open-loop attitude dynamics
%   Detailed explanation goes here
% Unpack state
q = x(1:4)/norm(x(1:4));
% q = x(1:4);
om = x(5:7);

% Compute angular velocity rate and quaternion rate
omd = Iinv*(-crossm(om)*I*om + u);                 % Assume I invertible
qd = 0.5*[-crossm(om) om; -om' 0]*q;

% Pack output
xd = [qd; omd];

end

