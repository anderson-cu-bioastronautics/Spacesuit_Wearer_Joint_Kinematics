function [ Gxk ] = getGxk( q, Bi )
%getGxk Get linearized discrete-time observation matrix
%   Detailed explanation goes here
Gxk = [-4*Bi*q(1:3)' + 2*(q(1:3)*Bi' + q(1:3)'*Bi*eye(3)) + 2*q(4)*crossm(Bi) -2*crossm(q(1:3))*Bi];

end

