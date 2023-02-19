function [ Hwk ] = getHwk( q, om, T )
%getHwk2 Get linearized discrete-time input matrix
%   Detailed explanation goes here

if norm(om) ~= 0
    ddqdom = [1/norm(om)*sin(norm(om)*T/2)*eye(3) + (T/(2*norm(om)^2)*cos(norm(om)*T/2) - 1/norm(om)^3*sin(norm(om)*T/2))*(om*om');
        -sin(norm(om)*T/2)*T/(2*norm(om))*om'];
    dhdom = (q(4)*eye(4) + [crossm(q(1:3)) q(1:3); -q(1:3)' 0])*ddqdom;
else
    dhdom = [crossm(q(1:3)) + q(4)*eye(3); -q(1:3)']*T/2;
end

Hwk = dhdom;

end

