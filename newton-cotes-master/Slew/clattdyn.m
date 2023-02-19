function [ xd ] = clattdyn( t, x, kp, kd, Iinv, I, qc )
%CLATTDYN Closed-loop spacecraft attitude dynamics with PD control
%   Detailed explanation goes here
% Unpack state
q = x(1:4)/norm(x(1:4));
% q = x(1:4);
om = x(5:7);

% Evaluate commanded quaternion
qc = qc(t);

% Compute error quaternion
qe = (q(4)*eye(4) + [-crossm(q(1:3)) q(1:3); -q(1:3)' 0])*[-qc(1:3); qc(4)];

% Compute control torques
if qe(4) == 0
    u = -kp*qe(1:3) - kd*om;
else
    u = -kp*sign(qe(4))*qe(1:3) - kd*om;
end

% Compute angular velocity rate and quaternion rate
omd = Iinv*(-crossm(om)*I*om + u);                 % Assume I invertible
qd = 0.5*[-crossm(om) om; -om' 0]*q;

% Pack output
xd = [qd; omd];

end

