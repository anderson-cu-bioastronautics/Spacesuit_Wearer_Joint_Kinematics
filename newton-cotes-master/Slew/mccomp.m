%% Attitude Toy Problem Monte Carlo Performance Comparison
% Y. Shen
% 1 April 2019

%% Monte Carlo Parameters
tic;
N = 5000;                   % Number of Monte Carlo runs

% Preallocate Monte-Carlo results
eth_max_zoh = zeros(N, 1);
eth_max_simp = zeros(N, 1);

%% Generate Ground Truth and Measurements

% Define constants
Re = 6.378137e6;                            % Earth radius (m)
mu = 3.986004415e14;                        % Earth gravitational parameter (m^3/s^2)
B0 = 3.12E-5;                               % Magnetic field strength at equator (T)
ome = 7.292115E-5;                          % Angular velocity of Earth's rotation (rad/s)

% Define initial orbital parameters
% 600 km polar orbit
a = Re + 600000;                            % Semi-major axis (m)
e = 0;                                      % Eccentricity
i = 90;                                     % Inclination (deg)
Om = 0;                                     % Right ascension of the ascending node (deg)
om = 0;                                     % Argument of periapsis (deg)
nu = 90;                                    % Initial true anomaly (deg)

% Convert orbital parameters into position and velocity vectors
[r0, v0] = coe2rv(a, e, i, Om, om, nu, mu);

% Define simulation parameters
fs = 1;                                     % Sampling rate (Hz)
T = 2*pi()*sqrt(a^3/mu);                    % Orbital period (s)
t = 1/fs:1/fs:17*T/32;                          % Timestamps (s)
% slew_time_div = 128;                          % Fraction of orbit at which slew command is initiated
% slew_time_div = 8;                          % Fraction of orbit at which slew command is initiated
% slew_time_div = 4;                          % Fraction of orbit at which slew command is initiated
% slew_time_div = 8/3;                          % Fraction of orbit at which slew command is initiated
slew_time_div = 2;                          % Fraction of orbit at which slew command is initiated

% Compute position and velocity vectors over simulation period
twobody = @(t, x) [x(4:6); -mu/(norm(x(1:3))^3)*x(1:3)];
xorb0 = [r0; v0];
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[tout, xorb] = ode45(twobody, t, xorb0, options);
xorb = xorb';

% figure;
% plot3(xorb(1, :), xorb(2, :), xorb(3, :));
% xlabel('x (m)');
% ylabel('y (m)');
% zlabel('z (m)');
% axis equal

% Compute magnetic field vectors
Bi = zeros(3, length(t));                    % Magnetic field vectors (T)
for k = 1:length(t)
    Bi(:, k) = B0*Re^3/norm(xorb(1:3, k))^5*[3*xorb(1, k)*xorb(3, k); 3*xorb(2, k)*xorb(3, k); 2*xorb(3, k)^2-xorb(1, k)^2-xorb(2, k)^2];
end

% figure;
% quiver(xorb(1, :), xorb(3, :), B(1, :), B(3, :));
% axis equal;

% Define target vectors
tvec1 = 1/sqrt(3)*[-1; 1; -1];              % Initial target vector
% tvec1 = [-1; -1; 1.01];              % Initial target vector
% tvec1 = tvec1/norm(tvec1);
tvec2 = 1/sqrt(3)*[-1; -1; 1];              % Desired target vector

% Define initial attitude 
% Note: Payload aperture -Z axis
ax = crossm([0; 0; -1])*tvec1;
ax = ax/norm(ax);
phi = acos([0; 0; -1]'*tvec1);
q0 = [ax*sin(phi/2); cos(phi/2)];

% Define target attitude 
ax = crossm(tvec1)*tvec2;
ax = ax/norm(ax);
phi = acos(tvec1'*tvec2);
dqf = [ax*sin(phi/2); cos(phi/2)];
qf = (dqf(4)*eye(4) + [-crossm(dqf(1:3)) dqf(1:3); -dqf(1:3)' 0])*q0;

% Generate command trajectory
qc = @(t) q0*(t < T/slew_time_div) + qf*(~(t < T/slew_time_div));

% Define spacecraft mass properties
I11 = +0.05;        % [kg m^2]
I12 = -0.002;        % [kg m^2]
I13 = +0.001;        % [kg m^2]
I22 = +0.05;        % [kg m^2]
I23 = +0.0001;        % [kg m^2]
I33 = +0.05;        % [kg m^2]
I = [I11 I12 I13; I12 I22 I23; I13 I23 I33];    % Satellite MOI (kg m^2)
Iinv = inv(I);

% Define control gains
% kp = 0.05;
% kd = 0.5;
% kp = 50e-3;                           % Continuous control torques
% kd = 100e-3;
kp = 20e-3;                             % Discrete control torques
kd = 40e-3;

% Generate slew trajectory
% xatt0 = [q0; 0; 0; 0];
% % xatt0 = [q0; 0; 0; pi()/36];
% % options = odeset('OutputFcn', @odeplot);
% [tout, xatt] = ode45(@clattdyn, t, xatt0, [], kp, kd, Iinv, I, qc);
% xatt = xatt';
xatt = zeros(7, length(t));
xatt(:, 1) = [q0; 0; 0; 0];
for k = 2:length(t)
    % Unpack
    q = xatt(1:4, k - 1);
    om = xatt(5:7, k - 1);
    qf = qc(t(k));
    
    % Compute error quaternion
    qe = (q(4)*eye(4) + [-crossm(q(1:3)) q(1:3); -q(1:3)' 0])*[-qf(1:3); qf(4)];

    % Compute control torques
    if qe(4) == 0
        u = -kp*qe(1:3) - kd*om;
    else
        u = -kp*sign(qe(4))*qe(1:3) - kd*om;
    end
    
    % Propagate dynamics
    [tout, xout] = ode45(@attdyn, [t(k - 1) t(k)], [q; om], [], Iinv, I, u);
    xatt(:, k) = xout(end, :)';
end

% Unpack trajectory components
qt = xatt(1:4, :);
omt = xatt(5:7, :);

% Compute tracking error over time
qet = zeros(4, length(t));
for k = 1:length(t)
    qf = qc(t(k));
    qet(:, k) = (qt(4, k)*eye(4) + [-crossm(qt(1:3, k)) qt(1:3, k); -qt(1:3, k)' 0])*[-qf(1:3); qf(4)];
end
etht = 2*acos(qet(4, :));

% figure;
% plot(t, qet);
% title('Error Quaternion vs. Time');
% legend('\epsilon_1', '\epsilon_2', '\epsilon_3', '\eta', 'location', 'best');

% figure;
% plot(t, etht*180/pi());
% title('Attitude Tracking Error vs. Time');
% xlabel('Time t (s)');
% ylabel('Tracking Error \phi_e_,_t (deg)');
% 
% figure;
% plot(t, omt*180/pi());
% title('Angular Velocity vs. Time');
% xlabel('Time t (s)');
% ylabel('Body Rates \omega (deg/s)');

% Generate true magnetic field in body frame
Bbt = zeros(3, length(t));
for k = 1:length(t)
    Bbt(:, k) = q2C(qt(:, k))*Bi(:, k);
end

% Define noise properties
qom = (0.1*pi()/180)^2;                                 % Gyro noise covariance (rad^2/s^2)
rmag = 3e-7^2;                                      % Magnetometer noise covariance (T^2)
Qtrue = qom*eye(3);                                     
Rtrue = rmag*eye(3);

% Run MC simulations
% rng(0);
for i = 1:N
    nom = sqrt(qom)*randn(3, length(t));
    nmag = sqrt(rmag)*randn(3, length(t));
    
    % Add noise to gyro and magnetometer measurements
    om = omt + nom;
    Bb = Bbt + nmag;

    % Assign to interoceptive and exteroceptive measurement vectors
    u = om;
    y = Bb;

    % figure;
    % plot(t, y);
    % title('y vs. t');
    % 
    % figure;
    % plot(t, u*180/pi());
    % title('u vs. t');
    
    eth_max_zoh(i) = max(slew(u, y, Qtrue, Rtrue, t, qt, qc, Bi, Bb, etht, 0));
    eth_max_simp(i) = max(slew_simp(u, y, Qtrue, Rtrue, t, fs, qt, qc, Bi, Bb, etht, 0));
end

%% Print and Plot Results
figure;
hist((eth_max_zoh - eth_max_simp)*180/pi(), 20);
% slew_time_div
Nless = length(find((eth_max_zoh - eth_max_simp) <= 0))
mu_zoh = mean(eth_max_zoh)*180/pi()
mu_simp = mean(eth_max_simp)*180/pi()
mu_d = mean(eth_max_zoh - eth_max_simp)*180/pi()
sig = std(eth_max_zoh - eth_max_simp)*180/pi()

toc;