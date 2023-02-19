%% Coop Localization Toy Problem, Zero Order Hold EKF
% Y. Shen
% 24 January 2019

% Define simulation parameters
fs = 1;                                         % Sample rate (Hz)
T = 400;                                        % End time (s)
t = 1/fs:1/fs:T;                                % Timestamps (s)

% Define vehicle properties
L = 0.5;                                        % UGV wheel separation (m)

% Define noise properties
% qxg = 1e-3;                                     % UGV position noise covariance (m^2)
% qthg = 0.01;                                    % UGV heading noise covariance (rad^2)
% qxa = 1e-3;                                     % UAV position noise covariance (m^2)
% qtha = 0.01;                                    % UAV heading noise covariance (rad^2)
% Q = diag([qxg qxg qthg qxa qxa qtha]);
% qvg = 0.01;                                      % UGV speed noise covariance (m^2/s^2)
% qphig = (0.1*pi()/180)^2;                                   % UGV steering angle noise covariance (rad^2)
% qvg = 1e-12;                                      % UGV speed noise covariance (m^2/s^2)
% qphig = 1e-12;                                   % UGV steering angle noise covariance (rad^2)
% qva = 0.1;                                      % UAV speed noise covariance (m^2/s^2)
qvg = 1e-12;                                      % UGV speed noise covariance (m^2/s^2)
qphig = 1e-12;                                   % UGV steering angle noise covariance (rad^2)
qva = 0.1;                                      % UAV speed noise covariance (m^2/s^2)
qoma = (pi()/180)^2;                                    % UAV angular rate noise covariance (rad^2)
Qtrue = diag([qvg qphig qva qoma]);
% Qtrue = zeros(4);
rrho = 64;                                      % Range measurement covariance (m^2)
rx = 16;                                        % UAV position covariance (m^2)
Rtrue = diag([rrho rx rx]);

% Define input parameters
% UGV steering angle 0.2 Hz sinusoid
vg = 5;                                         % UGV speed (m/s)
% phi = -pi()/18;                                 % UGV steering angle (rad)
phi = @(t) 10*pi()/180*sin(t);                 % UGV steering angle (rad)
va = 12;                                        % UAV speed (m/s)
om = pi/25;                                     % UAV rate of turn (rad/s)
% u = [vg; phi; va; om];
ufcn = @(t) [vg; phi(t); va; om];

% Define initial conditions
x0 = [10; 0; pi()/2; -60; 0; -pi()/2];

% Generate simulation data
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[tout, xt] = ode45(@hodefun, t, x0, options, ufcn, L);
xt = xt';

figure;
plot(xt(1, :), xt(2, :), xt(4, :), xt(5, :));
title('Ground Vehicle and Aircraft True Positions');
xlabel('Easting \xi (m)');
ylabel('Northing \eta (m)');
legend('Ground Vehicle', 'Aircraft');
grid on;
axis equal;

% Reseed RNG for consistent noise
rng(0);

% Generate interoceptive measurements
ut = zeros(4, length(t));
for k = 1:length(t)
    ut(:, k) = ufcn(t(k));
end
u = ut + sqrt(Qtrue)*randn(4, length(t));

% Generate exteroceptive measurements
yt = zeros(3, length(t));
for k = 1:length(t)
    yt(:, k) = g(xt(:, k));
end

% figure;
% plot(t, yt(1, :));
% title('True Range Measurement');
% xlabel('Time t (s)');
% ylabel('Range \rho (m)');
% grid on;

y = yt + sqrt(Rtrue)*randn(3, length(t));

% figure;
% plot(t, y(1, :));
% title('Noisy Range Measurement');
% xlabel('Time t (s)');
% ylabel('Range \rho (m)');
% grid on;
% 
% figure;
% plot(y(2, :), y(3, :));
% title('Aircraft Position Measurement');
% xlabel('Easting \xi (m)');
% ylabel('Northing \eta (m)');
% grid on;
% axis equal;

% Run EKF
Q = Qtrue;
% Q = diag([qvg qphig qva qoma]);
% R = Rtrue;
% R(2:3, 2:3) = 0.001*diag([rx rx]);
R = Rtrue(2:3, 2:3);

x = zeros(6, length(t));
P = zeros(6^2, length(t));
Kk = zeros(12, length(t));
Pmk = P;
% inno = zeros(3, length(t));
inno = zeros(2, length(t));                     % No range
x(:, 1) = x0;
Hwk = getHwk(x0, u(:, 1), L)/fs;
P(:, 1) = reshape(Hwk*Q*Hwk', 6^2, 1);
% hodefun2 = @(t, x, u, L) h(x, u, L);
for k = 2:length(t)
    dt = t(k) - t(k - 1);
    
    % Prediction
    Hxk = eye(6) + getHxk(x(:, k - 1), u(:, k - 1))*dt;
    Hwk = getHwk(x(:, k - 1), u(:, k - 1), L)*dt;
    Pm = Hxk*reshape(P(:, k), 6, 6)*Hxk' + Hwk*Q*Hwk';
    xm = x(:, k - 1) + h(x(:, k - 1), u(:, k - 1), L)*dt;
%     [tout, xout] = ode45(hodefun2, [t(k - 1) t(k)], x(:, k - 1), [], u(:, k - 1), L);
%     xm = xout(end, :)';
    
    xop = xm;
    for j = 1:1
%         % Kalman Gain
%         Gxk = Gxcoop(xop);
%         K = Pm*Gxk'/(Gxk*Pm*Gxk' + R);
%     %     K = zeros(6, 3);
% 
%         % Correction
%         inno(:, k) = y(:, k) - gcoop(xop);
%         xop = xm + K*(inno(:, k) - Gxk*(xm - xop));
        % Kalman Gain                       % No range
        Gxk = getGxk(xop);
        Gxk = Gxk(2:3, :);
        K = Pm*Gxk'/(Gxk*Pm*Gxk' + R);
    %     K = zeros(6, 3);

        % Correction
        yh = g(xop);
        yh = yh(2:3);
        inno(:, k) = y(2:3, k) - yh;
        xop = xm + K*(inno(:, k) - Gxk*(xm - xop));
    end
    
    P(:, k) = reshape((eye(6) - K*Gxk)*Pm, 6^2, 1);
    x(:, k) = xop;
    Kk(:, k) = reshape(K, 12, 1);
    Pmk(:, k) = reshape(Pm, 6^2, 1);
end

figure;
plot(x(1, :), x(2, :), x(4, :), x(5, :), x(4, 1), x(5, 1), 'd');
title('Ground Vehicle and Aircraft Estimated Positions');
xlabel('Easting \xi (m)');
ylabel('Northing \eta (m)');
legend('Ground Vehicle', 'Aircraft', 'location', 'best');
grid on;
axis equal;

figure;
plot(t, inno);
title('Innovation vs. Time');
xlabel('Time t (s)');
ylabel('Innovation');
grid on;

ex = x - xt;

figure;
plot(t, ex(1:2, :));
title('UGV Position Errors vs. Time');
xlabel('Time t (s)');
ylabel('Position Error (m)');
legend('Easting \Delta\xi', 'Northing \Delta\eta', 'location', 'best');
grid on;

figure;
plot(t, ex(3, :)*180/pi());
title('UGV Heading Error vs. Time');
xlabel('Time t (s)');
ylabel('Heading Error (deg)');
grid on;

sigax = sqrt(P(6*3 + 4, :));
sigay = sqrt(P(6*4 + 5, :));
figure;
plot(t, ex(4:5, :), t, 2*sigax, 'g-', t, -2*sigax, 'g-', t, 2*sigay, 'y-', t, -2*sigay, 'y-');
title('UAV Position Errors vs. Time');
xlabel('Time t (s)');
ylabel('Position Error (m)');
legend('Easting \Delta\xi', 'Northing \Delta\eta', 'location', 'best');
grid on;

figure;
plot(t, ex(6, :)*180/pi());
title('UAV Heading Error vs. Time');
xlabel('Time t (s)');
ylabel('Heading Error (deg)');
grid on;