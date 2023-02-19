%% Attitude Determination Toy Problem, ZOH EKF
% Y. Shen
% 28 January 2019

function [eth, q, P11, P22, P33, P44] = slew_simp(u, y, Qtrue, Rtrue, t, fs, qt, qc, Bi, Bb, etht, fPlot)
% nom - Gyro noise (rad/s)
% nmag - Magnetometer noise (T)
% fPlot - Plot flag

%% Run EKF

% Define noise covariances
Q = blkdiag(Qtrue, Qtrue, Qtrue);
R = Rtrue;

% Initialize state and covariance
x = zeros(8, length(t));
P = zeros(64, length(t));
inno = zeros(3, length(t));
x(:, 1) = [qt(:, 1); qt(:, 1)];
dt = 1/fs;
Hwk0 = 0.5*[getHwk(x(:, 1), u(:, 1), dt) getHwk(x(:, 1), u(:, 1), dt); zeros(4, 6)];
P(:, 1) = reshape(Hwk0*Q(1:6, 1:6)*Hwk0', 64, 1);

% Run EKF
xm = zeros(8, 1);
for k = 2:length(t)
    % Trapezoidal rule for every other step
    if mod(k, 2) == 0
        omtrap = 0.5*(u(:, k - 1) + u(:, k));
        
        % Prediction
        Hxk = [zeros(4) getHxk(omtrap, dt); eye(4) zeros(4)];
        Hwk = getHwk(x(:, k - 1), omtrap, dt);
        Hwk = 0.5*[Hwk Hwk; zeros(4, 6)];
        Pm = Hxk*reshape(P(:, k - 1), 8, 8)*Hxk' + Hwk*Q(1:6, 1:6)*Hwk';
        xm(1:4) = h(x(1:4, k - 1), omtrap, dt);
        xm(5:8) = x(1:4, k - 1);

        xop = xm;
        for j = 1:1
            % Kalman Gain
            Gxk = [getGxk(xop(1:4), Bi(:, k)) zeros(3, 4)];
            K = Pm*Gxk'/(Gxk*Pm*Gxk' + R);

            % Correction
            inno(:, k) = y(:, k) - q2C(xop)*Bi(:, k);
            xop = xm + K*(inno(:, k) - Gxk*(xm - xop));

            % Normalize quaternion
            xop(1:4) = xop(1:4)/norm(xop(1:4));
        end

        P(:, k) = reshape((eye(8) - K*Gxk)*Pm, 64, 1);
        x(:, k) = xop;
    else
        % Compute composite angular velocity
        omsimp = 1/6*(u(:, k - 2) + 4*u(:, k - 1) + u(:, k));
        
        % Prediction
        Hxk = getHxksimp(u(:, k), dt);
        Hwk = getHwksimp(x(:, k - 1), u(:, k), dt);
        Pm = Hxk*reshape(P(:, k - 1), 8, 8)*Hxk' + Hwk*Q*Hwk';
        xm(1:4) = h(x(5:8, k - 1), omsimp, 2*dt);
        xm(5:8) = x(1:4, k - 1);

        xop = xm;
        for j = 1:1
            % Kalman Gain
            Gxk = [getGxk(xop(1:4), Bi(:, k)) zeros(3, 4)];
            K = Pm*Gxk'/(Gxk*Pm*Gxk' + R);

            % Correction
            inno(:, k) = y(:, k) - q2C(xop)*Bi(:, k);
            xop = xm + K*(inno(:, k) - Gxk*(xm - xop));

            % Normalize quaternion
            xop(1:4) = xop(1:4)/norm(xop(1:4));
        end

        P(:, k) = reshape((eye(8) - K*Gxk)*Pm, 64, 1);
        x(:, k) = xop;

    %     P(:, k) = reshape(Pm, 16, 1);
    %     x(:, k) = xm;
    end
end

%% Plot

% Unpack output
q = x(1:4, :);
P11 = P(1, :);
P22 = P(8 + 2, :);
P33 = P(2*8 + 3, :);
P44 = P(3*8 + 4, :);

% Compute errors

% Attitude estimation error
qe = zeros(4, length(t));
for k = 1:length(t)
    qe(:, k) = (q(4, k)*eye(4) + [-crossm(q(1:3, k)) q(1:3, k); -q(1:3, k)' 0])*[-qt(1:3, k); qt(4, k)];
end
eth = 2*acos(qe(4, :));

% % Estimated tracking error
% qeth = zeros(4, length(t));
% for k = 1:length(t)
%     qf = qc(t(k));
%     qeth(:, k) = (q(4, k)*eye(4) + [-crossm(q(1:3, k)) q(1:3, k); -q(1:3, k)' 0])*[-qf(1:3); qf(4)];
% end
% ethth = 2*acos(qeth(4, :));
% 
% % Magnetic field vector estimation error
% eB = zeros(1, length(t));
% for k = 1:length(t)
%     Bbh = q2C(q(:, k))*Bi(:, k);
%     eB(k) = acosd(Bb(:, k)'*Bbh/(norm(Bb(:, k))*norm(Bbh)));
% end

if fPlot
    % figure;
    % plot(t, eth*180/pi());
    % title('Attitude Estimation Error vs. Time');
    % xlabel('Time t (s)');
    % ylabel('Attitude Estimation Error \Delta\phi (deg)');
    % grid on;
    % 
    % figure;
    % plot(t, etht*180/pi());
    % title('Estimated Attitude Tracking Error vs. Time');
    % xlabel('Time t (s)');
    % ylabel('Estimated Attitude Tracking Error \Delta\phi-hat (deg)');
    % grid on;

    figure;
    plot(t, etht*180/pi(), t, ethth*180/pi(), t, eth*180/pi());
    title('Attitude Errors vs. Time, Simpson''s Rule');
    xlabel('Time t (s)');
    ylabel('Attitude Error \Delta\phi (deg)');
    legend('True Tracking Error \Delta\phi', 'Estimated Tracking Error \Delta\phi-hat', 'Attitude Estimation Error \Delta\phi_e', 'location', 'best');
    grid on;
    
    figure;
    subplot(2, 2, 1);
    plot(t, q(1, :) - qt(1, :), t, 2*sqrt(P11), 'g-', t, -2*sqrt(P11), 'g-');
    title('q_1');
    xlabel('Time t (s)');
    grid on;
    subplot(2, 2, 2);
    plot(t, q(2, :) - qt(2, :), t, 2*sqrt(P22), 'g-', t, -2*sqrt(P22), 'g-');
    title('q_2');
    xlabel('Time t (s)');
    grid on;
    subplot(2, 2, 3);
    plot(t, q(3, :) - qt(3, :), t, 2*sqrt(P33), 'g-', t, -2*sqrt(P33), 'g-');
    title('q_3');
    xlabel('Time t (s)');
    grid on;
    subplot(2, 2, 4);
    plot(t, q(4, :) - qt(4, :), t, 2*sqrt(P44), 'g-', t, -2*sqrt(P44), 'g-');
    title('q_4');
    xlabel('Time t (s)');
    grid on;

    % figure;
    % plot(t, inno);
    % title('Innovation vs. Time');
    % xlabel('Time t (s)');
    % ylabel('Innovation (T)');
    % grid on;

    % figure;
    % plot(t, eB);
    % title('Magnetic Field Vector Direction Estimation Error vs. Time');
    % xlabel('Time t (s)');
    % ylabel('Magnetic Field Vector Direction Error \Delta\theta_B (deg)');
    % grid on;
end

%% Integrated Stabilizer (Controller with Estimator Feedback)

% % Initialize true states and measurements
% xatt = zeros(7, length(t));
% xatt(:, 1) = [q0; 0; 0; 0];
% qt = xatt(1:4, :);
% omt = zeros(3, length(t));
% omt(:, 1) = xatt(5:7, 1);
% Bbt = zeros(3, length(t));
% Bb = zeros(3, length(t));
% 
% % Initialize estimated state and covariance
% om = zeros(3, length(t));
% om(:, 1) = omt(:, 1) + nom(:, 1);
% u = om;
% y = zeros(3, length(t));
% x = zeros(8, length(t));
% P = zeros(64, length(t));
% inno = zeros(3, length(t));
% x(:, 1) = [qt(:, 1); qt(:, 1)];
% Hwk0 = 0.5*[getHwk(x(:, 1), u(:, 1), dt) getHwk(x(:, 1), u(:, 1), dt); zeros(4, 6)];
% P(:, 1) = reshape(Hwk0*Q(1:6, 1:6)*Hwk0', 64, 1);
% 
% for k = 2:length(t)
%     dt = t(k) - t(k - 1);
%     % Control
%     %   Unpack states
%     qf = qc(t(k));
%     q = x(1:4, k - 1);
%     
%     %   Compute error quaternion
%     qe = (q(4)*eye(4) + [-crossm(q(1:3)) q(1:3); -q(1:3)' 0])*[-qf(1:3); qf(4)];
% 
%     %   Compute control torques
%     if qe(4) == 0
%         uc = -kp*qe(1:3) - kd*om(:, k - 1);
%     else
%         uc = -kp*sign(qe(4))*qe(1:3) - kd*om(:, k - 1);
%     end
%     
%     %   Propagate dynamics
%     [tout, xout] = ode45(@attdyn, [0 dt], [qt(:, k - 1); omt(:, k - 1)], [], Iinv, I, uc);
%     xatt(:, k) = xout(end, :)';
%     
%     %   Assign outputs, generate measurements
%     qt(:, k) = xatt(1:4, k);
%     omt(:, k) = xatt(5:7, k);
%     om(:, k) = omt(:, k) + nom(:, k);
%     u(:, k) = om(:, k);
%     Bbt(:, k) = q2C(qt(:, k))*Bi(:, k);
%     Bb(:, k) = Bbt(:, k) + nmag(:, k);
%     y(:, k) = Bb(:, k);
%     
%     % Estimate    
%     % Trapezoidal rule for every other step
%     if mod(k, 2) == 0
%         omtrap = 0.5*(u(:, k - 1) + u(:, k));
%         
%         % Prediction
%         Hxk = [zeros(4) getHxk(omtrap, dt); eye(4) zeros(4)];
%         Hwk = getHwk(x(:, k - 1), omtrap, dt);
%         Hwk = 0.5*[Hwk Hwk; zeros(4, 6)];
%         Pm = Hxk*reshape(P(:, k - 1), 8, 8)*Hxk' + Hwk*Q(1:6, 1:6)*Hwk';
%         xm(1:4) = h(x(1:4, k - 1), omtrap, dt);
%         xm(5:8) = x(1:4, k - 1);
% 
%         xop = xm;
%         for j = 1:1
%             % Kalman Gain
%             Gxk = [getGxk(xop, Bi(:, k)) zeros(3, 4)];
%             K = Pm*Gxk'/(Gxk*Pm*Gxk' + R);
% 
%             % Correction
%             inno(:, k) = y(:, k) - q2C(xop)*Bi(:, k);
%             xop = xm + K*(inno(:, k) - Gxk*(xm - xop));
% 
%             % Normalize quaternion
%             xop(1:4) = xop(1:4)/norm(xop(1:4));
%         end
% 
%         P(:, k) = reshape((eye(8) - K*Gxk)*Pm, 64, 1);
%         x(:, k) = xop;
%     else
%         % Compute composite angular velocity
%         omsimp = 1/6*(u(:, k - 2) + 4*u(:, k - 1) + u(:, k));
%         
%         % Prediction
%         Hxk = getHxksimp(u(:, k), dt);
%         Hwk = getHwksimp(x(:, k - 1), u(:, k), dt);
%         Pm = Hxk*reshape(P(:, k - 1), 8, 8)*Hxk' + Hwk*Q*Hwk';
%         xm(1:4) = h(x(5:8, k - 1), omsimp, 2*dt);
%         xm(5:8) = x(1:4, k - 1);
% 
%         xop = xm;
%         for j = 1:1
%             % Kalman Gain
%             Gxk = [getGxk(xop, Bi(:, k)) zeros(3, 4)];
%             K = Pm*Gxk'/(Gxk*Pm*Gxk' + R);
% 
%             % Correction
%             inno(:, k) = y(:, k) - q2C(xop)*Bi(:, k);
%             xop = xm + K*(inno(:, k) - Gxk*(xm - xop));
% 
%             % Normalize quaternion
%             xop(1:4) = xop(1:4)/norm(xop(1:4));
%         end
% 
%         P(:, k) = reshape((eye(8) - K*Gxk)*Pm, 64, 1);
%         x(:, k) = xop;
%     end
% end
% 
% % Unpack output
% q = x;
% 
% % Compute errors
% 
% % True tracking error
% qet = zeros(4, length(t));
% for k = 1:length(t)
%     qf = qc(t(k));
%     qet(:, k) = (qt(4, k)*eye(4) + [-crossm(qt(1:3, k)) qt(1:3, k); -qt(1:3, k)' 0])*[-qf(1:3); qf(4)];
% end
% etht = 2*acos(qet(4, :));
% 
% % Attitude estimation error
% qe = zeros(4, length(t));
% for k = 1:length(t)
%     qe(:, k) = (q(4, k)*eye(4) + [-crossm(q(1:3, k)) q(1:3, k); -q(1:3, k)' 0])*[-qt(1:3, k); qt(4, k)];
% end
% eth = 2*acos(qe(4, :));
% 
% % Estimated tracking error
% qeth = zeros(4, length(t));
% for k = 1:length(t)
%     qf = qc(t(k));
%     qeth(:, k) = (q(4, k)*eye(4) + [-crossm(q(1:3, k)) q(1:3, k); -q(1:3, k)' 0])*[-qf(1:3); qf(4)];
% end
% ethth = 2*acos(qeth(4, :));
% 
% %% Plot
% 
% if fPlot
%     figure;
%     plot(t, etht*180/pi(), t, ethth*180/pi(), t, eth*180/pi());
%     title('Attitude Errors vs. Time, Simpson''s Rule Estimator Feedback');
%     xlabel('Time t (s)');
%     ylabel('Attitude Error \Delta\phi (deg)');
%     legend('True Tracking Error \Delta\phi', 'Estimated Tracking Error \Delta\phi-hat', 'Attitude Estimation Error \Delta\phi_e', 'location', 'best');
%     grid on;
% end
end