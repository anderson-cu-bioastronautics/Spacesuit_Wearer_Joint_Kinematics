%% Embedded Calibration Code Verification
% Y. Shen
% 15 November 2020

%% Define calibration parameters

ptom = [0.03 0.1 1 -20;                % Gyro thermal calibration polynomial coefficients (Temperature LSB counts -> gyro LSB counts linearized)
    0.02 0.2 1 40;
    -0.01 0.3 1 60];

pnlom = [0.01 0.1 1 -20;               % Gyro nonlinearity polynomial coefficients (gyro LSB counts -> gyro LSB counts linearized)
    0.02 0.2 1 40;
    -0.03 0.3 1 60];

Kom = 9.81*[0.000244 0 0;           % Gyro scale factor matrix ((m/s^2)/LSB)
    0 0.000244 0;
    0 0 0.000244];

Tom = [1 -0.001 0.001;              % Gyro nonorthogonality matrix
    0 1 -0.002;
    0 0 1];

TKom = Tom*Kom;                     % Gyro combined scale factor and nonorthogonality matrix

%% Define inputs

T = 500*0.001*sin(1:400/pi());          % Temperature (counts)

%% Calibrate raw measurements

omcal = zeros(size(omraw));

for i = 1:size(omraw, 2)
    bom = ptom(:, 1).*T(i).^3 + ptom(:, 2).*T(i).^2 + ptom(:, 3).*T(i) + ptom(:, 4);
    omcal(:, i) = TKom*(pnlom(:, 1).*omraw(:, i).^3 + pnlom(:, 2).*omraw(:, i).^2 + pnlom(:, 3).*omraw(:, i) + pnlom(:, 4) - bom);
end