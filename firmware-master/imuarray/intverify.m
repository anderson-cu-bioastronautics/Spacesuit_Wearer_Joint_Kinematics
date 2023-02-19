%% Embedded Numerical Integration Code Verification
% Y. Shen
% 29 September 2020

Nimu = 5;                               % Number of IMUs

dt = 1/800;                                 % Sample period (s);
tso = 0:dt:1;                               % Sample times (s)

phi = zeros(3, 100);                  % Rotation vector
ga = zeros(3, 100);                   % Bias correction coefficient
al = zeros(3, 1);                                     % Initialize alpha
be = zeros(3, 1);                                     % Initialize beta
j = 1;                                      % Initialize estimator sampling time counter
ij = zeros(1, 100);                   % Indices of polling events
ilast = 1;                                  % Index of last polling event
om = omc';
ombar = [mean(om(1:3:3*Nimu, :), 1); mean(om(2:3:3*Nimu, :), 1); mean(om(3:3:3*Nimu, :), 1)];            % Average angular velocities
% ombar = omt;

for i = 2:1:length(tso)
    if mod(i - ilast, 2)                                           % For every other timestep, use trapezoidal rule
        dt = tso(i) - tso(i - 1);

        al1 = al + dt/2*(ombar(:, i - 1) + ombar(:, i));

        gd0 = 0.5*(al - (tso(i - 1) - tso(ilast))*ombar(:, i - 1));
        gd1 = 0.5*(al1 - (tso(i) - tso(ilast))*ombar(:, i));

        bd0 = 0.5*crossm(al)*ombar(:, i - 1);
        bd1 = 0.5*crossm(al1)*ombar(:, i);

        allast = al;                                    % Save last values of alpha, beta, and gamma for Simpson's Rule
        belast = be;
        galast = ga(:, j);

        al = al1;
        be = be + dt/2*(bd0 + bd1);
        ga(:, j) = ga(:, j) + dt/2*(gd0 + gd1);
    else
        dt = tso(i) - tso(i - 2);

%         al1 = allast + dt/24*(5*om(:, i - 2) + 8*om(:, i - 1) - om(:, i));          % Recompute al1 using Simpson's Rule. Makes negligible difference in peak error.
        al2 = allast + dt/6*(ombar(:, i - 2) + 4*ombar(:, i - 1) + ombar(:, i));            % Integrate alpha using Simpson's Rule

%         bd1 = 0.5*crossm(al1)*om(:, i - 1);
        bd2 = 0.5*crossm(al2)*ombar(:, i);                                         % Integrate beta using Simpson's Rule

%         gd1 = 0.5*(al1 - (tso(i - 1) - tso(ilast))*om(:, i - 1));
        gd2 = 0.5*(al2 - (tso(i) - tso(ilast))*ombar(:, i));                       % Integrate gamma using Simpson's Rule

        al = al2;
        be = belast + dt/6*(bd0 + 4*bd1 + bd2);
        ga(:, j) = galast + dt/6*(gd0 + 4*gd1 + gd2);
    end

    if ~mod(i - 1, 8)                       % Record phi every 8 intervals
        phi(:, j) = al + be;                % Record phi
%         al = zeros(3*Nimu, 1);                                     % Initialize alpha
%         be = zeros(3*Nimu, 1);                                     % Initialize beta
        al = zeros(3, 1);                                     % Initialize alpha
        be = zeros(3, 1);                                     % Initialize beta

        ilast = i;                          % Set ilast
        ij(j) = i;                          % Record current index
        j = j + 1;                          % Increment estimator sampling time counter
    end
%     if j > length(t)
%         break;
%     end
end