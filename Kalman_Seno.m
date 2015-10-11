% Ejemplo del Welch & Bishop

%Example Random Constant using kalman filter

clear all 
%% Muestras 
duration = 10   %Duración [seg]
dt = .01;        %Tiempo entre muestras (Delta t)[seg]

%% Variables
Q_Process_noise_mag = 0.05;     %process noise (Q)
R_Measurement_noise_mag = 10;   %measurement noise (R)

%% Measure
X=0;    %Medición (constante) [ejemplo de kalman para una medicion constante mas ruido]
vel=0;
%% Init results
X_loc = [];
X_loc_meas = [];

%% Generación de la señal ruidosa
figure(2);clf
figure(1);clf
for t = 0 : dt: duration
    % Generate process noise
    QuailAccel_noise = Q_Process_noise_mag *randn;
    X = sin(2*pi*2*t) + QuailAccel_noise;
    %Ruido medicion
    noise = R_Measurement_noise_mag * randn;
    y = X + noise;
    X_loc = [X_loc; X(1)];
    X_loc_meas = [X_loc_meas; y];    
end

%plot
plot(0:dt:t, X_loc, '-r.')
hold on
plot(0:dt:t, X_loc_meas, '-k.')
hold on
plot(0:dt:t, smooth(X_loc_meas), '-g.')

%% Filtro Kalman
% Matrices
f=2;
w=2*pi*f
H = [1 0 0];
%initize estimation variables
R = R_Measurement_noise_mag^2; 
Q = Q_Process_noise_mag^2 * [1 1 1; 1 1 0; 1 0 1];
X_estimate = [0; 0; 0];     % re-init
P_estimate = 0;    % covarianza de estimacion (inicial)
X_loc_estimate = [];    % Estimado grafico
for t = 1:length(X_loc)
    A = [1 dt w*cos(w*t*dt); 0 1 0; 0 0 1];
    
    % Predict next state of the quail with the last state and predicted motion.
    X_estimate = A * X_estimate;
    %predict next covariance
    P_estimate = A * P_estimate * A' + Q;
    
    % predicted measurement covariance
    % Kalman Gain
    K = P_estimate*H'/(H*P_estimate*H'+R);
    % Update the state estimate.
    X_estimate = X_estimate + K * (X_loc_meas(t) - H * X_estimate);
    % update covariance estimation.
    P_estimate = P_estimate- K*H*P_estimate;
    
    %Store for plotting
    X_loc_estimate = [X_loc_estimate; X_estimate(1)];
end

% Plot kalman
figure(2);
tt = 0 : dt : duration;
plot(tt,X_loc,'-r.',tt,X_loc_meas,'-k.', tt,X_loc_estimate,'-g.');

