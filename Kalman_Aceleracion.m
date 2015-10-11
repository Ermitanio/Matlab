% Ejemplo Kalman
% Example Random Constant using kalman filter

clear all 
%% Muestras 
duration = 10   %Duración [seg]
dt = .01;        %Tiempo entre muestras (Delta t)[seg]

%% Variables
Q_Process_noise_mag = 0.2;     %process noise (Q)
R_Measurement_noise_mag = 20;   %measurement noise (R)

%% Measure
X=0;    %Medición (constante) [ejemplo de kalman para una medicion constante mas ruido]
vel=0;
acc=1;
%% Init results
X_loc = [];
X_loc_meas = [];

%% Generación de la señal ruidosa
figure(2);clf
figure(1);clf
for t = 0 : dt: duration
    % Generate process noise
    QuailAccel_noise = Q_Process_noise_mag *(dt^2/2)*randn;
    X = X + vel * dt + acc *dt^2/2 + QuailAccel_noise;
    vel = vel + acc * dt + Q_Process_noise_mag *dt*randn;
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
A = [1 dt; 0 1];
B = [dt^2/2 ; dt];
H = [1 0];
%initize estimation variables
Ez = R_Measurement_noise_mag^2; % Ez convert the measurement noise (stdv) into covariance matrix
Ex = Q_Process_noise_mag^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2];     % Ex convert the process noise (stdv) into covariance matrix  
X_estimate = [0; 0];     % re-init
P_estimate = 0;    % covarianza de estimacion (inicial)
X_loc_estimate = [];    % Estimado grafico
for t = 1:length(X_loc)
    % Predict next state of the quail with the last state and predicted motion.
    X_estimate = A * X_estimate + B * 1;
    %predict next covariance
    P_estimate = A * P_estimate * A' + Ex;
    
    % predicted measurement covariance
    % Kalman Gain
    K = P_estimate*H'/(H*P_estimate*H'+Ez);
    % Update the state estimate.
    X_estimate = X_estimate + K * (X_loc_meas(t) - H * X_estimate);
    % update covariance estimation.
    P_estimate =  (eye(2)-K*H)*P_estimate;
    
    %Store for plotting
    X_loc_estimate = [X_loc_estimate; X_estimate(1)];
end

% Plot kalman
figure(2);
tt = 0 : dt : duration;
plot(tt,X_loc,'-r.',tt,X_loc_meas,'-k.', tt,X_loc_estimate,'-g.');

