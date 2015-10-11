% Ejemplo del Welch & Bishop
% Example Random Constant using kalman filter

clear all 
%% Muestras 
duration = 20   %Duración [seg]
dt = .01;        %Tiempo entre muestras (Delta t)[seg]

%% Variables
Q_Process_noise_mag = 0.01;     %process noise (Q)
R_Measurement_noise_mag = 1;   %measurement noise (R)

%% Measure
X=0;    %Medición (constante) [ejemplo de kalman para una medicion constante mas ruido]
%accelerometer
%% Init results
X_loc = [];
X_loc_meas = [];

%% Generación de la señal ruidosa
figure(1);clf
i=0;
for t = 0 : dt: duration
    % Generate process noise
    QuailAccel_noise = Q_Process_noise_mag *randn;
    if t<2.5
        i = i+dt;
    elseif t<7.5
        i = i-dt;
    elseif t<10
        i = i+dt;
    end
        
    X =  i + QuailAccel_noise;
    %Ruido medicion
    noise = R_Measurement_noise_mag * randn;
    y = X + noise;
    X_loc = [X_loc; X(1)];
    X_loc_meas = [X_loc_meas; y]; 
end

%% Filtro Kalman
% Matrices
A = [1 dt dt^2/2; 0 1 dt; 0 0 1];
H = [0 0 1];
%initize estimation variables
Ez = R_Measurement_noise_mag; % Ez convert the measurement noise (stdv) into covariance matrix
Ex = Q_Process_noise_mag * [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 dt];     % Ex convert the process noise (stdv) into covariance matrix  
X_estimate = [0;0;0];     % re-init
P_estimate = Ex;    % covarianza de estimacion (inicial)
X_loc_estimate = [];    % Estimado grafico


for t = 1:length(X_loc)
    % Predict next state of the quail with the last state and predicted motion.
    X_estimate = A * X_estimate;
    %predict next covariance
    P_estimate = A * P_estimate * A' + Ex;
    
    % predicted measurement covariance
    % Kalman Gain
    K = P_estimate*H'*inv(H*P_estimate*H'+Ez);
    % Update the state estimate.
    X_estimate = X_estimate + K * (X_loc_meas(t) - H * X_estimate);
    % update covariance estimation.
    P_estimate =  (eye(3)-K*H)*P_estimate;
    
    %Store for plotting
    X_loc_estimate = [X_loc_estimate; X_estimate(1)];
end

% Plot kalman
figure(1);
tt = 0 : dt : duration;
plot(tt,X_loc,'-r.',tt,X_loc_meas,'-k.', tt,X_loc_estimate,'-g.');

