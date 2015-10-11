% Ejemplo del Welch & Bishop

%Example Random Constant using kalman filter

clear all 
%% Muestras 
duration = 10   %Duración [seg]
dt = .01;        %Tiempo entre muestras (Delta t)[seg]

%% Variables
Q_Process_noise_mag = 0.0005;     %process noise (Q)
R_Measurement_noise_mag = 0.4;   %measurement noise (R)

%% Measure
Z=-0.4;    %Medición (constante) [ejemplo de kalman para una medicion constante mas ruido]

%% Init results
X_loc = [];
X_loc_meas = [];

%% Generación de la señal ruidosa
for t = 0 : dt: duration
    % Generate process noise
    QuailAccel_noise = Q_Process_noise_mag * randn;
    X = Z + QuailAccel_noise;
    
    %Ruido medicion
    noise = R_Measurement_noise_mag * randn;
    y = X + noise;
    X_loc = [X_loc; X(1)];
    X_loc_meas = [X_loc_meas; y];
end
%plot
plot(0:dt:duration, X_loc, '-r.')
plot(0:dt:duration, X_loc_meas, '-k.')
hold on
plot(0:dt:t, smooth(X_loc_meas), '-g.')

%% Filtro Kalman
% Matrices
A = [1] ;
H = [1];
%initize estimation variables
Ez = R_Measurement_noise_mag^2; % Ez convert the measurement noise (stdv) into covariance matrix
Ex = Q_Process_noise_mag^2;     % Ex convert the process noise (stdv) into covariance matrix  
X_estimate = 0      % re-init
P_estimate = 0.4;    % covarianza de estimacion (inicial)
X_loc_estimate = [];    % Estimado grafico

for t = 1:length(X_loc)
    % Predict next state of the quail with the last state and predicted motion.
    X_estimate = A * X_estimate;
    %predict next covariance
    P_estimate = A * P_estimate * A' + Ex;
    
    % predicted measurement covariance
    % Kalman Gain
    K = P_estimate*H'/(H*P_estimate*H'+Ez);
    % Update the state estimate.
    X_estimate = X_estimate + K * (X_loc_meas(t) - H * X_estimate);
    % update covariance estimation.
    P_estimate =  (1-K*H)*P_estimate;
    
    %Store for plotting
    X_loc_estimate = [X_loc_estimate; X_estimate(1)];
end

% Plot kalman
figure(2);
tt = 0 : dt : duration;
plot(tt,X_loc,'-r.',tt,X_loc_meas,'-k.', tt,X_loc_estimate,'-g.');

%%
%FFT
figure(3);
freq = 0:(1/dt)/length(X_loc_meas):(1/dt)/2; %Genero vector de frecuencia
ydft = fft(X_loc_meas);  %Aplico FFT
ydft = ydft(1:length(X_loc_meas)/2+1);   %Hace algo para que se vea bien
subplot(2,1,1);
plot(freq,abs(ydft));   %Plot la FFT en valor absoluto con el vector frecuencia
grid on
%FFT
freq = 0:(1/dt)/length(X_loc_estimate):(1/dt)/2; %Genero vector de frecuencia
ydft = fft(X_loc_estimate);  %Aplico FFT
ydft = ydft(1:length(X_loc_estimate)/2+1);   %Hace algo para que se vea bien
subplot(2,1,2);
plot(freq,abs(ydft));   %Plot la FFT en valor absoluto con el vector frecuencia
grid on
