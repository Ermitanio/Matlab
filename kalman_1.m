N=100;
t=1:N;

measurements = t  + 1*randn(1,N); % Angulo entregado por el accel
measurements = measurements';
N = length(measurements);

g = 9.81;

dt = .1; % Tiempo entre las mediciones

u = 0; % Motion externa (rapidez angular del gyro)

x = [0]; % Estado inicial (alfa, bias -> valor del giroscopio en estado estacionario)
P = 0.5; % Incertidumbre inicial (aca no se cual seria)
B = [dt ; 0];
F = [1]; % Funcion del proximo estado
H = [1]; % Funcion de measurement

% Valor de la varianza falta para R
R = [500]; % Incertidumbre de la measurement
%

I = [1 0 ; 0 1]; % Matriz identidad
for i=1:N
   % Prediction
    x = (F*x);
    P = F*P*F';
    
    % Measurement Update
    Z = measurements(i);
    Y = Z - H*x;
    S = H*P*H' + R;
    K = P*H'/S;
    x = x + (K*Y);
    P = (1 - (K*H))*P;
    b(i)=x(1);                 %Salida
end

subplot(211);
plot(measurements);
subplot(212);
plot(b);
