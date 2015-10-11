N=10000;
t=1:N;
measurements = (t)  + 1000*randn(1,N); % Angulo entregado por el accel
measurements = measurements';
N = length(measurements);

dt = .01; % Tiempo entre las mediciones
 
u = 0;

x = [0]; % Estado inicial (alfa, bias -> valor del giroscopio en estado estacionario)
P = 1; % Incertidumbre inicial (aca no se cual seria)
B = [1 ; 0];
F = [1 -dt ; 0 1]; % Funcion del proximo estado
H = [1]; % Funcion de measurement

% Valor de la varianza falta para R
R = [.0001]; % Incertidumbre de la measurement
%

I = [1 0 ; 0 1]; % Matriz identidad
for i=1:N
   % Prediction
    x = x;
    P = F*P*F'+10;
    
    % Measurement Update
    Z = measurements(i);
    Y = Z - H*x;
    S = H*P*H' + R;
    K = P*H'/S;
    x = x + (K*Y);
    P = (eye(2) - (K*H))*P;
    b(i)=x(1);                  %Salida
    
    
end
subplot(211);
plot(measurements);
subplot(212);
plot(b);