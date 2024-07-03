% Parametri del percorso
R = 20;            % Raggio del cerchio (m)
omega = 0.5;       % Velocità angolare (rad/s)
L = 1;           % Lunghezza del veicolo (m)
T = 20;            % Durata della simulazione (s)
ti = 0;
tf = 100;

T = tf - ti;

t = 0:0.001:tf;      % Vettore del tempo

% Preallocazione delle variabili
x_d = zeros(size(t));
y_d = zeros(size(t));
theta_d = zeros(size(t));
phi_d = atan(L / R) * ones(size(t)); % Angolo di sterzata costante

% Calcolo delle traiettorie
for i = 1:length(t)
    x_d(i) = R * cos(omega * t(i));
    y_d(i) = R * sin(omega * t(i));
    theta_d(i) = omega * t(i) + pi/2;
end

% Creazione delle timeseries per Simulink
xd = timeseries(x_d, t);
yd = timeseries(y_d, t);
thetad = timeseries(theta_d, t);
phid = timeseries(phi_d, t);

figure
plot(x_d,y_d)
