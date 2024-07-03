% Creare una mappa di occupazione
map = binaryOccupancyMap(50, 50, 1); % Mappa 50x50 con risoluzione 1 cella per metro
% Aggiungere ostacoli alla mappa
setOccupancy(map, [10, 10; 15, 15; 20, 20; 35, 30; 40, 40;], 1); % Aggiungere ostacoli

% Definire i punti di partenza e di arrivo
start = [5, 5, 0]; % [x, y, theta]
goal = [45, 45, 0]; % [x, y, theta]

% Configurare il planner RRT
stateSpace = stateSpaceSE2([0 50; 0 50; -pi pi]); % Stato spazio 2D più angolo theta
stateValidator = validatorOccupancyMap(stateSpace);
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.1;

planner = plannerRRT(stateSpace, stateValidator);
planner.MaxConnectionDistance = 2.0;
planner.GoalReachedFcn = @(planner, state, goal) norm(state(1:2) - goal(1:2)) < 1.0;

% Pianificare il percorso
[pthObj, solnInfo] = plan(planner, start, goal);

% Estrarre i waypoints dal percorso pianificato
waypoints = pthObj.States(:, 1:3);

% Preallocazione delle variabili per i punti interpolati
x_d = [];
y_d = [];
theta_d = [];
phi_d = [];

% Interpolazione dei segmenti con polinomi cubici
for i = 1:size(waypoints, 1) - 1
    % Estrazione dei punti iniziali e finali per il segmento corrente
    p1 = waypoints(i, 1:2);
    p2 = waypoints(i+1, 1:2);
    
    % Tempo per il segmento corrente
    t_segment = linspace(0, 1, 100);
    
    % Polinomi cubici per x e y
    coeffs_x = polyfit([0, 1], [p1(1), p2(1)], 3);
    coeffs_y = polyfit([0, 1], [p1(2), p2(2)], 3);
    
    % Valori interpolati per x e y
    x_interp = polyval(coeffs_x, t_segment);
    y_interp = polyval(coeffs_y, t_segment);
    
    % Calcolo dell'orientamento theta per il segmento corrente
    theta_interp = atan2(diff(y_interp), diff(x_interp));
    theta_interp = [theta_interp, theta_interp(end)];
    
    % Concatenazione dei valori interpolati
    x_d = [x_d, x_interp];
    y_d = [y_d, y_interp];
    theta_d = [theta_d, theta_interp];
end

% Calcolo dell'angolo di sterzata phi
phi_d = atan2(diff(y_d), diff(x_d));
phi_d = [phi_d, phi_d(end)];

% Creazione delle timeseries per Simulink
T = 20; % Durata totale della simulazione
t = linspace(0, T, length(x_d));
xd = timeseries(x_d, t);
yd = timeseries(y_d, t);
thetad = timeseries(theta_d, t);
phid = timeseries(phi_d, t);

% Salva le timeseries in un file MAT
save('car_like_path_cubic.mat', 'xd', 'yd', 'thetad', 'phid');

% Visualizzare il percorso
show(map)
hold on
plot(pthObj.States(:,1), pthObj.States(:,2), 'b-', 'LineWidth', 2);
plot(x_d, y_d, 'r--', 'LineWidth', 2);
legend('Percorso iniziale RRT', 'Percorso interpolato con polinomi cubici')
hold off
