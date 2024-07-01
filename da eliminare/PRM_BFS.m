clc;
clear all;
close all;

%% Map and Points Visualization

% Load the map
load('image_map.mat');

% Definition of the start and goal points
q_start = [30, 125]; % Start point coordinates
q_goal = [135, 400]; % Goal point coordinates

% Plot initialization
figure;
imshow(image_map);
hold on;

% Plotting the start and goal points on the map
plot(q_start(2), q_start(1), 'ro', 'MarkerSize', 10); % Start point (red)
plot(q_goal(2), q_goal(1), 'bo', 'MarkerSize', 10); % Goal point (blue)

%% PRM Algorithm Implementation with BFS

% Algorithm parameters
max_iterations_per_attempt = 100; % Maximum iterations per attempt
max_attempts = 5; % Maximum number of attempts
delta = 10; % Connection threshold

% PRM parameters
num_samples = 500; % Number of samples (nodes) to generate
max_dist = 30; % Maximum distance for connections
min_dist = 15;

% Solution variable initialization
solution_found = false;

% Loop over attempts
for attempt = 1:max_attempts
    disp(['Attempt ', num2str(attempt), ':']);
    
    % Resetting the plot and tree for each attempt
    clf;
    imshow(image_map);
    hold on;
    plot(q_start(2), q_start(1), 'ro', 'MarkerSize', 10, 'LineWidth',2); % Start point (red)
    plot(q_goal(2), q_goal(1), 'bo', 'MarkerSize', 10,'LineWidth',2); % Goal point (blue)
    
    % Initialization of the nodes (samples) and adjacency matrix
    nodes = rand(num_samples, 2) .* [size(image_map, 1), size(image_map, 2)];
    adjacency_matrix = zeros(num_samples, num_samples);
    
    % Building the roadmap (PRM)
    for i = 1:num_samples
        for j = i+1:num_samples
            % Check if nodes i and j are within max_dist
            if ( (norm(nodes(i,:) - nodes(j,:)) <= max_dist) && (norm(nodes(i,:) - nodes(j,:)) >= min_dist) )
                % Check if the path between nodes i and j is obstacle-free
                line_points = round([linspace(nodes(i,1), nodes(j,1), 300); linspace(nodes(i,2), nodes(j,2), 300)]');
                valid_points = all(line_points >= 1, 2) & line_points(:,1) <= size(image_map,1) & line_points(:,2) <= size(image_map,2);
                if all(image_map(sub2ind(size(image_map), line_points(valid_points,1), line_points(valid_points,2))) == 1)
                    % If obstacle-free, connect i and j
                    adjacency_matrix(i,j) = 1;
                    adjacency_matrix(j,i) = 1;
                    % Plot the edge between nodes i and j
                    line([nodes(i,2), nodes(j,2)], [nodes(i,1), nodes(j,1)], 'Color', 'k');
                end
            end
        end
    end
    
    %% Finding optimal path using BFS
    
    % Find indices of start and goal nodes in nodes array
    [~, q_start_index] = min(pdist2(nodes, q_start));
    [~, q_goal_index] = min(pdist2(nodes, q_goal));
    
    % BFS initialization
    queue = q_start_index; % Start with the index of the start node
    parent = zeros(1, num_samples);
    visited = false(1, num_samples);
    visited(q_start_index) = true; % Mark start node as visited
    
    % BFS search
    while ~isempty(queue)
        current = queue(1);
        queue(1) = [];
        
        if current == q_goal_index
            % Reconstruct the path from goal to start
            optimal_path = reconstructPath(parent, current);
            % Plot the optimal path
            plotPath(optimal_path, nodes, 'r');
            disp(['Optimal path found at the Attempt number: ', num2str(attempt)]);
            solution_found = true;
            break;
        end
        
        % Explore neighbors (nodes connected to current)
        neighbors = find(adjacency_matrix(current,:) == 1);
        for i = 1:length(neighbors)
            if ~visited(neighbors(i))
                visited(neighbors(i)) = true;
                parent(neighbors(i)) = current;
                queue(end+1) = neighbors(i);
            end
        end
    end
    
    if solution_found
        break;
    end
    
    disp('No path found. Next attempt!');
end

if ~solution_found
    disp(['FAILURE! Optimal path is not found with ', num2str(attempt), ' Attempts.'])
end

%% Function definitions

function path = reconstructPath(parent, goal)
    path = [];
    current = goal;
    while current ~= 0
        path = [current, path];
        current = parent(current);
    end
end

function plotPath(path, nodes, color)
    for i = 1:length(path)-1
        line([nodes(path(i),2), nodes(path(i+1),2)], [nodes(path(i),1), nodes(path(i+1),1)], 'Color', color, 'LineWidth', 2);
    end
end
