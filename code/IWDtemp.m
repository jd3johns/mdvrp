clc; close all; clear all;
%% Variable initialization
iterations = 30;
% Initial variables
tempInit = 3000; % initial temperature for SA
temp = tempInit;
tempDec = 0.95; % temperature decrease rate
tempIter = 15;    % number of iterations at each step
soilInit = 1000;
velocityInit = 100;
a_v = 1000; a_s = 1000;
b_v = 0.01; b_s = 0.01;
c_v = 1; c_s = 1;


vel_params = [a_v b_v c_v];
soil_params = [a_s b_s c_s];

% Soil updating params
rho_o = 0.9;
rho_n = 1 - rho_o;
% Global soil updating params
rho_s = 0.8;
rho_iwd = -1.5;%1 - rho_s;

% Data sets
problempath = './../data/problems/';
problemfiles = { 'p01', 'p02' };

solutionpath = './../data/solutions/';
solutionfiles = { 'p01.res', 'p02.res' };

%% Load and format data
% Grab the first fileset
[desc, depot_desc, cust, depot] = parseProblemSet(strcat(problempath, problemfiles{1}));
num_depots = desc(4);
num_vehicles = desc(2);
all_coords = [cust(:,2:3); depot(:,2:3)];

% Build adjacency matrix row is source, col is dest
[ distMat globalSoilMat ] = prepareBoard(desc, depot_desc, cust, depot, soilInit);
soilMat = exp(distMat).*globalSoilMat;
soilMat = normalizeSoilMat(soilMat, soilInit);

%% Simulate water drops
% Initialize agents
drops = []; % TODO: Preallocate object array?
for d = 1:num_depots % for each depot
    for v = 1:num_vehicles % populate vehicles
        dep = depot(d,1);
        capacity = depot_desc(d,2);
        drops = [drops; WaterDrop(dep,capacity,velocityInit, ...
            vel_params, soil_params)];
    end
end

% Best solution
best_sol = drops;
best_cost = 0;

temp_counter = 0;

for it = 1:(iterations*tempIter)
    if (temp_counter>=tempIter)
        temp_counter = 0;
        temp = temp*tempDec;
    end
    % Build a full route set with multiple agents
    % Reset the parameters of each water drop on init
    for i = 1:length(drops)
        drops(i) = drops(i).reset(velocityInit);
    end
    
    customers = cust(:,1);
    while ~isempty(customers)
        for i = 1:length(drops)
            if isempty(customers)
                continue;
            end

            % Simulate water drop flow for one agent
            iwd = drops(i);
            iwd = iwd.flow(soilMat,customers);
            % n = length(iwd.route);
            % customers(customers == iwd.route(n)) = [];
            customers(iwd.route(end)) = [];

            % Update water drop
            iwd = iwd.updateVelocity(soilMat);
            iwd = iwd.updateSoil(distMat);
            drops(i) = iwd;

            % Update soil on edges
            dsoil = iwd.deltaSoil(distMat);
            soilMat(iwd.route(n - 1), iwd.route(n)) = ...
                rho_o * soilMat(iwd.route(n - 1), iwd.route(n)) ...
                - rho_n * dsoil;
            soilMat(iwd.route(n), iwd.route(n - 1)) = ...
                rho_o * soilMat(iwd.route(n), iwd.route(n - 1)) ...
                - rho_n * dsoil;
        end% end of looping over agents for allocation
    end % end of customer allocation

    % send agents home
    for i = 1:length(drops)
        drops(i) = drops(i).returnHome();
    end
    % solution construction complete
    
    % Calculate total solution cost (Euclidean distance; sum of routes)
    cost = 0; % cost of the solution
    for i = 1:length(drops)
        dropcost = drops(i).calcRouteCost(distMat);
        cost = cost + dropcost;
    end
    
    % Update the best solution
    if (cost < best_cost) || (best_cost == 0)
        best_cost = cost
        best_sol = drops;
    end
    
    % Update the global soil
    %if (cost < best_cost) || (best_cost == 0) % probablilstically accept worse tings
        %globalSoilMat = soilMat;
    %end
    
    
end % end solution generation

% Compare the route to the solution
[sol_cost sol_routes] = parseSolutionSet(strcat(solutionpath, solutionfiles{1}));

sol_cost
best_cost

% Visualize the best solution
figure();
colours = char('r', 'g', 'm', 'c', 'y', 'k', 'b');
gplot(globalSoilMat,all_coords,'.');
hold on;
for i = 1:length(best_sol)
    r = best_sol(i).route;
    r_coord = all_coords(r,:);
    A = zeros(length(r));
    for j = 2:length(r)
        A(j-1, j) = 1;
    end
    gplot(A, r_coord, colours(mod(i,7)+1));
end
hold off;

edges_use = length(find(globalSoilMat ~= 1000))
