clc; close all; clear all;
%% Variable initialization
iterations = 30;
% Initial variables
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
globalSoilMat = soilMat;

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
best_iwd = drops(1);
best_cost = 0;
best_route = 0;

for it = 1:iterations % TODO: Mark down iteration for convergencce plotting
    % Build a full route set with multiple agent
    customers = cust(:,1);
    while ~isempty(customers)
        for i = 1:length(drops)
            if isempty(customers)
                continue;
            end

            % Simulate water drop flow for one agent
            iwd = drops(i);
            iwd = iwd.flow(soilMat,customers);
            n = length(iwd.route);
            customers(customers == iwd.route(n)) = [];

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
        end
    end

    for i = 1:length(drops)
        drops(i) = drops(i).returnHome();
    end

    % Calculate total solution cost (Euclidean distance; sum of routes)
    cost = 0;
    bestiwd = drops(1);
    for i = 1:length(drops)
        dropcost = drops(i).calcRouteCost(distMat);
        cost = cost + dropcost;
        if dropcost < bestiwd.calcRouteCost(distMat)
            bestiwd = drops(i);
        end
    end
    
    % Update the best solution
    %cost % cost of this solution
    if (cost < best_cost) || (best_cost == 0)
        best_cost = cost
        best_sol = drops;
    end
    
    % Update the global soil
    current_best_route = bestiwd.calcRouteCost(distMat);
    if (current_best_route < best_route) || (best_route == 0)
        best_route = current_best_route;
        best_iwd = bestiwd;
        for n = 1:length(drops)
            drop = drops(n);
            r = drop.route;
            k_N = 1/((length(r) - 2) - 1);
            globalSoilMat = soilMat;
            for i = 2:length(r)
                globalSoilMat(r(i - 1), r(i)) = rho_s * globalSoilMat(r(i - 1), r(i)) ...
                    + rho_iwd * k_N * drop.soil;
            end
        end
            soilMat = globalSoilMat;
    end
    
    % Reset the parameters of each water drop on init
    for i = 1:length(drops)
        drops(i) = drops(i).reset(velocityInit);
    end
end

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