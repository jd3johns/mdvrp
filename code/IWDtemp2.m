clc; close all; clear all;


%% Variable initialization
iterations = 30;
pIter = 5; % # of parallel sol'ns calculated to find totalbest

% Initial variables
tempInit = 3000; % initial temperature for SA
temp = tempInit;
tempDec = 0.95; % temperature decrease rate
tempIter = 1%5;    % number of iterations at each step
soilInit = 2000;
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
soilMat = globalSoilMat;
%soilMat = log(distMat+1).*globalSoilMat;
%soilMat = distMat.*globalSoilMat;
%soilMat = exp(distMat).*globalSoilMat;
soilMat = normalizeSoilMat(soilMat, soilInit);
globalSoilMat = soilMat; % globalSoilMat contains the soil going forward


%% Initialize agents
% drops = []; % TODO: Preallocate object array?
%dropstemp = [];
for i = 1:pIter
    dropstemp = [];
    for d = 1:num_depots % for each depot
        for v = 1:num_vehicles % populate vehicles
            dep = depot(d,1);
            capacity = depot_desc(d,2);
            dropstemp =  [ dropstemp; WaterDrop(dep,capacity,velocityInit, ...
                vel_params, soil_params) ];
        end
    end
    drops(:, 1, i) = dropstemp;
end

clear dropstemp;

% Best solution
best_sol = 0;
best_cost = 0;
best_route = 0;

%% Simulate water drops

for it = 1:iterations
    % reset calcualtion variables
    for i = 1:pIter
        soilMat(:, :, i) = globalSoilMat;
        % reset droplets
        for dr = 1:length(drops)
            drops(dr, 1, i) = drops(dr, 1, i).reset(velocityInit);
        end
    end

    for s = 1:pIter % iterate to get parallel solutions and then will compare 
        % Build a full route set with multiple agents
        % Reset the parameters of each water drop on init

        customers = cust(:,1);
        while ~isempty(customers)

            for i = 1:length(drops)
                if isempty(customers)
                    break; % stop processing this loop since allocaiton complete
                end

                % Simulate water drop flow for one agent
                iwd = drops(i, 1, s);
                iwd = iwd.flow(soilMat(:, :, s),customers);
                n = length(iwd.route);
                customers(customers == iwd.route(n)) = [];
                %customers(iwd.route(end)) = [];

                % Update water drop
                iwd = iwd.updateVelocity(soilMat(:, :, s));
                iwd = iwd.updateSoil(distMat);
                drops(i, 1, s) = iwd;

                % Update soil on edges
                dsoil = iwd.deltaSoil(distMat);
                %n = length(iwd.route);
                soilMat(iwd.route(n - 1), iwd.route(n), s) = ...
                    rho_o * soilMat(iwd.route(n - 1), iwd.route(n)) ...
                    - rho_n * dsoil;
                soilMat(iwd.route(n), iwd.route(n - 1), s) = ...
                    rho_o * soilMat(iwd.route(n), iwd.route(n - 1)) ...
                    - rho_n * dsoil;
            end% end of looping over agents for allocation

        end % end of customer allocation
        
        % send agents home

        for i = 1:length(drops)
            drops(i, 1, s) = drops(i, 1, s).returnHome();
        end
    
    end % parallel solution construction
    
    
    parallelCost = zeros(pIter, 1);
    parallelBestCost = 0;
    
    % Calculate total solution cost (Euclidean distance; sum of routes)
    for s = 1:pIter
        dropcost = 0;
        for i = 1:length(drops)
            dropcost = drops(i, 1, s).calcRouteCost(distMat);
            parallelCost(s) = parallelCost(s) + dropcost;
        end
    end
    
    bestIndex = 0;
    for s = 1:pIter
        if parallelCost(s) == min(parallelCost)
            bestIndex = s;
            parallelBestCost = parallelCost(s);
            break;
        end
    end
    
    % Update the best solution
    if (parallelBestCost < best_cost) || (best_cost == 0)
        best_cost = parallelBestCost
        best_sol = drops(:, :, bestIndex);
    end
    
    % Update the global soil
    for n = 1:length(drops)
        drop = drops(n, 1, bestIndex);
        r = drop.route;
        k_N = 1/((length(r) - 2) - 1);
        for i = 2:length(r)
            soilMat(r(i - 1), r(i), bestIndex) = rho_s * soilMat(r(i-1), r(i), bestIndex) ...
                +rho_iwd * k_N * drop.soil;
            soilMat(r(i), r(i-1), bestIndex) = soilMat(r(i-1), r(i), bestIndex);
        end % end droplet splatter
    end % end solution splatter
    
    globalSoilMat = soilMat(:, :, bestIndex);    
    
end % end solution generation

% Compare the route to the solution
[sol_cost sol_routes] = parseSolutionSet(strcat(solutionpath, solutionfiles{1}));

sol_cost
best_cost

%% Visualize the best solution
figure();
colours = char('r', 'g', 'm', 'c', 'k', 'b');
gplot(globalSoilMat,all_coords,'.');
hold on;
for i = 1:length(best_sol)
    r = best_sol(i).route;
    r_coord = all_coords(r,:);
    A = zeros(length(r));
    for j = 2:length(r)
        A(j-1, j) = 1;
    end
    gplot(A, r_coord, colours(mod(i,6)+1));
end
hold off;

edges_use = length(find(globalSoilMat ~= 1000))
