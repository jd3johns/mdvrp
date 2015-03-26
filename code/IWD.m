clc; close all; clear all;
%% Prepare the data for processing
% Initial variables
soilInit = 1000;
velocityInit = 100;
a_v = 1000; a_s = 1000;
b_v = 0.01; b_s = 0.01;
c_v = 1; c_s = 1;

vel_params = [a_v b_v c_v];
soil_params = [a_s b_s c_s];

folderpath = './../data/problems/';
files = { 'p01', 'p02' };


% Grab the first fileset
[desc, depot_desc, cust, depot] = parseProblemSet(strcat(folderpath, files{1}));

% Build adjacency matrix row is source, col is dest
[ distMat soilMat ] = prepareBoard(desc, depot_desc, cust, depot, soilInit)

% Initialize agents
drops = [];
for i = 1:3
    drops = [drops; WaterDrop(depot(1,1),depot_desc(1,2),velocityInit, ...
        vel_params, soil_params)];
end

% Build a full route set with multiple agent
customers = cust(:,1);
while ~isempty(customers)
    for i = 1:length(drops)
        if isempty(customers)
            continue;
        end
        iwd = drops(i);
        iwd = iwd.flow(soilMat,customers);
        l = length(iwd.route);
        customers(customers == iwd.route(l)) = [];
    
        iwd = iwd.updateVelocity(soilMat);
        iwd = iwd.updateSoil(distMat);
        drops(i) = iwd;
    end
end

% Calculate total solution cost (Euclidean distance sum of routes)
cost = 0;
for i = 1:length(drops)
    cost = cost + drops(i).calcRouteCost(distMat);
end
cost
%