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

% Initialize a single agent
iwd = WaterDrop(depot(1,1),depot_desc(1,2),velocityInit, ...
    vel_params, soil_params)

% Build a full route with one agent
customers = cust(:,1);
while ~isempty(customers)
    iwd = iwd.flow(soilMat,customers);
    l = length(iwd.route);
    customers(customers == iwd.route(l)) = [];
end
iwd.route
%