function [ distMat soilMat ] = prepareBoard( desc, depot_desc, cust, depot, soilInit )
%PREPAREBOARD Summary of this function goes here
% Summary:  Function for parsing Cordeau problem data and
%           preparing board state
% 
% desc:     Problem Description
% depot_desc:    Depot Descriptions
% cust:     Customer Descriptions
% depot:    Depot locations
% initSoil: Initial soil Values

locations = [ cust(:, 1:3); depot(:, 1:3) ];
totalNodes = desc(3) + desc(4);
distMat = zeros(totalNodes);

distCalc = @(x1, y1, x2, y2) norm([ x1-x2 y1-y2 ], 'fro');

for i = 1:totalNodes
    for j = 1:totalNodes
        distMat(i, j) = distCalc(locations(i, 2), locations(i, 3), ...
                locations(j, 2), locations(j, 3));
    end
end

soilMat = ones(totalNodes)*soilInit - eye(totalNodes)*soilInit;

end

