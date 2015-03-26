%% Prepare the data for processing
folderpath = './../data/problems/';
files = { 'p01', 'p02' };

% grab the first fileset
[desc, depot_desc, cust, depot] = parseProblemSet(strcat(folderpath, files{1}));

% build adjacency matrix row is source, col is dest

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

%