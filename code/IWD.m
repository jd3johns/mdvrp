%% Prepare the data for processing
% initial variables
soilInit = 1000;

folderpath = './../data/problems/';
files = { 'p01', 'p02' };


% grab the first fileset
[desc, depot_desc, cust, depot] = parseProblemSet(strcat(folderpath, files{1}));

% build adjacency matrix row is source, col is dest

[ distMat soilMat ] = prepareBoard(desc, depot_desc, cust, depot, soilInit)

%