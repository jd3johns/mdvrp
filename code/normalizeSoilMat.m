function [ normalizedMat ] = normalizeSoilMat( soilMat, normalizer )
%NORMALIZESOILMAT Summary of this function goes here
% Summary:  Function for normalizing amount of soil in matrix, so soil
% total is equivalent to if it was initialized to the normalizer amount.
% 
% soilMat:     Soil Matrix
% normalizer:  Normalize this as if this amount was in all non-zero cells

% this normalizes by doing w_i (K/W) = k_i where w_i and k_i are elements
% in the un-normalized and normalized matrices respectively, and W and K
% is the total amount of soil in each of the matrices

arc_count = sum(sum(soilMat ~= 0));
K = normalizer*arc_count;
W = sum(sum(soilMat));

normalizedMat = soilMat .* (K/W);

end

