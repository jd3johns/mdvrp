% SYDE 522: Quiz 3
clear all; close all; clc;

% Load data
gs = readtable('GS.csv');
xdeg = gs.Inp1;
ydrive = gs.Out1;

figure()
plot(xdeg,ydrive)
title('Human-Generated Robot Control Input')
xlabel('Bearing with Respect to Goal (degrees)')
ylabel('Drive Control')

% Split input data
split = 0.7;
n_samples = length(xdeg);
n_train = ceil(split*n_samples);
n_test = n_samples - n_train;

idx = randperm(n_samples);
train = sortrows(gs(idx(1:n_train),:));
test = sortrows(gs(idx(n_train+1:n_samples),:));

%% Decision Tree

% Train the decision tree
% It's a binary tree split on values of robot orientation (x)
tree = fitrtree(train.Inp1,train.Out1);
view(tree,'mode','graph')

% Predict with the full DT
predtrain = predict(tree,train.Inp1);
predtest = predict(tree,test.Inp1);

% Plot predictions
figure()
plot(train.Inp1,train.Out1,'.')
hold on;
plot(train.Inp1,predtrain,'r-')
title('Decision Tree Prediction of Drive Controls on Training Set')
xlabel('Bearing with Respect to Goal (degrees)')
ylabel('Drive Control')
legend('Training data','Prediction')

figure()
plot(test.Inp1,test.Out1,'.')
hold on;
plot(test.Inp1,predtest,'r-')
title('Decision Tree Prediction of Drive Controls on Test Set')
xlabel('Bearing with Respect to Goal (degrees)')
ylabel('Drive Control')
legend('Test data','Prediction')

% Calculate RMSE values
% RMSE is much lower for predictions of training values.
% This is one example of overfitting to your training data.
rmse_train = sqrt(mean((train.Out1 - predtrain).^2))
rmse_test = sqrt(mean((test.Out1 - predtest).^2))

% Prune the tree
prune_level = tree.PruneList(1) - 10;
prune_tree = prune(tree,'Level',prune_level);
view(prune_tree,'mode','graph')

% Predict with the pruned DT
predtrain = predict(tree,train.Inp1,'SubTrees',prune_level);
predtest = predict(tree,test.Inp1,'SubTrees',prune_level);

% Plot predictions
figure()
plot(train.Inp1,train.Out1,'.')
hold on;
plot(train.Inp1,predtrain,'r-')
title('Pruned Decision Tree Prediction of Drive Controls on Training Set')
xlabel('Bearing with Respect to Goal (degrees)')
ylabel('Drive Control')
legend('Training data','Prediction')

figure()
plot(test.Inp1,test.Out1,'.')
hold on;
plot(test.Inp1,predtest,'r-')
title('Pruned Decision Tree Prediction of Drive Controls on Test Set')
xlabel('Bearing with Respect to Goal (degrees)')
ylabel('Drive Control')
legend('Test data','Prediction')

% Calculate RMSE values
% RMSE is slightly lower for test predictions than the
% full decision tree, but since it doesn't overfit,
% RMSE on the training set is better than the full DT
rmse_train_prune = sqrt(mean((train.Out1 - predtrain).^2))
rmse_test_prune = sqrt(mean((test.Out1 - predtest).^2))

% Observations
% The tree is "steppy" in that it has many discontinuities. This is an
% artifact of the DT as it splits classification on variables.