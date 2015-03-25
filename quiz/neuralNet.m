clc; clear all; close all;
s = RandStream('mt19937ar','Seed',1);
RandStream.setGlobalStream(s);

%%Load data
gs = readtable('GS.csv');
xdeg = gs.Inp1;
ydrive = gs.Out1;

% Split input data
split = 0.7;
n_samples = length(xdeg);
n_train = ceil(split*n_samples);
n_test = n_samples - n_train;

idx = randperm(n_samples);
trainingSet = table2array(sortrows(gs(idx(1:n_train),:)));
testingSet = table2array(sortrows(gs(idx(n_train+1:n_samples),:)));


%% Neural Nets

%1 - 1 hidden state
net1 = feedforwardnet(1);
net1 = train(net1,trainingSet(:,1)',trainingSet(:,2)');
predictSet1 = net1(testingSet(:,1)');
figure;
scatter(testingSet(:,1),testingSet(:,2));
hold on;
title('1 Hidden Units');
plot(testingSet(:,1),predictSet1,'r');
legend('Testing','Predicted Testing','Location','northwest');

%2 - 10 hidden state
net10 = feedforwardnet(10);
[net10,tr] = train(net10,trainingSet(:,1)',trainingSet(:,2)');
predictSet10 = net10(testingSet(:,1)');
plotperform(tr);
figure;
scatter(testingSet(:,1),testingSet(:,2));
hold on;
title('10 Hidden Units');
plot(testingSet(:,1),predictSet10,'r');
legend('Testing','Predicted Testing','Location','northwest');

RMSE1 = sqrt(mean((testingSet(:,2)' - predictSet1).^2))
RMSE10 = sqrt(mean((testingSet(:,2)' - predictSet10).^2))

%3 - Many hidden units
hU=[1,2,5,10,20,50,100,200];
RMSENtr = zeros(8,1);
RMSEN = zeros(8,1);
i=1;

for n=hU
    netN=feedforwardnet(n);
    netN = train(netN,trainingSet(:,1)',trainingSet(:,2)');
    predictSetN = netN(testingSet(:,1)');
    predictSetNtr = netN(trainingSet(:,1)');
    RMSENtr(i) = sqrt(mean((trainingSet(:,2)' - predictSetNtr).^2));
    RMSEN(i) = sqrt(mean((testingSet(:,2)' - predictSetN).^2));

    graphName = sprintf('%d Hidden Units', n);
    figure;
    scatter(testingSet(:,1),testingSet(:,2));
    hold on;
    title(graphName);
    plot(testingSet(:,1),predictSetN,'r');
    legend('Testing','Predicted Testing','Predicted Training','Location','northwest');

    i=i+1;
end

figure;
semilogx(hU, RMSENtr,'b');
hold on;
semilogx(hU, RMSEN,'r');
title('RMSE vs # of hidden units');
legend('RMSE of training','RMSE of testing','Location','northwest');
