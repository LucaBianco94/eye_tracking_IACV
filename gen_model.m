%% IMAGE ANALYSIS AND COMPUTER VISION COURSE - AA19/20
% Eye tracking and gaze estimation from facial features
% Luca Bianco - Lorenzo Randazzo

% Model generation script


%% initialization
close all
clear
clc

%% preparation
files = dir('data/dataset*');
load(files(1).name)
if size(files,1)>1
    for ii=2:size(files,1)
        L = load(files(ii).name);
        calibrationTargets = [calibrationTargets;L.calibrationTargets];
        horizontal_l = [horizontal_l;L.horizontal_l];
        horizontal_r = [horizontal_r;L.horizontal_r];
        vertical_l = [vertical_l;L.vertical_l];
        vertical_r = [vertical_r;L.vertical_r];
    end
end

%% regression
Xh_l = [ones(size(horizontal_l, 1), 1) horizontal_l(:,1) horizontal_l(:,2) horizontal_l(:,3)];
Xh_r = [ones(size(horizontal_r, 1), 1) horizontal_r(:,1) horizontal_r(:,2) horizontal_r(:,3)];
Xv_l = [ones(size(vertical_l, 1), 1) vertical_l(:,1) vertical_l(:,2) vertical_l(:,3)];
Xv_r = [ones(size(vertical_r, 1), 1) vertical_r(:,1) vertical_r(:,2) vertical_r(:,3)];
Yh = calibrationTargets(:,1);
Yv = calibrationTargets(:,2);
             
% horizontal model parameters
H_l = regress(Yh,Xh_l);
H_r = regress(Yh,Xh_r);
% vertical model parameters
V_l = regress(Yv,Xv_l);
V_r = regress(Yv,Xv_r);

%% results
predH_l = zeros(size(calibrationTargets,1),1);
predH_r = zeros(size(calibrationTargets,1),1);
predV_l = zeros(size(calibrationTargets,1),1);
predV_r = zeros(size(calibrationTargets,1),1);
for ii = 1:size(calibrationTargets,1)
    predH_l(ii) = [1, horizontal_l(ii,:)]*H_l;
    predH_r(ii) = [1, horizontal_r(ii,:)]*H_r;
    predV_l(ii) = [1, vertical_l(ii,:)]*V_l;
    predV_r(ii) = [1, vertical_r(ii,:)]*V_r;
end
predH=mean([predH_l predH_r],2);
predV=mean([predV_l predV_r],2);

subplot(211)
plot(Yh,'k'), hold on, plot(predH_l,'r:'), plot(predH_r,'b:'), plot(predH,'m') 
title('horizontal ax'), legend('real','predicted_l', 'predicted_r')
subplot(212)
plot(Yv,'k'), hold on, plot(predV_l,'r:'), plot(predV_r,'b:'), plot(predV,'m')
title('vertical ax'), legend('real','predicted_l','predicted_r')

%% closing and saving
clearvars -except H_l H_r V_l V_r
save('data/regressionModel.mat')