%% initialization
close all
clear
clc

%% preparation
files_v = dir('data/dataset_v_*'); 
files_h = dir('data/dataset_h_*'); 

T_v =[];
if size(files_v,1)>1
    for ii=1:size(files_v,1)
        L_v = readtable(files_v(ii).name);
        T_v = [T_v; L_v];
    end
end
T_h =[];
if size(files_h,1)>1
    for ii=1:size(files_h,1)
        L_h = readtable(files_h(ii).name);
        T_h = [T_h; L_h];
    end
end
%% 

Xh = [table2array(T_h(:,2)) table2array(T_h(:,3)) ...
    table2array(T_h(:,4)) table2array(T_h(:,5)) table2array(T_h(:,6))];
Xv = [table2array(T_v(:,2)) table2array(T_v(:,3)) ...
      table2array(T_v(:,4)) table2array(T_v(:,5)) table2array(T_v(:,6)) ...
      table2array(T_v(:,7)) table2array(T_v(:,8)) table2array(T_v(:,9))];
Yh = table2array(T_h(:,1));
Yv = table2array(T_v(:,1));
             
% horizontal model parameters
H = regress(Yh,Xh);
% vertical model parameters
V = regress(Yv,Xv);

