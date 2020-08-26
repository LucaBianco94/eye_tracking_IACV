%% IMAGE ANALYSIS AND COMPUTER VISION COURSE - AA19/20
% Eye tracking and gaze estimation from facial features
% Luca Bianco - Lorenzo Randazzo

% Heatmap generation script

%% 

files = dir('data/test1.mat'); 
load(files(1).name)


%% 
% remove values out of the screen

c = zeros(size(predsH));
r = zeros(size(predsV));
for ii = 1:length(predsH)
    if(predsH(ii)>0 && predsH(ii)<0.25)
        c(ii) = 1;
        if(predsV(ii)>0 && predsV(ii)<0.25)
            r(ii) = 1;
        elseif(predsV(ii)>0.25 && predsV(ii)<=0.50)
            r(ii) = 2;
        elseif(predsV(ii)>0.50 && predsV(ii)<=0.75)
            r(ii) = 3;
        elseif(predsV(ii)>0.75 && predsV(ii)<=1)
            r(ii) = 4;
        end
    elseif(predsH(ii)>0.25 && predsH(ii)<=0.50)
        c(ii) = 2;
        if(predsV(ii)>0 && predsV(ii)<0.25)
            r(ii) = 1;
        elseif(predsV(ii)>0.25 && predsV(ii)<=0.50)
            r(ii) = 2;
        elseif(predsV(ii)>0.50 && predsV(ii)<=0.75)
            r(ii) = 3;
        elseif(predsV(ii)>0.75 && predsV(ii)<=1)
            r(ii) = 4;
        end
    elseif(predsH(ii)>0.50 && predsH(ii)<=0.75)
        c(ii) = 3;
        if(predsV(ii)>0 && predsV(ii)<0.25)
            r(ii) = 1;
        elseif(predsV(ii)>0.25 && predsV(ii)<=0.50)
            r(ii) = 2;
        elseif(predsV(ii)>0.50 && predsV(ii)<=0.75)
            r(ii) = 3;
        elseif(predsV(ii)>0.75 && predsV(ii)<=1)
            r(ii) = 4;
        end
    elseif(predsH(ii)>0.75 && predsH(ii)<=1)
        c(ii) = 4;
        if(predsV(ii)>0 && predsV(ii)<0.25)
            r(ii) = 1;
        elseif(predsV(ii)>0.25 && predsV(ii)<=0.50)
            r(ii) = 2;
        elseif(predsV(ii)>0.50 && predsV(ii)<=0.75)
            r(ii) = 3;
        elseif(predsV(ii)>0.75 && predsV(ii)<=1)
            r(ii) = 4;
        end
    end
end
xval = {'1','2','3','4'};
yval = {'4','3','2','1'};
A = array2table([r c]);
n11 = find(A.Var1==1 & A.Var2==1);
n12 = find(A.Var1==1 & A.Var2==2);
n13 = find(A.Var1==1 & A.Var2==3);
n14 = find(A.Var1==1 & A.Var2==4);
n21 = find(A.Var1==2 & A.Var2==1);
n22 = find(A.Var1==2 & A.Var2==2);
n23 = find(A.Var1==2 & A.Var2==3);
n24 = find(A.Var1==2 & A.Var2==4);
n31 = find(A.Var1==3 & A.Var2==1);
n32 = find(A.Var1==3 & A.Var2==2);
n33 = find(A.Var1==3 & A.Var2==3);
n34 = find(A.Var1==3 & A.Var2==4);
n41 = find(A.Var1==4 & A.Var2==1);
n42 = find(A.Var1==4 & A.Var2==2);
n43 = find(A.Var1==4 & A.Var2==3);
n44 = find(A.Var1==4 & A.Var2==4);

% set squares values
data_map = [length(n41) length(n42) length(n43) length(n44); ...
            length(n31) length(n32) length(n33) length(n34); ...
            length(n21) length(n22) length(n23) length(n24); ...
            length(n11) length(n12) length(n13) length(n14); ...
            ]; 

% get accuracy 
tot = sum(data_map,'all');
correct = (length(n14)/tot)*100

%plot heatmap
heatFigure = figure(1);
heatFigure.WindowState = 'maximized';
heatFigure.Position = [0 0 1440 900];
heatFigure.Resize = 'off';
heatmap(heatFigure,xval,yval,data_map,'FontSize',0.001,'ColorMap',winter,'ColorbarVisible','off')







