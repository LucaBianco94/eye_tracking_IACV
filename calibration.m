runLoop = true;
firstTime = true;
recordingCounter = 1;
calibrationCounter = 1;
hPredictors = zeros(10,3);
vPredictors = zeros(10,2);
calibrationPoints=[0 0.5; 0.25 0.5; 0.5 0.5; 0.75 0.5; 1 0.5; 0.5 0; 0.5 0.25; 0.5 0.5; 0.5 0.75; 0.5 1];
while runLoop
     
        % --- HORIZONTAL DIRECTION FEATURES --- %
     
     % horizontal (H) distance between each eye center (EC) and the respective
     % ipsilateral anchor point (AC) 
     leftECACHdistance = norm(leftCenterTracked(1,1)-leftCenterInEntireFrame(1,1));
     rightECACHdistance = norm(rightCenterTracked(1,1)-rightCenterInEntireFrame(1,1));
     
     % horizontal (H) distance of the right eye center and the left anchor 
     % point at the opposite side 
     rightECleftACHdistance = norm(rightCenterTracked(1,1)-leftCenterInEntireFrame(1,1));
     
        % --- VERTICAL DIRECTION FEATURES --- %
     
     % vertical (V) distances between the y-coordinates of each of the eye centers and 
     % the respective anchor points
     leftECACVdistance = norm(leftCenterTracked(1,2)-leftCenterInEntireFrame(1,2));
     rightECACVdistance = norm(rightCenterTracked(1,2)-rightCenterInEntireFrame(1,2));
     
     
     
     if ~calibrationDone
         %%---------------------------- CALIBRATION -----------------------------%%
           
         % The first iteration that we are in calibration  
         if firstTime

            disp('Starting Calibration Mode...')
             
            % Draw the calibration figure 
            calibFigure = figure;
            calibFigure.ToolBar = 'none';
            calibFigure.MenuBar = 'none';
            calibFigure.WindowState = 'maximized';
            calibFigure.Units = 'Normalized';
            calibFigure.Position = [0 0 1 1];
            calibFigure.Resize = 'off';
            
            ax = gca;
            ax.Position = [0.1 0.1 0.8 0.8];
            ax.XLim = [0 1];
            ax.YLim = [0 1];
            ax.Visible = 'off';
            hold on
            
            plot(calibrationPoints(:,1), calibrationPoints(:,2),'k.','MarkerSize',30);
         end

         
         if recordingCounter < 4 
             if recordingCounter == 1
                 hold on
                 % color the active cirle red
                 plot(calibrationPoints(calibrationCounter,1),calibrationPoints(calibrationCounter,2),'r.','MarkerSize',35);
             end
             % make one of the three bips
             sound(bip1);
             pause(0.7)
             
             % allow recording after three bips
             recordingCounter = recordingCounter + 1;
         else   
             % final bip
             sound(bip2);
             % save predictors values
             hPredictors(calibrationCounter, :) = [leftECACHdistance, rightECACHdistance, rightECleftACHdistance];
             vPredictors(calibrationCounter, :) = [leftECACVdistance, rightECACVdistance];
             % color the circle green
             plot(calibrationPoints(calibrationCounter,1),calibrationPoints(calibrationCounter,2),'g.','MarkerSize',30);
             % wait a second
             pause(1)
             
             calibrationCounter = calibrationCounter + 1;
             recordingCounter = 1;
             
             if spacialContinuityCorrectionCounter > 0 && checkCorruptedCalibration
                % Clean up
                clear cam
                if debugImage, release(videoPlayer); end
                release(detectorFace);
                release(leftPointTracker);
                release(rightPointTracker);
                
                errordlg('Calibration corrupted! Please do not close eyelids during calibration phase and make sure ambient light condition is constant. Restart the program.')
            
                error('Calibration corrupted!')
             end
             
         end
         
         if calibrationCounter > 10
             calibrationDone = true;
             
             % --- LINEAR REGRESSION MODEL --- %
             Xh = [ones(size(hPredictors, 1), 1) hPredictors(:,1) hPredictors(:,2) hPredictors(:,3)];
             Xv = [ones(size(vPredictors, 1), 1) vPredictors(:,1) vPredictors(:,2)];
             
             Yh = calibrationPoints(:,1);
             Yv = calibrationPoints(:,2);
             
             % horizontal model parameters
             H = regress(Yh,Xh);
             % vertical model parameters
             V = regress(Yv,Xv);
         end
         
     else
         %%---------------------------- PREDICTION -----------------------------%%
       
        % Set screen the first time 
        if ~testWindowDrawn
           disp('Starting Final Test...')
           
           % Draw the prediction figure
           testFigure = figure;
           testFigure.ToolBar = 'none';
           testFigure.MenuBar = 'none';
           testFigure.WindowState = 'maximized';
           testFigure.Units = 'Normalized';
           testFigure.Position = [0 0 1 1];
           testFigure.Resize = 'off';
           testFigure.CloseRequestFcn = @my_closereq;

           
           ax = gca;
           ax.Position = [0.1 0.1 0.8 0.8];
           ax.XLim = [0 1];
           ax.YLim = [0 1];
           
           axis off
           
           % Insert end of page
           rectangle('Position',[0.75,0.175,0.1,0.15],'FaceColor','red')
           
           % Insert text
           text('String', 'Read this text to let the program', 'FontUnits', 'normalized', 'FontSize', 0.07, 'Position', [0.5 0.75], 'HorizontalAlignment', 'center')
           text('String', 'estimate your eyes gaze and check', 'FontUnits', 'normalized', 'FontSize', 0.07, 'Position', [0.5 0.5], 'HorizontalAlignment', 'center')
           text('String', 'if you reached the end of page or not.', 'FontUnits', 'normalized', 'FontSize', 0.07, 'Position', [0.5 0.25], 'HorizontalAlignment', 'center')
           hold on 
           
           testWindowDrawn = true;
        end
        
        % Estimates of the two outputs with the model found in calibration
        predH=sum([1, leftECACHdistance, rightECACHdistance, rightECleftACHdistance].*H');
        predV=sum([1, leftECACVdistance, rightECACVdistance].*V');
        
        % Plot the estimated screen point
        plot(predH,predV, 'b*','MarkerSize',3);
        
        % If the estimated gaze is inside the End of Page rectangle 5
        % times the program stops
        if predV>=0.175 && predV<=0.325 && predH>=0.75 && predH<=0.85  
            endOfPageCounter = endOfPageCounter+1;
            if endOfPageCounter == 3
                disp('End of page reached. Program closes.')
                rectangle('Position',[0.75,0.175,0.1,0.15],'FaceColor','green')
                text('String', 'End of page reached!', 'FontUnits', 'normalized', 'FontSize', 0.05, 'Position', [0.7 0.1], 'HorizontalAlignment', 'center')
                pause(3);
                runLoop = false;
                continue;
            end
        end
        
     end
     if firstTime
        firstTime = false;
     end

end
     
     