%
%%
% Gaze Tracker from facial features
% Pasquale Enrico Cortese

% Calibration version

%% initialization
close all
clear
clc
%%
% Add path
addpath('functions')

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();
leftEyeDetector = vision.CascadeObjectDetector('LeftEye');
rightEyeDetector = vision.CascadeObjectDetector('RightEye');
mouthDetector = vision.CascadeObjectDetector('Mouth');
noseDetector = vision.CascadeObjectDetector('Nose');

% Create the point tracker object.
left_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
right_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
leftAP_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
rightAP_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
left_eyelide_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
right_eyelide_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
mouth_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
nose_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [930 430 678 527]);
runLoop = true;
numPts_left = 0;
numPts_right = 0;
testWindowDrawn = false;
numPts_mouth = 0;
numPts_leftAP = 0;
numPts_rightAP = 0;
numPts_nose = 0;
numPts_left_eyelide = 0;
numPts_right_eyelide = 0;
frameCount = 0;
numPts_threshold = 1;
hWidthEye = 0.15;
hHeightEye = 0.05;
hHeightInt = 0.6;
hWidthInt = 0.25;
amp = 0.25;
fs = 250;  % sampling frequency
duration = 0.5;
freq = 280;
val = 0:1/fs:duration;
bip1 = amp*sin(2*pi* freq*val);
bip2 = amp*sin(2*pi* 2*freq*val);
calibrationDone = false;
firstTime = true;
recordingCounter = 1;
calibrationCounter = 1;
hPredictors = zeros(10,3);
vPredictors = zeros(10,2);
calibrationPoints=[0 0.5; 0.25 0.5; 0.5 0.5; 0.75 0.5; 1 0.5; 0.5 0; 0.5 0.25; 0.5 0.5; 0.5 0.75; 0.5 1];
    
%% loop
while runLoop && frameCount < 5000

    % Get the next frame.
    videoFrame = flip(snapshot(cam),2);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;
    
    if numPts_left < numPts_threshold || numPts_right < numPts_threshold ...
            || numPts_mouth < 2 || numPts_leftAP < numPts_threshold ...
            || numPts_rightAP < numPts_threshold || numPts_nose < numPts_threshold
        %% Detection mode.
        
        
        bboxFace = faceDetector.step(videoFrameGray);
        
        if ~isempty(bboxFace)
            bboxFace = findBiggerRect(bboxFace);
            
            %Roi parameters
            start_x = bboxFace(1);
            start_y = ceil(bboxFace(2)+bboxFace(4)/5);
            half_x = ceil(bboxFace(1)+bboxFace(3)/2);
            half_y = ceil(bboxFace(2)+bboxFace(4)/2);
            quarter_x = half_x-bboxFace(3)/4;
            threequarter_x = half_x+bboxFace(3)/4;
            threequarter_y = half_y+bboxFace(4)/4;
            secondthird_y = bboxFace(2)+2*bboxFace(4)/3;
            
            
            %NOSE DETECTION
            if (numPts_nose < numPts_threshold)
                bbox_nose = noseDetector.step(videoFrameGray);
                bbox_nose = findBiggerRect(bbox_nose);
                nose_coord = zeros(1);
                nose_coord(1) = (bbox_nose(1) + bbox_nose(3)/2);
                nose_coord(2) = (bbox_nose(2) + bbox_nose(4)/2);
            
            
                xyPoints_nose = nose_coord;
                numPts_nose = size(xyPoints_nose,1);
                release(nose_pointTracker);
                initialize(nose_pointTracker,nose_coord,videoFrameGray);
                oldPoints_nose = xyPoints_nose;
                bboxPoints_nose = bbox2points(bbox_nose(1, :));
        
            end
            %MOUTH DETECTION
            if (half_y<=frameSize(1) && numPts_mouth < 2)
                mouthROI = videoFrame(secondthird_y:secondthird_y+bboxFace(4)/3,...
                    quarter_x:threequarter_x);
                bbox_mouth = mouthDetector.step(mouthROI);
                if ~isempty(bbox_mouth)
                    bbox_mouth = findBiggerRect(bbox_mouth);
                    bbox_mouth(1:2) = bbox_mouth(1:2)+[quarter_x,secondthird_y];
                    bbox_mouth(1) = bbox_mouth(1)-10;
                    bbox_mouth(3) = bbox_mouth(3)+15;
                    mouthImage = videoFrame(bbox_mouth(2):bbox_mouth(2)+bbox_mouth(4),...
                        bbox_mouth(1):bbox_mouth(1)+bbox_mouth(3),:);
                    mouthCorners = findMouthCorners(mouthImage);
                    mouthCorners = mouthCorners+[bbox_mouth(1:2);bbox_mouth(1:2)];
                    
                    % Find corner points inside the detected region.

                    % Re-initialize the point tracker.
                    xyPoints_mouth = mouthCorners;
                    numPts_mouth = size(xyPoints_mouth,1);
                    release(mouth_pointTracker);
                    initialize(mouth_pointTracker, xyPoints_mouth, videoFrameGray);

                    % Save a copy of the points.
                    oldPoints_mouth = xyPoints_mouth;

                    % Convert the rectangle represented as [x, y, w, h] into an
                    % M-by-2 matrix of [x,y] coordinates of the four corners. This
                    % is needed to be able to transform the bounding box to display
                    % the orientation of the face.
                    bboxPoints_mouth = bbox2points(bbox_mouth(1, :));
                    
                else
                    continue
                end
               
           
            end
            
            %LEFT EYE DETECTION
            if (half_y<=frameSize(1)) && (half_x<=frameSize(2) ...
                   && (numPts_left < numPts_threshold || numPts_right < numPts_threshold ...
            || numPts_leftAP < numPts_threshold || numPts_rightAP < numPts_threshold))
            
                leftEyeROI = videoFrameGray(start_y:half_y,bboxFace(1):half_x);
                bbox_eye_left = leftEyeDetector.step(leftEyeROI);
        
                if ~isempty(bbox_eye_left)
                    bbox_eye_left = findBiggerRect(bbox_eye_left);
                    bbox_eye_left(1:2) = bbox_eye_left(1:2)+[start_x,start_y];
                    leftEyeImage = videoFrame(bbox_eye_left(2):bbox_eye_left(2)+bbox_eye_left(4),...
                        bbox_eye_left(1):bbox_eye_left(1)+bbox_eye_left(3),:);
                    eyeCenter_left = eyecenter_loc(leftEyeImage);
                    eyeCenter_left = eyeCenter_left+bbox_eye_left(1:2);

                    % Re-initialize the point tracker.
                    xyPoints_left = eyeCenter_left;
                    numPts_left = size(xyPoints_left,1);
                    release(left_pointTracker);
                    initialize(left_pointTracker, xyPoints_left, videoFrameGray);

                    % Save a copy of the points.
                    oldPoints_left = xyPoints_left;

                    % Convert the rectangle represented as [x, y, w, h] into an
                    % M-by-2 matrix of [x,y] coordinates of the four corners. This
                    % is needed to be able to transform the bounding box to display
                    % the orientation of the face.
                    bboxPoints_left = bbox2points(bbox_eye_left(1, :));

                else
                    continue
                end
             
            end
            
            %RIGHT EYE DETECTION AND LEFTAP AND RIGHT AP
            if (half_y<=frameSize(1)) && (half_x<=frameSize(2)) && (bboxFace(1)+bboxFace(3)<=frameSize(2) ...
                    && (numPts_left < numPts_threshold || numPts_right < numPts_threshold ...
                    || numPts_leftAP < numPts_threshold || numPts_rightAP < numPts_threshold))
                
                
                rightEyeROI = videoFrameGray(start_y:half_y,half_x:bboxFace(1)+bboxFace(3));
                bbox_eye_right = rightEyeDetector.step(rightEyeROI);
        
                if ~isempty(bbox_eye_right)
                    bbox_eye_right = findBiggerRect(bbox_eye_right);
                    bbox_eye_right(1:2) = bbox_eye_right(1:2)+[half_x,start_y];
                    rightEyeImage = videoFrame(bbox_eye_right(2):bbox_eye_right(2)+bbox_eye_right(4),...
                        bbox_eye_right(1):bbox_eye_right(1)+bbox_eye_right(3),:);
                    eyeCenter_right = eyecenter_loc(rightEyeImage);
                    eyeCenter_right = eyeCenter_right+bbox_eye_right(1:2);
                    
                    
                    % Re-initialize the point tracker.
                    xyPoints_right = eyeCenter_right;
                    numPts_right = size(xyPoints_right,1);
                    release(right_pointTracker);
                    initialize(right_pointTracker, xyPoints_right, videoFrameGray);

                    % Save a copy of the points.
                    oldPoints_right = xyPoints_right;

                    % Convert the rectangle represented as [x, y, w, h] into an
                    % M-by-2 matrix of [x,y] coordinates of the four corners. This
                    % is needed to be able to transform the bounding box to display
                    % the orientation of the face.
                    bboxPoints_right = bbox2points(bbox_eye_right(1, :));
                    
                    [ioc_dist,theta] = crossEyeCenters(eyeCenter_left,eyeCenter_right);
                    leftEyelid = findEyeLids(videoFrame,eyeCenter_left,ioc_dist,theta);
                    rightEyelid = findEyeLids(videoFrame,eyeCenter_right,ioc_dist,theta);
                    % end of eyelid detection
                    
                    xyPoints_left_eyelide = leftEyelid;
                    numPts_left_eyelide = size(xyPoints_left_eyelide,1);
                    release(left_eyelide_pointTracker);
                    initialize(left_eyelide_pointTracker, xyPoints_left_eyelide, videoFrameGray);

                    % Save a copy of the points.
                    oldPoints_left_eyelide = xyPoints_left_eyelide;
                    
                    xyPoints_right_eyelide = rightEyelid;
                    numPts_right_eyelide = size(xyPoints_right_eyelide,1);
                    release(right_eyelide_pointTracker);
                    initialize(right_eyelide_pointTracker, xyPoints_right_eyelide, videoFrameGray);

                    % Save a copy of the points.
                    oldPoints_right_eyelide = xyPoints_right_eyelide;
                    
                    % left anchor point detection
                    leftAnchorROI = floor([eyeCenter_left(1)+hWidthEye*bbox_eye_left(3),...
                    eyeCenter_left(2)-hHeightEye*bbox_eye_left(3)-hHeightInt*ioc_dist/2,...
                    hWidthInt*ioc_dist, hHeightInt*ioc_dist]);
                    leftAnchorPoint = [leftAnchorROI(1)+leftAnchorROI(3)/2,leftAnchorROI(2)+leftAnchorROI(4)/2];

                    % Re-initialize the point tracker.
                    xyPoints_leftAP = leftAnchorPoint;
                    numPts_leftAP = size(xyPoints_leftAP,1);
                    release(leftAP_pointTracker);
                    initialize(leftAP_pointTracker, xyPoints_leftAP, videoFrameGray);

                    % Save a copy of the points.
                    oldPoints_leftAP = xyPoints_leftAP;

                    % Convert the rectangle represented as [x, y, w, h] into an
                    % M-by-2 matrix of [x,y] coordinates of the four corners. This
                    % is needed to be able to transform the bounding box to display
                    % the orientation of the face.
                    bboxPoints_leftAP = bbox2points(leftAnchorROI(1, :));
                    % end of left anchor point detection 

                    % right anchor point detection
                    rightAnchorROI = floor([eyeCenter_right(1)-hWidthEye*bbox_eye_right(3)-hWidthInt*ioc_dist,...
                        eyeCenter_right(2)-hHeightEye*bbox_eye_right(3)-hHeightInt*ioc_dist/2,...
                        hWidthInt*ioc_dist, hHeightInt*ioc_dist]);
                    rightAnchorPoint = [rightAnchorROI(1)+rightAnchorROI(3)/2,rightAnchorROI(2)+rightAnchorROI(4)/2];


                    % Re-initialize the point tracker.
                    xyPoints_rightAP = rightAnchorPoint;
                    numPts_rightAP = size(xyPoints_rightAP,1);
                    release(rightAP_pointTracker);
                    initialize(rightAP_pointTracker, xyPoints_rightAP, videoFrameGray);

                    % Save a copy of the points.
                    oldPoints_rightAP = xyPoints_rightAP;

                    % Convert the rectangle represented as [x, y, w, h] into an
                    % M-by-2 matrix of [x,y] coordinates of the four corners. This
                    % is needed to be able to transform the bounding box to display
                    % the orientation of the face.
                    bboxPoints_rightAP = bbox2points(rightAnchorROI(1, :));
                else
                    continue
                end
                
          
            end
            %end of right eye detection
            
    
        
            % eyelid detection
            
            % end of right anchor point detection 
       
        end
        %END OF DETECTION
    else
        %% Tracking mode
        %MOUTH TRACKING
        [xyPoints_mouth, isFound_mouth] = step(mouth_pointTracker, videoFrameGray);
        visiblePoints_mouth = xyPoints_mouth(isFound_mouth, :);
        oldInliers_mouth = oldPoints_mouth(isFound_mouth, :);
        oldPoints_mouth = visiblePoints_mouth;
        if ~isempty(oldPoints_mouth)
            setPoints(mouth_pointTracker, oldPoints_mouth);
        end
        
        numPts_mouth = size(visiblePoints_mouth, 1);
        % end of mouth  tracking
        
        %LEFT EYELIDE TRACKING
        [xyPoints_left_eyelide, isFound_left_eyelide] = step(left_eyelide_pointTracker, videoFrameGray);
        visiblePoints_left_eyelide = xyPoints_left_eyelide(isFound_left_eyelide, :);
        oldInliers_left_eyelide = oldPoints_left_eyelide(isFound_left_eyelide, :);
        oldPoints_left_eyelide = visiblePoints_left_eyelide;
        if ~isempty(oldPoints_left_eyelide)
            setPoints(left_eyelide_pointTracker, oldPoints_left_eyelide);
        end
        
        numPts_left_eyelide = size(visiblePoints_left_eyelide, 1);
        
        
        %RIGHT EYELIDE TRACKING
        [xyPoints_right_eyelide, isFound_right_eyelide] = step(right_eyelide_pointTracker, videoFrameGray);
        visiblePoints_right_eyelide = xyPoints_right_eyelide(isFound_right_eyelide, :);
        oldInliers_right_eyelide = oldPoints_right_eyelide(isFound_right_eyelide, :);
        oldPoints_right_eyelide = visiblePoints_right_eyelide;
        if ~isempty(oldPoints_right_eyelide)
            setPoints(right_eyelide_pointTracker, oldPoints_right_eyelide);
        end
        
        numPts_right_eyelide = size(visiblePoints_right_eyelide, 1);
        
        
        %NOSE TRACKING
        [xyPoints_nose,isFound_nose] = step(nose_pointTracker,videoFrameGray); 
        visiblePoints_nose = xyPoints_nose(isFound_nose, :);
        oldInliers_nose = oldPoints_nose(isFound_nose, :);
        oldPoints_nose = visiblePoints_nose;
        if ~isempty(oldPoints_nose)
            setPoints(nose_pointTracker, oldPoints_nose);
        end
        
        numPts_nose = size(visiblePoints_nose, 1);
        
        %LEFT EYE TRACKING
        [xyPoints_left, isFound_left] = step(left_pointTracker, videoFrameGray);
        visiblePoints_left = xyPoints_left(isFound_left, :);
        oldInliers_left = oldPoints_left(isFound_left, :);

        numPts_left = size(visiblePoints_left, 1);
        oldPoints_left = visiblePoints_left;
        if ~isempty(oldPoints_left)
            setPoints(left_pointTracker, oldPoints_left);
        end
        
        % end of left eye tracking
        
        % RIGHT EYE TRACKING
        [xyPoints_right, isFound_right] = step(right_pointTracker, videoFrameGray);
        visiblePoints_right = xyPoints_right(isFound_right, :);
        oldInliers_right = oldPoints_right(isFound_right, :);

        numPts_right = size(visiblePoints_right, 1);
        oldPoints_right = visiblePoints_right;
        
        if ~isempty(oldPoints_right)
            setPoints(right_pointTracker, oldPoints_right);
        end
        
        % end of right eye tracking
        
        % eyelid tracking
        [ioc_dist,theta] = crossEyeCenters(eyeCenter_left,eyeCenter_right);
        leftEyelid = findEyeLids(videoFrame,eyeCenter_left,ioc_dist,theta);
        rightEyelid = findEyeLids(videoFrame,eyeCenter_right,ioc_dist,theta);
        % end of eyelid tracking
        
        % left anchor point tracking
        [xyPoints_leftAP, isFound_leftAP] = step(leftAP_pointTracker, videoFrameGray);
        visiblePoints_leftAP = xyPoints_leftAP(isFound_leftAP, :);
        oldInliers_leftAP = oldPoints_leftAP(isFound_leftAP, :);
        
        numPts_leftAP = size(visiblePoints_leftAP, 1);
        oldPoints_leftAP = visiblePoints_leftAP;
        if ~isempty(oldPoints_leftAP)
            setPoints(leftAP_pointTracker, oldPoints_leftAP);
        end
        
        % end of left anchor point tracking
        
        % right anchor point tracking
        [xyPoints_rightAP, isFound_rightAP] = step(rightAP_pointTracker, videoFrameGray);
        visiblePoints_rightAP = xyPoints_rightAP(isFound_rightAP, :);
        oldInliers_rightAP = oldPoints_rightAP(isFound_rightAP, :);
        oldPoints_rightAP = visiblePoints_rightAP;
        if ~isempty(oldPoints_rightAP)
             setPoints(rightAP_pointTracker, oldPoints_rightAP);
        end
       
        numPts_rightAP = size(visiblePoints_rightAP, 1);
        
        %end of right anchor point tracking

    end
    % end of tracking
    

    
    %----------------START CALIBRATION--------------------%
   
    % --- HORIZONTAL DIRECTION FEATURES --- %
     
    % horizontal (H) distance between each eye center (EC) and the respective
    % ipsilateral anchor point (AC) 
    leftECACHdistance = norm(xyPoints_left(1,1)-xyPoints_leftAP(1,1));
    rightECACHdistance = norm(xyPoints_right(1,1)-xyPoints_rightAP(1,1));
     
    % horizontal (H) distance of the right eye center and the left anchor 
    % point at the opposite side 
    rightECleftACHdistance = norm(xyPoints_left(1,1)-xyPoints_right(1,1));
     
    % --- VERTICAL DIRECTION FEATURES --- %
     
    % vertical (V) distances between the y-coordinates of each of the eye centers and 
    % the respective anchor points
    leftECACVdistance = norm(xyPoints_left(1,2)-xyPoints_left_eyelide(1,2));
    rightECACVdistance = norm(xyPoints_right(1,2)-xyPoints_right_eyelide(1,2));
    
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
        if firstTime
            firstTime = false;
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
    
    
    
     
    %--------------------END CALIBRATION----------------------%
   
    % Display tracked points.
    videoFrame = insertMarker(videoFrame, xyPoints_mouth, '+', 'Color', 'cyan');
    videoFrame = insertMarker(videoFrame, xyPoints_left, '*', 'Color', 'yellow');
    videoFrame = insertMarker(videoFrame, xyPoints_right, '*', 'Color', 'yellow');
    videoFrame = insertMarker(videoFrame, xyPoints_left_eyelide, '+', 'Color', 'white');
    videoFrame = insertMarker(videoFrame, xyPoints_right_eyelide, '+', 'Color', 'white');
    videoFrame = insertMarker(videoFrame, xyPoints_leftAP, '+', 'Color', 'cyan');
    videoFrame = insertMarker(videoFrame, xyPoints_rightAP, '+', 'Color', 'cyan');
    videoFrame = insertMarker(videoFrame, xyPoints_nose, '+', 'Color', 'white');
    
    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);

end

%% Clean up
release(videoPlayer);
release(left_pointTracker);
release(right_pointTracker);
release(leftAP_pointTracker);
release(rightAP_pointTracker);
release(mouth_pointTracker);
release(nose_pointTracker);
release(left_eyelide_pointTracker);
release(right_eyelide_pointTracker);
release(faceDetector);
release(leftEyeDetector);
release(rightEyeDetector);
release(mouthDetector);
release(noseDetector);
close all
calibrationTargets = calibrationTargets(1:end-1,:);
clearvars -except horizontal vertical calibrationTargets videoMontage

%% save video
if isempty(dir('C:/users/bianc/Google Drive/LM Milano 18_19/III Semestre/Image Analysis and Computer Vision/Project/previous years/eyeTracking-gazeEstimation-master/video/tracking*'))
    fileName = 'C:/users/bianc/Google Drive/LM Milano 18_19/III Semestre/Image Analysis and Computer Vision/Project/previous years/eyeTracking-gazeEstimation-master/video/tracking1.avi';
else
    files = dir('C:/users/bianc/Google Drive/LM Milano 18_19/III Semestre/Image Analysis and Computer Vision/Project/previous years/eyeTracking-gazeEstimation-master/video/tracking*');
    fileName = files(end).name;
    ord = str2double(fileName(9));
    ord = ord+1;
    fileName = append('C:/users/bianc/Google Drive/LM Milano 18_19/III Semestre/Image Analysis and Computer Vision/Project/previous years/eyeTracking-gazeEstimation-master/video/',fileName(1:8),num2str(ord),fileName(10:end));
end
outputVideo = VideoWriter(fileName);
outputVideo.FrameRate = 10;
open(outputVideo)
for ii = 1:size(videoMontage,4)
   img = uint8(videoMontage(:,:,:,ii));
   writeVideo(outputVideo,img)
end
close(outputVideo)
clearvars -except horizontal vertical calibrationTargets

%% saving
if isempty(dir('C:/users/bianc/Google Drive/LM Milano 18_19/III Semestre/Image Analysis and Computer Vision/Project/previous years/eyeTracking-gazeEstimation-master/data/dataset*'))
    save('C:/users/bianc/Google Drive/LM Milano 18_19/III Semestre/Image Analysis and Computer Vision/Project/previous years/eyeTracking-gazeEstimation-master/data/dataset1.mat')
else
    files = dir('C:/users/bianc/Google Drive/LM Milano 18_19/III Semestre/Image Analysis and Computer Vision/Project/previous years/eyeTracking-gazeEstimation-master/data/dataset*');
    filename = files(end).name;
    ord = str2double(filename(8));
    ord = ord+1;
    filename = append('C:/users/bianc/Google Drive/LM Milano 18_19/III Semestre/Image Analysis and Computer Vision/Project/previous years/eyeTracking-gazeEstimation-master/data/',filename(1:7),num2str(ord),filename(9:end));
    save(filename,'calibrationTargets','horizontal','vertical')
end
