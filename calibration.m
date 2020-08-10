%
%%
% Gaze Tracker from facial features
% Pasquale Enrico Cortese

% Calibration version
%via fontana 23 trasversa tra pzza cavour e piazza volta 16 porticato che unisce le piazze 
%% initialization
close all
clear all
clc
%%
% Add path
addpath('functions')
addpath('data')

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();
leftEyeDetector = vision.CascadeObjectDetector('LeftEye');
rightEyeDetector = vision.CascadeObjectDetector('RightEye');
noseDetector = vision.CascadeObjectDetector('Nose');

% Create the point tracker object.
left_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
right_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
leftAP_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
rightAP_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
left_eyelide_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
right_eyelide_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
nose_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object and initialize variables.
videoPlayer = vision.VideoPlayer('Position', [930 430 678 527]);
runLoop = true;
numPts_left = 0;
numPts_right = 0;
testWindowDrawn = false;
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
fs = 250; 
duration = 0.5;
freq = 280;
val = 0:1/fs:duration;
bip1 = amp*sin(2*pi* freq*val);
bip2 = amp*sin(2*pi* 2*freq*val);
calibrationDone = false;
firstTime = true;
recordingCounter = 1;
calibrationCounter = 1;
hPredictors = zeros(17,5);
vPredictors = zeros(17,8);
endOfPageCounter=0;
calibrationPoints=[0 0; 0.5 0; 1 0; ...
                   0.25 0.25; 0.5 0.25; 0.75 0.25; ...
                   0 0.5; 0.25 0.5; 0.5 0.5; 0.75 0.5; 1 0.5; ...
                   0.25 0.75; 0.5 0.75; 0.75 0.75; ...
                   0 1; 0.5 1; 1 1; ...
                  ];
    
%% loop
while ((runLoop && frameCount < 5000))

    % Get the next frame.
    videoFrame = flip(snapshot(cam),2);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;
    
    %check if one of the points is lost or initialize them
    if numPts_left < numPts_threshold || numPts_right < numPts_threshold ...
            || numPts_leftAP < numPts_threshold ...
            || numPts_rightAP < numPts_threshold || numPts_nose < numPts_threshold
        %% Detection mode.
        
        % detect the face
        bboxFace = faceDetector.step(videoFrameGray);
        
        if ~isempty(bboxFace)
            %from all the possible square recognized as face, find the
            %bigger one that corresponds to the face 
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
    % EC Nose
    leftECNHdistance = norm(xyPoints_left(1,1)-xyPoints_nose(1,1));
    rightECNHdistance = norm(xyPoints_right(1,1)-xyPoints_nose(1,1));
     
    % horizontal (H) distance of the right eye center and the left anchor 
    % point at the opposite side 
    rightECleftACHdistance = norm(xyPoints_left(1,1)-xyPoints_right(1,1));
     
    % --- VERTICAL DIRECTION FEATURES --- %
     
    % vertical (V) distances between the y-coordinates of each of the eye centers and 
    % the respective anchor points
    %up
    leftECACVdistance = norm(xyPoints_left(1,2)-xyPoints_left_eyelide(1,2));
    rightECACVdistance = norm(xyPoints_right(1,2)-xyPoints_right_eyelide(1,2));
    %down
    leftECACDVdistance = norm(xyPoints_left(1,2)-xyPoints_left_eyelide(2,2));
    rightECACDVdistance = norm(xyPoints_right(1,2)-xyPoints_right_eyelide(2,2));
    % ap and ec
    leftECAPVdistance = norm(xyPoints_left(1,2)-xyPoints_leftAP(1,2));
    rightECAPVdistance = norm(xyPoints_right(1,2)-xyPoints_rightAP(1,2));
    
    %EC Nose
    leftECNVdistance = norm(xyPoints_left(1,2)-xyPoints_nose(1,2));
    rightECNVdistance = norm(xyPoints_right(1,2)-xyPoints_nose(1,2));
    
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
            firstTime = false;
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
             hPredictors(calibrationCounter, :) = [leftECACHdistance, rightECACHdistance, rightECleftACHdistance, leftECNHdistance, rightECNHdistance];
             vPredictors(calibrationCounter, :) = [leftECACVdistance, rightECACVdistance, leftECACDVdistance, rightECACDVdistance, leftECNVdistance, rightECNVdistance, leftECAPVdistance, rightECAPVdistance];
             % color the circle green
             plot(calibrationPoints(calibrationCounter,1),calibrationPoints(calibrationCounter,2),'g.','MarkerSize',30);
             % wait a second
             pause(1)
             
             calibrationCounter = calibrationCounter + 1;
             recordingCounter = 1;
             
         end
         
         if calibrationCounter > 17
             calibrationDone = true;
             
             % --- LINEAR REGRESSION MODEL --- %
             Xh = [ones(size(hPredictors, 1), 1) hPredictors(:,1) hPredictors(:,2) hPredictors(:,3) hPredictors(:,4) hPredictors(:,5)];
             Xv = [ones(size(vPredictors, 1), 1) vPredictors(:,1) vPredictors(:,2) vPredictors(:,3) vPredictors(:,4) vPredictors(:,5) vPredictors(:,6) vPredictors(:,7) vPredictors(:,8)];
             
             T_h = array2table([calibrationPoints(:,1) hPredictors]);
             T_v = array2table([calibrationPoints(:,2) vPredictors]);
             %% saving hor
            if isempty(dir('data/dataset_h_*'))
                writetable(T_h,'data/dataset_h_1');
            else
                files = dir('data/dataset_h_*');
                filename = files(end).name;
                ord = str2double(filename(11));
                ord = ord+1;
                filename_h = append('data/',filename(1:10),num2str(ord));
                writetable(T_h,filename_h);
            end
            %% saving vert
            if isempty(dir('data/dataset_v_*'))
                writetable(T_v,'data/dataset_v_1');
            else
                files = dir('data/dataset_v_*');
                filename = files(end).name;
                ord = str2double(filename(11));
                ord = ord+1;
                filename_v = append('data/',filename(1:10),num2str(ord));
                writetable(T_v,filename_v);
            end
            close all
         end
        
     end
    
    
    
     
    %--------------------END CALIBRATION----------------------%
   
    % Display tracked points.
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
release(nose_pointTracker);
release(left_eyelide_pointTracker);
release(right_eyelide_pointTracker);
release(faceDetector);
release(leftEyeDetector);
release(rightEyeDetector);
release(noseDetector);
close all
clearvars -except horizontal vertical calibrationTargets videoMontage

