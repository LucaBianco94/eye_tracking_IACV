%% IMAGE ANALYSIS AND COMPUTER VISION COURSE - AA19/20
% Eye tracking and gaze estimation from facial features
% Luca Bianco - Lorenzo Randazzo

% Calibration script

%% initialization
close all
clear all
clc
%% 

% Add path
addpath('functions')
addpath('data')
addpath('video')

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();
leftEyeDetector = vision.CascadeObjectDetector('LeftEye');
rightEyeDetector = vision.CascadeObjectDetector('RightEye');
noseDetector = vision.CascadeObjectDetector('Nose','MergeThreshold',16);

% Create the point tracker object.
left_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
right_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
leftAP_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
rightAP_pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
nose_pointTracker = vision.PointTracker('MaxBidirectionalError', 2,'NumPyramidLevels',4,'MaxIterations',45);

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% calibration object
calibrationCounter = 0;
calibFigure = figure(1);
calibFigure.ToolBar = 'none';
calibFigure.MenuBar = 'none';
calibFigure.WindowState = 'maximized';
calibFigure.Units = 'Normalized';
calibFigure.Position = [0 0 1 1];
calibFigure.Resize = 'off';

calibrationPoints=[0 0; 0 0.5; 0 1; ...
                   0.5 1; 1 1; 1 0.5; ...
                   1 0; 0.5 0; 0.5 0.5; ...
                   0.5 0.5 ... % 
                   ];

window1 = 30;
window2 = 10;
num_of_points = 9;

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [50 50 [frameSize(2), frameSize(1)]]);
runLoop = true;
numPts_left = 0;
numPts_right = 0;
numPts_leftAP = 0;
numPts_rightAP = 0;
numPts_nose = 0;
frameCount = 0;
numPts_threshold = 5;
numPts_threshold_n = 1;
frame_number = window1*num_of_points+1;
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

% preallocation
h_l_feat = zeros(frame_number,4);
h_r_feat = zeros(frame_number,4);
v_l_feat = zeros(frame_number,3);
v_r_feat = zeros(frame_number,3);
horizontal_l = zeros(num_of_points,4);
horizontal_r = zeros(num_of_points,4);
vertical_l = zeros(num_of_points,3);
vertical_r = zeros(num_of_points,3);
calibrationTargets = zeros(num_of_points+1,2);
videoMontage = zeros(frameSize(1),frameSize(2),frameSize(3),frame_number);
    
%% loop
while runLoop && frameCount < frame_number

    % Get the next frame.
    videoFrame = flip(snapshot(cam),2);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;
    
    if numPts_left < numPts_threshold || numPts_right < numPts_threshold ...
            || numPts_leftAP < numPts_threshold...
            || numPts_rightAP < numPts_threshold || numPts_nose < numPts_threshold_n
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
            if (numPts_nose < numPts_threshold_n)
                                
                bbox_nose = noseDetector.step(videoFrameGray);
                if isempty(bbox_nose)
                    continue
                end    
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
            if (half_y<=frameSize(1)) && (half_x<=frameSize(2))
                leftEyeROI = videoFrameGray(start_y:half_y,bboxFace(1):half_x);
                bbox_eye_left = leftEyeDetector.step(leftEyeROI);
        
                if ~isempty(bbox_eye_left)
                    bbox_eye_left = findBiggerRect(bbox_eye_left);
                    bbox_eye_left(1:2) = bbox_eye_left(1:2)+[start_x,start_y];
                    leftEyeImage = videoFrame(bbox_eye_left(2):bbox_eye_left(2)+bbox_eye_left(4),...
                        bbox_eye_left(1):bbox_eye_left(1)+bbox_eye_left(3),:);
                    eyeCenter_left = eyecenter_loc(leftEyeImage);
                    eyeCenter_left = eyeCenter_left+bbox_eye_left(1:2);
                    
                    % Find corner points inside the detected region.
                    points_left = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox_eye_left(1, :));

                    % Re-initialize the point tracker.
                    xyPoints_left = points_left.Location;
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
                
            else
                continue
            end
            
            %RIGHT EYE DETECTION
            if (half_y<=frameSize(1)) && (half_x<=frameSize(2)) && (bboxFace(1)+bboxFace(3)<=frameSize(2))
                rightEyeROI = videoFrameGray(start_y:half_y,half_x:bboxFace(1)+bboxFace(3));
                bbox_eye_right = rightEyeDetector.step(rightEyeROI);
        
                if ~isempty(bbox_eye_right)
                    bbox_eye_right = findBiggerRect(bbox_eye_right);
                    bbox_eye_right(1:2) = bbox_eye_right(1:2)+[half_x,start_y];
                    rightEyeImage = videoFrame(bbox_eye_right(2):bbox_eye_right(2)+bbox_eye_right(4),...
                        bbox_eye_right(1):bbox_eye_right(1)+bbox_eye_right(3),:);
                    eyeCenter_right = eyecenter_loc(rightEyeImage);
                    eyeCenter_right = eyeCenter_right+bbox_eye_right(1:2);
                    
                    % Find corner points inside the detected region.
                    points_right = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox_eye_right(1, :));

                    % Re-initialize the point tracker.
                    xyPoints_right = points_right.Location;
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
                    
                else
                    continue
                end
                
            else
                continue
            end
            %end of right eye detection
            
            % eyelid detection
            [ioc_dist,theta] = crossEyeCenters(eyeCenter_left,eyeCenter_right);
            leftEyelid = findEyeLids(videoFrame,eyeCenter_left,ioc_dist,theta);
            rightEyelid = findEyeLids(videoFrame,eyeCenter_right,ioc_dist,theta);
            % end of eyelid detection
            
            % left anchor point detection
            leftAnchorROI = floor([eyeCenter_left(1)+hWidthEye*bbox_eye_left(3),...
                eyeCenter_left(2)-hHeightEye*bbox_eye_left(3)-hHeightInt*ioc_dist/2,...
                hWidthInt*ioc_dist, hHeightInt*ioc_dist]);
            leftAnchorPoint = [leftAnchorROI(1)+leftAnchorROI(3)/2,leftAnchorROI(2)+leftAnchorROI(4)/2];
            
            % Find corner points inside the detected region.
            points_leftAP = detectMinEigenFeatures(videoFrameGray, 'ROI', leftAnchorROI(1, :));

            % Re-initialize the point tracker.
            xyPoints_leftAP = points_leftAP.Location;
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
            
            % Find corner points inside the detected region.
            points_rightAP = detectMinEigenFeatures(videoFrameGray, 'ROI', rightAnchorROI(1, :));
            
            % Re-initialize the point tracker.
            xyPoints_rightAP = points_rightAP.Location;
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
            % end of right anchor point detection 
        else
            continue
        end
        %END OF DETECTION
    else
        %% Tracking mode
        
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
        if numPts_left >= numPts_threshold
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform_left, oldInliers_left, visiblePoints_left] = estimateGeometricTransform(...
                oldInliers_left, visiblePoints_left, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints_left = transformPointsForward(xform_left, bboxPoints_left);
            bbox_eye_left = ceil(points2bbox(bboxPoints_left));
            leftEyeImage = videoFrame(bbox_eye_left(2):bbox_eye_left(2)+bbox_eye_left(4),...
                bbox_eye_left(1):bbox_eye_left(1)+bbox_eye_left(3),:);
            eyeCenter_left = eyecenter_loc(leftEyeImage);
            eyeCenter_left = eyeCenter_left+bbox_eye_left(1:2);

            % Reset the points.
            oldPoints_left = visiblePoints_left;
            setPoints(left_pointTracker, oldPoints_left);
        end
        % end of left eye tracking
        
        % RIGHT EYE TRACKING
        [xyPoints_right, isFound_right] = step(right_pointTracker, videoFrameGray);
        visiblePoints_right = xyPoints_right(isFound_right, :);
        oldInliers_right = oldPoints_right(isFound_right, :);

        numPts_right = size(visiblePoints_right, 1);
        if numPts_right >= numPts_threshold
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform_right, oldInliers_right, visiblePoints_right] = estimateGeometricTransform(...
                oldInliers_right, visiblePoints_right, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints_right = transformPointsForward(xform_right, bboxPoints_right);
            bbox_eye_right = ceil(points2bbox(bboxPoints_right));
            rightEyeImage = videoFrame(bbox_eye_right(2):bbox_eye_right(2)+bbox_eye_right(4),...
                bbox_eye_right(1):bbox_eye_right(1)+bbox_eye_right(3),:);
            eyeCenter_right = eyecenter_loc(rightEyeImage);
            eyeCenter_right = eyeCenter_right+bbox_eye_right(1:2);
            
            % Reset the points.
            oldPoints_right = visiblePoints_right;
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
        if numPts_leftAP >= numPts_threshold
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform_leftAP, oldInliers_leftAP, visiblePoints_leftAP] = estimateGeometricTransform(...
                oldInliers_leftAP, visiblePoints_leftAP, 'similarity', 'MaxDistance', 4);
            
            % Apply the transformation to the bounding box.
            bboxPoints_leftAP = transformPointsForward(xform_leftAP, bboxPoints_leftAP);
            leftAnchorROI = ceil(points2bbox(bboxPoints_leftAP));
            leftAnchorPoint = [leftAnchorROI(1)+leftAnchorROI(3)/2,leftAnchorROI(2)+leftAnchorROI(4)/2];
            
            % Reset the points.
            oldPoints_leftAP = visiblePoints_leftAP;
            setPoints(leftAP_pointTracker, oldPoints_leftAP);
        end
        % end of left anchor point tracking
        
        % right anchor point tracking
        [xyPoints_rightAP, isFound_rightAP] = step(rightAP_pointTracker, videoFrameGray);
        visiblePoints_rightAP = xyPoints_rightAP(isFound_rightAP, :);
        oldInliers_rightAP = oldPoints_rightAP(isFound_rightAP, :);

        numPts_rightAP = size(visiblePoints_rightAP, 1);
        if numPts_rightAP >= numPts_threshold
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform_rightAP, oldInliers_rightAP, visiblePoints_rightAP] = estimateGeometricTransform(...
                oldInliers_rightAP, visiblePoints_rightAP, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints_rightAP = transformPointsForward(xform_rightAP, bboxPoints_rightAP);
            rightAnchorROI = ceil(points2bbox(bboxPoints_rightAP));
            rightAnchorPoint = [rightAnchorROI(1)+rightAnchorROI(3)/2,rightAnchorROI(2)+rightAnchorROI(4)/2];
            
            % Reset the points.
            oldPoints_rightAP = visiblePoints_rightAP;
            setPoints(rightAP_pointTracker, oldPoints_rightAP);
            
        end
        %end of right anchor point tracking

    end
    % end of tracking
    
    %% FEATURE COLLECTION
        
    %horiz
    h_scale = -(bbox_eye_left(1)-bbox_eye_right(1)+bbox_eye_right(3));   %scale factors
    diff = -(bbox_eye_left(1)-xyPoints_nose(1))-(bbox_eye_right(1)-xyPoints_nose(1));
    %left
    h_leftEyeNose = -(eyeCenter_left(1) - xyPoints_nose(1));
    h_leftEyeleftCorner = -(eyeCenter_left(1) - leftAnchorPoint(1));
    h_leftCornerNose = -(leftAnchorPoint(1) - xyPoints_nose(1));
    h_leftExtEyeNose_diff = diff;
    %right
    h_rightEyeNose = (eyeCenter_right(1) - xyPoints_nose(1));
    h_rightEyerightCorner = (eyeCenter_right(1) - rightAnchorPoint(1));
    h_rightCornerNose = (rightAnchorPoint(1) - xyPoints_nose(1));
    h_rightExtEyeNose_diff = diff;
    %vert
    v_scale = (bbox_eye_left(4)+bbox_eye_right(4))/2;   %scale factors
    %left
    v_leftEyeNose = (eyeCenter_left(2) - xyPoints_nose(2));
    v_leftCornerNose = (leftAnchorPoint(2) - xyPoints_nose(2));
    v_leftEyeleftEyelid = (leftEyelid(1,2) - eyeCenter_left(2));
    %right
    v_rightEyeNose = (eyeCenter_right(2) - xyPoints_nose(2));
    v_rightCornerNose = (rightAnchorPoint(2) - xyPoints_nose(2));
    v_rightEyerightEyelid = (rightEyelid(1,2) - eyeCenter_right(2));
    
    % horizontal distance between left eye center and left anchor point
    h_l_feat(frameCount,1) = h_leftEyeNose/h_scale;
    h_l_feat(frameCount,2) = h_leftEyeleftCorner/h_scale;
    h_l_feat(frameCount,3) = h_leftCornerNose/h_scale;
    h_l_feat(frameCount,4) = h_leftExtEyeNose_diff/h_scale;
    h_r_feat(frameCount,1) = h_rightEyeNose/h_scale;
    h_r_feat(frameCount,2) = h_rightEyerightCorner/h_scale;
    h_r_feat(frameCount,3) = h_rightCornerNose/h_scale;
    h_r_feat(frameCount,4) = h_rightExtEyeNose_diff/h_scale;
    
    % vertical features
    % vertical distance between left eye center and left anchor point
    v_l_feat(frameCount,1) = v_leftEyeNose/v_scale;
    v_l_feat(frameCount,2) = v_leftCornerNose/v_scale;
    v_l_feat(frameCount,3) = v_leftEyeleftEyelid/v_scale;
    v_r_feat(frameCount,1) = v_rightEyeNose/v_scale;
    v_r_feat(frameCount,2) = v_rightCornerNose/v_scale;
    v_r_feat(frameCount,3) = v_rightEyerightEyelid/v_scale;
    
    %% figure 
    if mod(frameCount,window1) == 1
        calibrationCounter = calibrationCounter+1;
        target = calibrationPoints(calibrationCounter,:);
        calibrationTargets(calibrationCounter,:) = target;
        sound(bip2);
        if calibrationCounter > 1
            horizontal_l(calibrationCounter-1,:) = mean(h_l_feat(frameCount-window1+window2:frameCount-window2,:));
            horizontal_r(calibrationCounter-1,:) = mean(h_r_feat(frameCount-window1+window2:frameCount-window2,:));
            vertical_l(calibrationCounter-1,:) = mean(v_l_feat(frameCount-window1+window2:frameCount-window2,:));
            vertical_r(calibrationCounter-1,:) = mean(v_r_feat(frameCount-window1+window2:frameCount-window2,:));
        end
    end
    figure(1)
    plot(target(1),target(2),'b.','MarkerSize',250);
    ax = gca;
    ax.Position = [0.1 0.1 0.8 0.8];
    ax.XLim = [0 1];
    ax.YLim = [0 1];
    ax.Visible = 'off';
   
    % Display tracked points.
    videoFrame = insertMarker(videoFrame, [bbox_eye_left(1),bbox_eye_left(2)+(bbox_eye_left(4)/2)], '*', 'Color', 'magenta');
    videoFrame = insertMarker(videoFrame, [bbox_eye_right(1)+bbox_eye_right(3),bbox_eye_right(2)+(bbox_eye_right(4)/2)], '*', 'Color', 'magenta');
    videoFrame = insertMarker(videoFrame, eyeCenter_left, '*', 'Color', 'yellow');
    videoFrame = insertMarker(videoFrame, eyeCenter_right, '*', 'Color', 'yellow');
    videoFrame = insertMarker(videoFrame, leftEyelid(1,:), '+', 'Color', 'blue');
    videoFrame = insertMarker(videoFrame, rightEyelid(1,:), '+', 'Color', 'blue');
    videoFrame = insertMarker(videoFrame, leftAnchorPoint, '+', 'Color', 'green');
    videoFrame = insertMarker(videoFrame, rightAnchorPoint, '+', 'Color', 'green');
    videoFrame = insertMarker(videoFrame, xyPoints_nose, '*', 'Color', 'black');

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    videoMontage(:,:,:,frameCount) = videoFrame;

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);

end

%% Clean up
release(videoPlayer);
release(left_pointTracker);
release(right_pointTracker);
release(faceDetector);
release(leftEyeDetector);
release(rightEyeDetector);
release(nose_pointTracker);
release(noseDetector);

close all
calibrationTargets = calibrationTargets(1:end-1,:);
clearvars -except horizontal_l horizontal_r vertical_l vertical_r videoMontage calibrationTargets

%% saving
if isempty(dir('data/dataset*'))
    save('data/dataset1.mat')
else
    files = dir('data/dataset*');
    filename = files(end).name;
    ord = str2double(filename(8));
    ord = ord+1;
    filename = append('data/',filename(1:7),num2str(ord),filename(9:end));
    save(filename,'calibrationTargets','horizontal_l','horizontal_r','vertical_l','vertical_r')
end

%% save video
 if isempty(dir('video/calibration*'))
     fileName = 'video/calibration1.avi';
 else
     files = dir('video/calibration*');
     fileName = files(end).name;
     ord = str2double(fileName(12));
     ord = ord+1;
     fileName = append('video/',fileName(1:11),num2str(ord),fileName(13:end));
 end
 outputVideo = VideoWriter(fileName);
 outputVideo.FrameRate = 10;
 open(outputVideo)
 for ii = 1:size(videoMontage,4)
    img = uint8(videoMontage(:,:,:,ii));
    writeVideo(outputVideo,img)
 end
close(outputVideo)
clearvars 