close all
clear all
clc

%%

frame_number = 500;
frameCount=0;
faceDetector = vision.CascadeObjectDetector();

% Read a video frame and run the detector.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
videoFrameGray = rgb2gray(videoFrame);
bbox = step(faceDetector, videoFrameGray);

% Draw the returned bounding box around the detected face.
videoOut = insertObjectAnnotation(videoFrameGray,'rectangle',bbox,'Face');
figure, imshow(videoOut), title('Detected face');

%%

% Display the Hue Channel data and draw the bounding box around the face.
figure, imshow(videoFrameGray), title('Hue channel data');
rectangle('Position',bbox(1,:),'LineWidth',2,'EdgeColor',[1 1 0])

%%

% Create a tracker object.
tracker = vision.HistogramBasedTracker;

% Initialize the tracker histogram using the Hue channel pixels from the
% nose.
initializeObject(tracker, videoFrameGray, bbox);

% Create a video player object for displaying video frames.
videoPlayer  = vision.VideoPlayer;

% Track the face over successive video frames until the video is finished.
while frameCount < frame_number
    
    videoFrame = flip(snapshot(cam),2);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    % Extract the next video frame

    % RGB -> HSV

    % Track using the Hue channel data
    bbox = tracker(videoFrameGray);

    % Insert a bounding box around the object being tracked
    videoOut = insertObjectAnnotation(videoFrame,'rectangle',bbox,'Face');

    % Display the annotated video frame using the video player object
    step(videoPlayer, videoOut);

end

% Release resources
release(videoPlayer);