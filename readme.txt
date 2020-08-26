
IMAGE ANALYSIS AND COMPUTER VISION COURSE - AA19/20
Eye tracking and gaze estimation from facial features
Luca Bianco matricola
Lorenzo Randazzo 905565


PROGRAM STRUCTURE:

Executable scripts:
1. eye_tracking_calibration.m
    This script contains the procedure of calibration to get data of facial features.
    It produces the files stored in the data folder and eventually the webcam take
    of the calibration procedure.
2. gen_model.m 
    This script generates the models of each eye from the dataset contained in the
    data folder.
    It produces the 4 models, vertical and horizontal regression of each eye.
3. eye_tracking_test.m
    This script contains the procedure of testing the models.
    The user is asked to look at the green panels.
    It gets frames from the webcam and plots the estimated gaze on the screen.
    It produces the test file stored in the data folder, saving the estimated gaze.
4. gen_heatmap.m
    This script generates the heatmap of the estimated gaze. 
    The color indicates the level of interest in the panel.
    Blue is low interest, green is high interest.
5. eye_features.m
    This script contains the procedure of plot important facial features.
    Changing the variable 'type' as described will display different features in real time. 
    type=1 plots important facial points 
    type=2 plots bounding boxes of eyes and head 
    type=3 plots horizontal features used and horizontal scale factor  
    type=4 plots vertical features used and vertical scale factor  

Folders:
1. 'data' folder: this folder will contain data collected during the calibration processes and that 
    will be used in to train the regression model, also saved in this folder.
    If a completely new calibration has to be done (new positions, new light conditions, ecc..), 
    be sure to delete any file inside this folder.
2. 'functions' folder: this folder contains all the useful functions used by the algorithm in the 
    tracking phase.
3. 'video' folder: this folder contains videos of the calibration phases, test phases and features
    tracked.
4. 'images' folder: this folder contains images and files of points acquired during our validation
    processes. 
5. 'reports' folder: this folder contains the pdf report and the ppt of the oral presentation.

   
INSTRUCTIONS

The face must be around 50cm from the webcam.
The nose must be at the height of the webcam.
The test program will be able to predict gaze when the distance changes during the test phase only if the calibration has been done at different distances.
The test program will be able to predict gaze when the head slightly rotates during the test phase only if the calibration has been done at different rotations of the head.

The first time the program is executed or every time the webcam is changed 
in its position, the Matlab's files must be runned again in the order presented
in the PROGRAM STRUCTURE section. The correct behaviour of the algorithm is not guaranteed if 
the calibration phase has been done in different light and position with the respect to the 
test phase.
File eye_tracking_calibration.m can be executed multiple times, in order to get more 
data to feed the model.

The program can be stopped in any moment closing the video stream window.
