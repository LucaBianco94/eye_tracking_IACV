IMAGE ANALYSIS AND COMPUTER VISION COURSE - AA19/20
Eye tracking and gaze estimation from facial features
Luca Bianco matricola
Lorenzo Randazzo 905565

PROGRAM STRUCTURE:
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
   
INSTRUCTIONS
The face must be around 50cm from the webcam.
The nose must be at the height of the webcam.
The first time the program is runned or every time the webcam is changed 
in its position, the Matlab's files must be runned again in the order presented
in the PROGRAM STRUCTURE section.
File eye_tracking_calibration.m can be runned multiple times, in order to get more 
data to feed the model.
The program can be stopped in any moment closing the video stream figure.
