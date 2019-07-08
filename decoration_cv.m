% MTRN4230 Group Project

% ----------------ChangeLog---------------
% v1. 6/7/19 Initial creation with comments and brief summary. Image capture
% v2. 

% ----------------ChangeLog---------------

% Computer Vision Engineer (Decoration)
% 1. Detect Quirkle blocks as they are supplied using the conveyor (ML)
% 2. Detect the customer's desired decorating pattern from conveyor camera (CV)

% Customer's IMAGE::
% Location = x,y for blocks? (target locations)
% Type of Block = Shape + Colour

% Shapes == (criss cross, clover, starburst, square, diamond, circle}
% Color == (red, blue, green, yellow, purple, orange)

%% Decoration Computer Vision Pipeline

% 1. RobotCell Image Laptop Test

% % Create Camera Object with discovered name of webcam
% % NOTE: change to the name of the webcam detected when 'webcamlist' command
% % is run in terminal
% Quirkle_Test = webcam('TOSHIBA Web Camera - HD'); 
% 
% % Preview image from webcam
% capture_image(Quirkle_Test,'Laptop Test');
% clear('Quirkle_Test');

% 1. Obtain customer's image

%RobotCellCamera([1]); % Camera above Quirkle Blocks

customerImage = imread('sample1.jpg');
imshow(customerImage)

% 2. Identify desired 'blocks' (Quirkle colored shapes)

% 3. Results of classification (which color and which shape)

% 4. x


%% FUNCTIONS

% Image capture function - saves image in current directory
function capture_image (vid,name)
 	robotCellImage = snapshot(vid);
    imshow(robotCellImage);
    imwrite(robotCellImage, [name, datestr(datetime('now'),'_mm_dd_HH_MM_SS'), '.jpg']);
    disp([name ' captured']);
end

function RobotCellCamera (varargin)
    close all;
    warning('off', 'images:initSize:adjustingMag');

    %Table Camera
    if nargin == 0 || nargin == 1
        fig1 =figure(1);
        axe1 = axes ();
        axe1.Parent = fig1;
        vid1 = videoinput('winvideo', 1, 'MJPG_1600x1200');
        video_resolution1 = vid1.VideoResolution;
        nbands1 = vid1.NumberOfBands;
        img1 = imshow(zeros([video_resolution1(2), video_resolution1(1), nbands1]), 'Parent', axe1);
        prev1 = preview(vid1,img1);
        src1 = getselectedsource(vid1);
        src1.ExposureMode = 'manual';
        src1.Exposure = -4;
%         src1.Contrast = 57;%57,32
%         src1.Saturation = 78;
        cam1_capture_func = @(~,~)capture_image(vid1,'table_img');
        prev1.ButtonDownFcn = cam1_capture_func;
        fig1.KeyPressFcn = cam1_capture_func;
    end

    %Conveyor Camera
    if nargin == 0 || nargin == 2
        fig2 =figure(2);
        axe2 = axes ();
        axe2.Parent = fig2;
        vid2 = videoinput('winvideo', 2, 'MJPG_1600x1200');
        video_resolution2 = vid2.VideoResolution;
        nbands2 = vid2.NumberOfBands;
        img2 = imshow(zeros([video_resolution2(2), video_resolution2(1), nbands2]), 'Parent', axe2);
        prev2 = preview(vid2,img2);
        src2 = getselectedsource(vid2);
        src2.ExposureMode = 'manual';    
        src2.Exposure = -4;
        cam2_capture_func = @(~,~)capture_image(vid2,'conveyor_img');
        fig2.KeyPressFcn = cam2_capture_func;
        prev2.ButtonDownFcn = cam2_capture_func;
    end
end
