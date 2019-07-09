% MTRN4230 Group Project

% ----------------ChangeLog---------------
% v1. 6/7/19 Initial creation with comments and brief summary. Image capture
% v2. 9/7/19 Modified structure and made basic function

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

% i. RobotCell Image

blockImage = webcam('USB Video Device'); 
capture_image(blockImage,'Robot Image');
clear('blockImage');

% 1. Obtain customer's image

%customerImage = imread('sample1.jpg');
%imshow(customerImage)

% 2. Identify desired 'blocks' (Quirkle colored shapes) on customer's image

% 3. Results of classification (which color and which shape)

% 4. x


% FUNCTIONS

function capture_image (vid,name)
 	robotCellImage = snapshot(vid);
    imshow(robotCellImage);
    imwrite(robotCellImage, [name, datestr(datetime('now'),'_mm_dd_HH_MM_SS'), '.jpg']);
    disp([name 'Complete']);
end


%Tried running this function on Robot cell. Had issue with the format
%of the image when trying to take image. (9/7/2019)
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
        prev1 = preview(img1);
        src1 = getselectedsource(vid1);
        src1.ExposureMode = 'manual';
        src1.Exposure = -4;
%         src1.Contrast = 57;%57,32
%         src1.Saturation = 78;
        cam1_capture_func = @(~,~)capture_image(vid1,'table_img');
        prev1.ButtonDownFcn = cam1_capture_func;
        fig1.KeyPressFcn = cam1_capture_func;
    end   
end
