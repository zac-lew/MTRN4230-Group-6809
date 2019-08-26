% Testing analyseCustomerImage.m and Detection.m

%load('FINAL_FRCNN_V5.mat');
load('CalibConv.mat');
load('CalibTable.mat');

global detector_updated_FINAL;
global camParam_Table R_Table t_Table;
global camParam_Conv R_Conv t_Conv;
global useRobotCellCamera;
global scanOnce;
global cLabels cBboxes;
global posMatchNum;
global cImage;

usesRobotCellCamera = false;
scanOnce = false;
posMatchNum = 0;
bdim = 100; %for conveyor block angle detect function
close all; clc;

%customerImage = imread('PnpTestT2.jpg'); 
customerImage = imread('PnPTestT3.jpg'); 
[shape_color,missingBlockMatch] = analyseCustomerImage(customerImage,0.20,375);
pause(1.0);
conv_match_ctr = 1;
while(conv_match_ctr ~= (size(shape_color,2)) - missingBlockMatch)
    [PnPMessage, shape_color, conv_match_ctr] = Detection(conv_match_ctr, shape_color,275,75);
        
    %SendMessage(socket_1,"PNP");              
    %SendMessage(socket_1,PnPArr(n));
    %SendMessage(socket_1,PnPMessage);
    %LookForMessage(socket_1,"DONE");
    
    fprintf("\n");
    fprintf("Next block \n");
    pause(1);
end
fprintf("Finished PNP! \n");

