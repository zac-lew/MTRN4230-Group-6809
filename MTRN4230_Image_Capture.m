function robot_image = MTRN4230_Image_Capture (varargin)
%Table Camera (Robot Cell)
   if nargin == 0 || nargin == 1
        fig1 =figure(1);
        axe1 = axes ();
        axe1.Parent = fig1;
        vid1 = videoinput('winvideo', 1, 'MJPG_1600x1200');
        video_resolution1 = vid1.VideoResolution;
        nbands1 = vid1.NumberOfBands;
        %img1 = imshow(zeros([video_resolution1(2), video_resolution1(1), nbands1]), 'Parent', axe1);
        src1 = getselectedsource(vid1);
        src1.ExposureMode = 'manual';
        src1.Exposure = -4;
        src1.Contrast = 21;
        src1.Brightness = 128;
        robot_image = getsnapshot(vid1);
   end
% Conveyor Camera
    if nargin == 0 || nargin == 2
        fig2 =figure(2);
        axe2 = axes ();
        axe2.Parent = fig2;
        vid2 = videoinput('winvideo', 2, 'MJPG_1600x1200');
        video_resolution2 = vid2.VideoResolution;
        nbands2 = vid2.NumberOfBands;
        src2 = getselectedsource(vid2);
        src2.ExposureMode = 'manual';    
        src2.Exposure = -4;        
        src2.Brightness = 165;
        src2.Contrast = 32;
        robot_image = getsnapshot(vid2);
    end
end