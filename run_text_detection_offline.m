% Run this file to image detect and trace all letters
% Aloysius 
% Ink Printing with offline data 
% requires 'b_stroke.mat'
% requires 'a_stroke.mat'

function run_text_detection_offline()
    close all; 
%     basename = '../cake-design-photos/'; 
%     files = dir(strcat(basename, 'table*'));
%     
%     for f = files'
%        img = iread(strcat(basename, f.name));
%        [paths, blob_im, n_blobs] = run_text_detection(img);
%        pause;
%        close all;
%     end
    
%     img = iread('table (8).jpg');
%     imshow(img)
%     [blob_paths, blob_im, n_blobs] = text_detection(img);
    n_blobs = 4; 
    blob_paths = cell(1, n_blobs); 
    data = load('b_stroke.mat');
    blob_paths{1}  = data.b_stroke_1; 
    blob_paths{2}  = data.b_stroke_2; 
    data = load('a_stroke.mat');
    blob_paths{3}  = data.a_stroke_1; 
    blob_paths{4}  = data.a_stroke_2; 
    for i = 1:n_blobs
        % iterate through all the strokes for this blob
        stroke = blob_paths{i};
        
        % Line Data
        thickOrThin = 1;
        
        % send the stroke
        ALCommunications(stroke, thickOrThin);
        
        % blob number 
        disp(i);
    end
    
    % send off
    done = 0;
    disp("finished with blobs");
    while done ~= 1
        done = ALEndCommunications();
    end  
end

