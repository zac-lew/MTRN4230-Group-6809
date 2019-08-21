function run_text_detection()
% Run this file to image detect and trace all letters
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
    
    img = iread('table (8).jpg');
    imshow(img)
    [blob_paths, blob_im, n_blobs] = text_detection(img);
    
    for i = 1:n_blobs - 4
        n_strokes = size(blob_paths{i}, 2);
        disp(i);
        % iterate through all the strokes for this blob
        for j = 1:n_strokes
            stroke = blob_paths{i}{j};
            % send the stroke
            
            % thick or thin 
            thickOrThin = 1;
            ALCommunications(stroke,thickOrThin)
        end
    end
    done = 0;
    disp("finished with blobs");
    while done ~= 1
        done = ALEndCommunications();
    end 
    
end

