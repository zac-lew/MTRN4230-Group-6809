function run_text_detection()
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
    
    img = iread('../w8-photos/table (6).jpg');
    [blob_paths, blob_im, n_blobs] = text_detection(img);
    
    for i = 1:n_blobs
        n_strokes = size(blob_paths{i}, 2);
        
        % iterate through all the strokes for this blob
        for j = 1:n_strokes
            stroke = blob_paths{i}{j};
            % send the stroke
        end
    end
        
end

