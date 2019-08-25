function MattCommTest_7Fn(app,runtype)
    global socket_1;
    global socket_2;
    global scanOnce;
    global cLabels cBboxes;
    global posMatchNum;
    global cImage;
    global bdim;
    
    scanOnce = false;
    posMatchNum = 0;
    bdim = 55;
    FinishedFlag = false;
    % runtype = 0(pnp then ink), 1 (pnp only), 2 (ink only), 3( conveyor
    % on), 4 (conveyor off), 5 (Vac on), 6 (Vac off)
    runtype = 0; % temporary

    while(~FinishedFlag)
        if(isequal(get(socket_1, 'Status'), 'open'))
            
            if((runtype == 0) || (runtype == 1))
                %PickNPlace (commented out to save time)
                % Getting PnP data
                customerImage = imread('PnpTestT2.jpg'); % Temporary
                conv_match_ctr = 0;
                [shape_color,missingBlockMatch] = analyseCustomerImage(customerImage,0.20,350);

                while(conv_match_ctr ~= (size(shape_color,2) - missingBlockMatch))
                    [PnPMessage, shape_color, conv_match_ctr]  = Detection(conv_match_ctr, shape_color,275);
                    if( PnPMessage.strlength > 1)
                        SendMessage(socket_1,"PNP");              
                        SendMessage(socket_1,PnPMessage);
                        LookForMessage(socket_1,"DONE");
                        fprintf("\n");
                        fprintf("Next block \n");
                    else
                        fprintf("Bad Message \n");
                    end
                    pause(1);
                end
            end
            
            app.TextArea.Value = 'Close Popup to Ink Trace';
            %uiwait(gcf);
            
            app.TracingoutEdibleInkTextLamp.Color = 'g';
            app.TextArea.Value = 'Ink Tracing...';

            if((runtype == 0) || (runtype == 2))
                % Ink
                fprintf("Starting Ink \n");
                img = iread('table (8).jpg');
                imshow(img)
                [blob_paths, blob_im, n_letters, thickVector] = text_detection(img);
                close all;
                figure; axis equal;
                for k=1:n_letters
                    blob_letter = blob_paths{k};
                    thickV = thickVector{k};

                    for j=1:length(blob_letter)

                        SendMessage(socket_1,"INK");                    
                        data = blob_letter{j};
                        thickOrThin = thickV(j);
                        initial = "[" + length(data) + "," + thickOrThin + "]"; 
                        SendMessage(socket_1,initial);

                        %  send the whole array 
                        for n=1:length(data)
                            fprintf("Array no. %d of %d \n", n, length(data));
                            send_str = "[" + num2str(data(n,1)*1000 ) + "," + num2str(data(n,2)*1000) + "]"; 
                            SendMessage(socket_1,send_str);
                            str = "";

                        if ( j == 1)
                            hold off;
                            subplot(n_letters,1,k);
                        end 
                        plot(data(:,1),data(:,2))
                        hold on;

                        end
                        LookForMessage(socket_1,"DONE");
                        fprintf("Finished INK! \n");
                        pause(2.0);
                    end
                end
            end

            if(runtype == 3) SendMessage(socket_1,"CON"); end
            if(runtype == 4) SendMessage(socket_1,"COF"); end
            if(runtype == 5) SendMessage(socket_1,"VON"); end
            if(runtype == 6) SendMessage(socket_1,"VOF"); end
            
            fprintf("Finished program! \n");
            FinishedFlag = true;
        else
            % Reconnect
            socket_1 = Connect(robot_IP_address, robot_port_1);
        end
        pause(2);
    end
    % Close the socket.
    fclose(socket_1);
end
%% Functions
    
function [blob_paths, n_letters, thickVector] = run_text_detection_offline()
    close all; 
    n_blobs = 4; 
    blob_paths = cell(1, n_blobs); 
    data = load('b_stroke.mat');
    blob_paths{1}  = data.b_stroke_1; 
    blob_paths{2}  = data.b_stroke_2; 
    data = load('a_stroke.mat');
    blob_paths{3}  = data.a_stroke_1; 
    blob_paths{4}  = data.a_stroke_2; 
    thickVector = ones(n_blobs,1);
    n_letters = 1;

    return 

end

function [blob_paths, blob_im,  n_letters, thickVector] = text_detection_Thick_Wrap(img)
    [blob_paths, blob_im, n_blobs] = text_detection(img);
    n_letters = n_blobs;
    thickVector = cell(1,n_letters);
    for i=1:n_letters
        thickVector{i} = ones(length(blob_paths{1}),1);
    end


end

function socket = Connect(robot_IP_address, robot_port)
    socket = tcpip(robot_IP_address, robot_port);    
    fclose(socket);
    set(socket, 'ReadAsyncMode', 'continuous');
    if(~isequal(get(socket, 'Status'), 'open'))
        try
            fopen(socket);
            disp('Connected');
            %app.TextArea.Value = 'Robot Connected SUCCESS';  
            %app.ConnectionStatusLamp.Color = 'g';
        catch
            fprintf('Could not open TCP connection to %s on port %d\n',robot_IP_address, robot_port);
            %app.TextArea.Value = 'Robot Connection FAILED';  
            %app.ConnectionStatusLamp.Color = 'r';
        end
    end
end

function SendMessage(socket, message)
    fprintf("Sending %s \n", message);
    str = "";
    i = 0;
    while (~strcmp(str,"ACK"))
        i = i+1;
        fprintf("Looking for ACK: %d \n", i);
        fwrite(socket, message);
        str = fgetl(socket);
        fprintf("%s \n" , char(str));        
    end
    fprintf("Sent %s \n", message);
end

function message = LookForMessage(socket,message)
    i = 0;
    str = "";
    while (~strcmp(str,message))
        fprintf("Looking for %s: %d \n", message, i);
        str = fgetl(socket);
        fprintf("Saw: %s \n", char(str));
        i = i + 1;
    end
    return 
end
