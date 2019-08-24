clear; close all;

robot_IP_address = '127.0.0.1'; % Simulation ip address
%robot_IP_address = '192.168.125.1';
robot_port_1 = 1025;
robot_port_2 = 1026;
socket_1 = Connect(robot_IP_address, robot_port_1);
socket_2 = Connect(robot_IP_address, robot_port_2);
set(socket_2, 'ReadAsyncMode', 'continuous');
set(socket_2, 'BytesAvailableFcn', 'dispcallback');

PnPArr = ["[-100,300,550,-20,45]"; "[-150,320,500,20,0]";];
FinishedFlag = false;

while(~FinishedFlag)
    if(isequal(get(socket_1, 'Status'), 'open'))
        %PickNPlace (commented out to save time)
        % Getting PnP data
        
        for n = 1:length(PnPArr)
            SendMessage(socket_1,"PNP");              
            SendMessage(socket_1,PnPArr(n));
            LookForMessage(socket_1,"DONE");
            fprintf("\n");
            fprintf("Next block \n");
            pause(1);
        end
        fprintf("Finished PNP! \n");

        %uiwait(gcf);
        %app.TracingoutEdibleInkTextLamp.Color = 'g';
        %app.TextArea.Value = 'Ink Tracing...';

        % Ink
        fprintf("Starting Ink \n");
        img = iread('table (8).jpg');
        imshow(img)
        [blob_paths, blob_im, n_letters, thickVector] = text_detection(img);
%         [blob_paths, thickVector, n_letters] = run_text_detection_offline();

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


        SendMessage(socket_1,"CON");% conveyor on
        SendMessage(socket_1,"COF");% conveyor off
        SendMessage(socket_1,"UPD");
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
        thickVector{i} = ones(length(blob_paths{i}),1);
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
        fprintf("Looking for %s: %d", message, i);
        str = fgetl(socket);
        fprintf(char(str));
        fprintf("\n");
        i = i + 1;
    end
    return 
end