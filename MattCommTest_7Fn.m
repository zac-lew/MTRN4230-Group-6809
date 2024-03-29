function MattCommTest_7Fn(app,runtype)
    global socket_1 socket_2 Offline;
    global scanOnce;
    global cLabels cBboxes;
    global posMatchNum;
    global cImage;    
    
    scanOnce = false;
    posMatchNum = 0;
    FinishedFlag = false;
    
    % runtype = 0(pnp then ink), 1 (pnp only), 2 (ink only), 3( conveyor
    % on), 4 (conveyor off), 5 (Vac on), 6 (Vac off), 9 (just take customer image)
    %runtype = 0; % temporary
    

    while(~FinishedFlag)
        if(isequal(get(socket_1, 'Status'), 'open'))
            
            if(runtype == 0 || runtype == 1 || runtype == 2 || runtype == 9)
                if(Offline)
                    textImg = iread('table (8).jpg');
                    customerImage = imread('PnpTestT2.jpg');
                else
                    % Take photo and set
                    img = MTRN4230_Image_Capture([]); 
                    textImg = img;
                    customerImage = img;
                end
            end       
            


            if((runtype == 0) || (runtype == 2))
                app.TracingoutEdibleInkTextLamp.Color = 'g';
                app.TextArea.Value = 'Ink Tracing...';
                % Ink
                fprintf("Starting Ink \n");
                imshow(textImg)
                [blob_paths, blob_im, n_letters, thickVector] = text_detection(textImg);
                close all;
                figure;
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
                        axis equal;
                        hold on;

                        end
                        LookForMessage(socket_1,"DONE");
                        fprintf("Finished INK! \n");
                        pause(2.0);
                    end
                end
            end

            
            if((runtype == 0) || (runtype == 1))
                %PickNPlace (commented out to save time)
                % Getting PnP data
                app.PlacingLetterBlocksLamp.Color = 'g';
                
                app.TextArea.Value = 'Placing Chocolates';
                conv_match_ctr = 0;
                [shape_color,missingBlockMatch] = analyseCustomerImage(customerImage,0.35,350);

                while(conv_match_ctr ~= (size(shape_color,2) - missingBlockMatch))
                    [moveConveyorFlag, PnPMessage, shape_color, conv_match_ctr,CmoveDirection] = Detection(conv_match_ctr, shape_color, 275,75);
                    if(moveConveyorFlag && CmoveDirection == 1)
                        PulseConv(socket_1,1);
                        posMatchNum = 0;
                        scanOnce = false;
                    elseif (moveConveyorFlag && CmoveDirection == 0)
                        PulseConv(socket_1,2);
                        posMatchNum = 0;
                        scanOnce = false;
                    elseif( PnPMessage.strlength > 1)
                        SendMessage(socket_1,"PNP");              
                        SendMessage(socket_1,PnPMessage);
                        LookForMessage(socket_1,"DONE");
                        fprintf("\n");
                        fprintf("Next block \n");
                        posMatchNum = 0;
                        scanOnce = false;
                    else
                        fprintf("Bad Message \n");
                    end
                    pause(0.5);
                end
            end
            
                        
            if(runtype == 3) SendMessage(socket_1,"CON"); end
            if(runtype == 4) SendMessage(socket_1,"COF"); end
            if(runtype == 5) SendMessage(socket_1,"VON"); end
            if(runtype == 6) SendMessage(socket_1,"VOF"); end
            if(runtype == 7) SendMessage(socket_1,"CFW"); end
            if(runtype == 8) SendMessage(socket_1,"CBK"); end
            
            SendMessage(socket_1,"HOM");
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

function PulseConv(socket_1,conveyorDirection)
    global app;
    time  = 1.0;
    if (conveyorDirection == 1)
        % send command for forward direction
        SendMessage(socket_1,"CFW");
        LookForMessage(socket_1,"DONE");
        SendMessage(socket_1,"CON");
        LookForMessage(socket_1,"DONE");
        app.ConRunLamp.Color = [0 1 0];
        app.ConRunButton.Value = 1;
        pause(time);
        SendMessage(socket_1,"COF");
        LookForMessage(socket_1,"DONE");
        app.ConRunLamp.Color = [1 0 0];
        app.ConRunButton.Value = 0;
    else
        % send command for backward direction
        SendMessage(socket_1,"CBK");
        LookForMessage(socket_1,"DONE");
        SendMessage(socket_1,"CON");
        LookForMessage(socket_1,"DONE");
        app.ConRunLamp.Color = [0 1 0];
        app.ConRunButton.Value = 1;
        pause(time);
        SendMessage(socket_1,"COF");
        LookForMessage(socket_1,"DONE");
        app.ConRunLamp.Color = [1 0 0];
        app.ConRunButton.Value = 0;
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

function message = FlushSocket(socket)
    flush = "";
    for i=1:2
        flush = flush + fgetl(socket);
    end
    fprintf("Flushed: %s \n", char(flush));
    return 
end
