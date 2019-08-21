% Aloysius Lee 
% Robot Engineer Ink Printing 
% This file transfer data to Robot Studios 
% 21/08

% This file is run with run_text_detection

function inkFlag = ALCommunications( data,thickOrThin)

%     robot_IP_address = '192.168.125.1';
    robot_IP_address = '127.0.0.1'; % Simulation ip address

    robot_port = 1025;

    socket = tcpip(robot_IP_address, robot_port);
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

    % data = load('sample_x_y.mat');
    % data = load('a_stroke.mat');
    % data.data = data.a_stroke_2; 
    % data = load('b_stroke.mat');
    % data = data.b_stroke_1; 
    % data.data = data.b_stroke_2;

    str = ""; 
    i = 0;
    while (isequal(get(socket, 'Status'), 'open')) 
        disp('Data Sending');
        initial = "[" + length(data) + "," + thickOrThin + "]"; 
        %initial = "[" + length(data) + "," + thickOrThin + "]"; 
        fwrite(socket,initial);
        % str = fgetl(socket);
        % get the all clear for the first set of data
        while (~strcmp(str,'DONE'))
            disp('Waiting for DONE');
            str = fgetl(socket);
            disp(str);
        end
        str = ""; 
        n = 1;
        %  send the whole array 
        while n <= length(data)
            send_str = "[" + num2str(data(n,1)*1000 ) + "," + num2str(data(n,2)*1000) + "]"; 
            disp(send_str)
            fwrite(socket, send_str); 
            % str = fgetl(socket);
            % get the all clear for array data 
            while (~strcmp(str,"DONE"))
                disp('Waiting for DONE');
                str = fgetl(socket);
                disp(str);
            end
            str = "";
            n=n+1;
            i = i + 1;
            disp(i);
        end
        % end line 
        while (~strcmp(str,"MoveComplete"))
            disp('Waiting for MoveComplete');
            str = fgetl(socket);
            disp(str);
        end
        break; 
    end
    pause(1);

    % Close the socket.
    fclose(socket);
    inkFlag = 1;
end









