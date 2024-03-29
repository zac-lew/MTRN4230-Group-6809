clc; close all;
dbstop if error;

%% IMPORTS 
if( ~exist('detector_updated_FINAL','var')) load('FINAL_FRCNN_V5.mat'); end
if( ~exist('R_Conv','var'))                 load('CalibConv.mat'); end
if( ~exist('R_Table','var'))                load('CalibTable.mat'); end
global detector_updated_FINAL;
global camParam_Conv R_Conv t_Conv;
global camParam_Table R_Table t_Table;

%% CONNECTION
global socket_1 socket_2 Offline;

robot_IP_address = '192.168.125.1'; % Real

Offline = false;
% Offline = true;
if(Offline)
    %robot_IP_address = '127.0.0.1'; % Simulated
end
robot_port_1 = 1025; robot_port_2 = 1026;

socket_1 = Connect(robot_IP_address, robot_port_1);
socket_2 = Connect(robot_IP_address, robot_port_2);
set(socket_2, 'BytesAvailableFcn', 'dispcallback');


%% GUI
global app;
%app = BasicGUINewRev3();
app = BasicGUINewRev4();
disp('GUI OPEN');

while(isequal(get(socket_1, 'Status'), 'open'))
    app.ReadyforCustomerOrderLamp,Color = 'g';
    uiwait(gcf); %Continues when figure is closed
    drawnow

    % --------------------------------CALL FUNCTIONS HERE----------------------------------
    MattCommTest_7Fn(app,0) % Change number depending on what is run

    app.CAKEREADYLamp.Color = 'g';
    app.TextArea.Value = 'Cake Finished';
    drawnow   

    if(app.DirectionSwitch_Changed)
        SwitchCommand(app.DirectionSwitch.Value,socket_1,'Backward','CBK','CFW');
        %MattCommTest_7Fn(app,8)
        app.DirectionSwitch_Changed = 0;
        drawnow
    end 
    
    if(app.ConRunButton_Pressed)
        ToggleCommand(app.ConRunButton.Value,socket_1,'CON','COF');
        app.ConRunButton_Pressed = 0;
        
        drawnow
    end
    
%     if(app.ReconnectButton_Pressed)
%         try
%             fopen(socket_1);
%             disp('Reconnected');
%             app.ConnectionStatusLamp.Color = 'g';
%         catch
%             disp('Reconnection Failed');
%             app.ConnectionStatusLamp.Color = 'r';
%         end
%         app.ReconnectButton_Pressed = 0;
%     end
end

%% Functions

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

function SwitchCommand(Switch,socket,state1,command0,command1)   
    if(strcmp(Switch,state1))
        fwrite(socket,command1);
        disp(command1);
    else
        fwrite(socket,command0);
        disp(command0);
    end
end

function ToggleCommand(Toggle,socket,command0,command1)
    if(Toggle == 1)
        fwrite(socket,command1);
        disp(command1);
    else
        fwrite(socket,command0);
        disp(command0);
    end
end

