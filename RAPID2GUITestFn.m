clc; close all;

%% IMPORTS 
if( ~exist('detector_updated_FINAL','var')) load('FINAL_FRCNN_V5.mat'); end
if( ~exist('R_Conv','var'))                 load('CalibConv.mat'); end
if( ~exist('R_Table','var'))                load('CalibTable.mat'); end
global detector_updated_FINAL;
global camParam_Conv R_Conv t_Conv;
global camParam_Table R_Table t_Table;

%% CONNECTION
global socket_1 socket_2;

robot_IP_address = '127.0.0.1'; % Simulated
%robot_IP_address = '192.168.125.1'; % Real
robot_port_1 = 1025; robot_port_2 = 1026;

socket_1 = Connect(robot_IP_address, robot_port_1);
socket_2 = Connect(robot_IP_address, robot_port_2);
set(socket_2, 'ReadAsyncMode', 'continuous');
set(socket_2, 'BytesAvailableFcn', 'dispcallback');


%% GUI
app = BasicGUINewRev3();
disp('GUI OPEN');

if(~isequal(get(socket_1, 'Status'), 'open'))
        fopen(socket_1);
        disp('Connected');
        app.TextArea.Value = 'Robot Connected SUCCESS';  
        app.ConnectionStatusLamp.Color = 'g';
else
        fprintf('Could not open TCP connection to %s on port %d\n',robot_IP_address, robot_port);
        app.TextArea.Value = 'Robot Connection FAILED';  
        app.ConnectionStatusLamp.Color = 'r';
        drawnow
end

while(isequal(get(socket_1, 'Status'), 'open'))
    app.ReadyforCustomerOrderLamp,Color = 'g';
    uiwait(gcf); %Continues when figure is closed
    app.PlacingLetterBlocksLamp.Color = 'g';
    app.TextArea.Value = 'Placing Chocolates';
    drawnow

    % --------------------------------CALL FUNCTIONS HERE----------------------------------

    MattCommTest_7Fn(app)

    app.CAKEREADYLamp.Color = 'g';
    app.TextArea.Value = 'Cake Finished';
    drawnow   

    if(app.DirectionSwitch_Changed)
        SwitchCommand(app.DirectionSwitch.Value,socket_1,'Backward','CBK','CFW');
        app.DirectionSwitch_Changed = 0;
        drawnow
    end 
    
    if(app.ConRunButton_Pressed)
        ToggleCommand(app.ConRunButton.Value,socket_1,'CON','COF');
        app.ConRunButton_Pressed = 0;
        drawnow
    end
    
    if(app.ReconnectButton_Pressed)
        try
            fopen(socket_1);
            disp('Reconnected');
            app.ConnectionStatusLamp.Color = 'g';
        catch
            disp('Reconnection Failed');
            app.ConnectionStatusLamp.Color = 'r';
        end
        app.ReconnectButton_Pressed = 0;
    end
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

function Str2GUI(app,str)
    switch str
        case ('CONVEYON')
            app.ConRunLamp.Color = 'g';
            app.ConRunLamp.Color = [0 1 0];
            app.ConRunButton.Value = 1;
            app.TextArea.Value = 'Conveyor Running...';
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case ('CONVEYOF')
            app.ConRunLamp.Color = 'r';
            app.ConRunLamp.Color = [1 0 0];
            app.ConRunButton.Value = 0;
            app.TextArea.Value = 'Conveyor Stopped'; 
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case ('CONVDIR1')
            app.DirectionSwitch_Changed = 1;
            app.DirectionSwitch.Value = 'Forward';
            app.TextArea.Value = 'Conveyor Direction Forward'; 
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case ('CONVDIR0')
            app.DirectionSwitch_Changed = 0;
            app.DirectionSwitch.Value = 'Backward'; 
            app.TextArea.Value = 'Conveyor Direction Reverse';  
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case('SETSOLE1')
            app.VacSolLamp.Color = 'g';
            app.VacSolSwitch.Value = 'On';
            app.TextArea.Value = 'Vac Sol Running...';
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case('SETSOLE0')
            app.VacSolLamp.Color = 'r'; 
            app.VacSolSwitch.Value = 'Off';
            app.TextArea.Value = 'Vac Sol Stopped';
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case('VACUUMON')
            app.VacRunLamp.Color = 'g';
            app.VacRunButton.Value = 1;
            app.TextArea.Value = 'Vacuum Running...';
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case('VACUUMOF')
            app.VacRunLamp.Color = 'r';
            app.TextArea.Value = 'Vacuum Stopped';
            app.VacRunButton.Value = 0;
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case('ESTOP')
            app.TabGroup.SelectedTab = app.EStopTab;
            app.EmergencyStopLamp.Color = [1 0 0];
            app.DirectionSwitch.Value = 'Backward';
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            app.VacSolSwitch.Value = 'Off';
            app.VacRunButton.Value = 0;
            app.VacSolLamp.Color = [1 0 0];
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case('ESTOP2')
            app.TabGroup.SelectedTab = app.EStopTab;
            app.EmergencyStopLamp.Color = [1 0 0];
            app.DirectionSwitch.Value = 'Backward';
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            app.VacSolSwitch.Value = 'Off';
            app.VacRunButton.Value = 0;
            app.VacSolLamp.Color = [1 0 0];
            fprintf(char(str));
            fprintf('\n');
            drawnow
        case('LIGHTCUR')
            app.TabGroup.SelectedTab = app.LightTab;
            app.LightCurtainBrokenLamp.Color = [1 0 0];
            app.DirectionSwitch.Value = 'Backward';
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            app.VacSolSwitch.Value = 'Off';
            app.VacRunButton.Value = 0;
            app.VacSolLamp.Color = [1 0 0];
             fprintf(char(str));
            fprintf('\n');
            drawnow 
        case('MOTIONSUS')
%             go back to home position
        case('CONVEY')
            app.TabGroup.SelectedTab = app.ConveyTab;
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            app.ConveyorInterlockBrokenLamp.Color = [1 0 0];
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            fprintf(char(str));
            fprintf('\n');
            drawnow             
    end        
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

