function dispcallback(obj, event)
    global socket_2 app;
    str = fscanf(socket_2);
    str = string(str(1:end-1));
    fprintf( "!!!!!!!!!!!!!!!!!! Got a: %s \n", str);  
    switch str
        case ("CONVEYON")
            app.ConRunLamp.Color = 'g';
            app.ConRunLamp.Color = [0 1 0];
            app.ConRunButton.Value = 1;
            app.TextArea.Value = 'Conveyor Running...';
            fprintf( "%s \n",char(str));            
            drawnow
        case ("CONVEYOF")
            app.ConRunLamp.Color = 'r';
            app.ConRunLamp.Color = [1 0 0];
            app.ConRunButton.Value = 0;
            app.TextArea.Value = 'Conveyor Stopped'; 
            fprintf( "%s \n",char(str));
            drawnow
        case ("CONVDIR1")
            app.DirectionSwitch_Changed = 1;
            app.DirectionSwitch.Value = 'Forward';
            app.TextArea.Value = 'Conveyor Direction Forward'; 
            fprintf( "%s \n",char(str));            
            drawnow
        case ("CONVDIR0")
            app.DirectionSwitch_Changed = 0;
            app.DirectionSwitch.Value = 'Backward'; 
            app.TextArea.Value = 'Conveyor Direction Reverse';  
            fprintf( "%s \n",char(str));            
            drawnow
        case("SETSOLE1")
            app.VacSolLamp.Color = 'g';
            app.VacSolSwitch.Value = 'On';
            app.TextArea.Value = 'Vac Sol Running...';
            fprintf( "%s \n",char(str));            
            drawnow
        case("SETSOLE0")
            app.VacSolLamp.Color = 'r'; 
            app.VacSolSwitch.Value = 'Off';
            app.TextArea.Value = 'Vac Sol Stopped';
            fprintf( "%s \n",char(str));            
            drawnow
        case("VACUUMON")
            app.VacRunLamp.Color = 'g';
            app.VacRunButton.Value = 1;
            app.TextArea.Value = 'Vacuum Running...';
            fprintf( "%s \n",char(str));            
            drawnow
        case("VACUUMOF")
            app.VacRunLamp.Color = 'r';
            app.TextArea.Value = 'Vacuum Stopped';
            app.VacRunButton.Value = 0;
            fprintf( "%s \n",char(str));            
            drawnow
        case("ESTOP")
            app.TabGroup.SelectedTab = app.EStopTab;
            app.EmergencyStopLamp.Color = [1 0 0];
            app.DirectionSwitch.Value = 'Backward';
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            app.VacSolSwitch.Value = 'Off';
            app.VacRunButton.Value = 0;
            app.VacSolLamp.Color = [1 0 0];
            fprintf( "%s \n",char(str));            
            drawnow
        case("ESTOP2")
            app.TabGroup.SelectedTab = app.EStopTab;
            app.EmergencyStopLamp.Color = [1 0 0];
            app.DirectionSwitch.Value = 'Backward';
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            app.VacSolSwitch.Value = 'Off';
            app.VacRunButton.Value = 0;
            app.VacSolLamp.Color = [1 0 0];
            fprintf( "%s \n",char(str));            
            drawnow
        case("LIGHTCUR")
            app.TabGroup.SelectedTab = app.LightTab;
            app.LightCurtainBrokenLamp.Color = [1 0 0];
            app.DirectionSwitch.Value = 'Backward';
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            app.VacSolSwitch.Value = 'Off';
            app.VacRunButton.Value = 0;
            app.VacSolLamp.Color = [1 0 0];
            fprintf( "%s \n",char(str));            
            drawnow 
        case("MOTIONSUS")
%             go back to home position
        case("CONVEY")
            app.TabGroup.SelectedTab = app.ConveyTab;
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            app.ConveyorInterlockBrokenLamp.Color = [1 0 0];
            app.ConRunButton.Value = 0;
            app.ConRunLamp.Color = [1 0 0];
            fprintf( "%s \n",char(str));
            
            drawnow             
    end
end