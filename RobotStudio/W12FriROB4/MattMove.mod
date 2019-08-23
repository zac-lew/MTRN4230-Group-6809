MODULE MattMove
	! Test code for picking and placing objects in a loop    
    CONST num zTab := 147;
    CONST num zCon := 22.1;
    CONST robtarget pZeros := [[0,0,0],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];   

    ! format is xi, yi, xf, yf, yaw angle change;
    VAR num aPnP{5};
    !VAR string myStrArr := "[-100,300,550,0,45]";
    VAR bool ok;
    
    PROC MattMain()
        TPWrite "------S------";
        TPWrite In_str;
        
        IF In_str = "0" THEN
            TPWrite "Empty string";
            TPWrite "In_str: " + In_str + "   Out_str: " + Out_str; 
        ELSEIF Out_str = "DONE" THEN
            TPWrite "Not yet sent";
            TPWrite "In_str: " + In_str + "   Out_str: " + Out_str; 
        ELSE
            ok := StrToVal(IN_str,aPnP);
            TPWrite ValToStr(aPnP{1}) + " " + ValToStr(aPnP{2}) + " " + ValToStr(aPnP{3}) + " " + ValToStr(aPnP{4}) + " " + ValToStr(aPnP{5});
            In_Str := "0";
            
            MoveToCalibPos;
            PrintCurPose;
            TurnVacOff;
            
            PickNPlaceXYA aPnP{1}, aPnP{2}, aPnP{3}, aPnP{4}, aPnP{5};
            
            MoveToCalibPos;
            Out_Str := "DONE";
            TPWrite "In_str: " + In_str + "   Out_str: " + Out_str;  
        ENDIF
        
        TPWrite "-------F------";
        
    ENDPROC

    PROC PickNPlaceXYA(num xi, num yi, num xf, num yf, num angle)
        VAR robtarget pPick := pZeros;
        VAR robtarget pPlace := pZeros;       
        VAR num upOff := 80;
        VAR speeddata spdFast := v150;
        VAR speeddata spdMidi := v50;
        VAR speeddata spdSlow := v20;

        IF TRUE THEN
            spdFast := v1500;
            spdMidi := v500;
            spdSlow := v200;
        ENDIF
        
        pPick.trans := [xi,yi,zCon];
        pPlace.trans := [xf,yf,zTab];
        pPlace.rot := OrientZYX(180 + angle,0,180);               
        
        TPWrite "Starting PNP xya";
        PrintPose(pPick);
        PrintPose(pPlace);
        
        ! Picking
        MoveJ Offs(pPick,0,0,upOff),    spdFast, fine, tSCup;  
        MoveL pPick,                    spdSlow, fine, tSCup;
        TurnVacOn;
        TPWrite "Vac on";   
        WaitTime 0.5;
        MoveL Offs(pPick,0,0,200),      spdMidi, fine, tSCup;
        
        ! Move just above the target place
        ! Angle anticlockwise
        MoveJ Offs(pPlace,0,0,upOff),   spdFast, fine, tSCup;
        MoveL pPlace,                   spdSlow, fine, tSCup;
        TurnVacOff;
        TPWrite "Vac off";  
        MoveJ Offs(pPlace,0,0, 100),    spdMidi, fine, tSCup;
    ENDPROC
ENDMODULE