MODULE InkMove  

    PROC PrintLetter(num thick_thin)
        VAR robtarget Pos1 := pZeros; 
        VAR num Table_upOff := zTab + 50;
        VAR speeddata inkSpeed := v100;
        
        IF thick_thin = 0 THEN 
            inkSpeed := v50;
        ELSEIF thick_thin = 1 THEN 
            inkSpeed := v100;
        ELSE 
            TPWrite "Thickness error: " + ValToStr(thick_thin);
        ENDIF
        
        ! move to the starting position 
        Pos1.trans := [DrawPt{1,1},DrawPt{1,2},Table_upOff];
        ! Move just above the object
        MoveJ Offs(Pos1,0,0,0), v200, fine, tSCup;
        
        ! ----------------            
        ! networking - ink printing has started
                    
        FOR i FROM 1 TO inkSize - 1 DO
            Pos1.trans := [DrawPt{i,1},DrawPt{i,2}, Table_upOff];
            TPWrite "i = " + ValToStr(i);
            !PrintPose Pos1;
            MoveL Pos1, inkSpeed, fine, tSCup;
        ENDFOR
            
        ! networking - say printing is completed
        Out_str := "DONE";
        
        ! ----------------
        ! A point just above the target 
        MoveJ Offs(Pos1,0,0,50), v200, fine, tSCup;
        ! MoveToCalibPos;
    ENDPROC
    
ENDMODULE