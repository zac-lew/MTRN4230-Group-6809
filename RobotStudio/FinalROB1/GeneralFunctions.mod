MODULE GeneralFunctions
    PROC PrintCurPose()
        PrintPose CRobT(\Tool:=tSCup);
    ENDPROC
    
    PROC TurnVacOn()
        ! Set VacRun on.
        SetDO DO10_1, 1;
        SetDO DO10_2, 1;
    ENDPROC
    
    PROC TurnVacOff()
        ! Set VacRun off.
        SetDO DO10_2, 0;
        SetDO DO10_1, 0;
    ENDPROC
    
    PROC ConForward()
        ! Sets conveyor to move forward (to table)
        SetDO DO10_4, 1;
    ENDPROC
    
    PROC ConBackward()
        ! Sets conveyor to move backward (away from table)
        SetDO DO10_4, 0;
    ENDPROC
    
    PROC TurnConOn()
        ! Turns conveyor on
        IF DI10_1 = 1 THEN
            SetDO DO10_3, 1;
        ELSE
            SetDO DO10_3, 0;
        ENDIF
    ENDPROC
    
    PROC TurnConOff()
        ! Turns conveyor off
        SetDO DO10_3, 0;
    ENDPROC
    
    PROC PrintPose(robtarget P)
        TPWrite "x: " + NumToStr(P.trans.x,2) +
                " y: " + NumToStr(P.trans.y,2) +
                " z: " + NumToStr(P.trans.z,2) +
                " Q: " + NumToStr(P.rot.q1,3) +
                ", " + NumToStr(P.rot.q2,3) +
                ", " + NumToStr(P.rot.q3,3) +
                ", " + NumToStr(P.rot.q4,3);         
    ENDPROC
    
    PROC PrintJoint(jointtarget J)
        TPWrite "Joints = "  + NumToStr(J.robax.rax_1, 3) +
                ", " + NumToStr(J.robax.rax_2, 3) +
                ", " + NumToStr(J.robax.rax_3, 3) +
                ", " + NumToStr(J.robax.rax_4, 3) +
                ", " + NumToStr(J.robax.rax_5, 3) +
                ", " + NumToStr(J.robax.rax_6, 3);
    ENDPROC
ENDMODULE