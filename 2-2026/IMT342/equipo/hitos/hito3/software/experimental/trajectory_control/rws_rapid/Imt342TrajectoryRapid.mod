MODULE Imt342TrajectoryRapid
    RECORD ImtTrjOrder
        num seq;
        jointtarget startPose;
        jointtarget target;
        num moveTime;
    ENDRECORD

    RECORD ImtTrjReply
        num seq;
        num phase;
    ENDRECORD

    CONST num trjVersion := 2;
    PERS ImtTrjOrder trjOrder := [0,[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],
                                  [[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],0];
    PERS ImtTrjReply trjReply := [0,0];

    ! Experimental endpoint service. Run only as the selected motion routine.
    ! Relaunch from this entry, never by continuing a stopped MoveAbsJ.
    PROC ServeTrajectoryOrders()
        VAR ImtTrjOrder requested;
        IF trjOrder.seq <> trjReply.seq OR trjReply.phase = 1 OR trjReply.phase = 2 THEN
            trjReply := [trjOrder.seq,5];
        ENDIF
        WHILE TRUE DO
            requested := trjOrder;
            IF requested.seq > trjReply.seq THEN
                IF requested.seq = trjReply.seq + 1 AND requested.seq <= 8000000 THEN
                    trjReply := [requested.seq,1];
                    RunTrajectoryOrder requested;
                ELSE
                    trjReply := [requested.seq,4];
                ENDIF
            ENDIF
            WaitTime 0.05;
        ENDWHILE
    ENDPROC

    LOCAL FUNC bool TrjNoExternal(jointtarget poseQ)
        RETURN poseQ.extax.eax_a = 9E9 AND poseQ.extax.eax_b = 9E9 AND
               poseQ.extax.eax_c = 9E9 AND poseQ.extax.eax_d = 9E9 AND
               poseQ.extax.eax_e = 9E9 AND poseQ.extax.eax_f = 9E9;
    ENDFUNC

    LOCAL FUNC bool TrjAt(jointtarget target)
        VAR jointtarget actual;
        actual := CJointT();
        RETURN TrjNoExternal(actual) AND
               Abs(actual.robax.rax_1 - target.robax.rax_1) <= 0.30 AND
               Abs(actual.robax.rax_2 - target.robax.rax_2) <= 0.30 AND
               Abs(actual.robax.rax_3 - target.robax.rax_3) <= 0.30 AND
               Abs(actual.robax.rax_4 - target.robax.rax_4) <= 0.30 AND
               Abs(actual.robax.rax_5 - target.robax.rax_5) <= 0.30 AND
               Abs(actual.robax.rax_6 - target.robax.rax_6) <= 0.30;
    ENDFUNC

    LOCAL PROC RunTrajectoryOrder(ImtTrjOrder requested)
        IF (requested.moveTime < 0.1) OR (requested.moveTime > 30) THEN
            trjReply := [requested.seq,4];
            RETURN;
        ENDIF
        IF TrjNoExternal(requested.startPose) AND TrjNoExternal(requested.target) AND
           TrjAt(requested.startPose) THEN
            trjReply := [requested.seq,2];
            MoveAbsJ requested.target,v10\T:=requested.moveTime,fine,tool0;
            WaitRob \InPos;
            IF TrjAt(requested.target) THEN
                trjReply := [requested.seq,3];
            ELSE
                trjReply := [requested.seq,4];
            ENDIF
        ELSE
            trjReply := [requested.seq,4];
        ENDIF
    ERROR
        trjReply := [requested.seq,4];
        RAISE;
    ENDPROC
ENDMODULE
