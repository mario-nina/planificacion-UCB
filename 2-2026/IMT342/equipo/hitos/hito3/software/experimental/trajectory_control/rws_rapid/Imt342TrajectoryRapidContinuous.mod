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

    RECORD ImtTrjTune
        num seq;
        num ratio;
    ENDRECORD

    RECORD ImtTrjLinear
        num seq;
        num mode;
    ENDRECORD

    RECORD ImtTrjTiming
        num seq;
        num moveElapsed;
        num waitElapsed;
        num baseAcc;
        num usedAcc;
    ENDRECORD

    CONST num trjVersion := 2;
    PERS ImtTrjOrder trjOrder := [0,[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],
                                  [[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],0];
    PERS ImtTrjReply trjReply := [0,0];
    PERS ImtTrjTune trjTune := [0,1];
    PERS ImtTrjLinear trjLinear := [0,0];
    PERS ImtTrjTiming trjTiming := [0,0,0,0,0];

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
        VAR num savedAcc;
        VAR num savedRamp;
        VAR num savedFine;
        VAR num limitedAcc;
        VAR bool changed := FALSE;
        VAR clock moveClock;
        VAR num elapsedMove;
        VAR num elapsedWait;
        VAR num usedAcc;
        VAR robtarget linearTarget;
        changed := FALSE;
        IF trjLinear.seq <> requested.seq OR (trjLinear.mode <> 0 AND trjLinear.mode <> 1) OR
           (requested.moveTime < 0.1) OR (requested.moveTime > 30) THEN
            trjReply := [requested.seq,4];
            RETURN;
        ENDIF
        IF TrjNoExternal(requested.startPose) AND TrjNoExternal(requested.target) AND
           TrjAt(requested.startPose) THEN
            ! The optional host ratio is empirical, never an absolute axis acceleration.
            ! It can only reduce the current modal limit, for this exact sequence.
            savedAcc := C_MOTSET.acc.acc;
            savedRamp := C_MOTSET.acc.ramp;
            savedFine := C_MOTSET.acc.finepramp;
            IF trjTune.seq = requested.seq AND trjTune.ratio >= 0.8 AND
               trjTune.ratio < 1 AND savedAcc >= 20 AND savedAcc <= 100 AND
               savedRamp >= 10 AND savedRamp <= 100 AND savedFine > 0 AND savedFine <= 100 THEN
                limitedAcc := savedAcc * trjTune.ratio;
                IF limitedAcc < 20 THEN
                    limitedAcc := 20;
                ENDIF
                IF limitedAcc < savedAcc THEN
                    changed := TRUE;
                    AccSet limitedAcc,savedRamp\FinePointRamp:=savedFine;
                ENDIF
            ENDIF
            usedAcc := C_MOTSET.acc.acc;
            trjReply := [requested.seq,2];
            ClkReset moveClock;
            ClkStart moveClock;
            IF trjLinear.mode = 1 THEN
                ! One controller-interpolated straight pass, no intermediate stops.
                ConfL \On;
                linearTarget := CalcRobT(requested.target,tool0 \WObj:=wobj0);
                MoveL linearTarget,v10\T:=requested.moveTime,fine,tool0\WObj:=wobj0;
            ELSE
                MoveAbsJ requested.target,v10\T:=requested.moveTime,fine,tool0;
            ENDIF
            elapsedMove := ClkRead(moveClock);
            WaitRob \InPos;
            ClkStop moveClock;
            elapsedWait := ClkRead(moveClock) - elapsedMove;
            IF changed THEN
                AccSet savedAcc,savedRamp\FinePointRamp:=savedFine;
                changed := FALSE;
            ENDIF
            trjTiming := [requested.seq,elapsedMove,elapsedWait,savedAcc,usedAcc];
            IF TrjAt(requested.target) THEN
                trjReply := [requested.seq,3];
            ELSE
                trjReply := [requested.seq,4];
            ENDIF
        ELSE
            trjReply := [requested.seq,4];
        ENDIF
    ERROR
        IF changed THEN
            AccSet savedAcc,savedRamp\FinePointRamp:=savedFine;
        ENDIF
        trjReply := [requested.seq,4];
        RAISE;
    ENDPROC
ENDMODULE
