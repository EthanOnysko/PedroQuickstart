package org.firstinspires.ftc.teamcode.robot.Bob.helpers;


public class BobState {
    public Integer shooterRPM;
    public Double spindexerAngle;
    public Double transferPosition;
    public Double intakePower;
    public LinkedState linkedState;


    public BobState(Integer shooterRPM,
                    Double spindexerAngle,
                    Double transferPosition,
                    Double intakePower,
                    LinkedState linkedState) {
        this.shooterRPM = shooterRPM;
        this.spindexerAngle = spindexerAngle;
        this.transferPosition = transferPosition;
        this.intakePower = intakePower;
        this.linkedState = linkedState;
    }

    public BobState(Integer shooterRPM,
                    Double spindexerAngle,
                    Double transferPosition) {
        this(shooterRPM, spindexerAngle, transferPosition, null, null);
    }

    public BobState(Integer shooterRPM) {
        this(shooterRPM, null, null, null, null);
    }
}