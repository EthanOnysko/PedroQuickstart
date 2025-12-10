package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

public class BobState {
    public Integer shooterRPM;
    public Double spindexerAngle;
    public Boolean spindexerAbsolute;  // ADD THIS - true = set absolute, false/null = increment
    public Double transferPosition;
    public PathThing path;
    public Double intakePower;
    public LinkedState linkedState;

    public BobState(Integer shooterRPM,
                    Double spindexerAngle,
                    Boolean spindexerAbsolute,  // ADD THIS
                    Double transferPosition,
                    Double intakePower,
                    PathThing path,
                    LinkedState linkedState) {
        this.shooterRPM = shooterRPM;
        this.spindexerAngle = spindexerAngle;
        this.spindexerAbsolute = spindexerAbsolute;  // ADD THIS
        this.transferPosition = transferPosition;
        this.intakePower = intakePower;
        this.path = path;
        this.linkedState = linkedState;
    }

    public BobState(Integer shooterRPM,
                    Double spindexerAngle,
                    Double transferPosition) {
        this(shooterRPM, spindexerAngle, null, transferPosition, null, null, null);
    }

    public BobState(Integer shooterRPM) {
        this(shooterRPM, null, null, null, null, null, null);
    }
}