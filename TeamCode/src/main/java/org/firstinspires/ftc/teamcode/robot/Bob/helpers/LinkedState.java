package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

public class LinkedState extends Link {
    public LinkedState(Link.LinkType type, int timeout, BobState nextState) {
        this.type = type;
        this.trigger = timeout;
        this.nextState = nextState;
    }
}