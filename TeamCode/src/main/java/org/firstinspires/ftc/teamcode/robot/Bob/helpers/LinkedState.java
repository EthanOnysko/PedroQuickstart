package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

public class LinkedState extends Link {
    public LinkedState(Link.LinkType type, int timeout, BobState nextState) {
        this.type = type;
        this.trigger = timeout;
        this.nextState = nextState;
    }
    public LinkedState(Link.LinkType type, BobState nextState) {
        this.type = type;
        this.nextState = nextState;
    }
}