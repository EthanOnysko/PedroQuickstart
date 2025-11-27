package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

public abstract class Link {
    public BobState nextState = null;
    public int trigger = 0;
    public LinkType type = LinkType.WAIT;

    public enum LinkType {
        WAIT
    }
}