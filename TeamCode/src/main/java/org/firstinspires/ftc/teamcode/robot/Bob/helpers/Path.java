package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import com.pedropathing.geometry.Pose;

public abstract class Path {
    public PathType pathType;
    public Double angle;
    public Pose pose;

    public enum PathType {
        ROTATE,
        MOVE
    }
}
