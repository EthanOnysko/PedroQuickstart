package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import com.pedropathing.geometry.Pose;

public class PathThing extends Path{
    public PathThing(Path.PathType type, Double angle) {
        this.pathType = type;
        this.angle = angle;
        this.pose = null;
    }
    public PathThing(Path.PathType type, Pose pose) {
        this.pathType = type;
        this.pose = pose;
        this.angle = null;
    }
}
