package org.firstinspires.ftc.teamcode.drive.autos;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_RIGHT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.testy;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

@Autonomous
public class forward extends OpMode {
    Bob bob = new Bob();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private boolean test = true;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0)); // Start Pose of our robot.
    public PathChain Path1;
    public PathChain Path2;

    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72, 72), new Pose(90, 72))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();


    }public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {

                    bob.runMacro(testy);
                    setPathState(-1); // done
                }
                break;
        }
    }


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        bob.tick();


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        bob.init(hardwareMap);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {


    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}

