package org.firstinspires.ftc.teamcode.drive.autos;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE_AUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

@Autonomous
public class Test_1_0 extends OpMode {
    Bob bob = new Bob();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean shoot = false;

    private final Pose startPose = new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)); // Start Pose of our robot.

public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;

    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(86.892, 9.354), new Pose(72.123, 71.631))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.123, 71.631),
                                new Pose(85.908, 83.200),
                                new Pose(101.908, 83.446)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(101.908, 83.446), new Pose(72.123, 71.631))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    private static final double SHOOT_TIMEOUT_S = 2500;
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    bob.runMacro(SHOOT_ALL_THREE_AUTO);

                    setPathState(2);
                }
                break;

            case 2:
                if (!bob.MACROING) {
                    setPathState(3);
                }
                break;

            case 3:
                follower.followPath(Path2);
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path3);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                // idle
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


        telemetry.addData("spincoder ticks", bob.spincoder.getCurrentPosition());
        telemetry.addData("spindexer power", bob.spindexer.getPower());
        telemetry.addData("MACROING", bob.MACROING);
        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        bob.tick();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        bob.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        bob.transferController.setDown();
        bob.runMacro(SHOOTER_ZONE1);

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}

