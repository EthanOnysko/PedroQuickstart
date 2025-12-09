package org.firstinspires.ftc.teamcode.drive.autos;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.BALL_PROX;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_RIGHT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

@Autonomous
public class forward extends OpMode {
    Bob bob = new Bob();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean waiting = false;

    private final Pose startPose = new Pose(104.123, 83.692, Math.toRadians(0)); // Start Pose of our robot.
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;


    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(104.123, 83.692),
                                new Pose(108, 83.692)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(108, 83.692),
                                new Pose(114, 83.692)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(114, 83.692),
                                new Pose(122.338, 83.692)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(122.338, 83.692),
                                new Pose(128, 83.692)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();


    }
    public void waitThenRun(double seconds){
        if (!waiting) {
            actionTimer.resetTimer();
            waiting = true;
        }
        if (seconds == 0 || actionTimer.getElapsedTimeSeconds() > seconds) {
            waiting = false;
            setPathState(pathState + 1);
        }
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (bob.isBall()){
                        bob.runMacro(SPINDEXER_RIGHT);
                        setPathState(2);
                    }

                }
                break;
            case 2:
                waitThenRun(.5);
                break;
            case 3:
                follower.followPath(Path2);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    if (bob.isBall()){
                        bob.runMacro(SPINDEXER_RIGHT);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                waitThenRun(.5);
                break;
            case 6:
                follower.followPath(Path3);
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1);
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
        actionTimer = new Timer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        bob.init(hardwareMap);
        bob.intakeController.intake();

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

