package org.firstinspires.ftc.teamcode.drive.autos;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.INTAKE_THEN_SPIN;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE_AUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

@Autonomous
public class Test_1_0 extends OpMode {
    Bob bob = new Bob();

    private boolean waiting = false;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean shoot = false;

    private final Pose startPose = new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)); // Start Pose of our robot.

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path3_5;
    public PathChain Path3_5_5;
    public PathChain Path4;

    public void buildPaths() {


            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.646, 9.354), new Pose(72.123, 71.877))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.123, 71.877),
                                    new Pose(83.938, 82.708),
                                    new Pose(104.123, 83.692)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(104.123, 83.692),
                                    new Pose(108, 83.692)
                            )
                    )
                    .setVelocityConstraint(5)

                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        Path3_5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(104.123, 83.692),
                                new Pose(114, 83.692)
                        )
                )
                .setVelocityConstraint(5)

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Path3_5_5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(104.123, 83.692),
                                new Pose(122.338, 83.692)
                        )
                )
                .setVelocityConstraint(5)

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(122.338, 83.692),
                                    new Pose(89.600, 83.446),
                                    new Pose(72.123, 72.123)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();


    }

    public boolean t(double seconds){
        if (!waiting) {
            actionTimer.resetTimer();
            waiting = true;
        }
        if (actionTimer.getElapsedTimeSeconds() > seconds) {
            waiting = false;
            return true;
        }
        return false;
    }

    public void waitThenRun(double seconds){
        if (!waiting) {
            actionTimer.resetTimer();
            waiting = true;
        }
        if (seconds == 0 || actionTimer.getElapsedTimeSeconds() > seconds) {
            waiting = false;
            setP(getP() + 1);
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);

                setP(1);

                break;

            case 1:
                if (!follower.isBusy()) {
                    bob.runMacro(SHOOT_ALL_THREE_AUTO);
                    setP(2);
                }
                break;
            case 2:

                    waitThenRun(2.4);

                break;

            case 3:

                follower.followPath(Path2);
                setP(4);

                break;

            case 4:
                if (!follower.isBusy()) {
                    bob.runMacro(INTAKE_THEN_SPIN);
                    t(.2);
                    follower.followPath(Path3);
                    setP(5);


                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    t(.3);
                    follower.followPath(Path3_5);
                    setP(6);

                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    t(.3);
                    follower.followPath(Path3_5_5);
                    setP(7);

                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Path4);
                    setP(8);

                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    bob.runMacro(SHOOT_ALL_THREE_AUTO);
                    setP(-1);
                }
                break;

            default:
                break;
        }
    }




    public void setP(int p) {
        pathState = p;
        pathTimer.resetTimer();
    }
    public int getP() {
        return pathState;
    }

    public void bigTick(){
        follower.update();
        autonomousPathUpdate();
        bob.tick();
    }
    @Override
    public void loop() {

        bigTick();


        telemetry.addData("spincoder ticks", bob.spincoder.getCurrentPosition());
        telemetry.addData("spindexer power", bob.spindexer.getPower());
        telemetry.addData("MACROING", bob.MACROING);
        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

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
        setP(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}

