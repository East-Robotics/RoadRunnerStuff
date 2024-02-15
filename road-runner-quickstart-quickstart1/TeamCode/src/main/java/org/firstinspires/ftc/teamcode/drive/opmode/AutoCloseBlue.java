package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
@Autonomous
public class AutoCloseBlue extends LinearOpMode {




    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private ElapsedTime runtime = new ElapsedTime();

    private DistanceSensor sensorDistance;

    private DistanceSensor sensorDistanceR;
    private DcMotor InTakeRight;

    private SampleMecanumDrive drive;

    String Prop;
    static final double     FORWARD_SPEED = 0.4;
    double DISTANCE = 28;



    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "LBDistance");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "RBDistance");
        int rotations = 900;
        drive = new SampleMecanumDrive(hardwareMap);
        //Pose2d startPose = new Pose2d(0, 0, 0);
        //drive.setPoseEstimate(startPose);
        Trajectory trajectoryBack = drive.trajectoryBuilder(new Pose2d())
                .back(DISTANCE)
                .build();
        Trajectory trajectoryBack1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(15)
                .build();
        Trajectory trajBack2 = drive.trajectoryBuilder(trajectoryBack.end())
                .forward(27)
                .build();
        TrajectorySequence trajBack3 = drive.trajectorySequenceBuilder(trajBack2.end())
                .turn(Math.toRadians(97))
                .build();
        TrajectorySequence trajBack4 = drive.trajectorySequenceBuilder(trajBack3.end())
                .back(35)
                .build();


        Trajectory trajectoryLeft = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineTo(new Vector2d(-15, 0))
                .build();
        Trajectory trajectoryLeft2 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(20)
                .build();
        TrajectorySequence trajectoryLeftturn = drive.trajectorySequenceBuilder(trajectoryLeft2.end())
                .turn(Math.toRadians(30))
                .build();
        Trajectory trajleft3 = drive.trajectoryBuilder(trajectoryLeftturn.end())
                .back(18)
                .build();
        Trajectory trajleft4 = drive.trajectoryBuilder(trajleft3.end())
                .forward(15)
                .build();
        TrajectorySequence trajleft5 = drive.trajectorySequenceBuilder(trajleft4.end())
                .turn(Math.toRadians(-30))
                .build();
        TrajectorySequence trajleft6 = drive.trajectorySequenceBuilder(trajleft5.end())
                .forward(18)
                .build();
        TrajectorySequence trajleft7 = drive.trajectorySequenceBuilder(trajleft6.end())
                .turn(Math.toRadians(97))
                .build();
        TrajectorySequence trajleft8 = drive.trajectorySequenceBuilder(trajleft7.end())
                .back(35)
                .build();
        TrajectorySequence trajleft9 = drive.trajectorySequenceBuilder(trajleft8.end())
                .back(40)
                .build();

        Trajectory trajectoryRight = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineTo(new Vector2d(-20, 0))
                .build();
        Trajectory trajectoryRight2 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(20)
                .build();

        TrajectorySequence trajectoryRightturn = drive.trajectorySequenceBuilder(trajectoryRight.end())
                .turn(Math.toRadians(-35))
                .build();
        Trajectory trajright3 = drive.trajectoryBuilder(trajectoryRightturn.end())
                .back(13)
                .build();
        Trajectory trajright4 = drive.trajectoryBuilder(trajright3.end())
                .forward(15)
                .build();
        TrajectorySequence trajright5 = drive.trajectorySequenceBuilder((trajright4.end()))
                .turn(Math.toRadians(35))
                .build();
        TrajectorySequence trajright6 = drive.trajectorySequenceBuilder(trajright5.end())
                .forward(20)
                .build();
        TrajectorySequence trajright7 = drive.trajectorySequenceBuilder(trajleft6.end())
                .turn(Math.toRadians(97))
                .build();
        TrajectorySequence trajright8 = drive.trajectorySequenceBuilder(trajleft7.end())
                .back(35)
                .build();
        telemetry.addData("Status", "Running");
        telemetry.update();


        double distanceInInches = sensorDistance.getDistance(DistanceUnit.INCH);
        double distanceRight = sensorDistanceR.getDistance(DistanceUnit.INCH);

        //check if distance is within 20 inches


        waitForStart();

        if (distanceInInches < 25) {

            telemetry.addData("Telemetry", "Prop is Right");
            telemetry.addData("Telemetry", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Telemetry", sensorDistanceR.getDistance(DistanceUnit.INCH));
            telemetry.update();
            Prop = "Right";

        } else if (distanceRight < 25) {

            telemetry.addData("Telemetry", "Prop is Left");
            telemetry.addData("Telemetry", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Telemetry", sensorDistanceR.getDistance(DistanceUnit.INCH));
            telemetry.update();
            Prop = "Left";
        }
        else{
            telemetry.addData("Telemetry", "Prop is Center");
            telemetry.addData("Telemetry", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Telemetry", sensorDistanceR.getDistance(DistanceUnit.INCH));
            telemetry.update();
            Prop = "Center";
        }
        if (Prop == "Center") {
            drive.followTrajectory(trajectoryBack1);
            drive.followTrajectory(trajectoryBack);
            drive.followTrajectory(trajBack2);
            drive.followTrajectorySequence(trajBack3);
            drive.followTrajectorySequence(trajBack4);
        } else if (Prop == "Left") {
            drive.followTrajectory(trajectoryLeft2);
            drive.followTrajectory(trajectoryLeft);
            drive.followTrajectorySequence(trajectoryLeftturn);
            drive.followTrajectory(trajleft3);
            drive.followTrajectory(trajleft4);
            drive.followTrajectorySequence(trajleft5);
            drive.followTrajectorySequence(trajleft6);
            drive.followTrajectorySequence(trajleft7);
            drive.followTrajectorySequence(trajleft8);
            LFMotor.setDirection(DcMotor.Direction.REVERSE);
            LBMotor.setDirection(DcMotor.Direction.FORWARD);
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
            RBMotor.setDirection(DcMotor.Direction.REVERSE);
            drive.followTrajectorySequence(trajleft9);
            RBMotor.setDirection(DcMotor.Direction.FORWARD);
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if (Prop == "Right") {
            drive.followTrajectory(trajectoryRight2);
            drive.followTrajectory(trajectoryRight);
            drive.followTrajectorySequence(trajectoryRightturn);
            drive.followTrajectory(trajright3);
            drive.followTrajectory(trajright4);
            drive.followTrajectorySequence(trajright5);
            drive.followTrajectorySequence(trajright6);
            drive.followTrajectorySequence(trajright7);
            drive.followTrajectorySequence(trajright8);

        }
        while (opModeIsActive()){

        }}}


