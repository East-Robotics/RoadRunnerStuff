/*package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Vector2d;


import java.util.List;

@Autonomous(name = "AutoTFODRedV2", group = "Concept")
public class AutoTFODRedV2 extends LinearOpMode {
    private DcMotor LFMotor, RFMotor, LBMotor, RBMotor, RightArm, LeftArm;
    private Servo LeftWrist, RightWrist, TrapdoorL, TrapdoorR;

    private boolean armRaised = false;
    private boolean trapdoorsRaised = false;
    private boolean wristRaised = false;

    private void raiseArm() {
        if (!armRaised) {
            RightArm.setPower(1);
            LeftArm.setPower(1);

            ElapsedTime raiseTime = new ElapsedTime();
            raiseTime.reset();
            while (opModeIsActive() && raiseTime.seconds() < 1.0) {
                idle();
            }

            RightArm.setPower(0);
            LeftArm.setPower(0);

            armRaised = true;
        }
    }

    private void raiseWrist() {
        if (!wristRaised) {
            LeftWrist.setPosition(1);
            RightWrist.setPosition(1);

            wristRaised = true;
        }
    }

    private void raiseTrapdoors() {
        if (!trapdoorsRaised) {
            TrapdoorL.setPosition(1);
            TrapdoorR.setPosition(1);

            trapdoorsRaised = true;
        }
    }

    RevBlinkinLedDriver lights;
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "AutoTFODRed.tflite";
    private static final String[] LABELS = {"RedProp"};
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean redPropDetected = false;

    // Constants
    private static final double BACKWARD_DISTANCE = 20.0;
    private static final double TURN_ANGLE_RIGHT = -35.0;
    private static final double TURN_ANGLE_LEFT = 35.0;
    private static final double DRIVE_CENTER_DISTANCE = 30.0;
    private static final double RESET_POSITION_DISTANCE = 7.5;

    private static final float MIN_CONFIDENCE = 0.45F;
    private static final double RIGHT_SIDE_MIN_X = 345;
    private static final double RIGHT_SIDE_MAX_X = 600;
    private static final double LEFT_SIDE_MIN_X = 60;
    private static final double LEFT_SIDE_MAX_X = 345;
    private static String prop;

    @Override
    public void runOpMode() throws InterruptedException {
        // Motor initialization...
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
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
                .turn(Math.toRadians(-97))
                .build();
        TrajectorySequence trajBack4 = drive.trajectorySequenceBuilder(trajBack3.end())
                .back(35)
                .build();
        TrajectorySequence trajBack5 = drive.trajectorySequenceBuilder(trajBack4.end())
                .back(15)
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
                .turn(Math.toRadians(-97))
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
                .turn(Math.toRadians(-32))
                .build();
        Trajectory trajright3 = drive.trajectoryBuilder(trajectoryRightturn.end())
                .back(15)
                .build();
        Trajectory trajright4 = drive.trajectoryBuilder(trajright3.end())
                .forward(15)
                .build();
        TrajectorySequence trajright5 = drive.trajectorySequenceBuilder((trajright4.end()))
                .turn(Math.toRadians(32))
                .build();
        TrajectorySequence trajright6 = drive.trajectorySequenceBuilder(trajright5.end())
                .forward(20)
                .build();
        TrajectorySequence trajright7 = drive.trajectorySequenceBuilder(trajright6.end())
                .turn(Math.toRadians(-97))
                .build();
        TrajectorySequence trajright8 = drive.trajectorySequenceBuilder(trajright7.end())
                .back(35)
                .build();
        TrajectorySequence trajright9 = drive.trajectorySequenceBuilder(trajright8.end())
                .back(15)
                .build();

        // Drive initialization...
        drive = new SampleMecanumDrive(hardwareMap);

        // TensorFlow initialization...
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && !redPropDetected) {
                telemetryTfod();
                // Push telemetry to the Driver Station.
                telemetry.update();
                // Share the CPU.
                sleep(10);
            }
        }
        // Save more CPU resources when the camera is no longer needed.
        visionPortal.close();
    }

    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(MIN_CONFIDENCE);
    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (recognition.getConfidence() >= MIN_CONFIDENCE) {
                redPropDetected = true;
                telemetry.addData("Red Prop Detected: ", "Confidence: %s", recognition.getConfidence());

                if (x > RIGHT_SIDE_MIN_X && x < RIGHT_SIDE_MAX_X) { // detects right side
                    telemetry.addData("Right", "");
                    prop = "Right";
                    resetPosition();
                    drive.followTrajectory(trajectoryRight2);
                    drive.followTrajectory(trajectoryRight);
                    drive.followTrajectorySequence(trajectoryRightturn);
                    drive.followTrajectory(trajright3);
                    drive.followTrajectory(trajright4);
                    drive.followTrajectorySequence(trajright5);
                    drive.followTrajectorySequence(trajright6);
                    // drive.followTrajectorySequence(trajright7);
                    //  drive.followTrajectorySequence(trajright8);
                    LFMotor.setDirection(DcMotor.Direction.REVERSE);
                    LBMotor.setDirection(DcMotor.Direction.FORWARD);
                    RFMotor.setDirection(DcMotor.Direction.FORWARD);
                    RBMotor.setDirection(DcMotor.Direction.REVERSE);
                    // drive.followTrajectorySequence(trajright9);
                    RBMotor.setDirection(DcMotor.Direction.FORWARD);
                    RFMotor.setDirection(DcMotor.Direction.FORWARD);
                } else if (x < LEFT_SIDE_MAX_X && x > LEFT_SIDE_MIN_X) { // detects left (center spike) side
                    telemetry.addData("Center", "");
                    prop = "Center";
                    resetPosition();
                    drive.followTrajectory(trajectoryBack1);
                    drive.followTrajectory(trajectoryBack);
                    drive.followTrajectory(trajBack2);
                    //   drive.followTrajectorySequence(trajBack3);
                    //   drive.followTrajectorySequence(trajBack4);
                    LFMotor.setDirection(DcMotor.Direction.REVERSE);
                    LBMotor.setDirection(DcMotor.Direction.FORWARD);
                    RFMotor.setDirection(DcMotor.Direction.FORWARD);
                    RBMotor.setDirection(DcMotor.Direction.REVERSE);
                    //  drive.followTrajectorySequence(trajBack5);
                    RBMotor.setDirection(DcMotor.Direction.FORWARD);
                    RFMotor.setDirection(DcMotor.Direction.FORWARD);
                }
            }
        }

        // Check if no recognitions meet the criteria for "Right" or "Center"
        if (!redPropDetected) {
            telemetry.addData("Left", "");
            prop = "Left";
            resetPosition();
            drive.followTrajectory(trajectoryLeft2);
            drive.followTrajectory(trajectoryLeft);
            drive.followTrajectorySequence(trajectoryLeftturn);
            drive.followTrajectory(trajleft3);
            drive.followTrajectory(trajleft4);
            drive.followTrajectorySequence(trajleft5);
            drive.followTrajectorySequence(trajleft6);
            //  drive.followTrajectorySequence(trajleft7);
            //   drive.followTrajectorySequence(trajleft8);
            LFMotor.setDirection(DcMotor.Direction.REVERSE);
            LBMotor.setDirection(DcMotor.Direction.FORWARD);
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
            RBMotor.setDirection(DcMotor.Direction.REVERSE);
            //   drive.followTrajectorySequence(trajleft9);
            RBMotor.setDirection(DcMotor.Direction.FORWARD);
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    private void driveForCenter() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(DRIVE_CENTER_DISTANCE).build());
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(-DRIVE_CENTER_DISTANCE).build());
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(15).build());
        strafeLeft(15);
        raiseArm();
        raiseWrist();
        raiseTrapdoors();
    }

    private void driveForRight() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(BACKWARD_DISTANCE).build());
        drive.turn(Math.toRadians(TURN_ANGLE_RIGHT));
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(DRIVE_CENTER_DISTANCE).build());
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(-DRIVE_CENTER_DISTANCE).build());
        drive.turn(Math.toRadians(-TURN_ANGLE_RIGHT));
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(-BACKWARD_DISTANCE).build());
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(15).build());
        strafeLeft(10);
        raiseArm();
        raiseWrist();
        raiseTrapdoors();
    }

    private void driveForLeft() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(BACKWARD_DISTANCE).build());
        drive.turn(Math.toRadians(TURN_ANGLE_LEFT));
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(DRIVE_CENTER_DISTANCE).build());
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(-DRIVE_CENTER_DISTANCE).build());
        drive.turn(Math.toRadians(-TURN_ANGLE_LEFT));
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(-BACKWARD_DISTANCE).build());
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(15).build());
        strafeLeft(20);
        raiseArm();
        raiseWrist();
        raiseTrapdoors();
    }

    private void resetPosition() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(RESET_POSITION_DISTANCE).build());
        drive.turn(Math.toRadians(RESET_POSITION_DISTANCE));
    }

    private void strafeLeft(double distance) {
        // Calculate the target position for strafing left
        double targetY = drive.getPoseEstimate().getY() - distance;

        // Create a trajectory to strafe left
        Trajectory strafeTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), targetY, 0))
                .build();

        // Follow the strafe trajectory
        drive.followTrajectory(strafeTrajectory);

        // Wait for the strafe to complete
        while (opModeIsActive() && drive.isBusy()) {
            // You can add additional actions or checks here if needed
            // For example, checking for collisions or other conditions
            idle();
        }
    }
}*/
