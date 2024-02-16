package org.firstinspires.ftc.teamcode.drive.opmode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous
public class AutoCloseRed extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor RightArm;
    private DcMotor LeftArm;
    private Servo LeftWrist;
    private Servo RightWrist;
    private Servo TrapdoorL;
    private Servo TrapdoorR;
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
    private void raiseArm() {
        // Set target position for the arm motors (example values)
        int targetPosition = 1000; // Adjust as needed
        RightArm.setTargetPosition(targetPosition);
        LeftArm.setTargetPosition(targetPosition);

        // Set power to raise the arm
        double power = 1.0; // Adjust as needed
        RightArm.setPower(power);
        LeftArm.setPower(power);

        // Wait until arm motors reach target position
        while (opModeIsActive() && (RightArm.isBusy() || LeftArm.isBusy())) {
            // Continue looping until arm motors reach target position
            // You can add additional logic or telemetry here if needed
            idle(); // Yield CPU to other threads
        }

        // Stop arm motors
        RightArm.setPower(0);
        LeftArm.setPower(0);
    }
    private void raiseWrist() {
        // Set position to raise the wrist (example values)
        double targetPosition = 0.65; // Adjust as needed
        LeftWrist.setPosition(targetPosition);
        RightWrist.setPosition(targetPosition);
    }
    private void raiseTrapdoors() {
        // Set position to raise the trapdoors (example values)
        double LtargetPosition = 0.6; // Adjust as needed
        double RtargetPosition = 0.4;
        TrapdoorL.setPosition(LtargetPosition);
        TrapdoorR.setPosition(RtargetPosition);
    }
    private void driveBacktoATOdometry() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Drive backward for the specified distance using odometry pods
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .back(5.0)
                .build()
        );
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void ATwork() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                // Check if the ID is 4 or the name is "RedAllianceLeft"
                if (detection.id == 6 || detection.metadata.name.equals("RedAllianceRight")) {
                    driveBacktoATOdometry();
                    raiseArm();
                    raiseWrist();
                    raiseTrapdoors();
                }
            } else if (detection.id == 5 || detection.metadata.name.equals("RedAllianceCenter")) {
                driveBacktoATOdometry();
                raiseArm();
                raiseWrist();
                raiseTrapdoors();
            } else if (detection.id == 4 || detection.metadata.name.equals("RedAllianceLeft")) {
                driveBacktoATOdometry();
                raiseArm();
                raiseWrist();
                raiseTrapdoors();
            }
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }

           // end for() loop

        // Add "key" information to telemetry


       // end method telemetryAprilTag()
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        RightArm = hardwareMap.get(DcMotor.class, "RightArm");
        LeftArm = hardwareMap.get(DcMotor.class, "LeftArm");
        LeftWrist = hardwareMap.get(Servo.class, "LeftWrist");
        RightWrist = hardwareMap.get(Servo.class, "RightWrist");
        TrapdoorL = hardwareMap.get(Servo.class, "TrapdoorL");
        TrapdoorR = hardwareMap.get(Servo.class, "TrapdoorR");
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
                .turn(Math.toRadians(-97))
                .build();
        TrajectorySequence trajBack4 = drive.trajectorySequenceBuilder(trajBack3.end())
                .back(25)
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
                .back(25)
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
                .back(13)
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
                .back(25)
                .build();
        TrajectorySequence trajright9 = drive.trajectorySequenceBuilder(trajright8.end())
                .back(15)
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
            LFMotor.setDirection(DcMotor.Direction.REVERSE);
            LBMotor.setDirection(DcMotor.Direction.FORWARD);
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
            RBMotor.setDirection(DcMotor.Direction.REVERSE);
            drive.followTrajectorySequence(trajBack5);
            RBMotor.setDirection(DcMotor.Direction.FORWARD);
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
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
            LFMotor.setDirection(DcMotor.Direction.REVERSE);
            LBMotor.setDirection(DcMotor.Direction.FORWARD);
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
            RBMotor.setDirection(DcMotor.Direction.REVERSE);
            drive.followTrajectorySequence(trajright9);
            RBMotor.setDirection(DcMotor.Direction.FORWARD);
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
            ATwork();

        while (opModeIsActive()){

        }}}}




