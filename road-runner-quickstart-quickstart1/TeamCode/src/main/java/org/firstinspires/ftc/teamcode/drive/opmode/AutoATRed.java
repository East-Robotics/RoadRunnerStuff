package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "AutoATRed", group = "Concept")
public class AutoATRed extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor LFMotor, RFMotor, LBMotor, RBMotor, RightArm, LeftArm;
    private Servo LeftWrist, RightWrist, TrapdoorL, TrapdoorR;

    private boolean armRaised = false;
    private boolean trapdoorsRaised = false;
    private boolean wristRaised = false;

    private void raiseArm(double power, double duration) {
        if (!armRaised) {
            RightArm.setPower(power);
            LeftArm.setPower(power);

            ElapsedTime raiseTime = new ElapsedTime();
            raiseTime.reset();
            while (opModeIsActive() && raiseTime.seconds() < duration) {
                idle();
            }

            RightArm.setPower(0);
            LeftArm.setPower(0);

            armRaised = true;
        }
    }

    private void raiseWrist(double targetPosition) {
        if (!wristRaised) {
            LeftWrist.setPosition(targetPosition);
            RightWrist.setPosition(targetPosition);

            wristRaised = true;
        }
    }

    private void raiseTrapdoors(double LtargetPosition, double RtargetPosition) {
        if (!trapdoorsRaised) {
            TrapdoorL.setPosition(LtargetPosition);
            TrapdoorR.setPosition(RtargetPosition);

            trapdoorsRaised = true;
        }
    }

    private void driveBacktoBSOdometry(double distance) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(distance).build());
    }

    @Override
    public void runOpMode() {
        initAprilTag();
        initializeMotorsAndServos();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag();
                telemetry.update();

                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                sleep(20);
            }
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void initializeMotorsAndServos() {
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
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                if (detection.id == 1 || detection.metadata.name.equals("BlueAllianceLeft")) {
                    driveBacktoBSOdometry(8.0);
                    raiseArm(-0.5, 3.5);
                    raiseWrist(0.8);
                    raiseTrapdoors(0.6, 0.4);
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
