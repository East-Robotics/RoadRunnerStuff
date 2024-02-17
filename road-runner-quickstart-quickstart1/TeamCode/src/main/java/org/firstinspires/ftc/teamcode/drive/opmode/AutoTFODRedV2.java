package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "AutoTFODRedV2", group = "Concept")
public class AutoTFODRedV2 extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "AutoTFODRed.tflite";
    private static final String[] LABELS = {"RedProp"};
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private DcMotor LFMotor, RFMotor, LBMotor, RBMotor;
    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean redPropDetected = false;

    // Constants
    private static final double BACKWARD_DISTANCE = 20.0;
    private static final double TURN_ANGLE_RIGHT = -35.0;
    private static final double TURN_ANGLE_LEFT = 35.0;
    private static final double DRIVE_CENTER_DISTANCE = 20.0;
    private static final double RESET_POSITION_DISTANCE = 7.5;

    private static final float MIN_CONFIDENCE = 0.45F;
    private static final double RIGHT_SIDE_MIN_X = 345;
    private static final double RIGHT_SIDE_MAX_X = 600;
    private static final double LEFT_SIDE_MIN_X = 60;
    private static final double LEFT_SIDE_MAX_X = 345;

    @Override
    public void runOpMode() throws InterruptedException {
        // Motor initialization...
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

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
                    resetposition();
                    driveBackwardAndTurnRight();
                    driveCenterAfterUsingOdometry();
                } else if (x < LEFT_SIDE_MAX_X && x > LEFT_SIDE_MIN_X) { // detects left (center spike) side
                    telemetry.addData("Center", "");
                    resetposition();
                    driveCenterUsingOdometry();
                }
            }
        }

        // Check if no recognitions meet the criteria for "Right" or "Center"
        if (!redPropDetected) {
            telemetry.addData("Left", "");
            resetposition();
            driveBackwardAndTurnLeft();
            driveCenterAfterUsingOdometry();
        }
    }

    private void driveCenterUsingOdometry() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(DRIVE_CENTER_DISTANCE).build());
    }

    private void driveBackwardAndTurnRight() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(BACKWARD_DISTANCE).build());
        drive.turn(Math.toRadians(TURN_ANGLE_RIGHT));
    }

    private void driveBackwardAndTurnLeft() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(BACKWARD_DISTANCE).build());
        drive.turn(Math.toRadians(TURN_ANGLE_LEFT));
    }

    private void driveCenterAfterUsingOdometry() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(DRIVE_CENTER_DISTANCE).build());
    }

    private void resetposition() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(RESET_POSITION_DISTANCE).build());
        drive.turn(Math.toRadians(RESET_POSITION_DISTANCE));
    }
}
