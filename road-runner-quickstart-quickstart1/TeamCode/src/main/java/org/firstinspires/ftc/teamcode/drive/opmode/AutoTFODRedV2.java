package org.firstinspires.ftc.teamcode.drive.opmode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import java.util.List;

@Autonomous(name = "AutoTFODRedV2", group = "Concept")

public class AutoTFODRedV2 extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "AutoTFODRed.tflite";
    private static final String[] LABELS = {
            "RedProp",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private DcMotor LFMotor, RFMotor, LBMotor, RBMotor;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean redPropDetected = false;
    private void driveCenterUsingOdometry() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Drive backward for the specified distance using odometry pods
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .back(32.0)
                .build()
        );
    }

    private void driveBackwardAndTurnRight() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Drive 20 inches backward
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .back(20.0)
                .build()
        );
        // Turn right
        drive.turn(Math.toRadians(-30)); // Turn right 30 degrees
    }

    private void driveBackwardAndTurnLeft() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Drive 20 inches backward
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .back(20.0)
                .build()
        );
        // Turn left
        drive.turn(Math.toRadians(30)); // Turn left 30 degrees
    }
    private void driveCenterAfterUsingOdometry() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Drive backward for the specified distance using odometry pods
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .build()
        );
    }
    private void resetposition() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .forward(7.5)
                .build()
        );
        drive.turn(Math.toRadians(7.5));
    }
    @Override
    public void runOpMode() throws InterruptedException {
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
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
                sleep(1000);
            }
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }   // end runOpMode()
    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
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
        builder.addProcessor(tfod);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.45f);
        // Disable or re-enable the TFOD processor at any time.
        // visionPortal.setProcessorEnabled(tfod, false);
    }   // end method initTfod()

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (recognition.getConfidence() >= 0.45) {
                redPropDetected = true;
                telemetry.addData("Red Prop Detected: ", "Confidence: %s", recognition.getConfidence());
            } else {
                telemetryTfod();
            }
            if (redPropDetected && (x > 345 && x < 600)) { //detects right side
                telemetry.addData("Right", "");
            } else if (redPropDetected && (x < 345 && x > 60)) { //detects left (center spike) side
                telemetry.addData("Center", "");
            } else {
                telemetry.addData("Left", "");
            }
        }

    }


}   // end class
