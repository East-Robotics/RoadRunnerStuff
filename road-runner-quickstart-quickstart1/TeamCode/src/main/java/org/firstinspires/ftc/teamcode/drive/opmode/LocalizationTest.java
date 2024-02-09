package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(group = "drive")
public class LocalizationTest extends LinearOpMode {
    private DcMotor LFMotor = null;
    private DcMotor RFMotor = null;
    private DcMotor LBMotor = null;
    private DcMotor RBMotor = null;

    private Encoder leftEncoder;

    private Encoder rightEncoder;

    private Encoder frontEncoder;

    //private DcMotor LeftArm = null;

    {

    }


    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        //LeftArm = hardwareMap.get(DcMotor.class, "LeftArm");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LBMotor"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RFMotor"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Encoder"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//nothing plugged into front encoder port

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()) {
            double RFtgtPower = 0;
            double LFtgtPower = 0;
            double RBtgtPower = 0;
            double LBtgtPower = 0;
            double py = -gamepad1.left_stick_y;
            double px = gamepad1.left_stick_x;
            double pa = gamepad1.right_stick_x;

            LFMotor.setPower((py - px + pa) / 2);
            RFMotor.setPower((py + px - pa) / 2);
            LBMotor.setPower((py + px + pa) / 2);
            RBMotor.setPower((py - px - pa) / 2);



        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );


            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.update();
            telemetry.addData("y", poseEstimate.getY());
            telemetry.update();
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
        }
    }
}
