package org.firstinspires.ftc.teamcode.drive.opmode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
@Autonomous
public class AutoFarRed extends LinearOpMode {
    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private ElapsedTime     runtime = new ElapsedTime();

    private DistanceSensor sensorDistance;

    private DistanceSensor sensorDistanceR;
    private DcMotor InTakeRight;

    private SampleMecanumDrive drive;

    String Prop;
    static final double     FORWARD_SPEED = 0.4;
 double DISTANCE = 30;

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


        Trajectory trajectoryLeft = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-15,-20), Math.toRadians(90))
                .build();
        Trajectory trajectoryLeft3 = drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .build();
        telemetry.addData("Status", "Running");
        telemetry.update();


        double distanceInInches = sensorDistance.getDistance(DistanceUnit.INCH);
        double distanceRight = sensorDistanceR.getDistance(DistanceUnit.INCH);

        //check if distance is within 20 inches


        waitForStart();

        if (distanceInInches < 25) {

            telemetry.addData("Telemetry", "Prop is Right");
            telemetry.update();
            Prop = "Right";

        } else if (distanceRight < 25) {

            telemetry.addData("Telemetry", "Prop is Left");
            telemetry.update();
            Prop = "Left";
        }
        else{
            telemetry.addData("Telemetry", "Prop is Center");
            telemetry.update();
            Prop = "Center";
        }
        if (Prop == "Center") {
            drive.followTrajectory(trajectoryBack1);
            drive.followTrajectory(trajectoryBack);
        } else if (Prop == "Left") {
            drive.followTrajectory(trajectoryLeft);
            drive.followTrajectory(trajectoryLeft3);
        } else if (Prop == "Right") {


        }
        while (opModeIsActive()){

    }}}

