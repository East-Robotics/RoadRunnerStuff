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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
@Autonomous
public class AutoFarBlue extends LinearOpMode {




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
        double DISTANCE = 33;

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
                    .forward(10)
                    .build();

            Trajectory trajectoryLeft = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .lineTo(new Vector2d(-25, 0))
                    .build();
            Trajectory trajectoryLeft2 = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(20)
                    .build();
            TrajectorySequence trajectoryLeftturn = drive.trajectorySequenceBuilder(trajectoryLeft2.end())
                    .turn(Math.toRadians(90))
                    .build();
            Trajectory trajleft3 = drive.trajectoryBuilder(trajectoryLeftturn.end())
                    .back(8)
                    .build();
            Trajectory trajleft4 = drive.trajectoryBuilder(trajleft3.end())
                    .forward(5)
                    .build();

            Trajectory trajectoryRight = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .lineTo(new Vector2d(-30, 0))
                    .build();
            Trajectory trajectoryRight2 = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(20)
                    .build();

            Trajectory trajectoryRightturn = drive.trajectoryBuilder(trajectoryRight.end())
                    .splineTo(new Vector2d(-20,10),0)
                    .build();
            Trajectory trajright3 = drive.trajectoryBuilder(trajectoryRightturn.end())
                    .back(5)
                    .build();
            Trajectory trajright4 = drive.trajectoryBuilder(trajright3.end())
                    .forward(5)
                    .build();

            telemetry.addData("Status", "Running");
            telemetry.update();


            double distanceInInches = sensorDistance.getDistance(DistanceUnit.INCH);
            double distanceRight = sensorDistanceR.getDistance(DistanceUnit.INCH);

            //check if distance is within 20 inches


            waitForStart();

            if (distanceInInches < 25) {

                telemetry.addData("Telemetry", "Prop is Left");
                telemetry.addData("Telemetry", sensorDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("Telemetry", sensorDistanceR.getDistance(DistanceUnit.INCH));
                telemetry.update();
                Prop = "Left";

            } else if (distanceRight < 25) {

                telemetry.addData("Telemetry", "Prop is Right");
                telemetry.addData("Telemetry", sensorDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("Telemetry", sensorDistanceR.getDistance(DistanceUnit.INCH));
                telemetry.update();
                Prop = "Right";
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
            } else if (Prop == "Left") {
                drive.followTrajectory(trajectoryLeft2);
                drive.followTrajectory(trajectoryLeft);
                drive.followTrajectorySequence(trajectoryLeftturn);
                drive.followTrajectory(trajleft3);
                drive.followTrajectory(trajleft4);
            } else if (Prop == "Right") {
                drive.followTrajectory(trajectoryRight2);
                drive.followTrajectory(trajectoryRight);
                drive.followTrajectory(trajectoryRightturn);
                drive.followTrajectory(trajright3);
                drive.followTrajectory(trajright4);

            }
            while (opModeIsActive()){

            }}}


