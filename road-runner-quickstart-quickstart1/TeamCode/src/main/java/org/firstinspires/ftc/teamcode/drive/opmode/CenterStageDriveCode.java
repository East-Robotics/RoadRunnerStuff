package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class CenterStageDriveCode extends LinearOpMode {
    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private DcMotor IntakeRight;
    private DcMotor RightArm;
    private DcMotor LeftArm;
    private Servo LeftWrist;
    private Servo RightWrist;
    private Servo TrapdoorL;
    private Servo TrapdoorR;
    private Servo Plane;

    @Override
    public void runOpMode() {
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        IntakeRight = hardwareMap.get(DcMotor.class, "IntakeRight");
        RightArm = hardwareMap.get(DcMotor.class, "RightArm");
        LeftArm = hardwareMap.get(DcMotor.class, "LeftArm");
        LeftWrist = hardwareMap.get(Servo.class, "LeftWrist");
        RightWrist = hardwareMap.get(Servo.class, "RightWrist");
        Plane = hardwareMap.get(Servo.class, "Plane");
        TrapdoorL = hardwareMap.get(Servo.class, "TrapdoorL");
        TrapdoorR = hardwareMap.get(Servo.class, "TrapdoorR");
        TrapdoorR.setDirection(Servo.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeRight.setDirection(DcMotor.Direction.FORWARD);
        RightWrist.setDirection(Servo.Direction.REVERSE);
        boolean rightwristIsOpen = true;
        boolean leftwristIsOpen = true;
        boolean intakerun = true;
        boolean LtrapdoorIsOpen = true;
        boolean RtrapdoorIsOpen = true;
        double Rightwristposition;
        double Leftwristposition;
        double Ltrapdoorposition;
        double Rtrapdoorposition;

        boolean currentAState = false;
        boolean lastAState = false;
        boolean lastXState = false;
        boolean currentXState = false;
        boolean lastLBState = false;
        boolean currentYState = false;
        boolean lastYState = false;
        boolean lastLB2State = false;
        boolean lastRB2State = false;
        boolean currentLB2State = false;
        boolean currentRB2State = false;
        boolean currentLBState = false;
//        boolean trapdoor = true;



        telemetry.addData("Status", "Running");
        telemetry.update();
        waitForStart();
//    public double PIDControl(double reference, double state){
//
//    }

        while(opModeIsActive()) {
            double RFtgtPower = 0;
            double LFtgtPower = 0;
            double RBtgtPower = 0;
            double LBtgtPower = 0;
            double py = -gamepad1.left_stick_y;
            double px = gamepad1.left_stick_x;
            double pa = gamepad1.right_stick_x;

            LFMotor.setPower((py-px+pa) / 1.5);
            RFMotor.setPower((py+px-pa) / 1.5);
            LBMotor.setPower((py+px+pa) / 1.5);
            RBMotor.setPower((py-px-pa) / 1.5);

            currentAState = gamepad2.a;
            currentXState = gamepad2.x;
            currentLBState = gamepad1.left_bumper;
            currentLB2State = gamepad2.left_bumper;
            currentRB2State = gamepad2.right_bumper;




            if (gamepad1.dpad_up){
                LFMotor.setPower(0.7);
                RFMotor.setPower(0.7);
                LBMotor.setPower(0.7);
                RBMotor.setPower(0.7);
            }
            if (gamepad1.dpad_down){
                LFMotor.setPower(-0.7);
                RFMotor.setPower(-0.7);
                LBMotor.setPower(-0.7);
                RBMotor.setPower(-0.7);
            }
            if (gamepad1.dpad_right){
                LFMotor.setPower(-0.7);
                RFMotor.setPower(0.7);
                LBMotor.setPower(0.7);
                RBMotor.setPower(-0.7);
            }
            if (gamepad1.dpad_left){
                LFMotor.setPower(0.7);
                RFMotor.setPower(-0.7);
                LBMotor.setPower(-0.7);
                RBMotor.setPower(0.7);
            }
            if (gamepad1.x){
                Plane.setPosition(0);
            }
            else {
                Plane.setPosition(1);
            }

            if(currentLBState && !lastLBState){
                intakerun = !intakerun;
            }

            lastLBState = currentLBState;

            if (intakerun){
                IntakeRight.setPower(0);
            }
            else {
                IntakeRight.setPower(1);
            }

            if(currentXState && !lastXState){
                leftwristIsOpen = !leftwristIsOpen;
                rightwristIsOpen = !rightwristIsOpen;
            }

            lastXState = currentXState;

            if (leftwristIsOpen && rightwristIsOpen){
                Leftwristposition = 0.23;
                Rightwristposition = 0.23;

            }
            else{
                Leftwristposition = 0.65;
                Rightwristposition = 0.65;
            }
            LeftWrist.setPosition(Leftwristposition);
            RightWrist.setPosition(Rightwristposition);


            if (gamepad2.dpad_up){

/*        int rotations = -5000;

        RightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightArm.setTargetPosition(rotations);
        LeftArm.setTargetPosition(rotations);

        RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

 */
                RightArm.setPower(-1);
                LeftArm.setPower(-1);
            }
            else{
                RightArm.setPower(0);
                LeftArm.setPower(0);
            }
            if (gamepad2.dpad_down){
/*                int rotations = 5000;

        RightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightArm.setTargetPosition(rotations);
        LeftArm.setTargetPosition(rotations);

        RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

  */
                RightArm.setPower(1);
                LeftArm.setPower(1);
            }
            else{
                RightArm.setPower(0);
                LeftArm.setPower(0);
            }
            if(currentLB2State && !lastLB2State){
        LtrapdoorIsOpen = !LtrapdoorIsOpen;
            }

            lastLB2State = currentLB2State;

            if (LtrapdoorIsOpen){
                TrapdoorL.setPosition(0.2);

            }
            else{
                TrapdoorL.setPosition(0.6);

            }
            if(currentRB2State && !lastRB2State){
                RtrapdoorIsOpen = !RtrapdoorIsOpen;
            }

            lastRB2State = currentRB2State;

            if (RtrapdoorIsOpen){
                TrapdoorR.setPosition(0.0);

            }
            else{
                TrapdoorR.setPosition(0.4);

            }
            //TrapdoorL.setPosition(Ltrapdoorposition);
            //TrapdoorR.setPosition(Rtrapdoorposition);
        }

    }

}
