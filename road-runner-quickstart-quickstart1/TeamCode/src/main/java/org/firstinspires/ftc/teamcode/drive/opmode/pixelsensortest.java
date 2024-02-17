import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp

public class pixelsensortest extends LinearOpMode {
    Servo minion;
    ColorSensor RColor;
    ColorSensor LColor;
    RevBlinkinLedDriver lights;

    private void UPJorgw() {
        double JorgeUPPosition = -0.5;
        minion.setPosition(JorgeUPPosition);
    }

    private void DOWNJorgw() {
        double JorgeDOWNPosition = 0.5;
        minion.setPosition(JorgeDOWNPosition);
    }

    @Override
    public void runOpMode() {
        RColor = hardwareMap.get(ColorSensor.class, "RColor");
        LColor = hardwareMap.get(ColorSensor.class, "LColor");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        minion = hardwareMap.get(Servo.class, "minion");

        waitForStart();

        while (opModeIsActive()) {
            int rRed = RColor.red();
            int rGreen = RColor.green();
            int rBlue = RColor.blue();

            int lRed = LColor.red();
            int lGreen = LColor.green();
            int lBlue = LColor.blue();

            telemetry.addData("RRed", rRed);
            telemetry.addData("RGreen", rGreen);
            telemetry.addData("RBlue", rBlue);

            telemetry.addData("LRed", lRed);
            telemetry.addData("LGreen", lGreen);
            telemetry.addData("LBlue", lBlue);
            telemetry.update();

            if ((rRed > 750 && rGreen > 750 && rBlue > 750) && (lRed > 750 && lGreen > 750 && lBlue > 750)) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                UPJorgw();
            } else if (rRed > 750 && rGreen > 750 && rBlue > 750) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                UPJorgw();
            } else if (lRed > 750 && lGreen > 750 && lBlue > 750) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                UPJorgw();
            } else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                DOWNJorgw();
            }
        }
    }
}
