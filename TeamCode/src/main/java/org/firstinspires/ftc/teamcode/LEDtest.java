package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Robot.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "LedTest")
public class LEDtest extends LinearOpMode{
    final double pat = 0.3;
    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);



        waitForStart();
        while (opModeIsActive()) {

            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;

            if(a1){
                while(true){
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK);
                    sleep(2500);
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM);
                    sleep(2500);


                }
            }
        }
    }
}







