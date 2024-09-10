package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Teleop1")
public class OutReachTeleOp extends LinearOpMode {

    int RBumper2switch = 0;
    int RTrigger2Switch = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);
        slidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        paperlaucher.setPosition(laucherready);
        pixelplacer.setPosition(placepixel);


        waitForStart();
        while (opModeIsActive()) {

            boolean LStickIn2 = gamepad2.left_stick_button;
            boolean RStickIn2 = gamepad2.right_stick_button;
            boolean LBumper1 = gamepad1.left_bumper;
            boolean RBumper1 = gamepad1.right_bumper;

            double LStickY = gamepad1.left_stick_y;
            double LStickX = gamepad1.left_stick_x;
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = -gamepad1.right_stick_x;

            double LTrigger1 = gamepad1.left_trigger; //
            double RTrigger1 = gamepad1.right_trigger; //

            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;

            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;

            double LTrigger2 = gamepad2.left_trigger;
            double RTrigger2 = gamepad2.right_trigger;
            boolean LBumper2 = gamepad2.left_bumper;
            boolean RBumper2 = gamepad2.right_bumper;

            double RStickY2 = -gamepad2.right_stick_y;
            double RStickX2 = gamepad2.right_stick_x;
            double LStickY2 = -gamepad2.left_stick_y;
            double LStickX2 = gamepad2.left_stick_x;

            boolean dpadUp1 = gamepad1.dpad_up;
            boolean dpadDown1 = gamepad1.dpad_down;
            boolean dpadRight1 = gamepad1.dpad_right;
            boolean dpadLeft1 = gamepad1.dpad_left;

            boolean dpadUp2 = gamepad2.dpad_up;
            boolean dpadDown2 = gamepad2.dpad_down;
            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;




            if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
                //Orientation angles = imu.getAngularOrientation();
                double rotation = 0; //Math.toRadians(angles.firstAngle);
                /*
                if (Math.abs(LStickX) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickY, LStickY, LStickY, LStickY);
                }
                else if (Math.abs(LStickY) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickX, -LStickX, -LStickX, LStickX);//+--+
                }
                */

                double newX = -LStickX * Math.cos(rotation) - -LStickY * Math.sin(rotation); //Angle Difference Identity
                double newY = LStickY * Math.cos(rotation) - -LStickX * Math.sin(rotation); //Trigonometry

                double r = Math.hypot(newX, newY);
                double robotAngle = Math.atan2(newY, newX) - Math.PI / 4;
                double rightX = -gamepad1.right_stick_x;

                double v1 = r * Math.cos(robotAngle) + rightX; //lf
                double v2 = r * Math.sin(robotAngle) - rightX; //rf
                double v3 = r * Math.sin(robotAngle) + rightX; //lb
                double v4 = r * Math.cos(robotAngle) - rightX; //rb


                SetPower(v1, v2, v3, v4);

                //Trigger Turning
            } else if (Math.abs(LTrigger1) > 0) {
                SetPower(.2 * LTrigger1, -.2 * LTrigger1, .2 * LTrigger1, -.2 * LTrigger1); //.25
            } else if (Math.abs(RTrigger1) > 0) {
                SetPower(-.2 * RTrigger1, .2 * RTrigger1, -.2 * RTrigger1, .2 * RTrigger1); //.25
            } else if (LBumper1) {
                SetPower(.5, -.5, -.5, .5);
            } else if (RBumper1) {
                SetPower(-.5, .5, .5, -.5);
                //dpad driving
            } else if (dpadUp1) {
                SetPower(-.3 * 1, -.3 * 1, -.3 * 1, -.3 * 1); //0.3
            } else if (dpadRight1) {
                SetPower(-.5, .5, .5, -.5); //0.5
            } else if (dpadLeft1) {
                SetPower(.5, -.5, -.5, .5);
            } else if (dpadDown1) {
                SetPower(.3 * 1, .3 * 1, .3 * 1, .3 * 1);
            } else {
                SetPower(0, 0, 0, 0);

            }
            if (x1) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

            } else if (a1) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            } else if (b1) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else if (y1) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }


            //failsafe

//controller 2 ;;;;1200, 5
            if (dpadDown2){
                if(slidelimiter.isPressed()) {
                    slidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }else {
                    slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slidemotor.setPower(-.35);
                }
            }else if(x2){
                slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slidemotor.setPower(0.35);

            } else if (a2) {
                slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slidemotor.setPower(-.35);
            }else{
                slidemotor.setPower(0);

           }
            if (RStickIn2) {
                pixelplacer.setPosition(pixelplaced);
            }


            if (dpadUp2 && LStickIn2) {
                paperlaucher.setPosition(laucherReleased);
            }


            /*
            if(a2 && slidemotor.getCurrentPosition()>150) {
                clawflipper.setPosition(clawflipped);
                sleep(900);
                claw.setPosition(clawopen);
                sleep(500);
                clawflipper.setPosition(backflipclaw);
            }

            */
            if(RBumper2) {
                if(RBumper2switch == 0){
                    claw.setPosition(clawgrab);
                    gamepad1.rumble(250);
                    gamepad2.rumble(250);
                    RBumper2switch = 1;
                }
                if (RBumper2switch == 2){
                    claw.setPosition(clawrelease);
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                    RBumper2switch = 3;
                }

            }else {
                if (RBumper2switch == 1 && !RBumper2) {
                    RBumper2switch = 2;
                } else if (RBumper2switch == 3 && !RBumper2) {
                    RBumper2switch = 0;
                }
            }
            if(RTrigger2 > 0.1) {
                if(RTrigger2Switch == 0){
                    clawflipper.setPosition(clawflipped);
                    RTrigger2Switch = 1;
                }
                if (RTrigger2Switch == 2){
                    clawflipper.setPosition(backflipclaw);
                    RTrigger2Switch = 3;
                }

            }else {
                if (RTrigger2Switch == 1 && RTrigger2 < 0.1) {
                    RTrigger2Switch = 2;
                }else if(RTrigger2Switch == 3 && RTrigger2 < 0.1) {
                    RTrigger2Switch = 0;
                }
            }



            if(b2){
                clawflipper.setPosition(clawretreat);
            }
            if (y2 && slidemotor.getCurrentPosition() <= 200){
                clawflipper.setPosition(backflipclaw);
                claw.setPosition(clawrelease);
            }


            if (dpadUp2){
                pixelplacer.setPosition(0);
            }



            if(dpadRight2){
                lift1.setPower(1);
                lift2.setPower(1);
            } else if (dpadLeft2) {
                lift1.setPower(-1);
                lift2.setPower(-1);
            } else {
                lift1.setPower(0);
                lift2.setPower(0);
            }




            /*
            if(slidemotor.getCurrentPosition() > 250) {
                clawflipper.setPosition(clawflipped);
            } else if(slidemotor.getCurrentPosition() > 240) {
                clawflipper.setPosition(backflipclaw);
            }
             */
                    //dpad driving

            /*
            if (Math.abs(LStickY) > 0) {
                SetPower(LStickY, LStickY, LStickY, LStickY);
            }
            else if(Math.abs(RStickX) > 0) {
                SetPower(RStickX, -RStickX, RStickX, -RStickX);
            }
            //turning
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(-LTrigger1, LTrigger1, LTrigger1, -LTrigger1);
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(RTrigger1, -RTrigger1, -RTrigger1, RTrigger1);
            }


             */

                    telemetry.addData("Slide Pos ", slidemotor.getCurrentPosition());
                    telemetry.addData("Slide Power ", slidemotor.getPower());
                    telemetry.addData("Claw Pos ", claw.getPosition());
                    telemetry.addData(" Clawflipper Pos ", clawflipper.getPosition());
                    telemetry.addData("Buttons",a2 +""+b2+""+x2+""+y2);
                    telemetry.addData("Clawasdasda", RBumper2switch);

                    telemetry.update();

            }
        }
    }

