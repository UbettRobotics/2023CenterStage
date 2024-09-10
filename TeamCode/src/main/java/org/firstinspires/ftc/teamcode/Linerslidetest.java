package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Linerslidetest")
public class Linerslidetest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);
        slidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();
        while (opModeIsActive()) {

            boolean islineup = false;

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
            boolean dxzpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;

             double margin = 0.1;

            if (x2){
               if(slidelimiter.isPressed()) {
                   slidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               }else {
                   slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                   slidemotor.setPower(-.35);
               }
            }else if(x1){
                slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slidemotor.setPower(0.35);

            } else if (a1) {
                slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slidemotor.setPower(-.35);
            } else if (RBumper1) {
                claw.setPosition(clawgrab);

            }else if(LBumper1) {
                clawflipper.setPosition(clawflipped);

            }else if(y1){
                clawflipper.setPosition(clawretreat);
            }else if(dpadDown2){
                slidemotor.setTargetPosition(2000);
                slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidemotor.setPower(0.7);
            }
            else{
                slidemotor.setPower(0);

            }
            if (dpadLeft1){
                paperlaucher.setPosition(laucherready);
            }
            if(dpadRight1){
                paperlaucher.setPosition(laucherReleased);
            }
            if (dpadUp1){

                clawflipper.setPosition(backflipclaw);
            }else{

            }
            if(b1){
                pixelplacer.setPosition(pixelplaced);
            }else{
                pixelplacer.setPosition(placepixel);
            }

            if(y2){
                paperlaucherv2.setPower(1);
            }
            if(a2){
                paperlaucherv2.setPower(0);
            }



            if(frontldist.getDistance(DistanceUnit.INCH) - frontrdist.getDistance(DistanceUnit.INCH) < margin){
             islineup = true;
            } else if (frontldist.getDistance(DistanceUnit.INCH) - frontrdist.getDistance(DistanceUnit.INCH) > -margin) {
                islineup = true;

            }else{
                islineup = false;
            }




            telemetry.addData("Slide Pos ", slidemotor.getCurrentPosition());
            telemetry.addData("Slide Power ", slidemotor.getPower());
            telemetry.addData("Servo Pos ", claw.getPosition());
            telemetry.addData("Flipper Pos:", clawflipper.getPosition());
            telemetry.addData("front left dist: ", frontldist.getDistance(DistanceUnit.INCH));
            telemetry.addData("front right dist: ", frontrdist.getDistance(DistanceUnit.INCH));
            telemetry.addData("Is line up? ", islineup);
            telemetry.addData("margin", margin);
            telemetry.addData("Color", linedectecter.red()+","+ linedectecter.green()+","+linedectecter.blue());
            telemetry.update();
        }
    }
}