package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.robotcore.external.navigation.*;

public class Robot {
//    public Robot() {}
        static DcMotor rightfront;
        static DcMotor leftfront;
        static DcMotor leftback;
        static DcMotor rightback;

        //static DcMotor intakemotor;

        static DcMotor slidemotor;

        static DcMotor paperlaucherv2;



        static DcMotor lift1;
        static DcMotor lift2;

        static TouchSensor slidelimiter;

        static DigitalChannel greenLED;


        static Servo claw;




    static Servo clawflipper;

        static Servo paperlaucher;



        static Servo pixelplacer;

        static DistanceSensor Distance;

        static DistanceSensor frontldist;

        static DistanceSensor frontrdist;

        static ColorSensor linedectecter;
        static RevBlinkinLedDriver led;







        static BNO055IMU imu;
        static Orientation lastAngles = new Orientation();

        static double globalAngle;

        //Slide heights
        static final int slideHigh = 9500;

        static final int slidemed = 100;
        static final int slideBottom = 25;

        static final int botreg = 1;

        static final double botslow = 0.25;

        static final int slidemin = 0;
        //Slide speed
        static final double slideSlow = .35;

        static double slowbot = 4;

        static int  slidepos;



        // change this one
        static final double clawoffset = 0.03;
        //
         static final double clawflipped = clawoffset + 0.05;

        static final double backflipclaw = clawoffset ;

        static final double clawgrab = .74;

        static final double clawrelease = .30;

        static final double clawretreat = clawoffset - 0.045;

        static final double laucherready = .43;

        static final double laucherReleased = .38;

        static final double pixelplaced = .6;

        static final double placepixel = 0.98;

        static final double pixelback = 0.2;





    public static void initMotors(OpMode opMode) {
        rightfront = opMode.hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = opMode.hardwareMap.get(DcMotor.class, "leftfront");
        leftback = opMode.hardwareMap.get(DcMotor.class, "leftback");
        rightback = opMode.hardwareMap.get(DcMotor.class, "rightback");
        //intakemotor = opMode.hardwareMap.get(DcMotor.class, "intakemotor");
        slidemotor = opMode.hardwareMap.get(DcMotor.class, "slidemotor");
        lift1 = opMode.hardwareMap.get(DcMotor.class, "leftEncoder");
        lift2 = opMode.hardwareMap.get(DcMotor.class, "rightEncoder");
        paperlaucherv2 = opMode.hardwareMap.get(DcMotor.class, "middleEncoder");

        claw = opMode.hardwareMap.get(Servo.class, "claw");
        clawflipper = opMode.hardwareMap.get(Servo.class, "clawflipper");
        paperlaucher = opMode.hardwareMap.get(Servo.class, "paperlaucher");
        pixelplacer = opMode.hardwareMap.get(Servo.class, "pixelplacer");

        led = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "led");

        slidelimiter = opMode.hardwareMap.get(TouchSensor.class, "slidelimiter");







        // Get the LED colors and touch sensor from the hardwaremap
        //redLED = opMode.hardwareMap.get(DigitalChannel.class, "red");
        //greenLED = opMode.hardwareMap.get(DigitalChannel.class, "green");

        frontldist = opMode.hardwareMap.get(DistanceSensor.class, "frontleftdistance");
        frontrdist = opMode.hardwareMap.get(DistanceSensor.class, "frontrightdistance");




        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        slidemotor.setDirection(DcMotor.Direction.REVERSE);
        paperlaucherv2.setDirection(DcMotor.Direction.REVERSE);


        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        paperlaucherv2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public static void resetMotors() {
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public static void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
//    public static void SetPower(double LPower, double RPower){
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        leftfront.setPower(LFPower);
        leftback.setPower(LBPower);
        rightfront.setPower(RFPower);
        rightback.setPower(RBPower);




    }
    public static void initIMU(OpMode opMode){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        //This line is only necessary if the the Control Hub is mounted vertically (as done this year)
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        opMode.telemetry.addData("Mode: ", "imu calibrating");
        opMode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
        }
        opMode.telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
        opMode.telemetry.update();
        resetAngle();
    }
    private static void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

}
