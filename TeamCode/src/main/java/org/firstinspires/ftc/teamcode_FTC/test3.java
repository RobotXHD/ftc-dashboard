/*

UltimateGoal01

Holonomic Drive

* sqrt transfer function
* normalized power

2020.12.06

*/

package org.firstinspires.ftc.teamcode_FTC;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@TeleOp
public class test3 extends OpMode {
    //private Gyroscope imu;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor intake;
    private DcMotorEx shuter;
    private DcMotorEx arm;
    private DcMotor grip;
    private Servo loader;
    private Servo grabber_left;
    private Servo grabber_right;
    double  intakeDir = 1;
    double  intakeChange = -1;
    double sm = 1;
    double poz = 0;
    double gpoz = 0;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    boolean v = true;
    boolean FirstTime = true;
    int okGrip = 1;
    //long VoltageSensor;
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25;
    int loaderState = -1;
    public void init() {
        motorBL = hardwareMap.get(DcMotor.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotor.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotor.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotor.class, "motorFR"); // Motor Front-Right

        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);


        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)

    }
    public void loop(){
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1.5;
        rx = gamepad1.right_stick_x;

        pmotorFL = y + x + rx;
        pmotorBL = y - x + rx;
        pmotorFR = y - x - rx;
        pmotorBR = y + x - rx;

        max = abs(pmotorFL);
        if (abs(pmotorFR) > max) {
            max = abs(pmotorFR);
        }
        if (abs(pmotorBL) > max) {
            max = abs(pmotorBL);
        }
        if (abs(pmotorBR) > max) {
            max = abs(pmotorBR);
        }
        if (max > 1) {
            pmotorFL /= max;
            pmotorFR /= max;
            pmotorBL /= max;
            pmotorBR /= max;
        }
        //SLOW-MOTION

        if(gamepad1.left_bumper)
        {
            sm=5;
            POWER(pmotorFR/sm, pmotorFL/sm, pmotorBR/sm, pmotorBL/sm);
            //arm.setPower(poz/sm);
        }
        else {
            //SLOWER-MOTION
            if (gamepad1.right_bumper) {
                sm = 10;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            } else {
                sm = 1;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            }
        }
        if(gamepad1.a)
        {
            motorFL.setPower(-0.08);
            motorFR.setPower(0.08);
            motorBL.setPower(-0.08);
            motorBR.setPower(0.08);
        }
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.update();
    }
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}

