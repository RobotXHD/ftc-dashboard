package org.firstinspires.ftc.teamcode_FTC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;
@TeleOp
public class MotoareMsoare extends OpMode {
    /**declare the motors*/
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    /**variable for changing the movement speed of the robot*/
    private int v = 2;
    /**variables for calculating the power for motors*/
    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;
    private double once=1;
    private double cn=0;
    double correction;
    double timeLimit = 0.3;
    /**variables for holding the gamepad joystick values;
     * we don't want to access them too many times in a loop */
    private double forward, right, clockwise;
    /**variable that stops the threads when programs stop*/
    private boolean stop, alast = false,cnlast=false;
    private int apoz = 0,cnpoz=0;
    int loaderState = -1;
    /**variables that count the thread's fps*/
    private long fpsC=0;
    private long fpsCLast;
    /** variable that  holds the system current time milliseconds*/
    private long sysTimeC;
    public ElapsedTime timer = new ElapsedTime();
    public boolean rotating = false,randoomshit = false;
    public double realAngle, targetAngle;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0.0,0.0,0.0);
    private long spasmCurrentTime = 0;
    private long pidTime = 0;
    public double difference,medie;
    public double medii[] = new double[10];
    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops*/
            while (!stop) {
                /**change the variable that controls the speed of the chassis using the bumpers*/
                if (gamepad1.right_bumper) {
                    v = 1;
                }
                else{
                    v = 2;
                }
                /**getting the gamepad joystick values*/
                forward = -gamepad1.left_stick_y;
                right = gamepad1.left_stick_x;
                clockwise = -gamepad1.right_stick_x;
                boolean abut = gamepad1.left_bumper;
                if (alast != abut) {
                    if (gamepad1.left_bumper) {
                        if (apoz == 0) randoomshit = false;
                        else if(apoz == 1) randoomshit = true;
                        apoz++;
                        if(apoz == 2){
                            apoz = 0;
                        }
                    }
                    alast = abut;
                }
                pid.setPID(constants.pGyro,constants.iGyro,constants.dGyro);
                if(clockwise != 0.0){
                    correction = 0.0;
                    rotating = true;
                }
                else{
                    if((forward != 0.0 || right != 0.0) && Math.abs(medie) < 0.5) {
                        if (rotating) {
                            targetAngle = realAngle;
                            rotating = false;
                            pid.setSetpoint(targetAngle);
                        }
                        correction = pid.performPID(realAngle);
                    }
                    else{
                        correction = 0.0;
                    }
                }


                /**calculating the power for motors */
                df = forward  + correction;
                ss = forward  - correction;
                sf = -forward  + correction;
                ds = -forward  - correction;

                /**normalising the power values*/
                max = abs(sf);
                if (abs(df) > max) {
                    max = abs(df);
                }
                if (abs(ss) > max) {
                    max = abs(ss);
                }
                if (abs(ds) > max) {
                    max = abs(ds);
                }
                if (max > 1) {
                    sf /= max;
                    df /= max;
                    ss /= max;
                    ds /= max;
                }
                /** fps counter*/
                fpsC++;
                if (sysTimeC + 1000 < System.currentTimeMillis()) {
                    fpsCLast = fpsC;
                    fpsC = 0;
                    sysTimeC = System.currentTimeMillis();
                }
                /** setting the speed of the chassis*/
                if (v == 1) {
                    POWER(df / 5, sf / 5, ds / 5, ss / 5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }
            }
        }
    });

    @Override
    public void init() {
        pid.enable();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /**initialization motors */
        motordf = hardwareMap.get(DcMotorEx.class, "motorFR");
        motords = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorsf = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorss = hardwareMap.get(DcMotorEx.class, "motorBL");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        /**set the mode of the  motors */

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**initialization system current time milliseconds */
        sysTimeC = System.currentTimeMillis();
        //stopper_left.setPosition(constants.stopperstin);
        //stopper_right.setPosition(constants.stopperdrin);
        /**start the thread*/
    }
    @Override
    public void start() {
        Chassis.start();
    };
    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
        telemetry.addData("motordf: ", motordf.getCurrentPosition());
        telemetry.addData("motorsf: ", motorsf.getCurrentPosition());
        telemetry.addData("motords: ", motords.getCurrentPosition());
        telemetry.addData("motorss: ", motorss.getCurrentPosition());
        telemetry.addData("Th Chassis: ", fpsCLast);
        telemetry.addData("realAngle:", realAngle);
        telemetry.addData("P:", pid.getP() * pid.getError());
        telemetry.addData("I:", pid.getI() * pid.getISum());
        telemetry.addData("D:", pid.getD() * pid.getDError());
        telemetry.addData("setPoint:", pid.getSetpoint());
        telemetry.addData("Error:", pid.getError());
        telemetry.addData("Correction:", correction);
        telemetry.addData("Difference:", medie);
        telemetry.update();
    }

    /**using the stop function to stop the threads */
    public void stop(){stop = true;}

    /**the power function sets the motor's power*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        if(randoomshit == false) {
            motordf.setPower(df1);
            motorss.setPower(ss1);
            motorsf.setPower(sf1);
            motords.setPower(ds1);
        }
        else{
            /*
            if(gamepad1.dpad_up) {
                motorss.setPower(1);
            }
            else if(gamepad1.dpad_down){
                motorss.setPower(-1);
            }
            else{
                motorss.setPower(0);
            }
            if(gamepad1.dpad_right) {
                motorsf.setPower(1);
            }
            else if(gamepad1.dpad_left){
                motorsf.setPower(-1);
            }
            else{
                motorsf.setPower(0);
            }
            if(gamepad1.y) {
                motords.setPower(1);
            }
            else if(gamepad1.a){
                motords.setPower(-1);
            }
            else{
                motords.setPower(0);
            }
            if(gamepad1.b) {
                motordf.setPower(1);
            }
            else if(gamepad1.x){
                motordf.setPower(-1);
            }
            else{
                motordf.setPower(0);
             */

            //motor N
            if(gamepad1.dpad_up) {
                motorss.setPower(1);
            }
            else if(gamepad1.y){
                motorss.setPower(-1);
            }
            else{
                motorss.setPower(0);
            }

            //motor E
            if(gamepad1.dpad_right) {
                motorsf.setPower(1);
            }
            else if(gamepad1.b){
                motorsf.setPower(-1);
            }
            else{
                motorsf.setPower(0);
            }

            //motor S
            if(gamepad1.dpad_down) {
                motords.setPower(1);
            }
            else if(gamepad1.a){
                motords.setPower(-1);
            }
            else{
                motords.setPower(0);
            }

            //motor V
            if(gamepad1.dpad_left) {
                motordf.setPower(1);
            }
            else if(gamepad1.x){
                motordf.setPower(-1);
            }
            else{
                motordf.setPower(0);

            }
        }
    }
}