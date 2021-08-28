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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;
@TeleOp
public class TeleOp_bun extends OpMode {
    private Servo grabber_left;
    private Servo grabber_right;
    private Servo loader;
    private Servo stopper_left;
    private Servo stopper_right;
    /**declare the motors*/
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    private DcMotorEx intake;
    private DcMotorEx arm;
    private DcMotorEx shuter;
    private DcMotorEx grip;
    private BNO055IMU imu;
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
    double VelVar = 2155;
    double poz=0, gpoz=0;
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
    public boolean rotating = false;
    public double realAngle, targetAngle;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0.0,0.0,0.0);
    private long spasmCurrentTime = 0;
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
                pid.setPID(constants.pGyro,constants.iGyro,constants.dGyro);
                if(clockwise != 0.0){
                    correction = 0.0;
                    rotating = true;
                }
                else{
                    if(rotating && clockwise == 0.0){
                        targetAngle = realAngle;
                        rotating = false;
                        pid.setSetpoint(targetAngle);
                    }
                    correction = pid.performPID(realAngle);
                }


                /**calculating the power for motors */
                df = forward + clockwise - right + correction;
                ss = forward - clockwise - right - correction;
                sf = -forward + clockwise - right + correction;
                ds = -forward - clockwise - right - correction;

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
    private Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop){
                shuter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(constants.p, constants.i, constants.d, constants.f));
                if(once==1){
                    intake.setPower(1);
                    once=0;
                }
                boolean abut = gamepad1.a;
                if (alast != abut) {
                    if (gamepad1.a) {
                        if (apoz == 0) intake.setPower(0);
                        else if(apoz == 1) intake.setPower(-1);
                        else if(apoz == 2) intake.setPower(0);
                        else if(apoz == 3) intake.setPower(1);
                        apoz++;
                        if(apoz == 4){
                            apoz = 0;
                        }
                    }
                    alast = abut;
                }
                if(gamepad1.left_trigger > 0.0){
                    stopper_left.setPosition(constants.stoppersts);
                    spasmCurrentTime = System.currentTimeMillis();
                    VelVar=VelVar + 5;
                }
                if(gamepad1.left_trigger > 0.8){
                    grabber_left.setPosition(1 - gamepad1.left_trigger);
                }
                else{
                    grabber_left.setPosition(0.8);
                }

                if(gamepad1.right_trigger > 0)
                {
                    stopper_right.setPosition(constants.stopperdrs);
                    spasmCurrentTime = System.currentTimeMillis();
                    VelVar=VelVar - 5;
                }
                if(gamepad1.right_trigger > 0.8){
                    grabber_right.setPosition(gamepad1.right_trigger);
                }
                else {
                    grabber_right.setPosition(0.2);
                }

                if(spasmCurrentTime + constants.spasmDelay < System.currentTimeMillis()){
                    stopper_left.setPosition(constants.stopperst);
                    stopper_right.setPosition(constants.stopperdr);
                }

                //stopper_left.setPosition()

                if(gamepad1.b && (loaderState == -1))
                {
                    loader.setPosition(0.6);
                    loaderState = 1;
                    timeLimit = 0.27;
                    timer.reset();

                }

                if(loaderState == 1)
                {
                    if(timer.time() > timeLimit)
                    {
                        loader.setPosition(0.3);
                        loaderState = 0;
                        timeLimit = 0.4;
                        timer.reset();
                    }
                }

                if(loaderState == 0)
                {
                    if (timer.time() > timeLimit)
                        loaderState = -1;
                }
                if(gamepad1.dpad_left)
                {
                    while(gamepad1.dpad_left)
                    {
                        gpoz++;
                        grip.setPower(gpoz);
                    }
                }
                grip.setPower(0);
                gpoz = 0;


                //GRIP-INCHIS
                if(gamepad1.dpad_right)
                {
                    while(gamepad1.dpad_right)
                    {
                        gpoz--;
                        grip.setPower(gpoz);
                    }
                }
                grip.setPower(0);
                gpoz = 0;
                if(gamepad1.dpad_up)
                {
                    while(gamepad1.dpad_up)
                    {
                        poz--;
                        arm.setPower(poz);
                    }
                }
                arm.setPower(0);
                poz=0;
                if(gamepad1.dpad_down)
                {
                    while(gamepad1.dpad_down)
                    {
                        poz++;
                        arm.setPower(poz);
                    }
                }
                arm.setPower(0);
                poz=0;
                boolean cnbut = gamepad1.y;
                if (cnlast != cnbut) {
                    if (gamepad1.y) {
                        if (cnpoz == 0) shuter.setVelocity(2155);//2155
                        else if(cnpoz == 1) shuter.setVelocity(0);
                        cnpoz++;
                        if(cnpoz == 2){
                            cnpoz = 0;
                        }
                    }
                    cnlast = cnbut;
                }

                //SHOOTER-POWER-SHOT
                if(gamepad1.x)
                {
                    VelVar = 1973;
                    shuter.setVelocity(1973);//1973
                }
                if(gamepad1.back)
                {
                    loader.setPosition(0.2);
                }
            }
        }
    });

    private Thread imuT = new Thread(new Runnable() {
        double angle, lastAngle;
        int rotations;
        @Override
        public void run() {
            while(!stop){
                lastAngle = angle;
                angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if(lastAngle > 170 && angle < -170) rotations++;
                else if (lastAngle < -170 && angle > 170) rotations --;
                realAngle = angle + 360 * rotations;
            }
        }
    });
    @Override
    public void init() {
        pid.disable();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /**initialization motors */
        motordf = hardwareMap.get(DcMotorEx.class, "motorFR");
        motords = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorsf = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorss = hardwareMap.get(DcMotorEx.class, "motorBL");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        grip = hardwareMap.get(DcMotorEx.class, "grip");
        shuter = hardwareMap.get(DcMotorEx.class, "shuter");
        grabber_left  = hardwareMap.servo.get("grabber_left");
        grabber_right  = hardwareMap.servo.get("grabber_right");
        loader  = hardwareMap.servo.get("loader");
        stopper_left = hardwareMap.servo.get("stopper_left");
        stopper_right = hardwareMap.servo.get("stopper_right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);
        shuter.setDirection(DcMotorSimple.Direction.REVERSE);
        /**set the mode of the  motors */

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**initialization system current time milliseconds */
        sysTimeC = System.currentTimeMillis();
        shuter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(constants.p, constants.i, constants.d, constants.f));
        //stopper_left.setPosition(constants.stopperstin);
        //stopper_right.setPosition(constants.stopperdrin);
        /**start the thread*/
    }
    @Override
    public void start() {
        Chassis.start();
        Systems.start();
        imuT.start();
    };
    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
        telemetry.addData("motordf: ", motordf.getCurrentPosition());
        telemetry.addData("motorsf: ", motorsf.getCurrentPosition());
        telemetry.addData("motords: ", motords.getCurrentPosition());
        telemetry.addData("motorss: ", motorss.getCurrentPosition());
        telemetry.addData("arm: ", arm.getCurrentPosition());
        telemetry.addData("Th Chassis: ", fpsCLast);
        telemetry.addData("Launch:", shuter.getVelocity());
        telemetry.addData("stopper_left", stopper_left.getPosition());
        telemetry.addData("realAngle:", realAngle);
        telemetry.addData("P:", pid.getP() * pid.getError());
        telemetry.addData("I:", pid.getI() * pid.getISum());
        telemetry.addData("D:", pid.getD() * pid.getDError());
        telemetry.addData("setPoint:", pid.getSetpoint());
        telemetry.addData("Error:", pid.getError());
        telemetry.addData("Correction:", correction);
        telemetry.update();
    }

    /**using the stop function to stop the threads */
    public void stop(){stop = true;}

    /**the power function sets the motor's power*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }
}