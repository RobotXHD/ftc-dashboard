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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class Test extends LinearOpMode {
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


    @Override
    public void runOpMode() 
    {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotor.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotor.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotor.class, "motorFR"); // Motor Front-Right 
        /*
        shuter  = (DcMotorEx) hardwareMap.dcMotor.get("shuter");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        arm     = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        grip    = (DcMotorEx) hardwareMap.dcMotor.get("grip");
        loader  = hardwareMap.servo.get("loader");
        grabber_left  = hardwareMap.servo.get("grabber_left");
        grabber_right  = hardwareMap.servo.get("grabber_right");
        //hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        */

        
       // shuter.setDirection(DcMotorSimple.Direction.REVERSE);
       motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
       motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
       
        
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //if (FirstTime == true)
        //{
         //FirstTime = false;
        
        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}
        
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //grip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        
        double gamePad1LeftY = 0;
        double gamePad1LeftX = 0;
        double gamePad1RightX = 0;
        
        double motorFLPower = 0;
        double motorFRPower = 0;
        double motorBLPower = 0;
        double motorBRPower = 0;
        double maxPower = 0;
        //shuter.setVelocity(500);
        //sleep(50);
        //shuter.setVelocity(1000);
        
        
        
        
        while (opModeIsActive()) 
        {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.5;
            rx = gamepad1.right_stick_x;
            
            pmotorFL = y + x + rx;
            pmotorBL = y - x + rx;
            pmotorFR = y - x - rx;
            pmotorBR = y + x - rx;
            
            if (Math.abs(pmotorFL) > 1 || Math.abs(pmotorBL) > 1 || Math.abs(pmotorFR) > 1 || Math.abs(pmotorBR) > 1)
            {
                max=0;
                max=Math.max(Math.abs(pmotorFL), max);
                max=Math.max(Math.abs(pmotorFR), max);
                max=Math.max(Math.abs(pmotorBR), max);
                max=Math.max(Math.abs(pmotorBL), max);
                
                pmotorFL /= max;
                pmotorBL /= max;
                pmotorFR /= max;
                pmotorBR /= max;
            }
            
            motorBL.setPower(pmotorBL);
            motorFL.setPower(pmotorFL);
            motorBR.setPower(pmotorBR);
            motorFR.setPower(pmotorFR);
            
            /*
            gamePad1LeftY = -gamepad1.left_stick_y;
            if(gamePad1LeftY >= 0){
                gamePad1LeftY = Math.sqrt(gamePad1LeftY);
            }
            else {
                gamePad1LeftY = -Math.sqrt(-gamePad1LeftY);
            }
            
            gamePad1LeftX =  gamepad1.left_stick_x;
            if(gamePad1LeftX >= 0){
                gamePad1LeftX = Math.sqrt(gamePad1LeftX);
            }
            else {
                gamePad1LeftX = -Math.sqrt(-gamePad1LeftX);
            }
            
            gamePad1RightX = gamepad1.right_stick_x / 2;
            if(gamePad1RightX >= 0){
                gamePad1RightX = Math.sqrt(gamePad1RightX);
            }
            else {
                gamePad1RightX = -Math.sqrt(-gamePad1RightX);
            }
            
            maxPower = 0;
        
            motorFLPower = -gamePad1LeftY - gamePad1LeftX - gamePad1RightX;
            if(Math.abs(motorFLPower) > maxPower){
                maxPower = Math.abs(motorFLPower);
            }
            motorFRPower =  gamePad1LeftY - gamePad1LeftX - gamePad1RightX;
            if(Math.abs(motorFRPower) > maxPower){
                maxPower = Math.abs(motorFRPower);
            }
            motorBRPower =  gamePad1LeftY + gamePad1LeftX - gamePad1RightX;
            if(Math.abs(motorBRPower) > maxPower){
                maxPower = Math.abs(motorBRPower);
            }
            motorBLPower = -gamePad1LeftY + gamePad1LeftX - gamePad1RightX;
            if(Math.abs(motorBLPower) > maxPower){
                maxPower = Math.abs(motorBLPower);
            }
            */
            
/*
            if (motorFLPower >= 0){ motorFLPower = Math.sqrt(motorFLPower);
            } else { motorFLPower = -Math.sqrt(-motorFLPower);}
        
            if (motorFRPower >= 0){ motorFRPower = Math.sqrt(motorFRPower);
            } else { motorFRPower = -Math.sqrt(-motorFRPower);}

            if (motorBLPower >= 0){ motorBLPower = Math.sqrt(motorBLPower);
            } else { motorBLPower = -Math.sqrt(-motorBLPower);}
        
            if (motorBRPower >= 0){ motorBRPower = Math.sqrt(motorBRPower);
            } else { motorBRPower = -Math.sqrt(-motorBRPower);}


            motorFLPower = Range.clip(motorFLPower, -1, 1);
            motorFRPower = Range.clip(motorFRPower, -1, 1);
            motorBLPower = Range.clip(motorBLPower, -1, 1);
            motorBRPower = Range.clip(motorBRPower, -1, 1);            
*/

            
            
            /*
            motorFL.setTargetPosition(-1389);
            motorFR.setTargetPosition(1448);
            motorBL.setTargetPosition(-1429);
            motorBR.setTargetPosition(1374);
            
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            motorBL.setPower(0.3);
            motorBR.setPower(0.3);
            motorFL.setPower(0.3);
            motorFR.setPower(0.3);
            
            while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());
         
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFL.setPower(0);
            motorFR.setPower(0);
         
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            */
            //shuter.setVelocity(2090);
            //mRunTime.reset();

            
            //SLOW-MOTION
            
            if(gamepad1.left_bumper)
            {
                sm=5;
            motorFL.setPower(pmotorFL/max/sm);
            motorBL.setPower(pmotorBL/max/sm);
            motorFR.setPower(pmotorFR/max/sm);
            motorBR.setPower(pmotorBR/max/sm);
            //arm.setPower(poz/sm);
            }
            else
            //SLOWER-MOTION
            if(gamepad1.right_bumper)
            {
                sm=10;
            motorFL.setPower(pmotorFL/max/sm);
            motorBL.setPower(pmotorBL/max/sm);
            motorFR.setPower(pmotorFR/max/sm);
            motorBR.setPower(pmotorBR/max/sm);
            }
            
            else
            {
                sm=1;
            motorFL.setPower(pmotorFL/max/sm);
            motorBL.setPower(pmotorBL/max/sm);
            motorFR.setPower(pmotorFR/max/sm);
            motorBR.setPower(pmotorBR/max/sm);
            }
            
            /*
            //loader.setPosition(0.1);
            
            //GRABBERS
            //grabber_left.setPosition(1.0);
            //grabber_right.setPosition(0.0);
             
            if(gamepad1.left_trigger > 0.7)
            {grabber_left.setPosition(0.1);}
            else {grabber_left.setPosition(0.8);}
            

            if(gamepad1.right_trigger > 0.7)
                {grabber_right.setPosition(1);}
            else {grabber_right.setPosition(0.2);}
            
            //INTAKE - REVERSE
            /*
            if(gamepad1.a)
            {
                intake.setPower(0);
                intakeDir = - intakeDir;
                sleep(200);
            }
            intake.setPower(intakeDir);
            */
            /*
            if(gamepad1.a)
            {
                intakeDir = intakeDir + intakeChange;
                if(intakeDir <-1 || intakeDir >1)
                    intakeChange = -intakeChange;
                sleep(100);
            }
            intake.setPower(intakeDir);
            */
            //shuter.setVelocity(500);
            //sleep(50);
            
            
            
            //INTAKE V2
            /*
            if(gamepad1.atRest() == false)
            {
                intake.Power(intakeDir);
            }
            */
            
            //LOADER
            /*
            if(gamepad1.b)
                {
                    loader.setPosition(1);
                    sleep(300);
                    loader.setPosition(0.1);
                    sleep(300);
                }
            */
            /*
            if(gamepad1.b && (loaderState == -1))
                {
                    
                   loader.setPosition(1);
                    loaderState = 1;
                    timer.reset();
                    
                }
            
            if(loaderState == 1)
            {
                if(timer.time() > timeLimit)
                {
                    loader.setPosition(0.3);
                    loaderState = 0;
                     //sleep(300);
                    //loader.setPosition(0);
                    timer.reset();
                }
            }
                
            if(loaderState == 0)
            {
                if (timer.time() > timeLimit)
                loaderState = -1;
            }
            
            
            //GRIP-EXTINS
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
            
                
            //ARM-UP
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
            
            
            //ARM-DOWN
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
            
            
            
            //SHOOTER-INELE
            if(gamepad1.y)
            shuter.setVelocity(2110); //2280 AUTONOM
            
            //SHOOTER-POWER-SHOT
            if(gamepad1.x)
            shuter.setVelocity(1950);
            
            /*
            if(okGrip == 1)
            {
            okGrip = 0;
            arm.setTargetPosition(-100);//-321; -400
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //grabber_left.setPosition(0.2);
            */
            if(gamepad1.left_bumper)
            {
                boolean lb=false;
                while (gamepad1.left_bumper)
                {
                    lb=true;
                }
                telemetry.addData("Left Bumper", gamepad1.left_bumper);
            }
            /*
            telemetry.addData("Velocity:", shuter.getVelocity());
            telemetry.addData("Arm position:", arm.getCurrentPosition());
            telemetry.addData("Grip position:", grip.getCurrentPosition());
            
            telemetry.addData("Front Left:",  motorFL.getCurrentPosition());
            telemetry.addData("Front Right:", motorFR.getCurrentPosition());
            telemetry.addData("Back Left:",   motorBL.getCurrentPosition());
            telemetry.addData("Back Right:",  motorBR.getCurrentPosition());
            telemetry.addData("loaderState:",  loaderState);
            telemetry.addData("timer.time():",  timer.time());
            */
            telemetry.update();
            
            if(gamepad1.a)
            {
                motorFL.setPower(-0.08);
                motorFR.setPower(0.08);
                motorBL.setPower(-0.08);
                motorBR.setPower(0.08);
                sleep(2000);
            }
            
        }
    }
}

