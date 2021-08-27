
package org.firstinspires.ftc.teamcode_FTC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.logging.Logger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name="TheRedNormalAutonomus")
//@Disabled
public class TheRedNormalAutonomus extends LinearOpMode {

    /* Declare OpMode members. */
    /*
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    */
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "4";
    private static final String LABEL_SECOND_ELEMENT = "1";
    private static final String VUFORIA_KEY = "AQ8JJOr/////AAABmcjKenKHVE9lpXrPPGnySdtzRBLpCctelkcCxiAsf9Rxqajtq1sLOzcwuMZhApnsrhr0q1wrHoc9cPu7vFLeA62XfiuQXYXdfkhW5wmVLltRIeXTZuahgIOuFdviant4dCE5C+oIu70HbG6yJP5/NxotVWF30ArftJ8mz5IUNpstlP+yAmjvkR3iBETGCyAdBZgXA+2Tm/snwnyH/9S09qsdDUuvGxhrLDIJU2pnq/DOQP5C4EMbrJfnYhr9TVVcMlgy1x+pfgMnZ73u0N/+lW8MtuGD+2x5o5yuAzEfDKkexBVoX1UebUvOe+RsGUhwMgxNyCiX4jFqmCfBZ2WKxbElyMUHjI9bxfIHoeK24ZjS";

    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor intake;
    private DcMotorEx arm;
    private DcMotorEx grip;
    private DcMotorEx shuter;
    private Servo loader;
    private Servo grabber_left;
    private Servo grabber_right;
    private Servo stopper_left;
    private Servo stopper_right;



    double Lpos = 0.7;

    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    int varrez;
    double lastTime;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);

    String rezultat = "None";

    @Override
    public void runOpMode() {



        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);
        motorBL = hardwareMap.get(DcMotor.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotor.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotor.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotor.class, "motorFR"); // Motor Front-Right 
        shuter  = (DcMotorEx) hardwareMap.dcMotor.get("shuter");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        arm     = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        grip    = (DcMotorEx) hardwareMap.dcMotor.get("grip");
        loader  = hardwareMap.servo.get("loader");
        grabber_left   = hardwareMap.servo.get("grabber_left");
        grabber_right  = hardwareMap.servo.get("grabber_right");
        stopper_left   = hardwareMap.servo.get("stopper_left");
        stopper_right  = hardwareMap.servo.get("stopper_right");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        loader.setPosition(0.4);//0.32
        shuter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(constants.p, constants.i, constants.d, constants.f));

        initVuforia( );
        initTfod();
        if(tfod!=null)
        {
            tfod.activate();
            tfod.setZoom(2.0, 16.0/9.0);
        }
        Detectam.start();

        if(!isStarted())

            waitForStart();

        if(rezultat == "None") varrez=1;
        else if(rezultat == LABEL_SECOND_ELEMENT) varrez=2;
        else if(rezultat == LABEL_FIRST_ELEMENT)  varrez=3;
        telemetry.addData("var",varrez );
        telemetry.update();

        if(varrez==1)
        {
         /*
         Translatare(20, 0, 0.3);
         Translatare(0, 50, 0.3);
         */
            shuter.setVelocity(-2200);//2080
            //pozitionare shoot 1
            stopper_left.setPosition(0.5);
            stopper_right.setPosition(0.5);

            Translatare(0, 160, 0.3);//x=-5
            lastTime = System.currentTimeMillis();
            while(lastTime + 200 > System.currentTimeMillis()){

            }
            Rotire(6, 0.3);

            //shoot 1 - 1
            shuter.setVelocity(-2220);//2080
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            shuter.setVelocity(-2210);//2080
            loader.setPosition(0.2);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            //shoot 1 - 2
            shuter.setVelocity(-2220);//2080
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }
            shuter.setVelocity(-2220);//2080
            loader.setPosition(0.2);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            // shoot 1 - 3
            shuter.setVelocity(-2180);//2080
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            shuter.setVelocity(-2170);//2080
            loader.setPosition(0.2);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //Translatare(80, 10, 0.3);
            Translatare(0, 60, 0.3);;
            Rotire(-130, 0.3);
            Translatare(10, 30, 0.2);
            //sleep(100);

            intake.setPower(0);

            //pozitionare livrare 1
            //Translatare(0, 60, 0.3);
            //sleep(100);

            //brat livrare 1
            arm.setTargetPosition(-400);//-400
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }

            //eliberare wobble 1
            grip.setTargetPosition(500); //-15
            grip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grip.setPower(1.0);
            while(grip.isBusy());
            grip.setPower(0);
            grip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }
            Translatare(0, -30, 0.3);
            Rotire(120, 0.3);

            Translatare(-30, -20, 0.3);

            arm.setTargetPosition(-100);//-321; -400
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(varrez == 2)
        {
         /*
         Translatare(20, 0, 0.3);
         Translatare(0, 50, 0.3);
         */
            shuter.setVelocity(-2400);

            stopper_right.setPosition(0.5);
            stopper_left.setPosition(0.5);


            while(shuter.isBusy())
                lastTime = System.currentTimeMillis();
            while(lastTime + 5 > System.currentTimeMillis()){

            }
            /*
            grabber_right.setPosition(0.2);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }

            grabber_left.setPosition(0.8);
            */


            //shuter.setVelocity(-2260);
            //pozitionare shoot 2
            Translatare(0, 70, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 550 > System.currentTimeMillis()){

            }

            //sleep(50);
            //shuter.setVelocity(-2310);
            //while(shuter.isBusy())
            //sleep(5);

            Rotire(5, 0.3);
            //shoot 2 - 1 un inel inainte de colectare
            shuter.setVelocity(-2520);
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            shuter.setVelocity(-2120);

            //corectie laterala 2
            Translatare(0, 10, 0.3);
            //pornire colectare
            //grabber_right.setPosition(1);
            intake.setPower(1.0);

            Rotire(-25,0.3);
            Translatare(0, 15, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 300 > System.currentTimeMillis()){
            }
            //sleep(100);

            Translatare(0, -15, 0.3);
            Rotire(22,0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){
            }




            //pozitionare shoot linie 2
            shuter.setVelocity(-2110);
            Translatare(-12, 90, 0.3);
            //sleep(150);

            Rotire(-7, 0.3);

            //shoot 2 - 2
            shuter.setVelocity(-2120);
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2150);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            shuter.setVelocity(-2170);
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2110);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            //shoot 2 - 3
            shuter.setVelocity(-2120);
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2110);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            shuter.setVelocity(-2120);
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2110);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            //shoot 2 - 4
            shuter.setVelocity(-2120);
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2110);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            shuter.setVelocity(-2120);
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2110);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            //pozitionare livreare
            Translatare(20, 100, 0.3); //x=25
            //sleep(100);

            intake.setPower(0);


            //pozitionare brat 2
            arm.setTargetPosition(-420);//-350
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //sleep(100);

            //eliberare wobble 1
            grip.setTargetPosition(500); //-15
            grip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grip.setPower(1.0);
            while(grip.isBusy());
            grip.setPower(0);
            grip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //sleep(100);

            Translatare(-30, -50, 0.3);

            arm.setTargetPosition(-100);//-350
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }


            //grabber_left.setPosition(0.5);
            //grabber_right.setPosition(0.5);

            //Translatare pana la wobble 2

            //sleep(50);
         /*
         grabber_left.setPosition(0.1);
         grabber_right.setPosition(1);
            
         
         Translatare(75, 0, 0.3); //x=55
         
         Translatare(0, -105, 0.3);//-10
         //sleep(100);
     
         Rotire(230, 0.3);
     
         //apucare wobble 2
         arm.setTargetPosition(-550);//-551
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setPower(1);
         while(arm.isBusy() && opModeIsActive());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //sleep(100);
     
         //Translatare(0, 5, 0.3);
        
         grip.setTargetPosition(33); //25
         grip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         grip.setPower(1);
         while(grip.isBusy());
         grip.setPower(0);
         grip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //sleep(100);
     
         arm.setPower(1);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setTargetPosition(-420);//-321; -400
         
         
         while(arm.isBusy());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         //sleep(100);
     
         Rotire(-195,0.3);
         //translatare pana la parcare wobble 2
         Translatare(-10, 155, 0.65);
         //sleep(100);
     /*
     arm.setTargetPosition(-350);
     arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     arm.setPower(1.0);
     while(arm.isBusy());
     arm.setPower(0);
     arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     sleep(100);
     */
     /*
     //eliberare wobble 2
     grip.setTargetPosition(150); //-15
     grip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     grip.setPower(1);
     while(grip.isBusy());
     grip.setPower(0);
     grip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     //sleep(100);
     
     Translatare(0, -30, 0.65);
     */
        }
        if(varrez == 3)
        {
         /*
         Translatare(20, 0, 0.3);
         Translatare(0, 50, 0.3);
         */
            stopper_right.setPosition(0.5);
            stopper_left.setPosition(0.5);

            /*
            grabber_right.setPosition(0.5);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }
            */

            //grabber_left.setPosition(0.5);

            //intake.setPower(1.0);

            shuter.setVelocity(-2530);
            //pozitionare shoot 3
            Translatare(10, 57, 0.3);

            //sleep(50);
               Rotire(5, 0.3);
//         shuter.setVelocity(-2290);
            //while(shuter.isBusy())
            //sleep(5);



            //grabber_right.setPosition(0.9);
            //sleep(300);
            //grabber_right.setPosition(0.2);
            Rotire(4, 0.3);

            //shoot 3 - 1
            shuter.setVelocity(-2370);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2420);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }//300

            //shoot 3 - 2
            shuter.setVelocity(-2380);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2390);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //shoot 3 - 3
            shuter.setVelocity(-2420);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2390);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //shoot 3 - 4
            shuter.setVelocity(-2400);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2390);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            //grabber_right.setPosition(0.9);
            //sleep(400);
            //grabber_right.setPosition(0.2);



            shuter.setVelocity(-2180);
            //Cnoc inele
            intake.setPower(1.0);
            Translatare(3, 0, 0.3);
            //Translatare(-5, -5, 0.5);
            Translatare(0, 30, 0.6);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()) {
            }
            Translatare(0, 25, 0.2);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){`
            }
            intake.setPower(1.0);



            Translatare(0, -47, 0.35);// X = -18
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }
            //sleep(50);

            Translatare(-10, 47, 0.35);// X = 40
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            } //sleep

            //Rotire(10, 0.3);
            //sleep(100);

            //Rotire(-8, 0.3);

            //grabber_right.setPosition(0.2);
            //grabber_left.setPosition(0.8);

            Rotire(2, 0.3);

            //shoot 3 - 4
            shuter.setVelocity(-2215);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2205);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            Translatare(0, 48, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }

            shuter.setVelocity(-2100);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2080);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            shuter.setVelocity(-2100);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2080);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            /*
            grabber_right.setPosition(0.4);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }
            grabber_right.setPosition(0.9);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }
            grabber_right.setPosition(0.2);

            grabber_left.setPosition(0.6);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }
            grabber_left.setPosition(0.1);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }
            grabber_left.setPosition(0.8);

            grabber_right.setPosition(0.4);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }
            grabber_right.setPosition(0.9);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }
            grabber_right.setPosition(0.2);

            grabber_left.setPosition(0.6);
            lastTime = System.currentTimeMillis();
            while(lastTime + 100 > System.currentTimeMillis()){

            }
            grabber_left.setPosition(0.1);
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }
            grabber_left.setPosition(0.8);

            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis()){

            }

            //grabber_right.setPosition(0.2);
            //grabber_left.setPosition(0.8);
            */

            //Rotire(5, 0.3);

            shuter.setVelocity(-2110);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2085);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            shuter.setVelocity(-2105);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2085);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }

            shuter.setVelocity(-2105);
            lastTime = System.currentTimeMillis();
            while(lastTime + 10 > System.currentTimeMillis()){

            }
            loader.setPosition(Lpos);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }
            loader.setPosition(0.2);
            shuter.setVelocity(-2085);
            lastTime = System.currentTimeMillis();
            while(lastTime + 400 > System.currentTimeMillis()){

            }



            //grabber_right.setPosition(0.9);


            Translatare(70, 140, 0.3);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }

            intake.setPower(0);

            arm.setTargetPosition(-400);//-507 original
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
            while(arm.isBusy())
                lastTime = System.currentTimeMillis();
                while(lastTime + 5 > System.currentTimeMillis()){

                }

            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }

            grip.setTargetPosition(150);
            grip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grip.setPower(1.0);
            while(grip.isBusy());
            grip.setPower(0);
            grip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }
         
         /*
         arm.setTargetPosition(-10);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setPower(0.8);
         while(arm.isBusy());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         sleep(50);
         */
            Translatare(-70, -110, 0.5);
            lastTime = System.currentTimeMillis();
            while(lastTime + 50 > System.currentTimeMillis()){

            }

            arm.setTargetPosition(-100);//-321; -400
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
            while(arm.isBusy());
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    /* 
    else if(varrez == 2)
    {
        Deplasare(-12,-12,-12,-12,0.5);
        sleep(500);
        
        Deplasare(-70,-70,70,70,0.5);
        sleep(500);
        
        arm.setTargetPosition(500);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setPower(1);
         while(arm.isBusy());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         grip.setTargetPosition(500);
         grip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         grip.setPower(1);
         while(grip.isBusy());
         grip.setPower(0);
         grip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         arm.setTargetPosition(500);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setPower(1);
         while(arm.isBusy());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         arm.setTargetPosition(0);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setPower(1);
         while(arm.isBusy());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         Deplasare(-50,-50,50,50,0.5);
    }
    
      else if(varrez == 3)
      {
        Deplasare(-20,-20,-20,-20,0.5);
        sleep(500);
        Deplasare(140,140,-140,-140,0.5);
        sleep(500);
        
        arm.setTargetPosition(500);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setPower(1);
         while(arm.isBusy());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         grip.setTargetPosition(500);
         grip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         grip.setPower(1);
         while(grip.isBusy());
         grip.setPower(0);
         grip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         arm.setTargetPosition(500);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setPower(1);
         while(arm.isBusy());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         arm.setTargetPosition(0);
         arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         arm.setPower(1);
         while(arm.isBusy());
         arm.setPower(0);
         arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        Deplasare(-100,-100,100,100,0.5);
        }*/

    }
     
     /*
     public void Deplasare(int tmotorBR, int tmotorFR, int tmotorBL, int tmotorFL, double speed)
     {
         currentmotorBL = motorBL.getCurrentPosition();
currentmotorBR = motorBR.getCurrentPosition();
         currentmotorFL = motorFL.getCurrentPosition();
         currentmotorFR = motorFR.getCurrentPosition();
         
         motorBL.setTargetPosition(currentmotorBL + (int) (tmotorBL * COUNTS_PER_CM));
         motorBR.setTargetPosition(currentmotorBR + (int) (tmotorBR * COUNTS_PER_CM));
         motorFL.setTargetPosition(currentmotorFL + (int) (tmotorFL * COUNTS_PER_CM));
         motorFR.setTargetPosition(currentmotorFR + (int) (tmotorFR * COUNTS_PER_CM));
         
         motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         
         motorBL.setPower(speed);
         motorBR.setPower(speed);
         motorFL.setPower(speed);
         motorFR.setPower(speed);
         
         while((motorFR.isBusy() && motorFL.isBusy() && motorBR.isBusy() && motorBL.isBusy()) && opModeIsActive());
         
         motorBL.setPower(0);
         motorBR.setPower(0);
         motorFL.setPower(0);
         motorFR.setPower(0);
         
         motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     }
     */

    public void Translatare(int deltaX, int deltaY, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm);
        targetBL= currentmotorBL + (int) (( -deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) (( -deltaY - deltaX) * cpcm);
         
         
         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));
         */
        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Rotire (int deltaA, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 15;
        int targetBL, targetBR, targetFL, targetFR;
        double cpdeg = 17.5 * 3.141 / 180 * COUNTS_PER_CM;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBL = currentmotorBL + (int) (deltaA * cpdeg);
        targetBR = currentmotorBR + (int) (deltaA * cpdeg);
        targetFL = currentmotorFL + (int) (deltaA * cpdeg);
        targetFR = currentmotorFR + (int) (deltaA * cpdeg);

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }
    }

    public Thread Detectam = new Thread(new Runnable()
    {
        @Override
        public void run()
        {
            if(!isStarted())
            {
                while (!isStarted())
                {
                    if (tfod != null)
                    {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null)
                        {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size()==0)
                                rezultat = "None";
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions)
                            {
                                telemetry.addData(String.format("label (%d)",i),recognition.getLabel());
                                rezultat = recognition.getLabel();
                                telemetry.addData(String.format(" left,top (%d)",i),"%.03f,%.03f", recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format(" right,bottom (%d)",i),"%.03f, %.03f", recognition.getRight(), recognition.getBottom());
                            }
                            telemetry.update();
                        }
                    }
                }
            }
            if(tfod != null)
            {
                tfod.shutdown();
            }
        }
    });
    private void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT );
    }
}
     
     
     
     
     
     
     
     
     
     
     
     
     /*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        
        // Ensure that the opmode is still active
        */
        /*
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
*/