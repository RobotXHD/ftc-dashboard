package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@Autonomous
public class AutonomusNou extends LinearOpMode {
    public DcMotorEx motorss;
    public DcMotorEx motords;
    public DcMotorEx motorfs;
    public DcMotorEx motordf;
    private DcMotorEx intake;
    private DcMotorEx arm;
    private DcMotorEx shuter;
    private DcMotorEx grip;

    private Servo grabber_left;
    private Servo grabber_right;
    private Servo loader;

    private static final String Vuforia_Key = "ATWiS67/////AAABmXkpvqSODEhyq17ZN/yB2zdOIygTMqLbBaHhR0Fy1ib0a+I4L1Zk8am3/U8LXQLtwAe6IYKDBCNiu9hddoOCP6io8nOFrRs/dnD2hkqAAetk50R28CHSvCNzx9B/j2PZLPjhQx1041PO5/WyrcrXI10SHBtDh47XTwf2f2S5FHi1QATr9sHEJ/SyjyoB2tc3Niq/80KBHWk2eEAcz+rcK84o9+KXEgsNU7RvhZhvMGbMg1makoV+wfxwR6ycWBEiRKx+W3ZbF426ot1eSIzqTUEvbUf31PbX0/xZVh7ZOIHiR1sRG3naVx1kDk98fo02PSVGB4f6brFvJzaeGrQO99y1f2s7KiiOUzaKVRgDOxYM";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private Thread Vuforia = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!isStopRequested() && opModeIsActive()) {

                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                }
                else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }
        }
    });
    @Override
    public void runOpMode() throws InterruptedException {
        motordf = hardwareMap.get(DcMotorEx.class, "motorFR");
        motords = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorfs = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorss = hardwareMap.get(DcMotorEx.class, "motorBL");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        grip = hardwareMap.get(DcMotorEx.class, "grip");
        shuter = hardwareMap.get(DcMotorEx.class, "shuter");
        grabber_left  = hardwareMap.servo.get("grabber_left");
        grabber_right  = hardwareMap.servo.get("grabber_right");
        loader  = hardwareMap.servo.get("loader");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);
        shuter.setDirection(DcMotorSimple.Direction.REVERSE);

        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = Vuforia_Key;

        parameters.cameraName = webcamName;

        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        final float CAMERA_FORWARD_DISPLACEMENT  = 0 * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT     = 0;

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }
        targetsUltimateGoal.activate();
        Vuforia.start();
        waitForStart();
        Fata(1,1,1,1);
        Spate(1,1,1,1);
        Stanga(1,1,1,1);
        Dreapta(1,1,1,1);
    }
    public void Fata(double ss,double ds,double fs,double fd){
        motordf.setPower(fd);
        motorfs.setPower(fs);
        motords.setPower(ds);
        motorss.setPower(ss);
    }
    public void Spate(double ss,double ds,double fs,double fd){
        motordf.setPower(-fd);
        motorfs.setPower(-fs);
        motords.setPower(-ds);
        motorss.setPower(-ss);
    }
    public void Stanga(double ss,double ds,double fs,double fd){
        motordf.setPower(fd);
        motorfs.setPower(-fs);
        motords.setPower(ds);
        motorss.setPower(-ss);
    }
    public void Dreapta(double ss,double ds,double fs,double fd){
        motordf.setPower(-fd);
        motorfs.setPower(fs);
        motords.setPower(-ds);
        motorss.setPower(ss);
    }
}
