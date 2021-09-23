package org.firstinspires.ftc.teamcode_FTC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class wow_hardcore extends OpMode {
    //CEO of brawl stars
    //Bo Ring
    private DcMotorEx motorst,motordr;
    private Servo yeeter;
    @Override
    public void init() {
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motordr = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorst = hardwareMap.get(DcMotorEx.class, "motorST");
        yeeter = hardwareMap.get(Servo.class, "yeeter");

        motordr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordr.setDirection(DcMotorSimple.Direction.REVERSE);

        motordr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        yeeter.setPosition(gamepad1.right_stick_y + 0.7);
        if(gamepad1.left_bumper){
            motordr.setVelocity(constants.power);
            motorst.setVelocity(constants.power);
        }
        else{
            motordr.setVelocity(0);
            motorst.setVelocity(0);
        }
        telemetry.addData("velocity", constants.power);
        telemetry.addData("realVelocity", motorst.getVelocity());
        telemetry.update();
    }
}
