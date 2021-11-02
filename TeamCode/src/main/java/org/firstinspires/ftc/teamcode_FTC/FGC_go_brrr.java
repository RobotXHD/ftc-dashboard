package org.firstinspires.ftc.teamcode_FTC;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FGC_go_brrr extends OpMode {
    public DcMotorEx motorS, motorD;
    public Servo gheara;
    @Override
    public void init() {
        motorS = hardwareMap.get(DcMotorEx.class, "motorS");
        motorD = hardwareMap.get(DcMotorEx.class, "motorD");
        gheara = hardwareMap.get(Servo.class, "Gheara");

        motorS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorD.setDirection(DcMotorSimple.Direction.REVERSE);

        motorS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        motorS.setPower(gamepad1.left_stick_y);
        motorD.setPower(gamepad1.right_stick_y);
        if(gamepad1.left_bumper){
            gheara.setPosition(0.6);
        }
        else{
            gheara.setPosition(0);
        }
    }
}
