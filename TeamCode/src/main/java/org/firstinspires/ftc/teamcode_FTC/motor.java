package org.firstinspires.ftc.teamcode_FTC;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class motor extends OpMode{
    DcMotorEx motorbl,motorbr,motorfl,motorfr;
    @Override
    public void
    init() {

        motorbl = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorbr = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorfl = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorfr = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        motorbl.setPower(-gamepad1.left_stick_y);
        motorfl.setPower(-gamepad1.left_stick_y);
        motorbr.setPower(gamepad1.left_stick_y);
        motorfr.setPower(gamepad1.left_stick_y);
        telemetry.addData("Pozitiebl:", motorbl.getCurrentPosition());
        telemetry.addData("Pozitiebr:", motorbr.getCurrentPosition());
        telemetry.addData("Pozitiefl:", motorfl.getCurrentPosition());
        telemetry.addData("Pozitiefr:", motorfr.getCurrentPosition());
        telemetry.update();
    }
}
