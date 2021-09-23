package org.firstinspires.ftc.teamcode_FTC;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Nou_Program_pentrucada extends OpMode {
    private DcMotorEx motorf;
    private DcMotorEx motorst;
    private DcMotorEx motordr;
    private DcMotorEx motors;
    public boolean randomshit = false;
    private boolean alast = false;
    private int apoz = 0;
    @Override
    public void init() {
        motorf = hardwareMap.get(DcMotorEx.class, "motorF");
        motordr = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorst = hardwareMap.get(DcMotorEx.class, "motorST");
        motors = hardwareMap.get(DcMotorEx.class, "motorS");
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y > 0){
            motordr.setPower(-1);
            motorst.setPower(-1);
            motors.setPower(-1);
            motorf.setPower(-1);
        }
        else if(gamepad1.left_stick_y < 0){
            motordr.setPower(1);
            motorst.setPower(1);
            motors.setPower(1);
            motorf.setPower(1);
        }
        if(gamepad1.left_stick_y == 0) {
            if (gamepad1.right_stick_y > 0) {
                motors.setPower(1);
                motorf.setPower(-1);
            }
            if (gamepad1.right_stick_y < 0) {
                motors.setPower(-1);
                motorf.setPower(1);
            }
            if (gamepad1.right_stick_x > 0) {
                motordr.setPower(1);
                motorst.setPower(-1);
            }
            if (gamepad1.right_stick_x < 0) {
                motordr.setPower(-1);
                motorst.setPower(1);
            }
        }
        if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_y == 0) {
            if (gamepad1.dpad_up) {
                motorf.setPower(1);
            } else if (gamepad1.y) {
                motorf.setPower(-1);
            } else {
                motorf.setPower(0);
            }

            //motor E
            if (gamepad1.dpad_right) {
                motordr.setPower(1);
            } else if (gamepad1.b) {
                motordr.setPower(-1);
            } else {
                motordr.setPower(0);
            }

            //motor S
            if (gamepad1.dpad_down) {
                motors.setPower(1);
            } else if (gamepad1.a) {
                motors.setPower(-1);
            } else {
                motors.setPower(0);
            }

            //motor V
            if (gamepad1.dpad_left) {
                motorst.setPower(1);
            } else if (gamepad1.x) {
                motorst.setPower(-1);
            } else {
                motorst.setPower(0);
            }
        }
    }
}
