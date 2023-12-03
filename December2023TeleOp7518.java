// Attempted dump of the code used by the JV team in the final round of the December 2nd 2023 competition
// Might be slightly different from the actual code used in the competition

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
public class December2023TeleOp7518 extends LinearOpMode {
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        /* Setting up hardware */
        // Wheels
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftDrive");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightDrive");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftDrive");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightDrive");
        // Reverse left-side motors
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Arm
        DcMotor arm = hardwareMap.dcMotor.get("should");
        int arm_min_pos = arm.getCurrentPosition() - 1600;
        int arm_max_pos = arm.getCurrentPosition();
        int arm_target_pos = arm_max_pos - 100; // initial value
        arm.setTargetPosition(arm_target_pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Claw
        Servo clawLeft = hardwareMap.servo.get("clawLeft");
        Servo clawRight = hardwareMap.servo.get("clawRight");
        boolean opened = false;
        
        waitForStart();
        if (isStopRequested()) return;
        
        while (opModeIsActive()) {
            
            /* Configuring Gamepad 1 */
            // Left Stick (Move)
            double LeftX = gamepad1.left_stick_x;
            double LeftY = gamepad1.left_stick_y;
            telemetry.addData("LeftX", LeftX);
            telemetry.addData("LeftY", LeftY);
            // Right Stick (Turn)
            double RightX = gamepad1.right_stick_x;
            double RightY = gamepad1.right_stick_y;
            telemetry.addData("RightX", RightX);
            telemetry.addData("RightY", RightY);
            
            // Mecanum Drive Calculations
            // Adapted from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
            double denominator = Math.max(Math.abs(LeftY) + Math.abs(LeftX) + Math.abs(RightX), 1);
            double frontLeftPower = (LeftY - LeftX - RightX) / denominator;
            double backLeftPower = (LeftY + LeftX - RightX) / denominator;
            double frontRightPower = (LeftY + LeftX + RightX) / denominator;
            double backRightPower = (LeftY - LeftX + RightX) / denominator;
            
            // Wheel Motors
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            
            // Claw Servos
            if (gamepad1.right_bumper) {
                opened = true;
            } else if (gamepad1.left_bumper) {
                opened = false;
            }
            
            if (opened) {
                clawLeft.setPosition(0.4);
                clawRight.setPosition(0.6);
            } else {
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0.5);
            }
            
            telemetry.addData("clawleftpos", clawLeft.getPosition());
            telemetry.addData("clawrightpos", clawRight.getPosition());
            
            
            // Arm Motor
            if (gamepad1.left_trigger > 0.1 /*&& arm_target_pos > arm_min_pos*/) {
                arm_target_pos -= 2;
            }
            if (gamepad1.right_trigger > 0.1 /*&& arm_target_pos < arm_max_pos*/) {
                arm_target_pos += 2;
            }
            arm.setTargetPosition(arm_target_pos);
            arm.setPower(0.7);
            
            telemetry.addData("arm_min", arm_min_pos);
            telemetry.addData("arm_target", arm_target_pos);
            telemetry.addData("arm_max", arm_max_pos);
            telemetry.addData("arm_current", arm.getCurrentPosition());
            
            telemetry.update();
            
        }
    }
}