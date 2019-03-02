package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


@TeleOp(name = "Driver Servo Test", group = "Driver")
public class DriverServoTest extends LinearOpMode {

    private Servo ServoL = null;
    private Servo ServoR = null;

    @Override
    public void runOpMode() {

        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");

        ServoL.setDirection(Servo.Direction.FORWARD);
        ServoR.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status :", "wait for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.right_bumper){
                ServoL.setPosition(Servo.MAX_POSITION);
                ServoR.setPosition(Servo.MAX_POSITION);
            }
            else if(gamepad1.left_bumper){
                ServoL.setPosition(Servo.MIN_POSITION);
                ServoR.setPosition(Servo.MIN_POSITION);
            }

        }


    }


}
