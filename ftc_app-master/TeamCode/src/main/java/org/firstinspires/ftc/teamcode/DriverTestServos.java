package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRGyro;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;

@TeleOp (name = "DriverTestServos", group = "Driver")

public class DriverTestServos extends LinearOpMode
{
    //servos
    private Servo servo_L = null;
    private Servo servo_R = null;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        while (opModeIsActive())
        {
            firstGamepad();
        }
    }

    private void initialise()
    {
        //servo
        servo_L = hardwareMap.servo.get("servo_L");
        servo_R = hardwareMap.servo.get("servo_R");
    }

    private void firstGamepad()
    {
        if (gamepad1.dpad_up) {
            servo_L.setPosition(0.5);
            servo_R.setPosition(0.5);
        }
        else if (gamepad1.a) {
            servo_L.setPosition(0.7);
            servo_R.setPosition(0.3);
        }
        else if (gamepad1.b){
            servo_L.setPosition(0.3);
            servo_R.setPosition(0.7);
        }
        else if (gamepad1.dpad_left){
            servo_L.setPosition(1.0);
            servo_R.setPosition(0);
        }
        else if (gamepad1.dpad_right){
            servo_L.setPosition(0);
            servo_R.setPosition(1.0);
        }
    }
}