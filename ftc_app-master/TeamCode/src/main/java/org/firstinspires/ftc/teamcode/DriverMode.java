package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRGyro;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;


@TeleOp (name = "DriverMode_Test", group = "Driver")

public class DriverMode extends LinearOpMode
{

    //motoare
    private DcMotor mers_left = null;
    private DcMotor mers_right = null;
    private DcMotor glisare = null;
    private DcMotor ridicare_cutie = null;
    private DcMotor ridicare_perii = null;
    private DcMotor rotire_perii = null;

    //senzori
    //private GyroSensor gyro = null;
    private ModernRoboticsI2cGyro gyro = null;
    private ModernRoboticsI2cRangeSensor range_right = null;
    private ModernRoboticsI2cRangeSensor range_left = null;
    private ModernRoboticsI2cColorSensor color = null;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        while (opModeIsActive())
        {
            firstGamepad();
            secondGamepad();
            telemetry.addData("gyro: ", gyro.isCalibrating() );
            telemetry.addData("gyro: ", gyro.getHeading() );
            telemetry.update();
        }
    }

    private void initialise()
    {
        //mapare
        mers_left = hardwareMap.dcMotor.get("mers_left");
        mers_right = hardwareMap.dcMotor.get("mers_right");

        //senzori
        //gyro = hardwareMap.gyroSensor.get("gyro");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class ,"gyro");
        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class ,"range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class ,"range_right");
        color = hardwareMap.get(ModernRoboticsI2cColorSensor.class , "color");

        //setare puteri la 0
        mers_left.setPower(0);
        mers_right.setPower(0);

        //setare directii
        mers_left.setDirection(DcMotorSimple.Direction.FORWARD);
        mers_right.setDirection(DcMotorSimple.Direction.REVERSE);

        //calibreat gyro
        gyro.calibrate();

        //senzor de culoare
        color.enableLed(true); //daca ma uit la un : obiect - true ; lumina - false
    }

    private void firstGamepad()
    {
        if (gamepad1.left_stick_y > 0.1) mers_left.setPower(min(gamepad1.left_stick_y , 0.9));
        else mers_left.setPower(0);
        if (gamepad1.left_stick_y < -0.1) mers_left.setPower(max(gamepad1.left_stick_y , -0.9));
        else mers_left.setPower(0);

        if (gamepad1.right_stick_y > 0.1) mers_right.setPower(min(gamepad1.right_stick_y , 0.9));
        else mers_right.setPower(0);
        if (gamepad1.right_stick_y < -0.1) mers_right.setPower(max(gamepad1.right_stick_y , -0.9));
        else mers_right.setPower(0);
    }

    private void secondGamepad()
    {
        if ( gamepad2.dpad_up ) ridicare_cutie.setPower(0.5);
        else ridicare_cutie.setPower(0);
        if ( gamepad2.dpad_down ) ridicare_cutie.setPower(-0.5);
        else ridicare_cutie.setPower(0);

        if ( gamepad2.a ) ridicare_perii.setPower(-0.4);
        else ridicare_perii.setPower(0);
        if ( gamepad2.b ) ridicare_perii.setPower(0.4);
        else ridicare_perii.setPower(0);

        if ( gamepad2.left_bumper ) rotire_perii.setPower(-0.4);
        else rotire_perii.setPower(0);
        if ( gamepad2.right_bumper ) rotire_perii.setPower(0.4);
        else rotire_perii.setPower(0);

        if (gamepad1.left_stick_y > 0.1) glisare.setPower(min(gamepad1.left_stick_y , 0.9));
        else mers_left.setPower(0);
        if (gamepad1.left_stick_y < -0.1) glisare.setPower(max(gamepad1.left_stick_y , -0.9));
        else mers_left.setPower(0);
     }
}