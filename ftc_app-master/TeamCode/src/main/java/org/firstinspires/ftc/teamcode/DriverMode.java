package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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
    //private ModernRoboticsI2cGyro gyro = null;
    //private ModernRoboticsI2cRangeSensor range_right = null;
    //private ModernRoboticsI2cRangeSensor range_left = null;
    //private ModernRoboticsI2cColorSensor color = null;

    private final double deadzone = 0.1;
    private final int max_glisare = 100 * 67;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        while (opModeIsActive())
        {
            firstGamepad();
            secondGamepad();
        }
    }

    private void initialise()
    {
        //mapare
        mers_left = hardwareMap.dcMotor.get("mers_left");
        mers_right = hardwareMap.dcMotor.get("mers_right");
        glisare = hardwareMap.dcMotor.get("glisare");
        ridicare_cutie = hardwareMap.dcMotor.get("ridicare_cutie");
        ridicare_perii = hardwareMap.dcMotor.get("ridicare_perii");
        rotire_perii = hardwareMap.dcMotor.get("rotire_perii");

        //senzori
        //gyro = hardwareMap.gyroSensor.get("gyro");
        //gyro = hardwareMap.get(ModernRoboticsI2cGyro.class ,"gyro");
        //range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class ,"range_left");
        //range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class ,"range_right");
        //color = hardwareMap.get(ModernRoboticsI2cColorSensor.class , "color");

        //setare puteri la 0
        mers_left.setPower(0);
        mers_right.setPower(0);
        glisare.setPower(0);
        ridicare_cutie.setPower(0);
        ridicare_perii.setPower(0);
        rotire_perii.setPower(0);

        //setare directii
        mers_left.setDirection(DcMotorSimple.Direction.REVERSE);
        mers_right.setDirection(DcMotorSimple.Direction.REVERSE);
        glisare.setDirection(DcMotorSimple.Direction.FORWARD);
        ridicare_cutie.setDirection(DcMotorSimple.Direction.FORWARD);
        ridicare_perii.setDirection(DcMotorSimple.Direction.FORWARD);
        rotire_perii.setDirection(DcMotorSimple.Direction.FORWARD);

        //setat encodere
        glisare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //calibreat gyro
        //gyro.calibrate();

        //senzor de culoare
        //color.enableLed(true); //daca ma uit la un : obiect - true ; lumina - false
    }

    private void firstGamepad()
    {
        if (abs(gamepad1.left_stick_y) > 0.1) mers_left.setPower(Range.clip(gamepad1.left_stick_y, -0.9, 0.9));
        else mers_left.setPower(0);

        if (abs(gamepad1.right_stick_y) > 0.1) mers_right.setPower(Range.clip(gamepad1.right_stick_y, -0.9, 0.9));
        else mers_right.setPower(0);
    }

    private void secondGamepad()
    {
        if ( gamepad2.dpad_up ) ridicare_cutie.setPower(0.6);
        else if ( gamepad2.dpad_down ) ridicare_cutie.setPower(-0.3);
        else ridicare_cutie.setPower(0);

        if(Math.abs(gamepad2.left_trigger) > deadzone)
            ridicare_perii.setPower(0.6);
        else
            ridicare_perii.setPower(0);

        if(Math.abs(gamepad2.right_trigger) > deadzone)
            ridicare_perii.setPower(-0.3);
        else
            ridicare_perii.setPower(0);
        /*if ( gamepad2.a ) ridicare_perii.setPower(-0.4);
        else if ( gamepad2.b ) ridicare_perii.setPower(0.4);
        else ridicare_perii.setPower(0);*/

        if ( gamepad2.left_bumper ) rotire_perii.setPower(-0.4);
        else if ( gamepad2.right_bumper ) rotire_perii.setPower(0.4);
        else rotire_perii.setPower(0);

        if (gamepad2.left_stick_y > deadzone && glisare.getCurrentPosition() < max_glisare) {
            glisare.setPower(0.5);
            telemetry.addData ("dist : " ,  glisare.getCurrentPosition());
            telemetry.update();
        }
        else if (gamepad2.left_stick_y < -deadzone && glisare.getCurrentPosition() > 0) {
            glisare.setPower(-0.5);
            telemetry.addData ("dist : " ,  glisare.getCurrentPosition());
            telemetry.update();
        }
        else glisare.setPower(0);
    }
}