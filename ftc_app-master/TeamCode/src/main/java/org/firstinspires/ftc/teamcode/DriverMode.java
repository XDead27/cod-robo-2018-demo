package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRGyro;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp (name = "DriverMode_Test", group = "Driver")

public class DriverMode extends LinearOpMode
{

    //motoare
    /*private DcMotor Motor_Cob_Sist = null;
    private DcMotor Motor_Perii = null;
    private DcMotor Motor_Mos_Dublu = null;
    private DcMotor Motor_Mos_Simplu = null;*/
    private DcMotor LeftMotors = null;
    private DcMotor RightMotors = null;

    //senzori
    private GyroSensor gyro = null;
    private I2cDevice range_right = null;
    private I2cDevice range_left = null;

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
        /*Motor_Cob_Sist = hardwareMap.dcMotor.get("MOTOR_COB_SIST");
        Motor_Perii = hardwareMap.dcMotor.get("MOTOR_PERII");
        Motor_Mos_Dublu = hardwareMap.dcMotor.get("MOTOR_MOS_DUBLU");
        Motor_Mos_Simplu = hardwareMap.dcMotor.get("MOTOR_MOS_SIMPLU");*/
        LeftMotors = hardwareMap.dcMotor.get("LeftMotors");
        RightMotors = hardwareMap.dcMotor.get("RightMotors");

        //senzori
        gyro = hardwareMap.gyroSensor.get("gyro");
        //range_right = hardwareMap.i2cDevice.get("range_right");
       // range_left = hardwareMap.i2cDevice.get("range_left");

        //setare puteri la 0
        /*Motor_Cob_Sist.setPower(0);
        Motor_Perii.setPower(0);
        Motor_Mos_Dublu.setPower(0);
        Motor_Mos_Simplu.setPower(0);*/
        LeftMotors.setPower(0);
        RightMotors.setPower(0);

        //setare directii
        /*Motor_Cob_Sist.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_Perii.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_Mos_Simplu.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_Mos_Dublu.setDirection(DcMotorSimple.Direction.FORWARD);*/
        LeftMotors.setDirection(DcMotorSimple.Direction.FORWARD);
        RightMotors.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.calibrate();
    }

    private void firstGamepad()
    {
        if (Math.abs(gamepad1.left_stick_y) > 0.1) LeftMotors.setPower(gamepad1.left_stick_y);
        else LeftMotors.setPower(0);

        if (Math.abs(gamepad1.right_stick_y) > 0.1) RightMotors.setPower(gamepad1.right_stick_y);
        else RightMotors.setPower(0);
    }

    private void secondGamepad()
    {
        /*if ( gamepad2.dpad_up ) Motor_Mos_Simplu.setPower(0.5);

        else Motor_Mos_Simplu.setPower(0);

        if ( gamepad2.dpad_down ) Motor_Mos_Simplu.setPower(-0.5);
        else Motor_Mos_Simplu.setPower(0);

        if ( gamepad2.left_trigger > 0.1 ) Motor_Perii.setPower(0.4);
        else Motor_Perii.setPower(0.4);

        if ( gamepad2.right_trigger > 0.1 ) Motor_Perii.setPower(-0.4);
        else Motor_Perii.setPower(0);

        if ( gamepad2.a ) Motor_Cob_Sist.setPower(0.4);
        else Motor_Cob_Sist.setPower(0);

        if ( gamepad2.b ) Motor_Cob_Sist.setPower(-0.4);
        else Motor_Cob_Sist.setPower(0);

        if ( gamepad2.left_bumper ) Motor_Mos_Dublu.setPower(0.4);
        else Motor_Mos_Dublu.setPower(0);

        if ( gamepad2.right_bumper ) Motor_Mos_Dublu.setPower(-0.4);
        else Motor_Mos_Dublu.setPower(0);*/
     }
}