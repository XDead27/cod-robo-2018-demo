package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;


@Autonomous(name = "Autonomous_test", group = "Autonomous")

public class AutonomousTest extends LinearOpMode
{

    //motoare
    private DcMotor LeftMotors = null;
    private DcMotor RightMotors = null;

    //senzori
    private GyroSensor gyro = null;
    private I2cDevice range_right = null;
    private I2cDevice range_left = null;

    int cnst = 1000;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        PID(90.0);

        sleep(3 * cnst);

        PID(-90.0);
    }

    private void initialise()
    {
        //mapare
        LeftMotors = hardwareMap.dcMotor.get("LeftMotors");
        RightMotors = hardwareMap.dcMotor.get("RightMotors");

        //senzori
        gyro = hardwareMap.gyroSensor.get("gyro");

        //setare puteri la 0
        LeftMotors.setPower(0);
        RightMotors.setPower(0);

        //setare directii
        LeftMotors.setDirection(DcMotorSimple.Direction.FORWARD);
        RightMotors.setDirection(DcMotorSimple.Direction.REVERSE);

        //calibreat gyro
        gyro.calibrate();
    }

     private void PID(double angle){

        //modific unghil momentat cu angle

         double now = gyro.getHeading();

         double pGain = 0.3 ;
         double iGain = 0.2;
         double dGain = 0.1;

         double error = angle;
         double sum = angle;

         while (error > 1 || error < 1){
             double acum = gyro.getHeading();
             double last = error;
             error = now + angle - acum;
             sum += error;
             double derivata = error - last;

             double speed = (error * pGain) + (sum * iGain) + (derivata * dGain);

             telemetry.addData("speed : ", speed );
             telemetry.addData("error : ", error );
             telemetry.addData("unghi : ", acum );
             telemetry.update();

             double absolut = Math.abs(speed);
             absolut = Math.min(absolut , 0.5);
             if (speed < 0){
                 speed = -absolut;
             }
             else{
                 speed = absolut;
             }

             LeftMotors.setPower(speed);
             RightMotors.setPower(-speed);
         }

         LeftMotors.setPower(0);
         RightMotors.setPower(0);
     }
}