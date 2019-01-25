package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.max;

@Autonomous(name = "Autonomous_test", group = "Autonomous")

public class AutonomousTest extends LinearOpMode
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

    int cnst = 1000;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        //crater simplu :

        //ma misc in fata pana ajung pe linia obiectelor
        //ma intorc ca sa fiu paralel cu obiectele
        //dau in spate pana la ultimul obiect
        //verific
        //merg pana la al doilea
        //verific
        //merg pana la al treilea
        //verific
        //cand gasesc cul == galben fac 90 grade , merg in fata si ma opresc pe crater

        //--------------------------------------------------------------------------

        //crater cu optimizare :

        //ma misc in fata pana ajung pe linia obiectelor
        //ma intorc ca sa fiu paralel cu obiectele
        //dau in spate pana la ultimul obiect
        //verific
        //merg pana la al doilea
        //verific
        //merg pana la al treilea
        //verific
        //cand gasesc cul == galben fac 90 , un fata spate , -90
        //ma duc pana la perete si ma pun paralel si ma pun pe crater

        //---------------------------------------------------------------------------

        boolean test_rotit = false;
        boolean test_mers = true;

        boolean test_senz = false;

        if (test_senz){
            while (opModeIsActive())
            {
                telemetry.addData("distanta dreapta : " , range_right.rawUltrasonic());
                telemetry.addData("distanta stanga : " , range_left.rawUltrasonic());
                telemetry.addData ("culoare : " , color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
                telemetry.addData ("calibrat : " , gyro.isCalibrating());
                telemetry.addData ("unghi : " , gyro.getHeading());
                telemetry.update();
            }
        }

        if (test_rotit){
            rotit (90);
            sleep (3 * cnst);
            rotit (-90);
            sleep (3 * cnst);
            rotit (180);
            sleep (3 * cnst);
            rotit (-180);
        }

        if (test_mers){
            mers(50);
            sleep (3 * cnst);
            mers(100);
            sleep (3 * cnst);
            mers(30);
            sleep (3 * cnst);
            mers(70);
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
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class ,"gyro");
        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class ,"range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class ,"range_right");
        color = hardwareMap.get(ModernRoboticsI2cColorSensor.class , "color");

        //setare puteri la 0
        mers_left.setPower(0);
        mers_right.setPower(0);
        glisare.setPower(0);
        ridicare_cutie.setPower(0);
        ridicare_perii.setPower(0);
        rotire_perii.setPower(0);

        //setare directii
        mers_left.setDirection(DcMotorSimple.Direction.FORWARD);
        mers_right.setDirection(DcMotorSimple.Direction.FORWARD);
        glisare.setDirection(DcMotorSimple.Direction.FORWARD);
        ridicare_cutie.setDirection(DcMotorSimple.Direction.FORWARD);
        ridicare_perii.setDirection(DcMotorSimple.Direction.FORWARD);
        rotire_perii.setDirection(DcMotorSimple.Direction.FORWARD);

        //calibreat gyro
        gyro.calibrate();
        while (gyro.isCalibrating()){
            idle();
        }

        //senzor de culoare
        color.enableLed(true); //daca ma uit la un : obiect - true ; lumina - false

    }

    private void rotit (double angle){
        double end = gyro.getHeading() + angle;
        while (end < 0){
            end += 360;
        }
        double speed = 0.5;
        if (angle < 0){
            speed = -speed;
        }
        mers_left.setPower(speed);
        mers_right.setPower(-speed);
        while (abs(gyro.getHeading() - end) > 5 && opModeIsActive()){
            telemetry.addData("angle rotit : " , gyro.getHeading());
            telemetry.update();
        }
        rotit_delicat(end , speed);
    }

    private void rotit_delicat (double end , double speed){
        if (speed > 0){
            speed = 0.3;
        }
        else{
            speed = -0.3;
        }
        mers_left.setPower(speed);
        mers_right.setPower(-speed);
        while (abs(gyro.getHeading() - end) > 1 && opModeIsActive()){
            telemetry.addData("angle delicat : " , gyro.getHeading());
            telemetry.update();
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
    }

    private void mers (double end){
        double speed = 0.5;
        if (range_left.rawUltrasonic() - end < 0){
            speed = -speed;
        }
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (abs(range_left.rawUltrasonic() - end) > 5 && opModeIsActive()){
            telemetry.addData("dist mers : " , range_left.rawUltrasonic());
            telemetry.update();
        }
        mers_delicat(end , speed);
    }

    private void mers_delicat (double end , double speed){
        if (speed > 0){
            speed = 0.3;
        }
        else{
            speed = -0.3;
        }
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (abs(range_left.rawUltrasonic() - end) > 1 && opModeIsActive()){
            telemetry.addData("dist delicat : " , range_left.rawUltrasonic());
            telemetry.update();
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
    }

     private boolean culoare(){
        int cul = color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
        return (cul == 9 || cul == 10);
     }


}