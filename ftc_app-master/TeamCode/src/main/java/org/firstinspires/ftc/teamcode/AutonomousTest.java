package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        boolean test_gyro = true;
        boolean test_PID_angle = false;
        boolean test_PID_walk = false;
        boolean test_cul = false;
        boolean test_range = false;
        boolean test_simple_walk = false;

        if (test_gyro){
            while (opModeIsActive())
            {
                telemetry.addData ("unghi : " , gyro.getHeading());
                telemetry.update();
            }
        }

        if (test_PID_angle){
            PID_angle(90.0);

            sleep(3 * cnst);

            PID_angle(-90.0);
        }

        if (test_PID_walk){
            PID_walk(5);

            sleep(3 * cnst);

            PID_walk(-5);
        }

        if (test_simple_walk){
            simple_walk(5);

            sleep(3 * cnst);

            simple_walk(-5);
        }

        if (test_cul){
            while (opModeIsActive())
            {
                telemetry.addData ("culoare : " , color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
                telemetry.update();
            }
        }

        if (test_range){
            while (opModeIsActive())
            {
                telemetry.addData ("left ultrasonic : " , range_left.rawUltrasonic());
                telemetry.addData ("left optical : " , range_left.rawOptical());
                telemetry.addData ("left cm : " , range_left.getDistance(DistanceUnit.CM));

                telemetry.addData ("right ultrasonic : " , range_right.rawUltrasonic());
                telemetry.addData ("right optical : " , range_right.rawOptical());
                telemetry.addData ("right cm : " , range_right.getDistance(DistanceUnit.CM));

                telemetry.update();
            }
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

     private void PID_angle (double angle){

         //modific unghil momentat cu angle
         //cred ca am pb cand sare la minus sau cacaturi dinastea

         double start = gyro.getHeading();

         double pGain = 0.3 ;
         double iGain = 0.2;
         double dGain = 0.1;

         double error = angle;
         double sum = angle;

         while (error > 1 || error < 1){
             double now = gyro.getHeading();
             if (now > 180){
                 now -= 360;
                 telemetry.addLine("sa imi bag pula");
                 break;
             }
             double last = error;
             error = angle - abs(now - start);
             sum += error;
             double derivata = error - last;

             double speed = (error * pGain) + (sum * iGain) + (derivata * dGain);

             telemetry.addData("speed : ", speed );
             telemetry.addData("error : ", error );
             telemetry.addData("unghi : ", now );
             telemetry.update();

             double absolut = abs(speed);
             absolut = min(absolut , 0.5);
             if (speed < 0){
                 speed = -absolut;
             }
             else{
                 speed = absolut;
             }

             mers_left.setPower(speed);
             mers_right.setPower(-speed);
         }

         mers_left.setPower(0);
         mers_right.setPower(0);
     }

     private boolean culoare(){
        int cul = color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
        return (cul == 8);
     }

    private void simple_walk (double walk){

        //merge walk cm (daca e cu minus in spate , altfel in fata)

        double start = range_right.getDistance(DistanceUnit.CM);
        double now = range_right.getDistance(DistanceUnit.CM);

        int dir = 1;
        if (walk < 0){
            dir = -1;
        }

        mers_left.setPower(0.5 * dir);
        mers_right.setPower(0.5 * dir);
        while (abs(start - now) < abs(walk)){
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
    }

     private void PID_walk (double walk){

         //merge walk cm (daca e cu minus in spate , altfel in fata)

         double start = range_right.getDistance(DistanceUnit.CM);

         double pGain = 0.3 ;
         double iGain = 0.2;
         double dGain = 0.1;

         double error = walk;
         double sum = walk;

         while (error > 0.1 || error < 0.1){
             double now = range_right.getDistance(DistanceUnit.CM);
             double last = error;
             error = walk - (now-start);
             sum += error;
             double derivata = error - last;

             double speed = (error * pGain) + (sum * iGain) + (derivata * dGain);

             telemetry.addData("speed : ", speed );
             telemetry.addData("error : ", error );
             telemetry.addData("dist : ", now );
             telemetry.update();

             double absolut = abs(speed);
             absolut = min(absolut , 0.5);
             if (speed < 0){
                 speed = -absolut;
             }
             else{
                 speed = absolut;
             }

             mers_left.setPower(speed);
             mers_right.setPower(speed);
         }

         mers_left.setPower(0);
         mers_right.setPower(0);
     }
}