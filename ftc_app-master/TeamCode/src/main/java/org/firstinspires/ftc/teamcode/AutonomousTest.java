package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.Range;

import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

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
    private ModernRoboticsI2cGyro gyro = null;
    private ModernRoboticsI2cRangeSensor range_right = null;
    private ModernRoboticsI2cRangeSensor range_left = null;
    private ModernRoboticsI2cColorSensor color = null;

    int cnst = 1000;

    static private Timer timer;

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

        //senzor de culoare
        color.enableLed(true); //daca ma uit la un : obiect - true ; lumina - false

        timer = new Timer();

    }


    private void setWheelsPower(double powerleft, double powerright){
        mers_right.setPower(powerright);
        mers_left.setPower(powerleft);
    }

    private void setWheelsPowerWithGyro(double powerleft, double powerright){
        // TODO adauga o variabila pentru a seta gradul la care se considera devierea
        // TODO daca trece peste 90 de grade sa se apeleze functia rotate ??
        // o ia la STANGA
        if (gyro.getHeading() > 270 && gyro.getHeading() <= 300) {
            mers_right.setPower(powerright - 0.4);
        }
        if (gyro.getHeading() > 300 && gyro.getHeading() <= 330) {
            mers_right.setPower(powerright - 0.3);
        }
        if (gyro.getHeading() > 330 && gyro.getHeading() <= 359) {
            mers_right.setPower(powerright - 0.2);
        }
        // o ia la DREAPTA
        if (gyro.getHeading() < 90 && gyro.getHeading() >= 60) {
            mers_left.setPower(powerright - 0.4);
        }
        if (gyro.getHeading() < 60 && gyro.getHeading() >= 30) {
            mers_left.setPower(powerright - 0.3);
        }
        if (gyro.getHeading() < 30 && gyro.getHeading() >= 1) {
            mers_left.setPower(powerright - 0.2);
        }
        if (gyro.getHeading() == 0) {
            setWheelsPower(powerleft, powerright);
        }
    }

    private void stopWheels(){
        setWheelsPower(0, 0);
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
             absolut = Range.clip(speed, -0.5, 0.5);
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
        gyro.resetZAxisIntegrator();

        double start = range_right.getDistance(DistanceUnit.CM);
        double now = range_right.getDistance(DistanceUnit.CM);

        int dir = 1;
        if (walk < 0){
            dir = -1;
        }

        mers_left.setPower(0.5 * dir);
        mers_right.setPower(0.5 * dir);

        while (abs(start - now) < abs(walk)){
            now = range_right.getDistance(DistanceUnit.CM);
            // TODO adauga o variabila pentru a seta gradul la care se considera devierea
            // TODO daca trece peste 90 de grade sa se apeleze functia rotate ??
            // o ia la STANGA
            if (gyro.getHeading() > 270 && gyro.getHeading() <= 300) {
                mers_right.setPower(0.2 * dir);
            }
            if (gyro.getHeading() > 300 && gyro.getHeading() <= 330) {
                mers_right.setPower(0.3 * dir);
            }
            if (gyro.getHeading() > 330 && gyro.getHeading() <= 359) {
                mers_right.setPower(0.4 * dir);
            }
            // o ia la DREAPTA
            if (gyro.getHeading() < 90 && gyro.getHeading() >= 60) {
                mers_left.setPower(0.2 * dir);
            }
            if (gyro.getHeading() < 60 && gyro.getHeading() >= 30) {
                mers_left.setPower(0.3 * dir);
            }
            if (gyro.getHeading() < 30 && gyro.getHeading() >= 1) {
                mers_left.setPower(0.4 * dir);
            }
            if (gyro.getHeading() == 0) {
                mers_left.setPower(0.5 * dir);
                mers_right.setPower(0.5 * dir);
            }
            idle();
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

     /*private void walk_with_range(double distanceFromWall, double approxDistance, boolean bHasToBeAllignedWithWall){
        //se aliniaza cu zidul din fata
        if(bHasToBeAllignedWithWall){
            while(Math.abs(range_right.rawUltrasonic() - range_left.rawUltrasonic()) < 10){
                //TODO: foloseste PID_angle dupa calcularea unghiului
                int dir = (int)Math.signum(range_right.rawUltrasonic() - range_left.rawUltrasonic());
                mers_right.setPower(0.2 * dir);
                mers_left.setPower(-0.2 * dir);
            }
        }

        //cat timp e pe loc verifica rapid daca se intrapta ceva spre robot
         TimerTask checkObstacle = new TimerTask() {
             double deltaRange_Right;
             double deltaRange_Left;

             double auxRange_Right = range_right.rawUltrasonic();
             double auxRange_Left = range_left.rawUltrasonic();

             long startTime = new Date().getTime();

             public void run() {
                 deltaRange_Right = range_right.rawUltrasonic() - auxRange_Right;
                 deltaRange_Left = range_left.rawUltrasonic() - auxRange_Left;

                 //daca se gaseste obstacol sau nu se gaseste obstacol pentru 1 secunde atunci iese din task
                 if(EvitareObstacol(deltaRange_Right, deltaRange_Left) || new Date().getTime() - startTime > 1)
                     cancel();
             }
         };

         long delay  = 0L;
         long period = 100L; //while ce opereaza la frecventa de 100 ms
         timer.scheduleAtFixedRate(checkObstacle, delay, period);
     }*/

     private void walk_with_obstacle_and_range(double distanceFromWall, double approxDistance, boolean bHasToBeAllignedWithWall){
         //se aliniaza cu zidul din fata
         if(bHasToBeAllignedWithWall){
             while(Math.abs(range_right.rawUltrasonic() - range_left.rawUltrasonic()) < 10/*magic number*/){
                 //TODO: foloseste PID_angle dupa calcularea unghiului
                 int dir = (int)Math.signum(range_right.rawUltrasonic() - range_left.rawUltrasonic());
                 mers_right.setPower(0.2 * dir);
                 mers_left.setPower(-0.2 * dir);
             }
         }

         final double target = distanceFromWall;

         TimerTask PIDwalk = new TimerTask() {
             double shortestDistanceToWall = Math.min(range_left.getDistance(DistanceUnit.CM), range_right.getDistance(DistanceUnit.CM));

             double iGain = 0.2;
             double pGain = 0.3;
             double dGain = 0.1;

             //TODO: creaza inca un pid, fiecare sa fie pentru cate un range si respectiva serie de roti

             double error = target - shortestDistanceToWall;
             double sum = error;

             double aux;

             public void run() {
                 shortestDistanceToWall = Math.min(range_left.getDistance(DistanceUnit.CM), range_right.getDistance(DistanceUnit.CM));

                 aux = error;
                 error = target - shortestDistanceToWall;
                 sum += error;

                 double derivative = error - aux;

                 double speed = (error * pGain) + (sum * iGain) + (derivative * dGain);

                 setWheelsPowerWithGyro(speed, speed);

             }
         };

         long delay  = 0L;
         long period = 100L; //while ce opereaza la frecventa de 100 ms
         timer.scheduleAtFixedRate(PIDwalk, delay, period);
     }
}

