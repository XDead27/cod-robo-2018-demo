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
import com.qualcomm.robotcore.util.Range;

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
    static double TOLERANCE = 0.0001;

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


        boolean test_rotit = false;
        boolean test_mers = false;

        boolean test_senz = false;
        boolean test_mers_culoare = true;

        boolean test_gyro = false;
        boolean test_PID_angle = false;
        boolean test_PID_walk = false;
        boolean test_cul = false;
        boolean test_range = false;
        boolean test_simple_walk = false;


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

        if (test_mers_culoare){
            mers_culoare();
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


        walk_with_obstacle_and_range(20, 20000, false);
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


    private void rotit (double angle) {
        if (angle <= 1) {
            return;
        }
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
        /*gyro.resetZAxisIntegrator();

        double start = range_right.getDistance(DistanceUnit.CM);
        double now = range_right.getDistance(DistanceUnit.CM);

        int dir = 1;
        if (walk < 0){
            dir = -1;
        }
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
        rotit_delicat(end , speed);*/
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

    private void mers (double pasi){
        double end = range_left.rawUltrasonic() - pasi;
        double speed = 0.5;
        if (range_left.rawUltrasonic() - end < 0){
            speed = -speed;
        }
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (abs(range_left.rawUltrasonic() - end) > 5 && opModeIsActive()){
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
        mers_delicat(end , speed);
    }

    private void mers_delicat (double end , double speed){
        if (speed > 0){
            speed = 0.15;
        }
        else{
            speed = -0.15;
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

    private void mers_cul (){
        double speed = 0.135;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 0){
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
    }

     private boolean etapa (double angle) {
        rotit(angle);
        double start = range_left.rawUltrasonic();
        mers_cul();
        double end = range_left.rawUltrasonic();
        boolean ok = false;
        if (culoare()) {
            mers(5);
            mers(-5);
            ok = true;
        }
        telemetry.addData("start : " , start);
        telemetry.addData("end : " , end);
        telemetry.update();
        mers(end - start);
        rotit(-angle);
        return ok;
     }

     private void mers_culoare() {
         if (etapa(0)) {
             return;
         }
         /*if (etapa(-20)) {
             return;
         }
         if (etapa(20)) {
             return;
         }*/
     }


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
         /*long delay  = 0L;
         final long period = 100L; //while ce opereaza la frecventa de 100 ms

         TimerTask PIDwalk = new TimerTask() {

             double steadyTimer = 0;

             double iGain = 0.0; //TODO: incearca sa setezi la 0 sa vedem daca se rezolva doar cu PD control
             double pGain = 1/target; //daca zidul sau alt robot se apropie mai mult decat trebuie atunci sa mearga la viteza maxima in spate
             double dGain = 0.0;

             double errorRight = target - range_right.getDistance(DistanceUnit.CM);
             double errorLeft = target - range_left.getDistance(DistanceUnit.CM);

             double sumRight = errorRight;
             double sumLeft = errorLeft;

             double auxRight;
             double auxLeft;

             public void run() {

                 if(Math.abs(errorLeft) < 5 || Math.abs(errorRight) < 5){
                     steadyTimer += period;
                 }else{
                     steadyTimer = 0;
                 }

                 auxRight = errorRight;
                 auxLeft = errorLeft;
                 errorRight = target - range_right.getDistance(DistanceUnit.CM);
                 errorLeft = target - range_left.getDistance(DistanceUnit.CM);
                 sumRight += errorRight;
                 sumLeft += errorLeft;

                 double derivativeRight = errorRight - auxRight;
                 double derivativeLeft = errorLeft - auxLeft;

                 double speedRight = (errorRight * pGain) + (sumRight * iGain) + (derivativeRight * dGain);
                 double speedLeft = (errorLeft * pGain) + (sumLeft * iGain) + (derivativeLeft * dGain);

                 //inversam viteza ca sa fie pozitiva
                 speedRight = Range.clip(-speedRight, -1, 1);
                 speedLeft = Range.clip(-speedLeft, -1, 1);

                 //setWheelsPowerWithGyro(speedLeft, speedRight);

                 setWheelsPower(speedLeft,speedRight);

                 if(steadyTimer > 1000){
                     cancel();
                 }

             }
         };

         timer.scheduleAtFixedRate(PIDwalk, delay, period);*/

         ///TEST**************
         double iGain = 0.0; //TODO: incearca sa setezi la 0 sa vedem daca se rezolva doar cu PD control
         double pGain = 1/target; //daca zidul sau alt robot se apropie mai mult decat trebuie atunci sa mearga la viteza maxima in spate
         double dGain = 0.0;

         double errorRight = target - range_right.getDistance(DistanceUnit.CM);
         double errorLeft = target - range_left.getDistance(DistanceUnit.CM);

         double sumRight = errorRight;
         double sumLeft = errorLeft;

         double auxRight;
         double auxLeft;

         while(opModeIsActive()){

             auxRight = errorRight;
             auxLeft = errorLeft;
             errorRight = target - range_right.getDistance(DistanceUnit.CM);
             errorLeft = target - range_left.getDistance(DistanceUnit.CM);
             sumRight += errorRight;
             sumLeft += errorLeft;

             double derivativeRight = errorRight - auxRight;
             double derivativeLeft = errorLeft - auxLeft;

             double speedRight = (errorRight * pGain)/* + (sumRight * iGain) + (derivativeRight * dGain)*/;
             double speedLeft = (errorLeft * pGain)/* + (sumLeft * iGain) + (derivativeLeft * dGain)*/;

             //inversam viteza ca sa fie pozitiva
             speedLeft = -speedLeft;
             speedRight = -speedRight;

             speedRight = Range.clip(speedRight, -0.9, 0.9);
             speedLeft = Range.clip(speedLeft, -0.9, 0.9);

             if(Math.abs(speedLeft) < TOLERANCE ){
                speedLeft = 0;
             }
             if(Math.abs(speedRight) < TOLERANCE ){
                 speedRight = 0;
             }


             setWheelsPower(speedLeft,speedRight);

             telemetry.addData("speed left", speedLeft);
             telemetry.addData("speed right", speedRight);

             telemetry.addData("error left ", errorLeft);
             telemetry.addData("error right ", errorRight);
             telemetry.addData("range left ", range_left.getDistance(DistanceUnit.CM));
             telemetry.addData("range right ", range_right.getDistance(DistanceUnit.CM));
             telemetry.update();
         }

         ///TEST END*****************
         stopWheels();
     }
}

