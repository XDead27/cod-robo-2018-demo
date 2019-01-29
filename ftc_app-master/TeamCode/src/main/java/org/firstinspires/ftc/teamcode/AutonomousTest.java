package org.firstinspires.ftc.teamcode;

import android.os.Handler;

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
=======
>>>>>>> postu_enache

import static java.lang.Math.abs;

@Autonomous(name = "Autonomous_test", group = "Autonomous")

public class AutonomousTest extends LinearOpMode {

    //motoare
    protected DcMotor mers_left = null;
    protected DcMotor mers_right = null;
    protected DcMotor glisare = null;
    protected DcMotor ridicare_cutie = null;
    protected DcMotor ridicare_perii = null;
    protected DcMotor rotire_perii = null;

    //senzori
<<<<<<< HEAD
    private ModernRoboticsI2cGyro gyro = null;
    private ModernRoboticsI2cRangeSensor range_right = null;
    private ModernRoboticsI2cRangeSensor range_left = null;
    private ModernRoboticsI2cColorSensor color = null;

    int cnst = 1000;
    protected static double TOLERANCE = 0.0001;
    protected static double DISTANCE_BETWEEN_SLOTS = 30;

    static private Timer timer;
=======
    protected ModernRoboticsI2cGyro gyro = null;
    protected ModernRoboticsI2cRangeSensor range_right = null;
    protected ModernRoboticsI2cRangeSensor range_left = null;
    protected ModernRoboticsI2cColorSensor color = null;

    final int const_sleep = 1000;
    final int const_encoder = 67;
>>>>>>> postu_enache

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

<<<<<<< HEAD
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
        boolean test_mers_culoare = false;

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
=======
        boolean test_rotit = false;
        boolean test_mers_encoder = false;

        boolean test_senz = false;
        boolean test_mers_culoare = true;

        if (test_senz) {
            while (opModeIsActive()) {
                telemetry.addData("distanta dreapta : ", range_right.rawUltrasonic());
                telemetry.addData("distanta stanga : ", range_left.rawUltrasonic());
                telemetry.addData("culoare : ", color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
                telemetry.addData("calibrat : ", gyro.isCalibrating());
                telemetry.addData("unghi : ", gyro.getHeading());
                telemetry.update();
            }
        }

        if (test_rotit) {
            rotit(90);
            sleep(3 * const_sleep);
            rotit(-90);
            sleep(3 * const_sleep);
            rotit(180);
            sleep(3 * const_sleep);
            rotit(-180);
            sleep(3 * const_sleep);
            rotit(20);
            sleep(3 * const_sleep);
            rotit(-20);
        }

        if (test_mers_encoder) {
            mers_encoder(50 * const_encoder);
            sleep(3 * const_sleep);

            mers_encoder(-100 * const_encoder);
            sleep(3 * const_sleep);

            mers_encoder(30 * const_encoder);
            sleep(3 * const_sleep);
>>>>>>> postu_enache

            mers_encoder(-70 * const_encoder);
        }

        if (test_mers_culoare) {
            mers_culoare();
        }
<<<<<<< HEAD


        walk_with_obstacle_and_range(20, 20000, false);
=======
>>>>>>> postu_enache
    }

    protected void initialise() {
        //mapare
        mers_left = hardwareMap.dcMotor.get("mers_left");
        mers_right = hardwareMap.dcMotor.get("mers_right");
        glisare = hardwareMap.dcMotor.get("glisare");
        ridicare_cutie = hardwareMap.dcMotor.get("ridicare_cutie");
        ridicare_perii = hardwareMap.dcMotor.get("ridicare_perii");
        rotire_perii = hardwareMap.dcMotor.get("rotire_perii");

        //senzori
        //gyro = hardwareMap.gyroSensor.get("gyro");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        color = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color");

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

        mers_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mers_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //calibreat gyro
        gyro.calibrate();
<<<<<<< HEAD
        while (gyro.isCalibrating()){
=======
        while (gyro.isCalibrating()) {
>>>>>>> postu_enache
            idle();
        }

        //senzor de culoare
        color.enableLed(true); //daca ma uit la un : obiect - true ; lumina - false
<<<<<<< HEAD

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
=======
    }

    private void rotit(double angle) {
        if (abs(angle) <= 1) {
            return;
        }
        double end = gyro.getHeading() + angle;
        while (end < 0) {
            end += 360;
        }
        double speed = 0.6;
        if (angle < 0) {
            speed = -speed;
        }
        mers_left.setPower(-speed);
        mers_right.setPower(speed);
        while (abs(gyro.getHeading() - end) > 5 && opModeIsActive()) {
            idle();
        }
        rotit_delicat(end, speed);
    }

    protected void rotit_delicat(double end, double speed) {
        if (speed > 0) {
            speed = 0.2;
        } else {
            speed = -0.2;
        }
        mers_left.setPower(-speed);
        mers_right.setPower(speed);
        while (abs(gyro.getHeading() - end) > 1 && opModeIsActive()) {
            idle();
>>>>>>> postu_enache
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
        mers_delicat(end , speed);
    }

<<<<<<< HEAD
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
         final double delay  = 2000;
         final long period = 10L; //while ce opereaza la frecventa de 10 ms

         boolean bIsUsingEncoder = false;

         ///IMPLEMENTARE GYRO
         gyro.resetZAxisIntegrator();
         double currentHeading = gyro.getHeading();

         ///TEST**************
         double iGain = 0.0; //TODO: incearca sa setezi la 0 sa vedem daca se rezolva doar cu PD control
         double pGain = 1/(target - 5); //daca zidul sau alt robot se apropie mai mult decat trebuie atunci sa mearga la viteza maxima in spate
         double dGain = 0.0;

         double errorRight = target - range_right.getDistance(DistanceUnit.CM);
         double errorLeft = target - range_left.getDistance(DistanceUnit.CM);

         double sumRight = errorRight;
         double sumLeft = errorLeft;

         double proportionalSpeedLeft = 0;
         double proportionalSpeedRight = 0;

         double derivativeSpeedLeft, finalSpeedLeft = 0, derivativeSpeedRight, finalSpeedRight = 0;

         double auxRight, auxLeft, initValueL = 0, initValueR = 0;

         float steadyTimer = 0;

         while(opModeIsActive() && steadyTimer < delay){

             //*****************************
             //conditia de timer
             if (Math.abs(errorLeft) < 2 || Math.abs(errorRight) < 2) {
                 steadyTimer += period;
             } else {
                 steadyTimer = 0;
             }


             //****************************
             //PID
             auxRight = finalSpeedRight;
             auxLeft = finalSpeedLeft;
             errorRight = target - range_right.getDistance(DistanceUnit.CM);
             errorLeft = target - range_left.getDistance(DistanceUnit.CM);
             sumRight += errorRight;
             sumLeft += errorLeft;

             proportionalSpeedRight = (errorRight * pGain);
             proportionalSpeedLeft = (errorLeft * pGain);

             //inversam viteza ca sa fie pozitiva
             finalSpeedLeft = -proportionalSpeedLeft;
             finalSpeedRight = -proportionalSpeedRight;

             finalSpeedLeft = Range.clip(finalSpeedLeft, -0.9, 0.9);
             finalSpeedRight = Range.clip(finalSpeedRight, -0.9, 0.9);

             //****************************
             //exceptii
             if(Math.abs(finalSpeedLeft) < TOLERANCE ){
                finalSpeedLeft = 0;
             }
             if(Math.abs(finalSpeedRight) < TOLERANCE ){
                 finalSpeedRight = 0;
             }

             //daca robotul trebuie sa se intoarca
             if(finalSpeedLeft > 0 && finalSpeedRight < 0){
                 finalSpeedLeft = 0;
             }
             if(finalSpeedRight > 0 && finalSpeedLeft < 0){
                 finalSpeedRight = 0;
             }

             //****************************
             //gyro
             currentHeading = gyro.getHeading();
             if(currentHeading > 180){
                 currentHeading = currentHeading - 360;
             }
             if(Math.abs(currentHeading) > 2 && !bIsUsingEncoder){
                 bIsUsingEncoder = true;
                 initValueL = mers_left.getCurrentPosition();
                 initValueR = mers_right.getCurrentPosition();
             }

             //****************************
             //retrack
             if(bIsUsingEncoder){
                 if(initValueL <= mers_left.getCurrentPosition()){
                     finalSpeedLeft = 0;
                 }
                 if(initValueR <= mers_right.getCurrentPosition()){
                     finalSpeedRight = 0;
                 }

                 if(initValueL < mers_left.getCurrentPosition() && initValueR < mers_right.getCurrentPosition()){
                     bIsUsingEncoder = false;
                     gyro.resetZAxisIntegrator();
                 }
             }

             setWheelsPower(finalSpeedLeft,finalSpeedRight);

             telemetry.addData("using encoder ", bIsUsingEncoder);

             //telemetry.addData("desired ", -proportionalSpeedRight);

             telemetry.addData("speed left", finalSpeedLeft);
             telemetry.addData("speed right", finalSpeedRight);

             telemetry.addData("error left ", errorLeft);
             telemetry.addData("error right ", errorRight);
             telemetry.addData("steady timer ", steadyTimer);
             telemetry.update();

             sleep(period);
         }

         ///TEST END*****************
         stopWheels();
     }

     //TODO: adauga functie de calculare a unghiului in functie de distanta parcursa
     protected double calculateTurnAngle(double encoderTicks){
        return Math.atan(DISTANCE_BETWEEN_SLOTS / (encoderTicks / 67));
     }


}

=======
    /*protected void mers_ultrasonic (double pasi){
        //cu pasi unde mai multe -> pasi poz se duce in spate , pasi neg in fata
        double end = range_left.rawUltrasonic() + pasi;
        double speed = 0.3;
        if (pasi > 0){
            speed = -speed;
        }
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (abs(range_left.rawUltrasonic() - end) > 5 && opModeIsActive()){
        }
        //mers_left.setPower(0);
        //mers_right.setPower(0);
        mers_delicat_ultrasonic(end , speed);
    }

    protected void mers_delicat_ultrasonic (double end , double speed){
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
    }*/

    protected void mers_encoder(int pasi) {
        if (abs(pasi) > 0) {
            mers_left.setTargetPosition(mers_left.getCurrentPosition() + pasi * const_encoder);
            mers_right.setTargetPosition(mers_right.getCurrentPosition() + pasi * const_encoder);
        }
        mers_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mers_left.setPower(0.7);
        mers_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mers_right.setPower(0.7);
        while (mers_left.isBusy() || mers_right.isBusy()) {
            idle();
        }
        mers_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mers_left.setPower(0);
        mers_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mers_right.setPower(0);
    }

    protected void mers_cul() {
        double speed = 0.2;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 0 && opModeIsActive()) {
            idle();
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
    }

    protected boolean culoare() {
        int cul = color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
        return (cul == 8 || cul == 9 || cul == 10);
    }

    private boolean etapa(double angle) {
        rotit(angle);
        int sl = mers_left.getCurrentPosition();
        int sr = mers_right.getCurrentPosition();
        mers_encoder(25);
        mers_cul();
        boolean ok = false;
        if (culoare()) {
            mers_encoder(5);
            mers_encoder(-5);
            ok = true;
        }
        mers_left.setTargetPosition(sl);
        mers_right.setTargetPosition(sr);
        mers_encoder(0);
        rotit(-angle);
        return ok;
    }

    protected void mers_culoare() {
        if (etapa(0)) {
            mers_left.setPower(0);
            mers_right.setPower(0);
            return;
        }
        sleep(300);
        if (etapa(-28)) {
            mers_left.setPower(0);
            mers_right.setPower(0);
            return;
        }
        sleep(300);
        if (etapa(28)) {
            mers_left.setPower(0);
            mers_right.setPower(0);
            return;
        }
    }

}

>>>>>>> postu_enache
