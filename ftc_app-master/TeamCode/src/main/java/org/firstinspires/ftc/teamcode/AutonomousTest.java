package org.firstinspires.ftc.teamcode;

import android.os.Handler;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;

public abstract class AutonomousTest extends LinearOpMode {

    ///********************************
    //Variables
    ///********************************

    //motoare
    protected DcMotor mers_left = null;
    protected DcMotor mers_right = null;
    protected DcMotor glisare = null;
    protected DcMotor ridicare_cutie = null;
    protected DcMotor ridicare_perii = null;
    protected DcMotor rotire_perii = null;

    //senzori
    private ModernRoboticsI2cGyro gyro = null;
    private ModernRoboticsI2cRangeSensor range_right = null;
    private ModernRoboticsI2cRangeSensor range_left = null;
    private ModernRoboticsI2cColorSensor color = null;

    //constante
    int cnst = 1000;
    protected static double TOLERANCE = 0.0001;
    protected static double DISTANCE_BETWEEN_SLOTS = 30;
    final int const_sleep = 1000;
    final int const_encoder = 67;

    //misc
    boolean bHasFoundCube;

    @Override
    public void runOpMode() {
        initialise();

        runOp();

        endOp();
    }

    ///********************************
    //FUNCTII
    ///********************************

    protected abstract void runOp();
    protected abstract void endOp();

    //TODO: adaugati comentarii


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

        while (gyro.isCalibrating()) {
            idle();
        }

        //senzor de culoare
        color.enableLed(true); //daca ma uit la un : obiect - true ; lumina - false

    }


    private void setWheelsPower(double powerleft, double powerright){
        mers_right.setPower(powerright);
        mers_left.setPower(powerleft);
    }

    //nefolosita
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

    //nefolosita
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

    //incearca sa mentina constanta distanta daca ca parametru fata de orice obiect din fata
    protected void walk_with_obstacle_and_range(double distanceFromWall, double approxDistance, boolean bHasToBeAllignedWithWall){
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

             finalSpeedLeft = Range.clip(finalSpeedLeft, -0.9, 0.7);
             finalSpeedRight = Range.clip(finalSpeedRight, -0.9, 0.7);

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

    //calculeaza un unghi in functie de tangenta lui
    protected double calculateTurnAngle(double encoderTicks){
        if(encoderTicks != 0)
            return Math.atan(DISTANCE_BETWEEN_SLOTS / (encoderTicks / const_encoder));
        else
            return 0;
    }


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
        //mers_cul();
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

    //functie ce incearca fiecare element in parte si returneaza true daca s-a fasit cubul sau false daca nu s-a gasit nici un cub
    protected boolean ChooseAndPushCube(){
        //prima incercare + rotire
        rotit(calculateTurnAngle(TryObject()));

        //a doua incercare + rotire
        if(!bHasFoundCube)
            rotit(-2 * calculateTurnAngle(TryObject()));

        //ultima incercare
        if(!bHasFoundCube)
            TryObject();

        //se roteste inapoi la valoarea 0
        rotit(-gyro.getHeading());

        if(bHasFoundCube)
            return true;
        else
            return false;
    }

    //functie care merge in linie dreapta pana gaseste un obiect si il impinge daca este gablen
    protected double TryObject(){
        int initPosition = mers_left.getCurrentPosition();

        //merge pana la culoare
        mers_cul();

        //impinge
        if (culoare()) {
            mers_encoder(5);
            mers_encoder(-5);
            bHasFoundCube = true;
        }

        int distMoved = mers_left.getCurrentPosition() - initPosition;
        mers_encoder(-distMoved);

        //returneaza distanta parcursa
        if(bHasFoundCube)
            return 0;
        else
            return Math.abs(distMoved);
    }
}
