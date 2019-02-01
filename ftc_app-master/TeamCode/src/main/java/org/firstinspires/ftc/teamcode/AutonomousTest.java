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
    protected ModernRoboticsI2cGyro gyro = null;
    protected ModernRoboticsI2cRangeSensor range_right = null;
    protected ModernRoboticsI2cRangeSensor range_left = null;
    protected ModernRoboticsI2cColorSensor color = null;

    //constante
    protected static double TOLERANCE = 0.0001;
    protected static double DISTANCE_BETWEEN_SLOTS = 30;
    protected final int const_sleep = 1000;
    protected final int const_encoder = 67;

    //misc
    protected boolean bHasFoundCube;
    protected boolean mers_mult = false;

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

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


    protected void setWheelsPower(double powerleft, double powerright){
        mers_right.setPower(powerright);
        mers_left.setPower(powerleft);
    }

    //nefolosita
    private void setWheelsPowerWithGyro(double powerleft, double powerright){

        // reseteaza axa z a gyro-ului
        gyro.resetZAxisIntegrator();

        // PID stuff
        double pGain = 0.003;
        double iGain = 0.002;
        double dGain = 0.001;

        int error = gyro.getHeading();
        int errorSum = 0;
        int errorLast = 0;
        double speed = 0;
        double derivata = 0;

        if (error > 180)
        {
            error -= 360;
            error = Math.abs(error);
        }

        while (error > 3) // TODO determina unghiul maxim permis
        {
            error = gyro.getHeading();

            if (error > 180)
            {
                error -= 360;
                error = Math.abs(error);
            }

            errorSum += error;
            derivata = errorLast - error;

            speed = pGain * error + iGain * errorSum + dGain * derivata;

            errorLast = error;

            if (gyro.getHeading() > 180)
            {
                setWheelsPower(powerleft + speed, powerright);
            }
            if (gyro.getHeading() < 180)
            {
                setWheelsPower(powerleft, powerright + speed);
            }
        }

    }


    protected void stopWheels(){
        setWheelsPower(0, 0);
    }

    protected void rotit(double angle) {
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
            telemetry.addData("gyro ", gyro.getHeading());
            telemetry.update();
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
            telemetry.addData("gyro ", gyro.getHeading());
            //telemetry.update();
            idle();
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
        //mers_delicat(end , speed);
    }

    //incearca sa mentina constanta distanta daca ca parametru fata de orice obiect din fata
    protected void walk_with_obstacle_and_range(double distanceFromWall, boolean bHasToBeAllignedWithWall){
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

    protected void mers_perete (){
        double speed = 0.6;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (range_left.getDistance(DistanceUnit.CM) > 7 && range_right.getDistance(DistanceUnit.CM) > 7 && opModeIsActive()){
            idle();
        }
        mers_delicat_perete();
    }

    protected void mers_delicat_perete (){
        double speed = 0.2;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (range_left.getDistance(DistanceUnit.CM) > 1 && range_right.getDistance(DistanceUnit.CM) > 1 && opModeIsActive()){
            idle();
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
    }

    protected void walk_with_single_PID(double distanceFromWall){
        final double target = distanceFromWall;
        final double delay  = 1000;
        final long period = 10L; //while ce opereaza la frecventa de 10 ms

        ///TEST**************
        double iGain = 0.0; //TODO: incearca sa setezi la 0 sa vedem daca se rezolva doar cu PD control
        double pGain = 1/(target - 5); //daca zidul sau alt robot se apropie mai mult decat trebuie atunci sa mearga la viteza maxima in spate
        double dGain = 0.0;

        double ShortestDistanceToWall = Math.min(range_left.getDistance(DistanceUnit.CM), range_right.getDistance(DistanceUnit.CM));

        double error = target - ShortestDistanceToWall;

        double  finalSpeed = 0;

        float steadyTimer = 0;

        while(opModeIsActive() && steadyTimer < delay){

            //*****************************
            //conditia de timer
            if (Math.abs(error) < 2) {
                steadyTimer += period;
            } else {
                steadyTimer = 0;
            }


            //****************************
            //PID

            ShortestDistanceToWall = Math.min(range_left.getDistance(DistanceUnit.CM), range_right.getDistance(DistanceUnit.CM));
            error = target - ShortestDistanceToWall;

            finalSpeed = error * pGain;

            //inversam viteza ca sa fie pozitiva
            finalSpeed = -finalSpeed;

            finalSpeed = Range.clip(finalSpeed, -0.9, 0.7);

            //****************************
            //exceptii
            if(Math.abs(finalSpeed) < TOLERANCE ){
                finalSpeed = 0;
            }

            setWheelsPower(finalSpeed, finalSpeed);

            sleep(period);
        }

        ///TEST END*****************
        stopWheels();
    }

    //calculeaza un unghi in functie de tangenta lui
    protected double calculateTurnAngle(double encoderTicks){

        if(encoderTicks != 0)
            return (360 * Math.atan(DISTANCE_BETWEEN_SLOTS / ((encoderTicks / const_encoder) + 23)) )/ (2 * Math.PI);
        else
            return 0;
    }


    protected void mers_encoder(int pasi, double speed) {
        speed = Math.abs(speed);

        if (abs(pasi) > 0) {
            mers_left.setTargetPosition(mers_left.getCurrentPosition() + pasi * const_encoder);
            mers_right.setTargetPosition(mers_right.getCurrentPosition() + pasi * const_encoder);
        }
        mers_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mers_left.setPower(speed);
        mers_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mers_right.setPower(speed);
        while ((mers_left.isBusy() || mers_right.isBusy()) && opModeIsActive()) {
            idle();
        }
        mers_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mers_left.setPower(0);
        mers_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mers_right.setPower(0);
    }


    protected void mers_cul() {
        if (mers_mult){
            mers_encoder(32 , 0.8);
        }
        else{
            mers_encoder(25 , 0.8);
        }
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


    /*private boolean etapa(double angle) {
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
    }*/

    //functie ce incearca fiecare element in parte si returneaza true daca s-a fasit cubul sau false daca nu s-a gasit nici un cub
    protected int ChooseAndPushCube(boolean returnToInitialPosition){

        //prima incercare + calcul unghi
        //am mai adaugat o variabila ca sa putem hardcoda daca nu merge trigo
        double unghiCuTrigo = calculateTurnAngle(TryObject(returnToInitialPosition));
        double unghiDeRotit = unghiCuTrigo;

        mers_mult = true;

        //a doua incercare + rotire
        if(!bHasFoundCube) {
            rotit(unghiDeRotit);
            TryObject(returnToInitialPosition);
            rotit(-2 * unghiDeRotit);
        }else {
            return 1;
        }

        //ultima incercare
        if(!bHasFoundCube) {
            TryObject(returnToInitialPosition);
        }else {
            if(returnToInitialPosition)
                rotit(-unghiDeRotit);
            return 2;
        }

        if(returnToInitialPosition)
            rotit(unghiDeRotit);

        return 3;
    }

    //functie care merge in linie dreapta pana gaseste un obiect si il impinge daca este galben
    protected double TryObject(boolean returnToInitialPosition){
        int initPosition = mers_left.getCurrentPosition();

        //merge pana la culoare

        mers_cul();

        //impinge
        if (culoare()) {
            mers_encoder(7, 0.7);
            mers_encoder(-7, 0.7);
            bHasFoundCube = true;
        }

        int distMoved = mers_left.getCurrentPosition() - initPosition;

        if(!bHasFoundCube || returnToInitialPosition)
            mers_encoder(-(distMoved/const_encoder), 0.7);

        //returneaza distanta parcursa
        if(bHasFoundCube)
            return 0;
        else
            return Math.abs(distMoved);
    }

    protected void ThrowTotem(){
        ridicare_perii_encoder(-10 , 0.2);

        rotire_perii.setPower(0.5);
        sleep(700);
        rotire_perii.setPower(0);

        ridicare_perii_encoder(10 , 0.5);
    }

    protected void optimizare(){
        //rotit -60
        //mers pana la perete
        //rotit -70
        //mers cu spatele pana pe crater

        rotit(-60);
        walk_with_single_PID(1);
        //mers_perete();
        rotit(-70);
        mers_encoder(-60 , 0.7);
        //mers_crater(-1);
    }

    protected void ridicare_perii_encoder(int pasi , double speed) {
        ridicare_perii.setTargetPosition(ridicare_cutie.getCurrentPosition() + pasi * const_encoder);
        ridicare_perii.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ridicare_perii.setPower(speed);
        while (ridicare_perii.isBusy()  && opModeIsActive()) {
            idle();
        }
        ridicare_perii.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ridicare_perii.setPower(0);
    }

}
