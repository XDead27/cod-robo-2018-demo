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

public abstract class AutonomousMode extends LinearOpMode {

    //motoare
    protected DcMotor mers_left = null;
    protected DcMotor mers_right = null;
    protected DcMotor glisare = null;
    protected DcMotor ridicare_cutie = null;
    protected DcMotor ridicare_perii = null;
    protected DcMotor rotire_perii = null;

    //senzori
    protected ModernRoboticsI2cGyro gyro = null;
    //protected ModernRoboticsI2cRangeSensor range_right = null;
    protected ModernRoboticsI2cRangeSensor range_left = null;
    protected ModernRoboticsI2cColorSensor color = null;

    protected final int const_sleep = 1000;
    protected final int const_encoder = 67;

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
        //range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        color = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color");

        //setare puteri la 0
        mers_left.setPower(0);
        mers_right.setPower(0);
        glisare.setPower(0);
        ridicare_cutie.setPower(0);
        ridicare_perii.setPower(0);
        rotire_perii.setPower(0);

        //setare directii
        mers_left.setDirection(DcMotorSimple.Direction.FORWARD); //TODO ?
        mers_right.setDirection(DcMotorSimple.Direction.FORWARD); //TODO ?
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

    protected void rotit(double angle) {
        if (abs(angle) <= 1) {
            return;
        }
        double end = gyro.getHeading() + angle;
        while (end < 0) {
            end += 360;
        }
        while (end > 360) {
            end -= 360;
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
    }

    protected void mers_perete (){
        double speed = 0.6;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (range_left.getDistance(DistanceUnit.CM) > 7 /*&& range_right.getDistance(DistanceUnit.CM) > 7*/ && opModeIsActive()){
            idle();
        }
        mers_delicat_perete();
    }

    protected void mers_delicat_perete (){
        double speed = 0.2;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (range_left.getDistance(DistanceUnit.CM) > 1 /*&& range_right.getDistance(DistanceUnit.CM) > 1*/ && opModeIsActive()){
            idle();
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
    }

    protected void mers_encoder(int pasi , double speed) {
        if (abs(pasi) > 0) {
            mers_left.setTargetPosition(mers_left.getCurrentPosition() + pasi * const_encoder);
            mers_right.setTargetPosition(mers_right.getCurrentPosition() + pasi * const_encoder);
        }
        mers_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mers_left.setPower(speed);
        mers_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mers_right.setPower(speed);
        while (mers_left.isBusy() || mers_right.isBusy() && opModeIsActive()) {
            idle();
        }
        mers_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mers_left.setPower(0);
        mers_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mers_right.setPower(0);
    }

    protected void mers_pana_la_cul() {
        double speed = 0.2;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        int sl = mers_left.getCurrentPosition();
        while (color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 0 && opModeIsActive() && mers_left.getCurrentPosition() - sl < 15 * const_encoder) {
            idle();
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
    }

    protected boolean culoare() {
        int cul = color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
        return (cul >= 8 &&  cul <= 10);
    }

    protected boolean etapa(double angle , int tip) {
        int semn = -1;
        if (angle < 0){
            semn = 1;
        }
        else if (angle == 0){
            semn = 0;
        }
        rotit(angle);
        int sl = mers_left.getCurrentPosition();
        int sr = mers_right.getCurrentPosition();
        if (angle != 0){
            mers_encoder(30 , 0.7);
        }
        else{
            mers_encoder(19 , 0.7);
        }
        mers_pana_la_cul();
        boolean ok = false;
        if (culoare()) {
            if (tip == 1){
                //mers_crater(1);
                mers_encoder(100 , 0.7);
                return true;
            }
            if (tip == 2){
                mers_encoder(10 , 0.7);
                sleep(500);
                rotit(-angle + 15*semn);
                sleep(500);
                mers_encoder(20 , 0.7);
                lasat_mascota();
                return true;
            }
            mers_encoder(5 , 0.7);
            mers_encoder(-5 , 0.7);
            ok = true;
        }
        mers_left.setTargetPosition(sl);
        mers_right.setTargetPosition(sr);
        mers_encoder(0 , 0.7);
        rotit(-angle);
        return ok;
    }

    protected void gasit_cub(int tip) {
        //TODO unghi cu trigo, nu hardcodat, cu functia lui dani
        if (etapa(0 , tip)) {
            return;
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
        if (etapa(-30 , tip)) {
            return;
        }
        mers_left.setPower(0);
        mers_right.setPower(0);
        if (etapa(37 , tip)) {
            return;
        }
        if (tip == 2){
            mers_encoder(40 , 0.7);
            lasat_mascota();
        }
        if (tip == 1){
            mers_encoder(100 , 0.7);
        }
    }

    /*protected void mers_crater(double dir){
        double speed = 0.5 * dir;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (opModeIsActive()){
            idle();
        }
    }*/

    protected void optimizare(){
        //rotit -60
        //mers pana la perete
        //rotit -70
        //mers cu spatele pana pe crater

        rotit(-60);
        mers_perete();
        rotit(-70);
        mers_encoder(-60 , 0.7);
        //mers_crater(-1);
    }

    /*protected void ridicare_cutie_encoder(int pasi , double speed) {
        ridicare_cutie.setTargetPosition(ridicare_cutie.getCurrentPosition() + pasi * const_encoder);
        ridicare_cutie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ridicare_cutie.setPower(speed);
        while (ridicare_cutie.isBusy()  && opModeIsActive()) {
            idle();
        }
        ridicare_cutie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ridicare_cutie.setPower(0);
    }*/

    protected void ridicare_perii_timp(int timp , double speed) {
        ridicare_perii.setPower(speed);
        sleep(timp * const_sleep);
        ridicare_perii.setPower(0);
    }

    protected void rotire_perii_timp(int timp , double speed) {
        rotire_perii.setPower(speed);
        sleep(timp * const_sleep);
        rotire_perii.setPower(0);
    }

    protected void lasat_mascota(){
        ridicare_perii_timp(1 , -0.2);
        rotire_perii_timp(2 , 0.6);
        ridicare_perii_timp(1 , 0.5);
    }

}

