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
    protected ModernRoboticsI2cRangeSensor range_right = null;
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

    protected void mers_pana_la_cul() {
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

    protected boolean etapa(double angle , boolean simplu) {
        rotit(angle);
        int sl = mers_left.getCurrentPosition();
        int sr = mers_right.getCurrentPosition();
        if (angle != 0){
            mers_encoder(30);
        }
        else{
            mers_encoder(25);
        }
        mers_pana_la_cul();
        boolean ok = false;
        if (culoare()) {
            if (simplu){
                mers_crater(1);
                return true;
            }
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

    protected void gasit_cub(boolean simplu) {
        if (etapa(0 , simplu)) {
            mers_left.setPower(0);
            mers_right.setPower(0);
            return;
        }
        //sleep(300);
        if (etapa(-28 , simplu)) {
            mers_left.setPower(0);
            mers_right.setPower(0);
            return;
        }
        //sleep(300);
        if (etapa(28 , simplu)) {
            mers_left.setPower(0);
            mers_right.setPower(0);
            return;
        }
    }

    protected void mers_crater(double dir){
        double speed = 0.5 * dir;
        mers_left.setPower(speed);
        mers_right.setPower(speed);
        while (opModeIsActive()){
            idle();
        }
    }

    protected void optimizare(){
        //rotit -90
        //mers pana la perete
        //rotit -45
        //mers cu spatele pana pe crater

        rotit(-90);
        mers_perete();
        rotit(-45);
        mers_crater(-1);
    }

    protected void ridicare_perii_encoder(int pasi) {
        ridicare_perii.setTargetPosition(ridicare_perii.getCurrentPosition() + pasi * const_encoder);
        ridicare_perii.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ridicare_perii.setPower(0.3);
        while (ridicare_perii.isBusy()) {
            idle();
        }
        ridicare_perii.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ridicare_perii.setPower(0);
    }

    protected void rotire_perii_encoder(int pasi) {
        rotire_perii.setTargetPosition(rotire_perii.getCurrentPosition() + pasi * const_encoder);
        rotire_perii.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotire_perii.setPower(0.3);
        while (rotire_perii.isBusy()) {
            idle();
        }
        rotire_perii.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotire_perii.setPower(0);
    }


    protected void lasat_mascota(){
        mers_encoder(30);
        ridicare_perii_encoder(10);
        rotire_perii_encoder(10);
        ridicare_perii_encoder(-10);
    }

}

