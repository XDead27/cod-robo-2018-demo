package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    protected ModernRoboticsI2cGyro gyro = null;
    protected ModernRoboticsI2cRangeSensor range_right = null;
    protected ModernRoboticsI2cRangeSensor range_left = null;
    protected ModernRoboticsI2cColorSensor color = null;

    final int const_sleep = 1000;
    final int const_encoder = 67;

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

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

            mers_encoder(-70 * const_encoder);
        }

        if (test_mers_culoare) {
            mers_culoare();
        }
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
        while (gyro.isCalibrating()) {
            idle();
        }

        //senzor de culoare
        color.enableLed(true); //daca ma uit la un : obiect - true ; lumina - false
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
    }

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

