package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.abs;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")

public abstract class AutonomousTest extends AutonomousMode {

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        boolean test_senz = false;
        boolean test_rotit = false;
        boolean test_mers_encoder = false;
        boolean test_gasit_cub = false;
        boolean test_optimizare = false;

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
            mers_encoder(50 * const_encoder , 0.7);
            sleep(3 * const_sleep);

            mers_encoder(-100 * const_encoder ,0.7);
            sleep(3 * const_sleep);

            mers_encoder(30 * const_encoder , 0.7);
            sleep(3 * const_sleep);

            mers_encoder(-70 * const_encoder , 0.7);
        }

        if (test_gasit_cub) {
            gasit_cub(0);
        }

        if (test_optimizare){
            optimizare();
        }
    }
}

