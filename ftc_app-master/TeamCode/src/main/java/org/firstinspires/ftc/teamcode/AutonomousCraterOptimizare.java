package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Crater_Optimizare", group = "Autonomous")

public final class AutonomousCraterOptimizare extends AutonomousMode {

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        gasit_cub(false);
        optimizare();
    }

}
