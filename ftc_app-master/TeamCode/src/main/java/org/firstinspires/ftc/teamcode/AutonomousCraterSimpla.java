package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Crater_Simpla", group = "Autonomous")

public final class AutonomousCraterSimpla extends AutonomousMode {

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        gasit_cub(1);
    }

}
