package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Patrat_Aliat", group = "Autonomous")

public abstract class AutonomousPatratAliat extends AutonomousMode {

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        lasat_mascota();
    }

}
