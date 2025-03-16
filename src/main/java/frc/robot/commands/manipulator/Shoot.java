package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class Shoot extends Command {
    private Manipulator m_manipulator;

    public Shoot(Manipulator manipulator) {
        this.m_manipulator = manipulator;
        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter initialized");
    }

    @Override
    public void execute() {
        m_manipulator.spinOuttakeMotor();
        System.out.println("Shooter Executed");
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.stopShooterMotor();
        System.out.println("Shooter Ended");
    }

    @Override
    public boolean isFinished() {
        if (m_manipulator.isCoralAtFrontSensor() != 1) {
            System.out.println("Shooter isFinished returned true");
            return true;
        } else {
            System.out.println("Shooter isFinished returned false");
            return false;
        }
    }
}
