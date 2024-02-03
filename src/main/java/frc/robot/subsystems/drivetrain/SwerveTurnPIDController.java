package frc.robot.subsystems.drivetrain;


import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

// PID Controller for a single swerve pod turn module
public class SwerveTurnPIDController extends PIDController {
    // encoder for the swerve module
    private CANcoder encoder;
    
    // max & min motor speeds
    private double maxOutput = 1; 
    private double minOutput = -1; 

    public SwerveTurnPIDController(CANcoder encoder, double p, double i, double d) {
        super(p, i, d);
        this.encoder = encoder; 

        // allow the motor to rotate past 1 rotation and not break
        enableContinuousInput(-180, 180);
    }

    private Rotation2d getRotation() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble()); 
    }

    // calculates the PID output and clamps it
    public double calculate() {
        return MathUtil.clamp(calculate(getRotation().getDegrees()), minOutput, maxOutput); 
    }

    // returns max PID output
    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput; 
    }

    // returns min PID output
    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput; 
    }

    
}
