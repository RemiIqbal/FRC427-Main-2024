package frc.robot.subsystems.leds;
import java.util.List;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlState;
import frc.robot.subsystems.leds.patterns.LEDPattern;

import edu.wpi.first.wpilibj.Timer;


public class Led extends SubsystemBase {
    
    private static Led instance; 
    // = new Led();

    public static Led getInstance() {
        return instance; 
    }
    //Create Led, ledstrips, buffer, pattern, and timer
    AddressableLED m_Led;

    private List<LEDStrip> ledStrips;

    private LEDPattern pattern;

    private AddressableLEDBuffer buffer;

    public boolean isMovingToAmp = false; 
    public boolean isMovingToSpeaker = false; 
    public boolean isShooting = false; 
    public boolean isIntaking = false; 
    public boolean isShootingToAmp = false;
    public boolean isShootingToSpeaker = false;


    private Timer timer = new Timer();

    //If port and Length aren't given, take from Constants. Will then run next Led below
    private Led() {
        this(Constants.LEDs.kLedPort, Constants.LEDs.kLedLength); 
    }

    //If Pattern isn't given, will run with default pattern and then run next Led below
    private Led(int port, int length) {
        // default pattern
        this(port, length, Constants.LEDs.Patterns.kDefault); 
    }

    //Tells Leds what to do with given port, legnth, and pattern
    private Led(int port, int length, LEDPattern pattern) {
        //Sets Port
        this.m_Led = new AddressableLED(port);
        //Sets Length to buffer
        this.m_Led.setLength(length);
        //Actually sets length from buffer to Leds
        this.buffer = new AddressableLEDBuffer(length); 

        this.ledStrips = List.of(
            new LEDStrip(buffer, Constants.LEDs.kLed1Start, Constants.LEDs.kLed1End)
        ); 
        //Starts timer
        this.timer.start();
        //Starts Leds!
        this.m_Led.start();
        
        //Set pattern
        setPattern(pattern);


        
        
    }
    //Actual method for Setting Patterns
    public void setPattern(LEDPattern pattern) {
        //If  pattern is already current pattern, exit method
        if (this.pattern == pattern) return; 

        //Set Pattern of each strip
        for (LEDStrip strip : this.ledStrips)
            strip.setPattern(pattern);
        this.pattern = pattern; 
        //Restart timer for whatever reason?
        this.timer.restart();
    }
   

    @Override
    public void periodic() {

        LEDPattern decidedPattern = LEDPattern.kEmpty; 

        // lower priorities
        if (DriverStation.isEnabled()) decidedPattern = Constants.LEDs.Patterns.kEnabled; 
        if (DriverStation.isDisabled()) decidedPattern = Constants.LEDs.Patterns.kDisabled; 
       
        if (this.isMovingToAmp || this.isMovingToSpeaker) decidedPattern = Constants.LEDs.Patterns.kMoving;
        if (this.isShooting) decidedPattern = Constants.LEDs.Patterns.kShootAnywhere;
        if (this.isIntaking) decidedPattern = Constants.LEDs.Patterns.kIntake;
        if (Arm.getInstance().getArmControlState() == ArmControlState.TRAVEL) decidedPattern = Constants.LEDs.Patterns.kArmMoving;
        if (Arm.getInstance().getArmControlState() == ArmControlState.AMP) decidedPattern = Constants.LEDs.Patterns.kArmAtAmp;
        if (Arm.getInstance().getArmControlState() == ArmControlState.SPEAKER) decidedPattern = Constants.LEDs.Patterns.kArmAtSpeaker;
        if (Arm.getInstance().getArmControlState() == ArmControlState.GROUND) decidedPattern = Constants.LEDs.Patterns.kArmAtGround;
        if (this.isShootingToAmp) decidedPattern = Constants.LEDs.Patterns.kShootAmp;
        if  (this.isShootingToSpeaker) decidedPattern = Constants.LEDs.Patterns.kShootSpeaker;

        setPattern(decidedPattern);

        //Constantly updates leds with respect to time
        for (int i = 0; i < ledStrips.size(); i++) {
            ledStrips.get(i).update(timer.get());
        }
        //Sets LED output data
        m_Led.setData(buffer);
    }
    //Method to return how many Led Strips there are
    public List<LEDStrip> getLedStrips() {
        return this.ledStrips; 
    }
    //Method to getLED
    public AddressableLED getLED() {
        return this.m_Led;
    }
}
