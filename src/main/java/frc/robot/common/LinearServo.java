package frc.robot.common;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class LinearServo extends Servo{
    
    private final double m_speed;
    private final double m_length;
    private double setPos;
    private double curPos;
    private double lastTime = Timer.getFPGATimestamp();
    
    /**
    * Parameters for L16-R Actuonix Linear Actuators
    *
    * @param channel PWM channel used to control the servo
    * @param length max length of the servo [mm]
    * @param speed max speed of the servo [mm/second]
    */
    public LinearServo(int channel, double length, double speed) {
        super(channel);
        setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        m_length = length;
        m_speed = speed;
    }
    
    /**
    * Run this method in any periodic function to update the position estimation of your servo
    *
    * @param setpoint the target position of the servo [mm]
    */
    public void setPosition(double setpoint){
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed((setPos/m_length *2)-1);
    }
    
    /**
    * Run this method in any periodic function to update the position estimation of your servo
    */
    public void updateCurPos(){
        double current = Timer.getFPGATimestamp();
        double dt = current - lastTime;
        lastTime = current;

        if (curPos > setPos + m_speed*dt){
            curPos -= m_speed*dt;
        } else if(curPos < setPos - m_speed*dt){
            curPos += m_speed*dt;
        }else{
            curPos = setPos;
        }
    }
    /**
    * Current position of the servo, must be calling
    * {@link #updateCurPos() updateCurPos()} periodically
    * @return Servo Position [mm]
    */
    public double getPosition(){
        return curPos;
    }
}