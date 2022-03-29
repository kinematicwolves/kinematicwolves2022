package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import edu.wpi.first.util.math.MathUtil;
import java.lang.Math;

public class LinearActuator extends SubsystemBase {

public static Servo angleActuator_1 = new Servo(Constants.LINEAR_ACTUATOR_1); // PWM controlled

public void setLinearActuatorPosition(double m_linearActuator){
    // Check to make sure position is not out of bounds
    m_linearActuator = clipLinearActuatorPositionCommand(m_linearActuator);
    angleActuator_1.set(m_linearActuator);
  }

  
private double clipLinearActuatorPositionCommand(double position){
    // Make sure command does not exceed the hardware limit
    if (position > Constants.UPPER_SERVO_POS_LIMIT){
      position = Constants.UPPER_SERVO_POS_LIMIT;
    }

    else if (position < Constants.LOWER_SERVO_POS_LIMIT){
      position = Constants.LOWER_SERVO_POS_LIMIT;
    }
    
    return position;
  }

  public boolean servo_at_position(double endPosition){
    double actuator_1_position = angleActuator_1.getPosition();

    double actuator_1_position_delta = Math.abs(actuator_1_position - endPosition);

    return ((actuator_1_position_delta) < 0.04);
  }


public void setLinearActuatorPosition(LinearActuator m_linearActuator) {
}


}