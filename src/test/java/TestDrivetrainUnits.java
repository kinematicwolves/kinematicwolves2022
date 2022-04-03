import static org.junit.Assert.*;

import frc.robot.subsystems.DifferentialDrivetrain;
import org.junit.*;

public class TestDrivetrainUnits {
    DifferentialDrivetrain drivetrain;
    @Before // this method will run before each test
  public void setup() {
   drivetrain = new DifferentialDrivetrain();
  }

  @Test // marks this method as a test
  public void testCountConversionHighGear() {
    assertEquals(0.933, drivetrain.countsToDistanceDrivenInches(2048), 0.1);
    ;
  }
}
