package lib.Kinematics;

/**
 * Helper class that converts a chassis velocity (dx and dtheta components) to
 * left and right wheel velocities for a differential drive.
 *
 * <p>Inverse kinematics converts a desired chassis speed into left and right
 * velocity components whereas forward kinematics converts left and right
 * component velocities into a linear and angular chassis speed.
 */
public class DifferentialDriveKinematics {
  private final double m_driveRadius;

  /**
   * Constructs a differential drive kinematics object.
   *
   * @param driveRadius The radius of the drivetrain. Theoretically, this is
   *                    half the distance between the left wheels and right wheels. However, the
   *                    empirical value may be larger than the physical measured value due to
   *                    scrubbing effects.
   */
  public DifferentialDriveKinematics(double driveRadius) {
    m_driveRadius = driveRadius;
  }

  /**
   * Returns a chassis speed from left and right component velocities using
   * forward kinematics.
   *
   * @param wheelSpeeds The left and right velocities.
   * @return The chassis speed.
   */
  public ChassisSpeeds toChassisSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    return new ChassisSpeeds(
        (wheelSpeeds.left + wheelSpeeds.right) / 2, 0,
        (wheelSpeeds.right - wheelSpeeds.left) / (2 * m_driveRadius)
    );
  }

  /**
   * Returns left and right component velocities from a chassis speed using
   * inverse kinematics.
   *
   * @param chassisSpeeds The linear and angular (dx and dtheta) components that
   *                      represent the chassis' speed.
   * @return The left and right velocities.
   */
  public DifferentialDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    return new DifferentialDriveWheelSpeeds(
        chassisSpeeds.vx - m_driveRadius * chassisSpeeds.omega,
        chassisSpeeds.vx + m_driveRadius * chassisSpeeds.omega
    );
  }
}
