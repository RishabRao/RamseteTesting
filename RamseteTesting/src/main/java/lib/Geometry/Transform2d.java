package lib.Geometry;

/**
 * Represents a transformation for a Pose2d.
 */
public class Transform2d {
  private final Translation2d m_translation;
  private final Rotation2d m_rotation;

  /**
   * Constructs the transform that maps the initial pose to the final pose.
   *
   * @param initial The initial pose for the transformation.
   * @param last    The final pose for the transformation.
   */
  public Transform2d(Pose2d initial, Pose2d last) {
    // We are rotating the difference between the translations
    // using a clockwise rotation matrix. This transforms the global
    // delta into a local delta (relative to the initial pose).
    m_translation = last.getTranslation().minus(initial.getTranslation())
        .rotateBy(initial.getRotation().unaryMinus());

    m_rotation = last.getRotation().minus(initial.getRotation());
  }

  /**
   * Constructs a transform with the given translation and rotation components.
   *
   * @param translation Translational component of the transform.
   * @param rotation    Rotational component of the transform.
   */
  public Transform2d(Translation2d translation, Rotation2d rotation) {
    m_translation = translation;
    m_rotation = rotation;
  }

  /**
   * Constructs the identity transform -- maps an initial pose to itself.
   */
  public Transform2d() {
    m_translation = new Translation2d();
    m_rotation = new Rotation2d();
  }

  /**
   * Scales the transform by the scalar.
   *
   * @param scalar The scalar.
   * @return The scaled Transform2d.
   */
  public Transform2d times(double scalar) {
    return new Transform2d(m_translation.times(scalar), m_rotation.times(scalar));
  }

  /** 
   * Returns the translation component of the transformation.
   *
   * @return The translational component of the transform.
   */
  public Translation2d getTranslation() {
    return m_translation;
  }

  /**
   * Returns the rotational component of the transformation.
   *
   * @return Reference to the rotational component of the transform.
   */
  public Rotation2d getRotation() {
    return m_rotation;
  }
}