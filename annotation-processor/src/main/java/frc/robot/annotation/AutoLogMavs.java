package frc.robot.annotation;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface AutoLogMavs {
  /**
   * The key to use when logging the field or method. Use {...} to reference constant fields for
   * disambiguation.
   *
   * @return The value of the key parameter.
   */
  public String key() default "";

  public String overrideKey() default "";

  /**
   * Whether or not to force the Logger to use a serialized data method.
   *
   * @return Whether or not to force the Logger to use a serialized data method.
   */
  public boolean forceSerializable() default false;

  /**
   * The unit to save as metadata, used when visualizing the field in AdvantageScope.
   *
   * @return The value of the unit parameter.
   */
  public String unit() default "";
}
