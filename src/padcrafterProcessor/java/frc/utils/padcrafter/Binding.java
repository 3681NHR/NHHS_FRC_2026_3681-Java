package frc.utils.padcrafter;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.SOURCE)
@Target(ElementType.LOCAL_VARIABLE)
public @interface Binding {
    String value();

    Controller controller() default Controller.DRIVER;

    enum Controller {
        DRIVER,
        OPERATOR
    }
}
