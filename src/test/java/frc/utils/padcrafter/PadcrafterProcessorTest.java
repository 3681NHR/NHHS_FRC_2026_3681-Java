package frc.utils.padcrafter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.net.URI;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PadcrafterProcessorTest {
    @TempDir
    Path temporaryDirectory;

    @Test
    void generatesBindingsForRepeatedInputsAndControllers() throws Exception {
        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        assertNotNull(compiler);

        Path sourceDirectory = Files.createDirectories(temporaryDirectory.resolve("source"));
        Path classDirectory = Files.createDirectories(temporaryDirectory.resolve("classes"));
        Path generatedDirectory = Files.createDirectories(temporaryDirectory.resolve("generated"));
        Path sourceFile = sourceDirectory.resolve("ExampleBindings.java");
        Files.writeString(sourceFile, exampleSource(), StandardCharsets.UTF_8);

        DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<>();
        try (StandardJavaFileManager fileManager =
                compiler.getStandardFileManager(diagnostics, null, StandardCharsets.UTF_8)) {
            Iterable<? extends JavaFileObject> compilationUnits =
                    fileManager.getJavaFileObjects(sourceFile.toFile());
            String classPath = Path.of(PadcrafterProcessor.class
                            .getProtectionDomain()
                            .getCodeSource()
                            .getLocation()
                            .toURI())
                    .toString();
            var task = compiler.getTask(
                    null,
                    fileManager,
                    diagnostics,
                    java.util.List.of(
                            "-proc:only",
                            "-classpath",
                            classPath,
                            "-processorpath",
                            classPath,
                            "-processor",
                            PadcrafterProcessor.class.getName(),
                            "-d",
                            classDirectory.toString(),
                            "-s",
                            generatedDirectory.toString(),
                            "-Afrc.padcrafter.timestamp=1"),
                    null,
                    compilationUnits);
            assertNotNull(task);
            assertTrue(task.call(), () -> diagnostics.getDiagnostics().toString());
        }

        String url = Files.readString(
                generatedDirectory.resolve("padcrafter.url"), StandardCharsets.UTF_8).trim();
        Map<String, String> parameters = Arrays.stream(URI.create(url).getRawQuery().split("&"))
                .map(parameter -> parameter.split("=", 2))
                .collect(Collectors.toMap(
                        parameter -> URLDecoder.decode(parameter[0], StandardCharsets.UTF_8),
                        parameter -> URLDecoder.decode(parameter[1], StandardCharsets.UTF_8)));

        assertEquals("Driver|Operator", parameters.get("templates"));
        assertEquals("Fire / Charge|", parameters.get("aButton"));
        assertEquals("Home|", parameters.get("dpadRight"));
        assertEquals("|Raise", parameters.get("leftTrigger"));
        assertEquals("1", parameters.get("timestamp"));
    }

    private String exampleSource() {
        return """
                package example;

                import frc.utils.padcrafter.Binding;

                class ExampleBindings {
                    private final Pad driverController = new Pad();
                    private final Pad operatorController = new Pad();

                    void configureBindings() {
                        @Binding("Fire")
                        Trigger fire = new Trigger(() -> driverController.getRawButton(1));
                        @Binding("Charge")
                        Trigger charge = new Trigger(() -> driverController.getRawButton(1));
                        @Binding("Home")
                        Trigger home = new Trigger(() -> driverController.getPOV() == 90);
                        @Binding(
                                value = "Raise",
                                controller = Binding.Controller.OPERATOR)
                        Trigger raise = new Trigger(() -> operatorController.getRawAxis(2) > 0.5);
                    }

                    private static final class Trigger {
                        private Trigger(java.util.function.BooleanSupplier condition) {}
                    }

                    private static final class Pad {
                        private boolean getRawButton(int input) {
                            return false;
                        }

                        private double getRawAxis(int input) {
                            return 0.0;
                        }

                        private int getPOV() {
                            return 0;
                        }
                    }
                }
                """;
    }
}
