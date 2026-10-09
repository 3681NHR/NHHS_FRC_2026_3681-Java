package frc.utils.padcrafter;

import com.sun.source.tree.AnnotationTree;
import com.sun.source.tree.BinaryTree;
import com.sun.source.tree.ExpressionTree;
import com.sun.source.tree.IdentifierTree;
import com.sun.source.tree.LiteralTree;
import com.sun.source.tree.MemberSelectTree;
import com.sun.source.tree.MethodInvocationTree;
import com.sun.source.tree.NewClassTree;
import com.sun.source.tree.ParenthesizedTree;
import com.sun.source.tree.Tree;
import com.sun.source.tree.TypeCastTree;
import com.sun.source.tree.UnaryTree;
import com.sun.source.tree.VariableTree;
import com.sun.source.util.SourcePositions;
import com.sun.source.util.TreePath;
import com.sun.source.util.TreePathScanner;
import com.sun.source.util.Trees;
import java.io.IOException;
import java.io.Writer;
import java.math.BigDecimal;
import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.Filer;
import javax.annotation.processing.Messager;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedOptions;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.tools.Diagnostic;

@SupportedAnnotationTypes("*")
@SupportedOptions({
    "frc.padcrafter.url",
    "frc.padcrafter.colors",
    "frc.padcrafter.outline",
    "frc.padcrafter.platforms",
    "frc.padcrafter.timestamp"
})
@SupportedSourceVersion(SourceVersion.RELEASE_17)
public final class PadcrafterProcessor extends AbstractProcessor {
    private static final String OUTPUT_FILE = "padcrafter.url";
    private static final String DEFAULT_URL = "https://www.padcrafter.com/";
    private static final String DEFAULT_COLORS = "#242424,#606A6E,#FFFFFF";
    private static final String DEFAULT_OUTLINE = "0";
    private static final String DEFAULT_PLATFORMS = "0";

    private Trees trees;
    private Messager messager;
    private Filer filer;
    private boolean generated;
    private boolean validationFailed;

    @Override
    public synchronized void init(ProcessingEnvironment processingEnvironment) {
        super.init(processingEnvironment);
        trees = Trees.instance(processingEnvironment);
        messager = processingEnvironment.getMessager();
        filer = processingEnvironment.getFiler();
    }

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnvironment) {
        if (generated || roundEnvironment.processingOver()) {
            return false;
        }

        List<SourceVariable> variables = findBindings(roundEnvironment);
        variables.sort(Comparator.comparingLong(SourceVariable::sourcePosition));

        List<RBinding> bindings = new ArrayList<>();
        for (SourceVariable sourceVariable : variables) {
            if (sourceVariable.element() instanceof VariableElement variable) {
                collectBinding(variable, bindings);
            } else {
                messager.printMessage(
                        Diagnostic.Kind.ERROR,
                        "Could not read @Binding variable source");
                validationFailed = true;
            }
        }

        if (!validationFailed) {
            try {
                writeConfiguration(bindings);
                generated = true;
            } catch (IOException exception) {
                error(null, "Could not write Padcrafter configuration: %s", exception.getMessage());
            }
        }
        return false;
    }

    private void collectBinding(VariableElement variable, List<RBinding> bindings) {
        if (variable.getKind() != ElementKind.LOCAL_VARIABLE) {
            error(variable, "@Binding is only supported on local variables");
            return;
        }

        Binding annotation = variable.getAnnotation(Binding.class);
        String action = annotation.value().trim();
        if (action.isEmpty()) {
            error(variable, "@Binding action must not be blank");
            return;
        }
        if (action.indexOf('\n') >= 0 || action.indexOf('\r') >= 0) {
            error(variable, "@Binding action must be a single line");
            return;
        }

        Input input = resolveInput(variable, annotation.controller());
        if (input != null) {
            bindings.add(new RBinding(annotation.controller(), input, action));
        }
    }

    private Input resolveInput(
            VariableElement variable, Binding.Controller expectedController) {
        TreePath variablePath = trees.getPath(variable);
        if (variablePath == null || !(variablePath.getLeaf() instanceof VariableTree variableTree)) {
            error(variable, "Could not read @Binding variable source");
            return null;
        }

        ExpressionTree initializer = variableTree.getInitializer();
        if (initializer == null) {
            error(variable, "@Binding variable must have an initializer");
            return null;
        }
        if (!(initializer instanceof NewClassTree newClass)
                || !simpleClassName(newClass.getIdentifier()).equals("Trigger")) {
            error(variable, "@Binding must annotate a Trigger initializer");
            return null;
        }

        InputResolver resolver = new InputResolver(variable, expectedController);
        resolver.scan(new TreePath(variablePath, initializer), null);
        if (resolver.inputs.isEmpty()) {
            error(variable, "@Binding Trigger does not read a supported gamepad input");
            return null;
        }
        if (resolver.distinctInputs.size() > 1) {
            error(variable, "@Binding Trigger must read exactly one gamepad input");
            return null;
        }
        return resolver.inputs.get(0);
    }

    private List<SourceVariable> findBindings(RoundEnvironment roundEnvironment) {
        List<SourceVariable> bindings = new ArrayList<>();
        for (Element root : roundEnvironment.getRootElements()) {
            TreePath rootPath = trees.getPath(root);
            if (rootPath != null) {
                new BindingFinder(bindings).scan(rootPath, null);
            }
        }
        return bindings;
    }

    private boolean hasBinding(VariableTree variable, TreePath variablePath) {
        TreePath modifiersPath = new TreePath(variablePath, variable.getModifiers());
        for (AnnotationTree annotation : variable.getModifiers().getAnnotations()) {
            Element annotationType = trees.getElement(new TreePath(modifiersPath, annotation));
            if (annotationType instanceof TypeElement typeElement
                    && typeElement
                            .getQualifiedName()
                            .contentEquals("frc.utils.padcrafter.Binding")) {
                return true;
            }
        }
        return false;
    }

    private String simpleClassName(ExpressionTree identifier) {
        String name = identifier.toString();
        int separator = name.lastIndexOf('.');
        return separator < 0 ? name : name.substring(separator + 1);
    }

    private Integer resolveInt(TreePath path) {
        Tree tree = path.getLeaf();
        if (tree instanceof LiteralTree literal && literal.getValue() instanceof Number number) {
            return intValue(number);
        }
        if (tree instanceof IdentifierTree || tree instanceof MemberSelectTree) {
            Element element = trees.getElement(path);
            if (element instanceof VariableElement variable
                    && variable.getConstantValue() instanceof Number number) {
                return intValue(number);
            }
        }
        if (tree instanceof ParenthesizedTree parenthesized) {
            return resolveInt(new TreePath(path, parenthesized.getExpression()));
        }
        if (tree instanceof TypeCastTree cast) {
            return resolveInt(new TreePath(path, cast.getExpression()));
        }
        if (tree instanceof UnaryTree unary
                && (unary.getKind() == Tree.Kind.UNARY_PLUS
                        || unary.getKind() == Tree.Kind.UNARY_MINUS)) {
            Integer value = resolveInt(new TreePath(path, unary.getExpression()));
            if (value == null) {
                return null;
            }
            return unary.getKind() == Tree.Kind.UNARY_MINUS ? -value : value;
        }
        return null;
    }

    private Integer intValue(Number number) {
        try {
            return new BigDecimal(number.toString()).intValueExact();
        } catch (ArithmeticException | NumberFormatException exception) {
            return null;
        }
    }

    private void writeConfiguration(List<RBinding> bindings) throws IOException {
        Map<Binding.Controller, Map<Input, List<String>>> actions =
                new EnumMap<>(Binding.Controller.class);
        Set<Input> usedInputs = EnumSet.noneOf(Input.class);

        for (RBinding RBinding : bindings) {
            Map<Input, List<String>> controllerActions =
                    actions.computeIfAbsent(RBinding.controller(), ignored -> new EnumMap<>(Input.class));
            controllerActions
                    .computeIfAbsent(RBinding.input(), ignored -> new ArrayList<>())
                    .add(RBinding.action());
            usedInputs.add(RBinding.input());
        }

        List<Binding.Controller> controllers =
                new ArrayList<>(actions.keySet());
        Map<String, String> parameters = new LinkedHashMap<>();
        parameters.put("col", option("frc.padcrafter.colors", DEFAULT_COLORS));
        parameters.put("outline", option("frc.padcrafter.outline", DEFAULT_OUTLINE));
        parameters.put(
                "templates",
                controllers.stream().map(PadcrafterProcessor::controllerName).reduce((a, b) -> a + "|" + b)
                        .orElse("Driver"));
        parameters.put("plat", option("frc.padcrafter.platforms", DEFAULT_PLATFORMS));
        parameters.put(
                "timestamp",
                option("frc.padcrafter.timestamp", Long.toString(System.currentTimeMillis())));

        for (Input input : Input.values()) {
            if (!usedInputs.contains(input)) {
                continue;
            }
            String value = controllers.stream()
                    .map(controller -> actions.getOrDefault(controller, Map.of())
                            .getOrDefault(input, List.of())
                            .stream()
                            .distinct()
                            .reduce((a, b) -> a + " / " + b)
                            .orElse(""))
                    .reduce((a, b) -> a + "|" + b)
                    .orElse("");
            parameters.put(input.queryName, value);
        }

        String baseUrl = option("frc.padcrafter.url", DEFAULT_URL);
        StringBuilder url = new StringBuilder(baseUrl);

        char separator = baseUrl.contains("?") ? '&' : '?';
        for (Map.Entry<String, String> parameter : parameters.entrySet()) {
            url.append(separator)
                    .append(encode(parameter.getKey()))
                    .append('=')
                    .append(encode(parameter.getValue()));
            separator = '&';
        }
        url.append('\n');
        System.out.println(url);
        try (Writer writer = filer.createResource(
                        javax.tools.StandardLocation.SOURCE_OUTPUT, "", OUTPUT_FILE)
                .openWriter()) {
            writer.write(url.toString());
        }

    }

    private String option(String name, String defaultValue) {
        String value = processingEnv.getOptions().get(name);
        return value == null || value.isBlank() ? defaultValue : value;
    }

    private String encode(String value) {
        return URLEncoder.encode(value, StandardCharsets.UTF_8);
    }

    private static String controllerName(Binding.Controller controller) {
        String name = controller.name().toLowerCase(Locale.ROOT);
        return Character.toUpperCase(name.charAt(0)) + name.substring(1);
    }

    private void error(Element element, String message, Object... arguments) {
        validationFailed = true;
        messager.printMessage(
                Diagnostic.Kind.ERROR, String.format(message, arguments), element);
    }

    private Input methodInput(String method, Integer value, Element element) {
        if (value == null) {
            return null;
        }
        return switch (method) {
            case "getRawButton" -> switch (value) {
                case 1 -> Input.A_BUTTON;
                case 2 -> Input.B_BUTTON;
                case 3 -> Input.X_BUTTON;
                case 4 -> Input.Y_BUTTON;
                case 5 -> Input.LEFT_BUMPER;
                case 6 -> Input.RIGHT_BUMPER;
                case 7 -> Input.BACK_BUTTON;
                case 8 -> Input.START_BUTTON;
                case 9 -> Input.LEFT_STICK_BUTTON;
                case 10 -> Input.RIGHT_STICK_BUTTON;
                default -> {
                    error(element, "Raw button %d is not supported by Padcrafter", value);
                    yield null;
                }
            };
            case "getRawAxis" -> switch (value) {
                case 0, 1 -> Input.LEFT_STICK;
                case 2 -> Input.LEFT_TRIGGER;
                case 3 -> Input.RIGHT_TRIGGER;
                case 4, 5 -> Input.RIGHT_STICK;
                default -> {
                    error(element, "Raw axis %d is not supported by Padcrafter", value);
                    yield null;
                }
            };
            case "getPOV" -> switch (value) {
                case 0 -> Input.DPAD_UP;
                case 90 -> Input.DPAD_RIGHT;
                case 180 -> Input.DPAD_DOWN;
                case 270 -> Input.DPAD_LEFT;
                default -> {
                    error(element, "POV value %d is not supported by Padcrafter", value);
                    yield null;
                }
            };
            default -> null;
        };
    }

    private final class BindingFinder extends TreePathScanner<Void, Void> {
        private final List<SourceVariable> bindings;

        private BindingFinder(List<SourceVariable> bindings) {
            this.bindings = bindings;
        }

        @Override
        public Void visitVariable(VariableTree variable, Void unused) {
            TreePath variablePath = getCurrentPath();
            if (hasBinding(variable, variablePath)) {
                SourcePositions positions = trees.getSourcePositions();
                long sourcePosition = positions.getStartPosition(
                        variablePath.getCompilationUnit(), variable);
                bindings.add(new SourceVariable(
                        sourcePosition, trees.getElement(variablePath)));
            }
            return super.visitVariable(variable, unused);
        }
    }

    private final class InputResolver extends TreePathScanner<Void, Void> {
        private final VariableElement variable;
        private final Binding.Controller controller;
        private final List<Input> inputs = new ArrayList<>();
        private final Set<Input> distinctInputs = EnumSet.noneOf(Input.class);

        private InputResolver(
                VariableElement variable, Binding.Controller controller) {
            this.variable = variable;
            this.controller = controller;
        }

        @Override
        public Void visitMethodInvocation(MethodInvocationTree invocation, Void unused) {
            String method = invocation.getMethodSelect() instanceof MemberSelectTree memberSelect
                    ? memberSelect.getIdentifier().toString()
                    : invocation.getMethodSelect().toString();

            if (method.equals("getRawButton") || method.equals("getRawAxis")) {
                if (invocation.getArguments().size() != 1) {
                    error(variable, "%s must have one input argument", method);
                } else {
                    TreePath invocationPath = getCurrentPath();
                    TreePath argumentPath =
                            new TreePath(invocationPath, invocation.getArguments().get(0));
                    Integer value = resolveInt(argumentPath);
                    if (value == null) {
                        error(variable, "%s input must be an integer constant", method);
                    } else {
                        addInput(methodInput(method, value, variable));
                    }
                }
            } else if (method.equals("getPOV")) {
                TreePath invocationPath = getCurrentPath();
                TreePath parentPath = invocationPath.getParentPath();
                if (parentPath == null
                        || !(parentPath.getLeaf() instanceof BinaryTree comparison)
                        || comparison.getKind() != Tree.Kind.EQUAL_TO) {
                    error(variable, "getPOV must be compared to an integer constant");
                } else {
                    ExpressionTree direction = comparison.getLeftOperand() == invocationPath.getLeaf()
                            ? comparison.getRightOperand()
                            : comparison.getLeftOperand();
                    Integer directionValue = resolveInt(
                            new TreePath(parentPath, direction));
                    if (directionValue == null) {
                        error(variable, "getPOV comparison must use an integer constant");
                    } else {
                        addInput(methodInput(method, directionValue, variable));
                    }
                }
            }
            return super.visitMethodInvocation(invocation, unused);
        }

        private void addInput(Input input) {
            if (input == null) {
                return;
            }
            if (!distinctInputs.add(input)) {
                error(
                        variable,
                        "@Binding %s Trigger contains the same gamepad input more than once",
                        controller.name());
                return;
            }
            inputs.add(input);
        }
    }

    private record SourceVariable(long sourcePosition, Element element) {}

    private record RBinding(Binding.Controller controller, Input input, String action) {}

    private enum Input {
        A_BUTTON("aButton"),
        B_BUTTON("bButton"),
        X_BUTTON("xButton"),
        Y_BUTTON("yButton"),
        LEFT_BUMPER("leftBumper"),
        RIGHT_BUMPER("rightBumper"),
        BACK_BUTTON("backButton"),
        START_BUTTON("startButton"),
        LEFT_STICK_BUTTON("leftStickClick"),
        RIGHT_STICK_BUTTON("rightStickClick"),
        LEFT_TRIGGER("leftTrigger"),
        RIGHT_TRIGGER("rightTrigger"),
        LEFT_STICK("leftStick"),
        RIGHT_STICK("rightStick"),
        DPAD_UP("dpadUp"),
        DPAD_DOWN("dpadDown"),
        DPAD_LEFT("dpadLeft"),
        DPAD_RIGHT("dpadRight");

        private final String queryName;

        Input(String queryName) {
            this.queryName = queryName;
        }
    }
}
