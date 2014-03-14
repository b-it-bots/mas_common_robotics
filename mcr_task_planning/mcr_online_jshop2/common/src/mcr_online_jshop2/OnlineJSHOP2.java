package mcr_online_jshop2;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaCompiler.CompilationTask;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.ToolProvider;

import JSHOP2.Plan;

public class OnlineJSHOP2 {

	protected JavaFileManager fileManager;
	protected String domainName;
	protected String problemName;

	public OnlineJSHOP2(HashMap<String, String> nameToCode) {
		fileManager = compile(nameToCode);
	}

	public JavaFileManager compile(HashMap<String, String> files) {
		// create a list of virtual files, where each "file" contains source
		// code
		List<JavaFileObject> javaFileObjects = new ArrayList<JavaFileObject>();
		for (Entry<String, String> entry : files.entrySet()) {
			javaFileObjects.add(new InMemoryJavaFileObject(entry.getKey(),
					entry.getValue()));
		}

		// Determine the system's compiler
		JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
		// Diagnostics include information about errors when the compilation
		// failed
		DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<JavaFileObject>();
		// A virtual file system that contains the compiled files
		JavaFileManager fileManager = new ClassFileManager(
				compiler.getStandardFileManager(null, null, null));

		// Create the compilation task and run it
		CompilationTask task = compiler.getTask(null, fileManager, diagnostics,
				null, null, javaFileObjects);
		boolean success = task.call();

		// Print compilation information when an error occurred
		for (Diagnostic<? extends JavaFileObject> diagnostic : diagnostics
				.getDiagnostics()) {
			System.out.println(diagnostic.getCode());
			System.out.println(diagnostic.getKind());
			System.out.println(diagnostic.getPosition());
			System.out.println(diagnostic.getStartPosition());
			System.out.println(diagnostic.getEndPosition());
			System.out.println(diagnostic.getSource());
			System.out.println(diagnostic.getMessage(null));
		}

		if (success) {
			return fileManager;
		} else {
			return null;
		}
	}

	public List<Plan> getPlans() {
		List<Plan> plans = null;

		if (fileManager != null) {
			try {
				Class<?> problemClass = fileManager.getClassLoader(null)
						.loadClass("problem");
				Method getPlans = problemClass.getDeclaredMethod("getPlans");
				plans = (List<Plan>) getPlans.invoke(null, (Object[]) null);
			} catch (ClassNotFoundException e) {
				System.err.println("Class not found: " + e);
			} catch (NoSuchMethodException e) {
				System.err.println("No such method: " + e);
			} catch (IllegalAccessException e) {
				System.err.println("Illegal access: " + e);
			} catch (InvocationTargetException e) {
				System.err.println("Invocation target: " + e);
			}
		}

		return plans;
	}

}
