package de.hbrs.task_planning.online_jshop2;

import java.io.File;
import java.io.FileInputStream;
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

	public static void main(String[] args) throws Exception {
		// -- The number of solution plans to be returned.
		int planNo = -1;

		//-- Handle the number of solution plans the user wants to be returned.
		if (args.length == 3 || args[0].substring(0, 2).equals("-r")) {
			if (args[0].equals("-r")) {
				planNo = 1;
			} else if (args[0].equals("-ra")) {
				planNo = Integer.MAX_VALUE;
			} else {
				try {
					planNo = Integer.parseInt(args[0].substring(2));
				} catch (NumberFormatException e) {
				}
			}
		}

		//-- Check the number of arguments.
		if (args.length != 3) {
			System.err.println("usage: java OnlineJSHOP2 " +
					"(-r|-ra|-rSomePositiveInteger) domain problem");
			System.exit(1);
		}


		// Create the domain description Java code
		String domainFile = args[1];
		Domain domain = new Domain(new FileInputStream(new File(domainFile)));
		String domainCode = domain.generate();

		// Create the problem description Java code
		String problemFile = args[2];
		Problem problem = new Problem(
				new FileInputStream(new File(problemFile)), planNo,
				domain.getDomainDescription());
		String problemCode = problem.generate();

		HashMap<String, String> nameToCode = new HashMap<String, String>();
		nameToCode.put(domain.getName(), domainCode);
		nameToCode.put(problem.getName(), problemCode);

		// Create the online JSHOP planner
		OnlineJSHOP2 shop = new OnlineJSHOP2(nameToCode);
		List<Plan> plans = shop.getPlans();

		for (Plan plan : plans) {
			System.out.println(plan.toString());
		}
	}

}
