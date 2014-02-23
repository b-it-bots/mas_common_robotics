package mcr_online_jshop2;

import java.io.File;
import java.io.FileInputStream;
import java.util.HashMap;
import java.util.List;

import JSHOP2.Plan;

public class Main {

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