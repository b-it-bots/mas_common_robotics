package mcr_online_jshop2;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.LinkedList;
import java.util.Vector;

import JSHOP2.InternalDomain;
import JSHOP2.Predicate;
import JSHOP2.TaskList;

public class Problem extends InternalDomain {

	/**
	 * Stores the names of the constant symbols, the compound tasks and the
	 * primitive tasks in the domain description. This data will be used when
	 * compiling planning problems in this domain.
	 */
	protected byte[] domainDescription;

	/**
	 * The Java code that implements the problem.
	 */
	protected String problemCode;

	public Problem(InputStream sin, int planNoIn, byte[] description)
			throws IOException {
		super(sin, planNoIn);
		domainDescription = description;
	}

	@Override
	public void commandToCode(LinkedList<Vector<Predicate>> states,
			LinkedList<TaskList> taskLists) throws IOException {
		problemCode = generateProblemCode(states, taskLists);
	}

	@Override
	public void commandInitialize() throws IOException {
		// -- To read the text file that stores the names of the constant
		// symbols
		// -- that appeared in the domain description.
		BufferedReader src;

		// -- Open the file.
		ByteArrayInputStream stream = new ByteArrayInputStream(
				domainDescription);
		src = new BufferedReader(new InputStreamReader(stream));

		// -- Read in the constant symbols.
		constantsSize = readStringArray(src, constants);

		// -- Read in the compound task names.
		readStringArray(src, compoundTasks);

		// -- Read in the primitive task names.
		readStringArray(src, primitiveTasks);

		// -- Close the file.
		src.close();
	}

	public String generate() throws Exception {
		parser.command();

		return problemCode;
	}

	public String getName() {
		return probName;
	}

}
