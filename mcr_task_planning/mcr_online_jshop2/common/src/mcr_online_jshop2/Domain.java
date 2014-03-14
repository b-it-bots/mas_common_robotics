package mcr_online_jshop2;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;

import JSHOP2.InternalDomain;

public class Domain extends InternalDomain {

	/**
	 * The Java code that implements the domain-specific planner.
	 */
	protected String domainCode;

	/**
	 * Stores the names of the constant symbols, the compound tasks and the
	 * primitive tasks in the domain description. This data will be used when
	 * compiling planning problems in this domain.
	 */
	private byte[] domainDescription;

	public Domain(InputStream inputStream) throws IOException {
		super(inputStream, -1);
	}

	@Override
	public void close(int varsMaxSize) throws IOException {
		domainCode = generateDomainCode(varsMaxSize);

		ByteArrayOutputStream stream = new ByteArrayOutputStream();

		// -- Store the String names of the constant symbols, the compound tasks
		// and
		// -- the primitive tasks in the domain description. This data will be
		// used
		// -- when compiling planning problems in this domain.
		BufferedWriter dest = new BufferedWriter(new OutputStreamWriter(stream));

		// -- Store the constant symbols.
		dumpStringArray(dest, constants);

		// -- Store the compound tasks.
		dumpStringArray(dest, compoundTasks);

		// -- Store the primitive tasks.
		dumpStringArray(dest, primitiveTasks);

		dest.close();

		byte[] src = stream.toByteArray();
		domainDescription = new byte[src.length];
		System.arraycopy(src, 0, getDomainDescription(), 0, src.length);
	}

	public String generate() throws Exception {
		parser.domain();

		return domainCode;
	}

	public byte[] getDomainDescription() {
		return domainDescription;
	}

	public String getName() {
		return name;
	}

}
