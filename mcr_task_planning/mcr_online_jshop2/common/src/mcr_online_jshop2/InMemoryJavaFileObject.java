package mcr_online_jshop2;

import java.net.URI;

import javax.tools.SimpleJavaFileObject;

class InMemoryJavaFileObject extends SimpleJavaFileObject {
	final String code;

	InMemoryJavaFileObject(String name, String code) {
		super(URI.create("string:///" + name.replace('.', '/')
				+ Kind.SOURCE.extension), Kind.SOURCE);
		this.code = code;
	}

	@Override
	public CharSequence getCharContent(boolean ignoreEncodingErrors) {
		return code;
	}
}