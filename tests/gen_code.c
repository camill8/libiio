// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2014, 2019 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *         Robin Getz <robin.getz@analog.com>
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <iio.h>

#include "iio_common.h"

static FILE *fd = NULL;
static char *uri;
static enum languages {
	C_LANG,
	PYTHON_LANG,
	UNSUPPORTED_LANG,
} lang = UNSUPPORTED_LANG;

static int gen_fopen(FILE** pFile, const char *filename, const char *mode)
{
	int ret = 0;


#ifdef _MSC_BUILD
	ret = fopen_s(pFile, filename, mode);
#else
	*pFile = fopen(filename, mode);
	if (!*pFile)
		ret = -errno;
#endif

	return ret;
}

bool gen_test_path(const char *gen_file)
{
	FILE *test;
	char *last;
	int ret;

	if (!gen_file)
		return false;

	if (gen_file[0] == '-')
		return false;

	last = strrchr(gen_file, '.');
	if (*last != '.')
		return false;
	last++;

	if (!strcmp(last, "c"))
		lang = C_LANG;
	else if (!strcmp(last, "py"))
		lang = PYTHON_LANG;
	else {
		fprintf(stderr, "Not a 'c' or 'py' file\n");
		return false;
	}

	ret = gen_fopen(&test, gen_file, "w");
	if (ret)
		return false;
	fclose(test);

	return true;
}

void gen_start(const char *gen_file)
{
	int ret;

	if (!gen_file)
		return;

	if (lang == UNSUPPORTED_LANG)
		return;

	ret = gen_fopen(&fd, gen_file, "w");
	if (ret) {
		char buf[1024];
		iio_strerror(-ret, buf, sizeof(buf));
		fprintf(stderr, "Error '%s' opening file: %s\n", buf, gen_file);
		return;
	}
	if (lang == C_LANG) {
		fprintf(fd, "/*******************************************************************\n"
		    " * This is autogenerated code from the iio_utils package\n"
		    " * Code snippets in this file are released under the WTFPL.\n"
		    " * For more information, check out : http://www.wtfpl.net/.\n"
		    " * This does not effect the license for libiio or iio-utils.\n"
		    " * If this helps - great, if it does not - stop using it.\n"
		    " *******************************************************************\n");
		fprintf(fd, " * Compile with 'gcc %s -o /tmp/aout -liio'\n", gen_file);
		fprintf(fd, " *******************************************************************/\n");
		fprintf(fd, "#include <stdio.h>\n#include <errno.h>\n#include <iio.h>\n\n");

		fprintf(fd, "/* These macros are for illustrative purposes only */\n");
		fprintf(fd, "#define IIO_ASSERT(expr) { \\\n");
		fprintf(fd, "\tif (!(expr)) { \\\n");
		fprintf(fd, "\t\tiio_strerror(errno, buf, sizeof(buf)); \\\n");
		fprintf(fd, "\t\t(void) fprintf(stderr, \"Assertion triggered:\\n\"); \\\n");
		fprintf(fd, "\t\t(void) fprintf(stderr, \"\\t%%s (file:%%s, line:%%d)\\n\", \\\n"
			    "\t\t\tbuf, __FILE__, __LINE__); \\\n");
		fprintf(fd, "\t\t(void) abort(); \\\n");
		fprintf(fd, "\t} \\\n");
		fprintf(fd, "}\n\n");

		fprintf(fd, "#define RET_ASSERT(expr) { \\\n");
		fprintf(fd, "\tif ((expr) <= 0) { \\\n");
		fprintf(fd, "\t\tiio_strerror(-ret, buf, sizeof(buf)); \\\n");
		fprintf(fd, "\t\t(void) fprintf(stderr, \"Assertion triggered:\\n\"); \\\n");
		fprintf(fd, "\t\t(void) fprintf(stderr, \"%%s (file:%%s, line:%%d)\\n\", \\\n"
			    "\t\t\tbuf, __FILE__, __LINE__); \\\n");
		fprintf(fd, "\t\t(void) abort(); \\\n");
		fprintf(fd, "\t} \\\n");
		fprintf(fd, "}\n\n");

		fprintf(fd, "int main(int argc, char **argv)\n{\n"
			"\tstruct iio_context *ctx;\n\tstruct iio_device *dev;\n\tstruct iio_channel *ch;\n"
			"\tconst char* val_str;\n\tssize_t ret;\n\tchar buf[256];\n\n");

	} else if (lang == PYTHON_LANG) {
		/* https://www.python.org/dev/peps/pep-0008 says to use 4 spaces for indent */
		fprintf(fd, "####################################################################\n"
			    "# This is autogenerated code from the iio_utils package\n"
			    "# Code snippets in this file are released under the WTFPL.\n"
			    "# For more information, check out : http://www.wtfpl.net/.\n"
			    "# This does not effect the license for libiio or iio-utils.\n"
			    "# If this helps - great, if it does not - stop using it.\n"
			    "####################################################################\n");
		fprintf(fd, "# Execute with python : 'python3 %s'\n", gen_file);
		fprintf(fd, "####################################################################\n");
		fprintf(fd, "import sys\n\n");
		fprintf(fd, "try:\n"
			    "    import iio\n"
			    "except:\n"
			    "    # By default the iio python bindings are not in path\n"
			    "    print(\"you must fix your PYTHONPATH to include iio\")\n"
			    "    exit(1)\n\n\n");
		fprintf(fd, "def main():\n");
	}
}

void gen_context (const char *uri_in)
{
	if (!fd)
		return;

	if (uri_in)
		uri = cmn_strndup(uri_in, NAME_MAX);
	else
		uri = cmn_strndup("unknown:", NAME_MAX);

	if (lang == C_LANG) {
		fprintf(fd, "\t/* Create IIO Context */\n"
		    "\tIIO_ASSERT(ctx = iio_create_context_from_uri(\"%s\"));\n\n", uri);
	} else if (lang == PYTHON_LANG) {
		fprintf(fd, "    # Create IIO Context\n"
			    "    try:\n"
			    "        ctx = iio.Context(\"%s\")\n", uri);
		fprintf(fd, "    except OSError as e:\n"
			    "        print(\"Unable to open context %s\")\n", uri);
		fprintf(fd, "        exit(1)\n\n");
	}
}

void gen_context_destroy()
{
	if (!fd)
		return;

	if (lang == C_LANG) {
		fprintf(fd, "\n\t/* Close context at %s, can release/destroy things */\n", uri);
		fprintf(fd, "\tiio_context_destroy(ctx);\n\treturn EXIT_SUCCESS;\n}\n");
	} else if (lang == PYTHON_LANG) {
		/* No need to close context in python
		 * destroy is called when the destructor is called for iio objects
		 */
		fprintf(fd, "\n\nif __name__ == \"__main__\":\n"
			    "    main()\n");
	}

	fclose(fd);
	free(uri);
}

void gen_context_attr(const char *key)
{
	if (!fd)
		return;

	if (lang == C_LANG) {
		fprintf(fd, "\t/* Read IIO Context attribute and return result as string */\n");
		fprintf(fd, "\tval_str = iio_context_get_attr_value(ctx, \"%s\");\n", key);
		fprintf(fd, "\tprintf(\"%s : %%s\\n\", val_str);\n", key);
	} else if (lang == PYTHON_LANG) {
		fprintf(fd, "    # Read IIO Context attribute and return result as string\n");
		fprintf(fd, "    print(\"%s : \" + ctx.attrs[\"%s\"])\n", key, key);
	}
}

void gen_dev(const struct iio_device *dev)
{
	if (!fd)
		return;
	if (lang == C_LANG) {
		fprintf(fd, "\t/* Find IIO device in current context */\n");
		fprintf(fd, "\tIIO_ASSERT(dev = iio_context_find_device(ctx, \"%s\"));\n\n",iio_device_get_name(dev));
	} else if (lang == PYTHON_LANG) {
		fprintf(fd, "    # Find IIO device in current context\n");
		fprintf(fd, "    dev = ctx.find_device(\"%s\")\n\n",iio_device_get_name(dev));
	}
}

void gen_ch(const struct iio_channel *ch)
{
	const char *name;

	if (!fd)
		return;

	name = iio_channel_get_name(ch);
	if (!name)
		name = iio_channel_get_id(ch);

	if (lang == C_LANG) {
		fprintf(fd, "\t/* Find the IIO %s channel in the current device */\n",
			iio_channel_is_output(ch) ? "Output" : "Input");
		fprintf(fd, "\tIIO_ASSERT(ch = iio_device_find_channel(dev, \"%s\", %s));\n\n",
			name, iio_channel_is_output(ch) ? "true" : "false");
	} else if (lang == PYTHON_LANG) {
		fprintf(fd, "    #Find the IIO %s channel in the current device */\n",
			iio_channel_is_output(ch) ? "Output" : "Input");
		fprintf(fd, "    ch = dev.find_channel('%s', %s)\n",
			name, iio_channel_is_output(ch) ? "True" : "False");
	}
}

void gen_context_timeout(unsigned int timeout_ms)
{
	if (!fd)
		return;

	if (lang == C_LANG) {
		fprintf(fd, "\t/* Set the context timeout in ms */\n");
		fprintf(fd, "\tiio_context_set_timeout(ctx, %ui);\n", timeout_ms);
	}
}

void gen_function(const char* prefix, const char* target,
		const char* attr, const char* wbuf)
{
	char *rw = wbuf ? "write" : "read";

	if(!fd)
		return;

	if (lang == C_LANG) {
		if (wbuf) {
			fprintf(fd, "\t/* Write null terminated string to %s attribute: */\n", prefix);
			fprintf(fd, "\tRET_ASSERT(ret = iio_%s_attr_write(\n"
				    "\t\t\t%s, \"%s\", \"%s\"));\n",
				prefix, target, attr, wbuf);
		} else {
			fprintf(fd, "\t/* Read IIO %s attribute, and put result in string */\n", prefix);
			fprintf(fd, "\tRET_ASSERT(ret = iio_%s_attr_read(\n"
				    "\t\t\t%s, \"%s\", buf, sizeof(buf)));\n",
				prefix, target, attr);
		}
		fprintf(fd, "\t/* For other types, use:\n");
		fprintf(fd, "\t *  ret = iio_%s_attr_%s_bool(%s, \"%s\", v_bool);\n",
			prefix, rw, target, attr);
		fprintf(fd, "\t *  ret = iio_%s_attr_%s_double(%s, \"%s\", v_double);\n",
			prefix, rw, target, attr);
		fprintf(fd, "\t *  ret = iio_%s_attr_%s_longlong(%s, \"%s\", v_ll);\n",
			prefix, rw, target, attr);
		fprintf(fd, "\t *******************************************************************/\n");
		if (wbuf) {
			fprintf(fd, "\tprintf(\"Wrote %%zi bytes\\n\", ret);\n\n");
		} else {
			fprintf(fd, "\tprintf(\"%s : %%s\\n\", buf);\n\n", attr);
		}
	} else if (lang == PYTHON_LANG) {
		if (wbuf) {
			fprintf(fd, "    # Write string to %s attribute:\n", prefix);
			if (!strcmp(prefix, "device") || !strcmp(prefix, "channel")) {
				fprintf(fd, "    %s.attrs[\"%s\"].value = str(\"%s\")\n", target, attr, wbuf);
			} else if (!strcmp(prefix, "device_debug")) {
				fprintf(fd, "    %s.debug_attrs[\"%s\"].value = str(\"%s\")\n", target, attr, wbuf);
			} else {
				fprintf(fd, "    # Write for %s / %s not implemented yet\n", prefix, target);
			}
			fprintf(fd, "    print(\"wrote %s into %s\")\n", wbuf, attr);
		} else {
			fprintf(fd, "    # Read IIO %s attribute\n", prefix);
			if (!strcmp(prefix, "device") || !strcmp(prefix, "channel")) {
				fprintf(fd, "    print(\"%s : \" + %s.attrs[\"%s\"].value)\n",
						attr, target, attr);
			} else if (!strcmp(prefix, "device_debug")) {
				fprintf(fd, "    print(\"%s : \" + %s.debug_attrs[\"%s\"].value)\n",
						attr, target, attr);
			} else {
				fprintf(fd, "    # Read for %s / %s not implemented yet\n", prefix, target);
			}
		}

	}
}

