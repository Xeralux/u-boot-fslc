/*
 * (C) Copyright 2000-2008
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * Command line user interface to firmware (=U-Boot) environment.
 *
 * Implements:
 *	fw_printenv [[ -n name ] | [ name ... ]]
 *              - prints the value of a single environment variable
 *                "name", the ``name=value'' pairs of one or more
 *                environment variables "name", or the whole
 *                environment if no names are specified.
 *	fw_setenv name [ value ... ]
 *		- If a name without any values is given, the variable
 *		  with this name is deleted from the environment;
 *		  otherwise, all "value" arguments are concatenated,
 *		  separated by single blank characters, and the
 *		  resulting string is assigned to the environment
 *		  variable "name"
 */

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/file.h>
#include <unistd.h>
#include "fw_env.h"

#define	CMD_PRINTENV	"fw_printenv"
#define CMD_SETENV	"fw_setenv"

static struct option long_options[] = {
	{"script", required_argument, NULL, 's'},
	{"default", no_argument, NULL, 'd'},
	{"all", no_argument, NULL, 'a'},
	{"help", no_argument, NULL, 'h'},
	{NULL, 0, NULL, 0}
};

void usage(void)
{

	fprintf(stderr, "fw_printenv/fw_setenv, "
		"Command line tool to inspect/alter U-Boot environment variables.\n\n"
		"usage:\tfw_printenv [-n] <variable_name>\n"
		"\tfw_setenv <variable_name> <variable_value>\n"
		"\tfw_setenv -d <variable_name>\n"
		"\t\tRestore <variable_name> to compile-time default.\n"
		"\tfw_setenv -d -a\n"
		"\t\tRestore all variables to compile-time defaults.\n"
		"\tfw_setenv -s [ script_file ]\n"
		"\tfw_setenv -s - < [ script_file ]\n\n"
		"The script_file passed as argument contains only pairs "
		"name / value\n"
		"Example:\n"
		"# Any line starting with # is treated as comment\n"
		"\n"
		"\t      netdev         eth0\n"
		"\t      kernel_addr    400000\n"
		"\t      var1\n"
		"\t      var2          The quick brown fox jumps over the "
		"lazy dog\n"
		"\n"
		"A variable without value will be dropped. It is possible\n"
		"to put any number of spaces between the fields, but any\n"
		"space inside the value is treated as part of the value "
		"itself.\n\n"
	);
}

int main(int argc, char *argv[])
{
	char *p;
	char *cmdname = *argv;
	char *script_file = NULL;
	int force_default = 0;
	int single_var = 0;
	int c;
	const char *lockname = "/var/lock/" CMD_PRINTENV ".lock";
	int lockfd = -1;
	int retval = EXIT_SUCCESS;

	lockfd = open(lockname, O_WRONLY | O_CREAT | O_TRUNC, 0666);
	if (-1 == lockfd) {
		fprintf(stderr, "Error opening lock file %s\n", lockname);
		return EXIT_FAILURE;
	}

	if (-1 == flock(lockfd, LOCK_EX)) {
		fprintf(stderr, "Error locking file %s\n", lockname);
		close(lockfd);
		return EXIT_FAILURE;
	}

	if ((p = strrchr (cmdname, '/')) != NULL) {
		cmdname = p + 1;
	}

	while ((c = getopt_long (argc, argv, "ns:dah",
		long_options, NULL)) != EOF) {
		switch (c) {
		case 'n':
			/* handled in fw_printenv */
			single_var = 1;
			break;
		case 's':
			script_file = optarg;
			break;
		case 'd':
			force_default = 1;
			break;
		case 'a':
			if (force_default) {
				force_default++;
			} else {
				fprintf (stderr,
				         "Error: "
				         "Option -a only valid after -d.\n");
				goto exit;
			}
			break;
		case 'h':
			usage();
			goto exit;
		default: /* '?' */
			fprintf(stderr, "Try `%s --help' for more information."
				"\n", cmdname);
			retval = EXIT_FAILURE;
			goto exit;
		}
	}

	if (strcmp(cmdname, CMD_PRINTENV) == 0) {
		if (fw_printenv (argc, optind, argv,
		                 single_var, force_default) != 0)
			retval = EXIT_FAILURE;
	} else if (strcmp(cmdname, CMD_SETENV) == 0) {
		if (single_var) {
			fprintf (stderr, "\"-n\" option makes no sense here.\n");
			retval = EXIT_FAILURE;
		} else if (!script_file) {
			if (fw_setenv (argc, optind, argv, force_default) != 0)
				retval = EXIT_FAILURE;
		} else {
			if (fw_parse_script(script_file) != 0)
				retval = EXIT_FAILURE;
		}
	} else {
		fprintf(stderr,
			"Identity crisis - may be called as `" CMD_PRINTENV
			"' or as `" CMD_SETENV "' but not as `%s'\n",
			cmdname);
		retval = EXIT_FAILURE;
	}

exit:
	flock(lockfd, LOCK_UN);
	close(lockfd);
	return retval;
}
