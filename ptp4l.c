/**
 * @file ptp4l.c
 * @brief PTP Boundary Clock or Transparent Clock main program
 * @note Copyright (C) 2011 Richard Cochran <richardcochran@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <threads.h>

#include "clock.h"
#include "config.h"
#include "ntpshm.h"
#include "pi.h"
#include "print.h"
#include "raw.h"
#include "sk.h"
#include "transport.h"
#include "udp6.h"
#include "uds.h"
#include "util.h"
#include "version.h"

/* fix for getopt not thread-safe */
#define OPTPARSE_IMPLEMENTATION
#include "optparse.h"

static void usage(char *progname)
{
	fprintf(stderr,
		"\nusage: %s [options]\n\n"
		" Delay Mechanism\n\n"
		" -A        Auto, starting with E2E\n"
		" -E        E2E, delay request-response (default)\n"
		" -P        P2P, peer delay mechanism\n\n"
		" Network Transport\n\n"
		" -2        IEEE 802.3\n"
		" -4        UDP IPV4 (default)\n"
		" -6        UDP IPV6\n\n"
		" Time Stamping\n\n"
		" -H        HARDWARE (default)\n"
		" -S        SOFTWARE\n"
		" -L        LEGACY HW\n\n"
		" Other Options\n\n"
		" -f [file] read configuration from 'file'\n"
		" -i [dev]  interface device to use, for example 'eth0'\n"
		"           (may be specified multiple times)\n"
		" -p [dev]  Clock device to use, default auto\n"
		"           (ignored for SOFTWARE/LEGACY HW time stamping)\n"
		" -s        client only synchronization mode (overrides configuration file)\n"
		" -l [num]  set the logging level to 'num'\n"
		" -m        print messages to stdout\n"
		" -q        do not print messages to the syslog\n"
		" -v        prints the software version and exits\n"
		" -h        prints this message and exits\n"
		"\n",
		progname);
}

struct test_thread_info {
    int argc;
    const char** argv;
};

void * inner_main(void * ti)
{
	char *config = NULL, *req_phc = NULL, *progname;
	enum clock_type type = CLOCK_TYPE_ORDINARY;
	int option, err = -1, index, print_level;
	struct clock *clock = NULL;
	struct option *opts;
	struct config *cfg;

    int argc;
    char **argv;

    struct test_thread_info* _ti = (struct test_thread_info*)ti;
    argc = _ti->argc;
    argv = _ti->argv;

    //fprintf(stderr ,"thread test started with ID - %lu, argc = %d, argv[0] = %s, %s\n",thrd_current(), argc,argv[0], argv[1]);

    t_msg_pool_init();
    t_tc_pool_init();
    t_tlv_pool_init();

	cfg = config_create();
	if (!cfg) {
		return (void*)-1;
	}
	opts = config_long_options(cfg);

	/* Process the command line arguments. */
	progname = strrchr(argv[0], '/');
	progname = progname ? 1+progname : argv[0];

    struct optparse options;

    optparse_init(&options, argv);

	while ((option = optparse(&options, "VAEP246HSLf:i:p:sl:mqvh"))  != -1) {
		switch (option) {
		case 0:
			//if (config_parse_option(cfg, opts[index].name, optarg))
			//	goto out;
            fprintf(stderr, "Please do not use long opt.\n");
			break;
		case 'V':
			if (config_set_int(cfg, "virtual", 1))
				goto out;
			break;
		case 'A':
			if (config_set_int(cfg, "delay_mechanism", DM_AUTO))
				goto out;
			break;
		case 'E':
			if (config_set_int(cfg, "delay_mechanism", DM_E2E))
				goto out;
			break;
		case 'P':
			if (config_set_int(cfg, "delay_mechanism", DM_P2P))
				goto out;
			break;
		case '2':
			if (config_set_int(cfg, "network_transport",
					    TRANS_IEEE_802_3))
				goto out;
			break;
		case '4':
			if (config_set_int(cfg, "network_transport",
					    TRANS_UDP_IPV4))
				goto out;
			break;
		case '6':
			if (config_set_int(cfg, "network_transport",
					    TRANS_UDP_IPV6))
				goto out;
			break;
		case 'H':
			if (config_set_int(cfg, "time_stamping", TS_HARDWARE))
				goto out;
			break;
		case 'S':
			if (config_set_int(cfg, "time_stamping", TS_SOFTWARE))
				goto out;
			break;
		case 'L':
			if (config_set_int(cfg, "time_stamping", TS_LEGACY_HW))
				goto out;
			break;
		case 'f':
			config = options.optarg;
			break;
		case 'i':
			if (!config_create_interface(options.optarg, cfg))
				goto out;
			break;
		case 'p':
			req_phc = options.optarg;
			break;
		case 's':
			if (config_set_int(cfg, "clientOnly", 1)) {
				goto out;
			}
			break;
		case 'l':
			if (get_arg_val_i(option, options.optarg, &print_level,
					  PRINT_LEVEL_MIN, PRINT_LEVEL_MAX))
				goto out;
			config_set_int(cfg, "logging_level", print_level);
			break;
		case 'm':
			config_set_int(cfg, "verbose", 1);
			break;
		case 'q':
			config_set_int(cfg, "use_syslog", 0);
			break;
		case 'v':
			version_show(stdout);
			return 0;
		case 'h':
			usage(progname);
			return 0;
		case '?':
			usage(progname);
			goto out;
		default:
			usage(progname);
			goto out;
		}
	}
    if (config_get_int(cfg, NULL, "virtual")) {
		config_set_int(cfg, "domainNumber",config_get_int (cfg, NULL, "domainNumber")+1);
    }

	if (config && (option = config_read(config, cfg))) {
		return (void *)option;
	}

	print_set_progname(progname);
	print_set_tag(config_get_string(cfg, NULL, "message_tag"));
	print_set_verbose(config_get_int(cfg, NULL, "verbose"));
	print_set_syslog(config_get_int(cfg, NULL, "use_syslog"));
	print_set_level(config_get_int(cfg, NULL, "logging_level"));

	assume_two_step = config_get_int(cfg, NULL, "assume_two_step");
	sk_check_fupsync = config_get_int(cfg, NULL, "check_fup_sync");
	sk_tx_timeout = config_get_int(cfg, NULL, "tx_timestamp_timeout");
	sk_hwts_filter_mode = config_get_int(cfg, NULL, "hwts_filter");

	if (config_get_int(cfg, NULL, "clock_servo") == CLOCK_SERVO_NTPSHM) {
		config_set_int(cfg, "kernel_leap", 0);
		config_set_int(cfg, "sanity_freq_limit", 0);
	}

	if (STAILQ_EMPTY(&cfg->interfaces)) {
		fprintf(stderr, "no interface specified\n");
		usage(progname);
		goto out;
	}

	type = config_get_int(cfg, NULL, "clock_type");
	switch (type) {
	case CLOCK_TYPE_ORDINARY:
		if (cfg->n_interfaces > 1) {
			type = CLOCK_TYPE_BOUNDARY;
		}
		break;
	case CLOCK_TYPE_BOUNDARY:
		if (cfg->n_interfaces < 2) {
			fprintf(stderr, "BC needs at least two interfaces\n");
			goto out;
		}
		break;
	case CLOCK_TYPE_P2P:
		if (cfg->n_interfaces < 2) {
			fprintf(stderr, "TC needs at least two interfaces\n");
			goto out;
		}
		if (DM_P2P != config_get_int(cfg, NULL, "delay_mechanism")) {
			fprintf(stderr, "P2P_TC needs P2P delay mechanism\n");
			goto out;
		}
		break;
	case CLOCK_TYPE_E2E:
		if (cfg->n_interfaces < 2) {
			fprintf(stderr, "TC needs at least two interfaces\n");
			goto out;
		}
		if (DM_E2E != config_get_int(cfg, NULL, "delay_mechanism")) {
			fprintf(stderr, "E2E_TC needs E2E delay mechanism\n");
			goto out;
		}
		break;
	case CLOCK_TYPE_MANAGEMENT:
		goto out;
	}

	clock = clock_create(type, cfg, req_phc);
	if (!clock) {
		fprintf(stderr, "failed to create a clock\n");
		goto out;
	}

	err = 0;

	while (is_running()) {
		if (clock_poll(clock))
			break;
	}
out:
	if (clock)
		clock_destroy(clock);
	config_destroy(cfg);

	return (void *)err;
}


int main(int argc, char *argv[]) {
    thrd_t test_thread, test_thread2;
    int rc = 0;
    struct test_thread_info ti;
    struct test_thread_info ti2;

    ti.argc = argc;
    ti.argv = argv;

    ti2.argc = argc+1; // add for virtual flag
    char** new_argv = malloc((argc+2) * sizeof *new_argv);
    for(int i = 0; i < argc; ++i)
    {
        size_t length = strlen(argv[i]); // excluding the null byte at end
        new_argv[i] = strndup(argv[i], length); //malloc(length);
        //memcpy(new_argv[i], argv[i], length);
    }
    new_argv[argc+1] = NULL;
    new_argv[argc] = "-V";

    ti2.argv = new_argv;

	if (handle_term_signals())
		return -1;

    rc = thrd_create(&test_thread, (thrd_start_t) inner_main, (void *)&ti);
    if (rc == thrd_error) {
        printf("ERORR; thrd_create() call failed\n");
        exit(EXIT_FAILURE);
    }

    rc = thrd_create(&test_thread2, (thrd_start_t) inner_main, (void *)&ti2);
    if (rc == thrd_error) {
        printf("ERORR; thrd_create() call failed\n");
        exit(EXIT_FAILURE);
    }

    int retval2, retval;
    thrd_join(test_thread,  &retval);
    thrd_join(test_thread2, &retval2);

    return retval | retval2;
}
