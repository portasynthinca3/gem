#include "cpu.h"
#include "machine.h"
#include <string.h>
#include <esp_log.h>
#include <stdlib.h>
#include <stdio.h>
#include <esp_console.h>
#include <linenoise/linenoise.h>
#include <argtable3/argtable3.h>

#define TAG "8086-repl"

static int _cpu_repl_r(int argc, char** argv) {
    cpu_print_state();
    return 0;
}

static struct {
    struct arg_int* num;
    struct arg_end* end;
} s_args;
static int _cpu_repl_s(int argc, char** argv) {
    int arg_errors = arg_parse(argc, argv, (void**)&s_args);
    if(arg_errors != 0) {
        arg_print_errors(stderr, s_args.end, argv[0]);
        return 0;
    }

    if(s_args.num->count == 0)
        s_args.num->ival[0] = 1;

    cpu_single_step(s_args.num->ival[0]);
    return 0;
}

static int _cpu_repl_c(int argc, char** argv) {
    cpu_set_running(!cpu_running());
    return 0;
}

static int _cpu_repl_t(int argc, char** argv) {
    cpu_set_trace(!cpu_trace());
    return 0;
}

static struct {
    struct arg_str* ip;
    struct arg_str* cs;
    struct arg_end* end;
} b_args;
static int _cpu_repl_b(int argc, char** argv) {
    int arg_errors = arg_parse(argc, argv, (void**)&b_args);
    if(arg_errors != 0) {
        arg_print_errors(stderr, b_args.end, argv[0]);
        return 0;
    }

    // get IP and CS
    uint32_t ip, cs = cpu_state().cs;
    sscanf(b_args.ip->sval[0], "%x", &ip);
    if(b_args.cs->count != 0)
        sscanf(b_args.cs->sval[0], "%x", &cs);

    cpu_breakpoint(cs, ip);
    return 0;
}

static int _cpu_repl_lb(int argc, char** argv) {
    cpu_list_breakpoints();
    return 0;
}

static struct {
    struct arg_int* num;
    struct arg_end* end;
} rb_args;
static int _cpu_repl_rb(int argc, char** argv) {
    int arg_errors = arg_parse(argc, argv, (void**)&rb_args);
    if(arg_errors != 0) {
        arg_print_errors(stderr, rb_args.end, argv[0]);
        return 0;
    }

    cpu_del_breakpoint(rb_args.num->ival[0]);
    return 0;
}

void cpu_repl_init() {
    esp_console_repl_t* repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    repl_config.prompt = "8086>";

    // register commands
    esp_console_register_help_command();
    const esp_console_cmd_t r_cmd = {
        .command = "r",
        .help = "Print register values",
        .hint = NULL,
        .func = &_cpu_repl_r
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&r_cmd));

    const esp_console_cmd_t s_cmd = {
        .command = "s",
        .help = "Single CPU step",
        .hint = NULL,
        .func = &_cpu_repl_s,
        .argtable = &s_args
    };
    s_args.num = arg_int0(NULL, NULL, "<num>", "how many steps to perform");
    s_args.end = arg_end(1);
    ESP_ERROR_CHECK(esp_console_cmd_register(&s_cmd));

    const esp_console_cmd_t c_cmd = {
        .command = "c",
        .help = "Start/stop execution",
        .hint = NULL,
        .func = &_cpu_repl_c
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&c_cmd));

    const esp_console_cmd_t t_cmd = {
        .command = "t",
        .help = "Start/stop tracing",
        .hint = NULL,
        .func = &_cpu_repl_t
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&t_cmd));

    const esp_console_cmd_t b_cmd = {
        .command = "b",
        .help = "Add breakpoint",
        .hint = NULL,
        .func = &_cpu_repl_b,
        .argtable = &b_args
    };
    b_args.ip = arg_str1(NULL, NULL, "<ip>", "hexadecimal Instruction Pointer register value");
    b_args.cs = arg_str0(NULL, NULL, "<cs>", "hexadecimal Code Segment register value (set to current if omitted)");
    b_args.end = arg_end(1);
    ESP_ERROR_CHECK(esp_console_cmd_register(&b_cmd));

    const esp_console_cmd_t lb_cmd = {
        .command = "lb",
        .help = "List breakpoints",
        .hint = NULL,
        .func = &_cpu_repl_lb
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&lb_cmd));

    const esp_console_cmd_t rb_cmd = {
        .command = "rb",
        .help = "Remove breakpoint",
        .hint = NULL,
        .func = &_cpu_repl_rb,
        .argtable = &rb_args
    };
    rb_args.num = arg_int1(NULL, NULL, "<num>", "which breakpoint to remove");
    rb_args.end = arg_end(1);
    ESP_ERROR_CHECK(esp_console_cmd_register(&rb_cmd));

    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}