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

static int _cpu_repl_s(int argc, char** argv) {
    cpu_single_step(1);
    return 0;
}

static int _cpu_repl_c(int argc, char** argv) {
    cpu_set_running(!cpu_running());
    return 0;
}

static int _cpu_repl_t(int args, char** argv) {
    cpu_set_trace(!cpu_trace());
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
        .func = &_cpu_repl_s
    };
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

    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}