importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var script = ScriptingEnvironment.instance();
script.setScriptTimeout(60000);

var server = script.getServer("DebugServer.1");
server.setConfig("C2000Lib_F280049C/targetConfigs/TMS320F280049C_LaunchPad.ccxml");

var session = server.openSession(".*C28xx_CPU1");
session.target.connect();
session.memory.loadProgram("F280049C_Debug/GMP_C2000_LAUNCHPAD_FREERTOS.out");
session.target.runAsynch();
Thread.sleep(5000);
session.target.halt();

function report(name) {
    try {
        print(name + "=" + session.expression.evaluate(name));
    } catch (error) {
        print(name + "=<ERROR:" + error.message + ">");
    }
}

report("startup_task_runs");
report("heartbeat_task_runs");
report("datalink.service_run_count");
report("dl_scope.resource.sample_rate_hz");
report("dl_scope.sample_divider");
report("dl_scope.generation");
report("dl_scope_control_steps");
report("can_task_runs");
report("gmp_rtos_boot_state");
report("gmp_rtos_runtime_iterations");
report("gmp_rtos_user_task_heartbeats");
report("xTickCount");
report("control_isr_runs");
report("DSPC2000_SystemTick");
report("launchpad_adc_raw");
report("launchpad_adc_pu");
report("launchpad_output_pu");
report("launchpad_pwm_compare");
report("datalink.err_hdr_crc_cnt");
report("datalink.err_pld_crc_cnt");
report("datalink.err_fifo_ovf_cnt");
report("datalink.err_timeout_cnt");
report("sci_rx_overrun_count");

session.target.runAsynch();
session.terminate();
server.stop();
