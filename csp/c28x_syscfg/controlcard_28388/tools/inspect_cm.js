importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)

var args = arguments;
if (args.length < 2) {
    throw "Usage: inspect_cm.js <ccxml> <cm-out> [resume]";
}
var forceResume = args.length >= 3 && args[2] == "resume";

var script = ScriptingEnvironment.instance();
script.setScriptTimeout(30000);
var server = script.getServer("DebugServer.1");
server.setConfig(args[0]);
var session = server.openSession(".*Cortex_M4_0.*");

function show(expression) {
    try {
        System.out.println(expression + " = " + session.expression.evaluate(expression));
    } catch (exception) {
        System.out.println(expression + " = <unavailable: " + exception + ">");
    }
}

session.target.connect();
var wasRunning = !session.target.isHalted();
if (wasRunning) {
    session.target.halt();
}
session.symbol.load(args[1]);

System.out.println("CM was running: " + wasRunning);
show("PC");
show("SP");
show("cm_scheduler_heartbeats");
show("cm_dl_errors");
show("cm_last_network_error");
show("cm_rx_frames");
show("cm_tx_frames");
show("'user_main.c'::cm_scope.state");
show("'user_main.c'::cm_scope.generation");
show("'user_main.c'::cm_scope.history_count");
show("cm_tx_retry_pending");
show("cm_err_mem_count");
show("cm_rx_ring_backpressure_count");
show("cm_rx_ring_reset_count");
show("cm_tcp_accept_count");
show("cm_tcp_close_count");
show("cm_tcp_abort_count");
show("cpu2_to_cm_snapshot.magic");
show("cpu2_to_cm_snapshot.sample_count");
show("cm_ethercat_ready");
show("systickPeriodValue");

if (forceResume) {
    System.out.println("Running CM for two seconds before a second snapshot...");
    session.target.runAsynch();
    Thread.sleep(2000);
    session.target.halt();
    show("PC");
    show("cm_scheduler_heartbeats");
    show("cm_dl_errors");
    show("cm_last_network_error");
    show("cm_rx_frames");
    show("cm_tx_frames");
    show("cm_ethercat_ready");
    show("systickPeriodValue");
}

/* Connecting through DSS halts CM before isHalted() can describe its prior
 * state. Always restore execution so this read-only probe cannot accidentally
 * stop the communication core. */
session.target.runAsynch();
session.target.disconnect();
session.terminate();
server.stop();
