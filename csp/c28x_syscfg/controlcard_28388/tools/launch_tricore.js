importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)

if (arguments.length < 4) {
    throw "Usage: launch_tricore.js <ccxml> <cpu1-out> <cpu2-out> <cm-out> [run-ms] [entry|resetisr|natural] [snapshot|nosnapshot]";
}

var runMilliseconds = arguments.length >= 5 ? parseInt(arguments[4], 10) : 30000;
var cmStartMode = arguments.length >= 6 ? String(arguments[5]).toLowerCase() : "entry";
var takeInitialSnapshot = arguments.length < 7 ||
                          String(arguments[6]).toLowerCase() != "nosnapshot";
if (cmStartMode != "entry" && cmStartMode != "resetisr" &&
    cmStartMode != "natural") {
    throw "CM start mode must be entry, resetisr, or natural";
}

function readExpression(session, label, expression)
{
    try {
        System.out.println(label + " = " + session.expression.evaluate(expression));
    }
    catch (error) {
        System.out.println(label + " = <unavailable: " + error + ">");
    }
}

function printState(cpu1, cpu2, cm, phase)
{
    System.out.println("--- " + phase + " ---");
    readExpression(cpu1, "CPU1 PC", "PC");
    readExpression(cpu2, "CPU2 PC", "PC");
    readExpression(cpu2, "CPU2 heartbeat", "cpu2_scheduler_heartbeats");
    readExpression(cpu2, "CPU2 command updates", "cpu2_command_updates");
    readExpression(cpu2, "CPU2 wave samples", "'user_main.c'::wave_sample_count");
    readExpression(cm, "CM PC", "PC");
    readExpression(cm, "CM heartbeat", "cm_scheduler_heartbeats");
    readExpression(cm, "CM DL errors", "cm_dl_errors");
    readExpression(cm, "CM last network error", "cm_last_network_error");
    readExpression(cm, "CM Rx frames", "cm_rx_frames");
    readExpression(cm, "CM Tx frames", "cm_tx_frames");
    readExpression(cm, "CM Tx retry pending", "cm_tx_retry_pending");
    readExpression(cm, "CM ERR_MEM count", "cm_err_mem_count");
    readExpression(cm, "CM ERR_MEM stage", "cm_err_mem_stage");
    readExpression(cm, "CM last TX frame size", "cm_last_tx_frame_size");
    readExpression(cm, "CM last TCP send buffer", "cm_last_tcp_sndbuf");
    readExpression(cm, "CM last TCP queue length", "cm_last_tcp_queuelen");
    readExpression(cm, "CM DL TX state", "'user_main.c'::cm_datalink.tx_state");
    readExpression(cm, "CM DL RX state", "'user_main.c'::cm_datalink.rx_state");
    readExpression(cm, "CM DL RX FIFO overflows", "'user_main.c'::cm_datalink.err_fifo_ovf_cnt");
    readExpression(cm, "CM DL header CRC errors", "'user_main.c'::cm_datalink.err_hdr_crc_cnt");
    readExpression(cm, "CM DL payload CRC errors", "'user_main.c'::cm_datalink.err_pld_crc_cnt");
    readExpression(cm, "CM scope state", "'user_main.c'::cm_scope.state");
    readExpression(cm, "CM scope generation", "'user_main.c'::cm_scope.generation");
    readExpression(cm, "CM scope history count", "'user_main.c'::cm_scope.history_count");
    readExpression(cm, "CM SysTick period", "systickPeriodValue");
    readExpression(cm, "Ethernet RX interrupt count", "genericISRCustomRIcount");
    readExpression(cm, "Ethernet generic interrupt count", "genericISRCustomcount");
    readExpression(cm, "Ethernet RX-buffer-unavailable count", "genericISRCustomRBUcount");
    readExpression(cm, "Ethernet RX-overflow count", "genericISRCustomROVcount");
    readExpression(cm, "Ethernet RX buffer requests", "Ethernet_numGetPacketBufferCallback");
    readExpression(cm, "CM RX ring backpressure", "cm_rx_ring_backpressure_count");
    readExpression(cm, "CM RX parser resets", "cm_rx_ring_reset_count");
    readExpression(cm, "CM TCP accepts", "cm_tcp_accept_count");
    readExpression(cm, "CM TCP closes", "cm_tcp_close_count");
    readExpression(cm, "CM TCP aborts/errors", "cm_tcp_abort_count");
    readExpression(cm, "lwIP netif flags", "'lwiplib.c'::g_sNetIF.flags");
    readExpression(cm, "lwIP local IPv4", "'lwiplib.c'::g_sNetIF.ip_addr.addr");
    readExpression(cm, "TCP listener", "'xplt.peripheral.c'::cm_tcp_listener");
    readExpression(cm, "TCP client", "'xplt.peripheral.c'::cm_tcp_client");
    readExpression(cm, "TCP send buffer", "'xplt.peripheral.c'::cm_tcp_client == 0 ? 0 : 'xplt.peripheral.c'::cm_tcp_client->snd_buf");
    readExpression(cm, "TCP queued segments", "'xplt.peripheral.c'::cm_tcp_client == 0 ? 0 : 'xplt.peripheral.c'::cm_tcp_client->snd_queuelen");
    readExpression(cpu1, "CPU1->CM flags",
        "IPC_Instance[1].IPC_Flag_Ctr_Reg->IPC_FLG");
    readExpression(cpu1, "CM->CPU1 flags",
        "IPC_Instance[1].IPC_Flag_Ctr_Reg->IPC_STS");
    readExpression(cpu1, "CM boot mode",
        "IPC_Instance[1].IPC_Boot_Pump_Reg->IPC_BOOTMODE");
    readExpression(cpu1, "CM boot status",
        "IPC_Instance[1].IPC_Boot_Pump_Reg->IPC_BOOTSTS");
}

var script = ScriptingEnvironment.instance();
script.setScriptTimeout(30000);
var server = script.getServer("DebugServer.1");
server.setConfig(arguments[0]);
var cpu1 = server.openSession(".*C28xx_CPU1.*");
var cpu2 = server.openSession(".*C28xx_CPU2.*");
var cm = server.openSession(".*Cortex_M4_0.*");

cpu1.target.connect();
cpu2.target.connect();
cm.target.connect();
cpu1.symbol.load(arguments[1]);
cpu2.symbol.load(arguments[2]);
cm.symbol.load(arguments[3]);

/* Connecting after a multicore flash leaves the secondary-core boot ROM
 * halted. Restart CPU1 so it republishes the boot command. The optional start
 * modes isolate the debugger entry point, ResetISR, and natural boot-ROM flow. */
cpu1.target.restart();
cpu1.target.runAsynch();
Thread.sleep(250);
if (cmStartMode == "entry" || cmStartMode == "resetisr") {
    cm.target.restart();
}
if (cmStartMode == "resetisr") {
    cm.expression.evaluate("PC = ResetISR");
}
System.out.println("CM start mode = " + cmStartMode);
cm.target.runAsynch();
cpu2.target.restart();
cpu2.target.runAsynch();
Thread.sleep(3000);

/* Halting CM while Ethernet is active can itself provoke an RX-buffer-
 * unavailable condition. Hardware communication tests therefore use the
 * nosnapshot mode and observe state only after traffic is complete. */
if (takeInitialSnapshot) {
    cm.target.halt();
    cpu2.target.halt();
    cpu1.target.halt();
    printState(cpu1, cpu2, cm, "initial state");

    cpu1.target.runAsynch();
    cpu2.target.runAsynch();
    cm.target.runAsynch();
}
System.out.println("Running all three cores for " + runMilliseconds +
    " ms; exercise serial/Ethernet DL now.");
Thread.sleep(runMilliseconds);

cm.target.halt();
cpu2.target.halt();
cpu1.target.halt();
printState(cpu1, cpu2, cm, "final state");

cpu1.target.runAsynch();
cpu2.target.runAsynch();
cm.target.runAsynch();
