importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)

if (arguments.length < 3) {
    throw "Usage: inspect_cold_boot.js <ccxml> <cpu1-out> <cm-out>";
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

var script = ScriptingEnvironment.instance();
script.setScriptTimeout(30000);
var server = script.getServer("DebugServer.1");
server.setConfig(arguments[0]);

/* Reset the device while only CPU1 is attached. This preserves the real
 * CPU1-owned CM release sequence instead of letting the debugger resume a
 * secondary core that is still held in reset. */
var cpu1 = server.openSession(".*C28xx_CPU1.*");
cpu1.target.connect();
cpu1.symbol.load(arguments[1]);
cpu1.target.reset();
/* The debugger reset follows the physical boot straps into CPU1 boot ROM.
 * Restart selects the loaded CPU1 ELF entry while retaining the device-wide
 * reset state, matching the normal CPU1 application initialization path. */
cpu1.target.restart();
cpu1.target.runAsynch();
Thread.sleep(5000);
if (!cpu1.target.isHalted()) cpu1.target.halt();
readExpression(cpu1, "CPU1 after reset PC", "PC");
readExpression(cpu1, "CPU1 after reset CM reset control",
    "*(unsigned long *)0x0005DC00");
readExpression(cpu1, "CPU1 after reset CM boot mode",
    "IPC_Instance[1].IPC_Boot_Pump_Reg->IPC_BOOTMODE");
readExpression(cpu1, "CPU1 after reset CM boot status",
    "IPC_Instance[1].IPC_Boot_Pump_Reg->IPC_BOOTSTS");

var cmResetControl = Number(cpu1.expression.evaluate(
    "*(unsigned long *)0x0005DC00"));
if ((cmResetControl & 1) != 0) {
    System.out.println("CM remains held in reset; skipping CM attach.");
    cpu1.target.runAsynch();
    throw "CPU1 did not release CM after device reset";
}
cpu1.target.runAsynch();

var cm = server.openSession(".*Cortex_M4_0.*");
cm.target.connect();
var cmWasRunning = !cm.target.isHalted();
if (cmWasRunning) cm.target.halt();
cm.symbol.load(arguments[2]);
if (!cpu1.target.isHalted()) cpu1.target.halt();

System.out.println("CM was running before inspection = " + cmWasRunning);
readExpression(cm, "CM pre-resume PC", "PC");
readExpression(cm, "CM pre-resume SP", "SP");
readExpression(cm, "CM VTOR", "*(unsigned long *)0xE000ED08");
readExpression(cm, "CM CFSR", "*(unsigned long *)0xE000ED28");
readExpression(cm, "CM HFSR", "*(unsigned long *)0xE000ED2C");
readExpression(cm, "CM BFAR", "*(unsigned long *)0xE000ED38");
/* Attaching to CM can halt it just after the boot ROM branches to Flash.
 * Resume both sides so the two IPC barriers can complete, then sample. */
cpu1.target.runAsynch();
var cmCanRun = true;
try {
    cm.target.runAsynch();
}
catch (error) {
    cmCanRun = false;
    System.out.println("CM resume failed = " + error);
}
if (cmCanRun) Thread.sleep(5000);
if (cmCanRun && !cm.target.isHalted()) cm.target.halt();
cpu1.target.halt();
readExpression(cpu1, "CPU1 PC", "PC");
readExpression(cm, "CM PC", "PC");
readExpression(cm, "CM CFSR after resume", "*(unsigned long *)0xE000ED28");
readExpression(cm, "CM HFSR after resume", "*(unsigned long *)0xE000ED2C");
readExpression(cm, "CM heartbeat", "cm_scheduler_heartbeats");
readExpression(cm, "CM DL errors", "cm_dl_errors");
readExpression(cm, "lwIP netif flags", "'lwiplib.c'::g_sNetIF.flags");
readExpression(cpu1, "CM boot mode",
    "IPC_Instance[1].IPC_Boot_Pump_Reg->IPC_BOOTMODE");
readExpression(cpu1, "CM boot status",
    "IPC_Instance[1].IPC_Boot_Pump_Reg->IPC_BOOTSTS");

cpu1.target.runAsynch();
if (cmCanRun) cm.target.runAsynch();
