importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)

if (arguments.length < 4) {
    throw "Usage: resume_tricore.js <ccxml> <cpu1-out> <cpu2-out> <cm-out>";
}

var script = ScriptingEnvironment.instance();
script.setScriptTimeout(30000);
var server = script.getServer("DebugServer.1");
server.setConfig(arguments[0]);
var cpu1 = server.openSession(".*C28xx_CPU1.*");
var cpu2 = server.openSession(".*C28xx_CPU2.*");
var cm = server.openSession(".*Cortex_M4_0.*");

/* UniFlash/GEL can leave secondary boot ROMs halted after programming. CPU1
 * then waits forever at its first CM IPC barrier. Resume the already-loaded
 * CPU1 image, let CM acknowledge that barrier, then let CPU2 consume the boot
 * command CPU1 publishes. No core is restarted here, so the programmed Flash
 * entry and CPU1-owned initialization order remain authoritative. */
cpu1.target.connect();
cpu2.target.connect();
cm.target.connect();
cpu1.symbol.load(arguments[1]);
cpu2.symbol.load(arguments[2]);
cm.symbol.load(arguments[3]);

/* Explicitly select each programmed image's C runtime entry. Debug attach can
 * leave a core halted in boot ROM even when UniFlash used --run. Assigning PC
 * is the same entry selection CCS performs after loading an ELF; code_start
 * and ResetISR still initialize the language runtime and stack normally. */
cpu1.expression.evaluate("PC = code_start");
cpu1.target.runAsynch();
Thread.sleep(250);
cm.expression.evaluate("PC = ResetISR");
cm.target.runAsynch();
Thread.sleep(750);
cpu2.expression.evaluate("PC = code_start");
cpu2.target.runAsynch();
Thread.sleep(1250);

/* A core can be halted again by a late debug event while the other session is
 * connecting. Reassert run once after both IPC barriers have had time to
 * complete, then detach without resetting the device. */
if (cpu1.target.isHalted()) cpu1.target.runAsynch();
if (cm.target.isHalted()) cm.target.runAsynch();
if (cpu2.target.isHalted()) cpu2.target.runAsynch();
Thread.sleep(250);

cpu1.target.disconnect();
cpu2.target.disconnect();
cm.target.disconnect();
cpu1.terminate();
cpu2.terminate();
cm.terminate();
server.stop();
System.out.println("CPU1, CM, and CPU2 resumed after flash programming.");
