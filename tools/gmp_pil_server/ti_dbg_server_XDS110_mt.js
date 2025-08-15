/*********************************************************************
 * File: ccs_tcp_server_final_v3.js
 * Function: A CCS scripting server that runs in a background thread.
 * - All console messages are in English.
 * - Fixes the ReferenceError on client connection.
 * - Removes the incompatible isBusy() check.
 * - Adds input trimming to handle different client line endings.
 * Author: Javnson
 * Date: 2025-08-14
 *********************************************************************/

// Import necessary Java packages for networking and threading
importPackage(Packages.java.net);
importPackage(Packages.java.io);
importPackage(Packages.java.lang);

// command
// + Read memory
//   R < address > <size> -> R <address> <content>
//   example: R 0x00000500 2 -> R 500 0001 0001
// + Write memory
//   W < address > <size> <content> -> W <address> <content>
//   example: W 0x00000500 1 0001 (hex value)
//            W 0x00000500 2 0001 0001 -> W 500 0001 0001
// + C -> C <address> # current PC pointer

// TODO add more commands
// + Read register
//   RR <REG Name> -> RR <REG Nmae> <content>
// + Write register
//   WR <REG Nmae> <content> -> WR <REG Nmae> <content>
// + Read Flash
//   RF <address> <length>

// --- USER CONFIGURATION ---
var PORT = 19990;
var HOST = "127.0.0.1";

// [IMPORTANT!] You must fill this in with the exact, full session name
// from the CCS Debug view.
var EXACT_SESSION_NAME = "Texas Instruments XDS110 USB Debug Probe_0/C28xx_CPU1";
// --------------------------

var session = null;
try {
    var script = ScriptingEnvironment.instance();
    var debugServer = script.getServer("DebugServer.1");
    session = debugServer.openSession(EXACT_SESSION_NAME);

} catch (e) {
    print("--> [ERROR] Failed to get debug session: " + e.message);
    print("--> Please ensure a debug session is active and EXACT_SESSION_NAME is correct.");
}

if (session != null) {
    print("--> Successfully attached to debug session: " + session.getName());

    var serverLogic = new Runnable({
        run: function () {
            var serverSocket = null;
            var clientSocket = null;
            var out = null;
            var inReader = null;
            var wasHalted = session.target.isHalted();

            try {
                serverSocket = new ServerSocket(PORT);
                print("--> TCP Server started on " + HOST + ":" + PORT);
                print("--> Waiting for a client to connect...");

                clientSocket = serverSocket.accept();
                print("--> Client connected from: " + clientSocket.getInetAddress().getHostAddress());

                out = new PrintWriter(clientSocket.getOutputStream(), true);
                inReader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));

                var inputLine;

                while (true) {
                    var isHalted = session.target.isHalted();
                    if (isHalted && !wasHalted) {
                        var pc = session.registers.read("PC");
                        var breakMessage = "B " + pc.toString(16).toUpperCase();
                        print("--> Target halted at 0x" + pc.toString(16).toUpperCase() + ". Notifying client...");
                        out.println(breakMessage);
                    }
                    wasHalted = isHalted;

                    if (inReader.ready()) {
                        inputLine = inReader.readLine();
                        if (inputLine == null) break;

                        // ======================== CORRECTION ========================
                        // Trim the input line to remove leading/trailing whitespace and control chars like '\r'
                        var trimmedLine = inputLine.trim();
                        if (trimmedLine.length() == 0) continue; // Skip empty lines
                        // ==========================================================

                        print("--> Received command: " + trimmedLine);
                        var parts = trimmedLine.split(" ");
                        var command = parts[0].toUpperCase();
                        var response = "ERROR: Unknown command";

                        try {
                            //command = "R";
                            if (command == "R") {
                                var addr = parseInt(parts[1], 16);
                                var size = parseInt(parts[2]);
                                var data = session.memory.readData(0, addr, 16, size);
                                var content = Array.prototype.slice.call(data).map(function (d) {
                                    return ("0000" + (d & 0xFFFF).toString(16)).slice(-4);
                                }).join(" ");
                                response = "R " + addr.toString(16).toUpperCase() + " " + content.toUpperCase();
                            }

                            else if (command == "W") {

                                var addr = parseInt(parts[1], 16);
                                var size = parseInt(parts[2]);
                                var values = [];
                                for (var i = 0; i < size; i++) {
                                    values.push(parseInt(parts[3 + i], 16));
                                }
                                session.memory.writeData(0, addr, values, 16);
                                var content = parts.slice(3).join(" ").toUpperCase();
                                response = "W " + addr.toString(16).toUpperCase() + " " + content;
                            }

                            else if (command == "C") {
                                var pc = session.registers.read("TBCTL");
                                session.target.run();
                                response = "C " + pc.toString(16).toUpperCase();
                            }

                            else {

                                // This default case was being hit incorrectly before
                                response = "ERROR: Invalid command '" + command + "'";
                            }
                        } catch (e) {
                            response = "ERROR: " + e.message;
                        }

                        print("--> Sending response: " + response);
                        out.println(response);
                    }

                    // milliseconds
                    Thread.sleep(10);
                }

            } catch (e) {
                print("--> [ERROR] An error occurred in the server thread: " + e);
            } finally {
                print("--> Server is shutting down...");
                if (inReader != null) inReader.close();
                if (out != null) out.close();
                if (clientSocket != null) clientSocket.close();
                if (serverSocket != null) serverSocket.close();
            }
        }
    });

    var serverThread = new Thread(serverLogic);
    serverThread.start();

    print("--> Server thread has been started.");
}