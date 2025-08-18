// MB convention: in methods naming, write the object name first, then the verb, resulting in to objectVerb: workspaceSetup and not setupWorkspace
//                It is more easy to find CmdFileCreate, CmdFileDelete, CmdFileRefresh than CreateCmdFile, DeleteCmdFile, RefreshCmdFile

// i Start Sequence: 
//  1) Sik: command_artec_to_cobot_scan = "[artec] cobot goto all_scan_positions"
//	2) Fanuc:
//		a) Restart Fanuc CRX control
//		b) Restart Fanuc app
//	3) Teach pendant: 
//		a) Non attivo 
//		b) Fungo via
//		c) Ascolta il fischio  
//		d) Msg IP resettato 
//		e) Override: 40%
//	4) Artec Studio deve essere chiuso
//	5) Ricordare ad Oliver: non deve ripetere comandi se non glielo dico io!
//	6) Start Sikulix, then server, then client
//	7) On Server errors, try to continue

// - Oliver: dire che il PC parla con Realwear and the cobot, and controls artec studio

// Commands in order:
// "scanner start" => Open Artec Studio
// "scanning functionality" => position scanner
// "scanning procedure" => record frames
// "scanner stop" => Stop scanning
// "scanner save", 
// "scanner open result" => Open processed scanning

// Commands not shown:
// "scanner tools", 
// "scanner global registration", 
// "scanner fast fusion" + Mesh simplification + Object filter for removing the fixturing + Small hole filling


// Server Concept 1.0: (Server & Sikulix support client commands)
// "The one who orders, writes the command file. The one who executes it, deletes it"
// Server receives a command from client
//      Server sends command to Sikulix: Server writes command file into "D:/Banfi/cmd/cmdForSikulix/"
//      Server waits for Sikulix feedback: when done, Sikulix deletes the Server command file
// Server sends command aknowledge to client

// Server Concept 2.0 (Server & Sikulix also support cobot commands):
// "The one who orders, writes the command file. The one who executes it, deletes it"
// 01) Server receives the user command from the client (e.g. "Cobot start")
// 02)      Server "sends" the user command to Sikulix: the user command file is written into the directory "D:\Banfi\cmd\commandForSikulix"
// 03)          If the user command involves the cobot, Sikulix writes a cobot command into the directory "D:\Banfi\cmd\cmdFromSikulix" and blocking waits for cobot aknowledge
// 04)          Server checks the directory "D:\Banfi\cmd\cmdFromSikulix", and reads the cobot command from Sikulix
// 05)              If cobot command exists, server sends it to the client, and then deletes it
// 06)              Server blocking waits for "cobot finished aknowledge" from client 
// 07)              When cobot aknowledge is received, Server writes it to Sikulix
// 08)      When all user command tasks are done, Sikulix deletes the user command file
// 09)      Server blocking waits for Sikulix to delete the server command file
// 10) Server sends command aknowledge to client (which is not using it, for now)

// OA: Meeting del 19.9 con me
//     Per facilitare il controllo della cella e non confondere l'utente, io devo esporre solo i comandi necessari, accorpando i comandi singoli
//     Ad esempio, invece di esporre artec open, poi artec scan, poi artec preview, espongo solo artec preview che fa le 3 cose insieme
//     All'utente non serve separare i movimenti del robot dalla funzionalità di scanning, lo user non ha accesso ai movimenti del robot
//     Lui vuole ottenere una scansione completa e chiusa, quindi non ha senso per l'utente separare top centre da top peripheral, o da bottom peripheral o bottom centre
//     Global registration e fast fusion: farglieli dire, ma prima consigliandoglieli

// i Robot: Status > Current position > Open old teach pendant, li dentro menu > 0 next > 6 system type > config > scroll down to line 42 > remote local setup > choice > remote (PC) vs Soft local OP Panel
// i Robot: spline si ferma, mentre joint no
// i Sik Do demo con Sik ver 15 + Server + Client
// i Window explorer search for two strings: search within files + string1 AND string2
// i Notepad search for two strings: regular expression + Find all in current doc + (string1).*(string2)
// i Robot Si è spento da solo 26.09.23 ore 16:15 circa, restart control fixed it
// i Sikulix: how to kill Sikulix (At run, I get info "Terminating IDE already running"): app is either javaPlatformLib or javaw in the process tab.
// i Sikulix: the "Orphan string delimiter" error shown at save sometime disappear by themself, seems to be created by doing cut and paste
// i I had to setup an external USBToLAN device, then re-run the net trick for removing the second IP, tied to DHCP somehow
// i I could not use both LAN adaptors, since I could not remove the second IP, which was randomly set, and the adapter used as first IP!
