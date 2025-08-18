// i Start Sequence: 
//  1) Sik: command_artec_to_cobot_scan = "[artec] cobot goto all_scan_positions"
//	2) Fanuc:
//		a) Fanuc CRX control
//		b) app fresh started
//	3) Teach pendant: 
//		a) Non attivo 
//		b) Fungo via
//		c) Ascolta il fischio  
//		d) Msg IP resettato 
//		e) Override: 20%
//	4) Artec Studio deve essere chiuso
//	5) Ricordare ad Oliver: non deve ripetere comandi se non glielo dico io!
//	6) Start Sikulix, then server, then client
//	7) On Server errors, try to continue


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
// 05)              If cobot command exists, server sends it to the client, and eventually deletes it
// 06)              Server blocking waits for "robot finished aknowledge" from client 
// 07)              If aknowledge is received, Server removes the Sikulix cobot command file
// 08)      When all user command tasks are done, Sikulix deletes the user command file
// 09)      Server blocking waits for Sikulix to delete the server command file
// 10) Server sends command aknowledge to client (which is not using it, for now)

// Come riattivare Realwear: 
// 1) Wear them, accenderle, senti i suoni di avvio, entrarci e connettersi al BT della beast (cfr doc Realwear su ArtecPC.D:\)
// my programs > my controls > bluetooth > bluetooth settings => connected devices
// 2) Thinkpad>roscore non serve rilanciarlo
// 3) Se non va forse devi rilanciare il nodo di whisper che si è staccato da BT
// 4) Launch ArtecPC>server, poi Thinkpad>Client relaunch
// 5) ThinkCentre: /home/fluently1>rosrun artec artec_node, since client is in blocking mode, ctrl-c works only if server is down
// 5b) ThinkCentre launch client: /home/fluentlyy/supsi_ws>roslaunch artec normal.launch 
// Use "terminator" terminal
// Pwd beast windows user cat 2023

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




// x Test this:
// v Vincenzo Cobot va a scatti se vado a 30%?
// v Filtraggio dei messaggi non pertinenti: Gestire cobot, start // Gestire Kobot // Robot invece che Cobot / try scan abbreviato
// v Say robot e non cobot  
// v Sik sik non deve perdersi alcun comando (workaround: il server dopo un tot di tentativi di aspettare il suo feedback, esce)
// v Server protocol deve scrivere a quale client e collegato, il mio o quello di Ale
// v Server logFileWrite deve poter accettare stringhe infinite
// v Sik state_change is correct?
// v Sik write a protocol, in caso spariscano ancora dei cmd files
// v Server e Client add time to log file
// v Sik non ha visto ARTEC Start cosa è arrivato? Fixed lower case
// v Server Artex non accettato => Va bene cosi, "artec" va ripetuto dall'utente
// v Sik State.recording_is_done: "Artec Stop", # ["artec tools"]
// v Scanner invece di Artec: scanner start
// v arpec start / artax start / artec, start / etc
// v Sik ui params deve essere top responsive
// v Server error riga 355 remove file, now used a variable 

// i Demo con Oliver, Server in errore ma non è andato giu, era in pause, e poi ha potuto continuare
// i How to launch the client on ThinkCentre (first run the server): $roslaunch artec normal.launch // For killing it, close the server, then do ctrl-c

// Do NOW:
// x Vincenzo: write only twice the finished msg

// x Com Il primo "goto topscanposition finished" si perde, quindi Vincenzo ne deve fare due: why?
// x "scanning functionality" se viene detto due volte, il server invece che leggere [artec] cobot goto top_scan_position.txt, legge il feedback.txt
// x Sik Next command deve essere available solo quando il feedback finished è arrivato, non prima
// x Velocità di scan at beginning 30% va a scatti?
// x Sik test scan breve
// x Sik close another opened artec studio
// x Backup of server client sikulix
// x Write pairing how to do it 
// x Sik SHUTDOWN sikulix does not remove roscomand file, and server hangs on it?
// x Server 0) Empty the cmd folder: Sik o server, not both 1) Chi cancella i cmd files? See fs::remove 2) non dovrebbe andare in ascolto se non ha ancora ricevuto il feedback, altrimenti ascolta cobot scan, lo manda, e sik lo cancella
// x Client problem at 0:45: Server receives from client "finishedcobot scan" quando il client legge i comandi da dir e lo fa velocemente
// x Check if files are lost, if yes, Server has to write unique file names from 00 to 99
// x Server client fix ver with client ip nr

// x Install java 8+, Jython 2.71, 
//    https://sikulix.github.io/docs/scripts/python/ how to setup PyCharm, using other IDE such as IntelliJ PyCharm and IDEA or Eclipse
// x Sik on PyCharm
// Clear the buffer before use in c++????
// x Sik command_do legge il comando ma permette solo transizioni definite prima di lanciarlo, es se cobot in scan, cobot start fa un macello
// x Sik unify commands_list con methods_list in a set
// x Sik         #artec_studio_opened_close()
// x Sik command_cobot_start: handle Return error from cobot
// x Vincenzo: 1) Prima di lanciare i comandi di Sikulix, verificare di essere nella posizione giusta, altrimenti return error, 2) Return error
// x Vincenzo teach pendant error con ROS va risolto: come vuole provarci? help from Fanuc?
// x Vincenzo 1) Sik verify che sia partita la rec, 2) Server send robot command al momento giusto, 3) check la sua override speed acquisendo con Studio
// x Sik command_remove non dovrebbe chiamarsi command_, visto che solo i veri comandi vocali si chiamano cosi. Idem x diverse altre funzioni
// x Tommaso add program "scan top reduced", che torna a casa subito
// x Vincenzo add program "scan top reduced"
// x Tommaso: riportare il programma sotto ai valori normali, chiudere il giro con una posizione compatibile con la start position
// x Tommaso: sottosquadra: 1) override < 15%, 2) fill them, 3) new positions, fillare alcune pale, time-out problem dopo 50 frames
// x Tommaso: 3) Hole filling 500 è troppo bisogna provare 50 (mm), 4) Smoothing ripetuto quante volte?
// x Tommaso: fare tutto il giro del fumo
// x Sik Add click window yes/no global geometry on global registration
// x Sik remove all 4 orphan string delimiters
// x Sik typing error that Sikulix does 1 time each 30 approx, during writing of Artec Studio launch path, can be removed?
// x Sik Write cobot command into a file, and not into the name of the file
// x Sik add stop recording at the end of the scan, when program flashes
// x Sik read command from file contents, and not from file name
// x Sik automation: 1) Test enable real time fusion flag, 2) Check enable real-time processing (should be on), 3) Hole filling 500, 4) Smoothing ripetuto quante volte?
// x Docu https://graphviz.org/Gallery/directed/fsm.html 
// x Server remove il remove dei caratteri: quello che mi arriva dal client, torna indietro al client identico
// x copy ROS commands here / >rostopic echo /arte/state
// x Server limita la lunghezza del file name, o meglio ancora, non scrivere comandi assurdi a sikulix
// x Sik dopo il cmd close, il preview non riparte poiche' non cancella il cmd: deve finisce sikulix?
// x Sik check preview > stop > preview 
// x Sik stop scanning
// x Sik #artec_studio_opened_close()
// x Sik feedback visivo dei commands quando user chiede help:     def command_help(self):
// x Sik Use camtasia per visualizzare all'utente la distanza che lo scanner ha avuto durante la scansione, senza dover far rifare al robot lo scan path
// x Sik https://sikulix-2014.readthedocs.io/en/latest/region.html#find-more-than-one-image-in-a-region-at-the-same-time
// x Sik comando fuori contesto:
//       crea una macchina a stati in jython per dire all'utente quali comandi ci si puo' aspettare, e quali rifiutare
//       Use case 1) Ad Artec Studio off lo user dice record invece che preview: dirgli "comando fuori contesto", o "prima ordinare preview"
//       Use case 2) User dice record quando l'ha gia' detto: dirgli che il comando è già in funzione
//       Use case 3) Il comando non è ancora disponibile, es chiedere fast fusion quando la global registration non è ancora terminata: dirgli di attendere
//       Use case 3) Va sempre bene dire shut down: non dirgli niente, e farlo
//       Stati: StudioOff (preview), ScannerOn(record, start, stop), FramesAcquired(tools), non puoi ripetere un comando che è già in atto
// Sik is there a pop up time based, to show msg without the need to click on ok?
// x Firewall 1) insert rules per firewall, per poter riattivare il wifi
// x Firewall 2) come diavolo faceva la wifi del beast a girare i pacchetti al thinkpad via rete supsi? ROS1 manda su 11311, mentre ROS2 manda su tutti i canali disponibili!!
// x Sik insert command robot move, fare una scansione completa, con il robot in azione
// x Sik close_opened_Artec_Studio deve gestire il caso in cui Studio è aperta e maximized e il caso in cui e' aperta ma minimized: esiste l'operatore or?
// x Sik Java works, Jython not: print("Function name:", my_function.__name__)    
// x Client copy logFile methods to client
// x Github setup, and move these parts over there
// x Sik test close multiple Artec instances: solve the problem that when clicking Artec Studio is not maximized, but two small Artec Studios are shown
// x Server Use non blocking client server, since feedback has to be sent
// x Server logfile functionality has to become a separate class, used by server and client
// x Fissare il mio profilo su https://www.supsi.ch/en/michele-banfi: manca foto, competenze, anno d'entrata, papers ecc

// i Sikulix: how to kill Sikulix (At run, I get info "Terminating IDE already running"): app is either javaPlatformLib or javaw in the process tab.
// i Sikulix: the "Orphan string delimiter" error shown at save sometime disappear by themself, seems to be created by doing cut and paste
// i I had to setup an external USBToLAN device, then re-run the net trick for removing the second IP, tied to DHCP somehow
// i I could not use both LAN adaptors, since I could not remove the second IP, which was randomly set, and the adapter used as first IP!

// Attention: MB added _WINSOCK_DEPRECATED_NO_WARNINGS to the Preprocessor Definitions, to be removed if this code goes into production

// MB convention: in methods naming, write the object name first, then the verb, resulting in to objectVerb: workspaceSetup and not setupWorkspace
//                It is more easy to find CmdFileCreate, CmdFileDelete, CmdFileRefresh than CreateCmdFile, DeleteCmdFile, RefreshCmdFile

// Define this before including any headers, for suppressing messages against the (legal!) use of localtime

#define _CRT_SECURE_NO_WARNINGS 1 // for suppressing warning of the so called "deprecated" localtime

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <cstring>
#include <winsock2.h>
#include <Ws2tcpip.h> // Include this header for inet_ntop
#include <windows.h>
#include <set>
#include <cctype> // For tolower function

#pragma comment(lib, "ws2_32.lib")
namespace fs = std::filesystem; 

// Customisation
const int TIME_WAIT_FOR_CHECK_AKNOWLEDGE = 1000; // ms debug 1000, release 50, if extremely short log file will be bigger
std::string commandFileName = "ROSClientCommand.txt";
std::string feedbackFileName = "ROSClientFeedback.txt";
std::string cmdForSikulixPath = "D:/Banfi/cmd/cmdForSikulix/";
std::string cmdFromSikulixPath = "D:/Banfi/cmd/cmdFromSikulix/";
std::string logDirPath = "D:/Banfi/codeUtility/TCPServer/logs/TCPServer_";
const char* addressIP = "192.168.1.142"; // Setup the server IP only, since the client IP has not to be known to the server
const int PORT = 22;// 12345;
const int BUFFER_SIZE = 1024;

// Global variables
// Create an empty set of strings
std::set<std::string> cmdSetGlob;
std::unique_ptr<std::ofstream> logFilePtrGlob;

void cmdFileCreate(const std::string filename);

void doExit();

void logFileClose();
int logFileCreate(std::string IPNr);
int logFileRedirect(std::unique_ptr<std::ofstream> logFilePtr);
void logFileWrite(const std::string msg);
