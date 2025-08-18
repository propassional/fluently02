// .gitignore somehow looses some settings, do this to fix build errors: 
// VS > project > properties > C++ > Language > Select ISO20 then rebuild

// Attention: MB added _WINSOCK_DEPRECATED_NO_WARNINGS to the Preprocessor Definitions, to be removed if this code goes into production
#define _CRT_SECURE_NO_WARNINGS 1 // for suppressing warning of the so called "deprecated" localtime
#define _WINSOCK_DEPRECATED_NO_WARNINGS // for suppressing warning of inet_addr

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

#include "..//..//Logger/Logger/Logger.hpp"

// Customisation
std::string logDirPath = "D:/Banfi/Github/fluently/Code/TCPServer/logs/TCPServer_";
const int TIME_WAIT_FOR_CHECK_AKNOWLEDGE = 1000; // ms debug 1000, release 50, if extremely short log file will be bigger
std::string commandFileName = "ROSClientCommand.txt";
std::string feedbapingckFileName = "ROSClientFeedback.txt";
std::string cmdForSikulixPath = "D:/Banfi/Github/Fluently/Commands/cmdForSikulix/"; // "D: / Banfi / cmd / cmdForSikulix / ";
std::string cmdFromSikulixPath = "D:/Banfi/Github/Fluently/Commands/cmdFromSikulix/"; // "D:/Banfi/cmd/cmdFromSikulix/";

//std::string cmdForSikulixPath = "D:/Banfi/Github/fluently/Commands/cmdForSikulix/"; // "D: / Banfi / cmd / cmdForSikulix / ";
//std::string cmdFromSikulixPath = "D:/Banfi/Github/fluently/Commands/cmdFromSikulix/"; // "D:/Banfi/cmd/cmdFromSikulix/";

// Setup the server IP only, since the client IP has not to be known to the server
// Works with server IP, with windows defender "on" on all 3 levels: "10.11.31.135"; // Old IP:"192.168.1.142"; 
const char* addressIP = "10.11.31.153"; // Until 24.4.24: "10.11.31.135"; // It does not work: "127.0.0.1" nor "localhost"; 
const int PORT = 22;// 12345;
const int BUFFER_SIZE = 1024;

// Global variables
// Create an empty set of strings
Logger logger(logDirPath);

std::set<std::string> cmdSetGlob;
std::unique_ptr<std::ofstream> logFilePtrGlob;

void cmdFileCreate(const std::string filename);

void doExit();
