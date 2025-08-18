// .gitignore somehow looses some settings, do this to fix build errors: 
// VS > project > properties > C++ > Language > Select ISO20 then rebuild

#define _CRT_SECURE_NO_WARNINGS 1 // for suppressing warning of the so called "deprecated" localtime
#define _WINSOCK_DEPRECATED_NO_WARNINGS // for suppressing warning of inet_addr

#include <iostream>
#include <string>
#include <cstring>
#include <winsock2.h>
#include <fstream>
#include <filesystem> // Original
//#include <experimental/filesystem>
//#include <boost/filesystem.hpp>

#pragma comment(lib, "ws2_32.lib")
//namespace fs = std::filesystem;
//namespace fs = boost::filesystem;

#include "..//..//Logger/Logger/Logger.hpp"

// Customisation
std::string logDirPath = "D:/Banfi/Github/fluently/Code/TCPClient/logs/TCPClient_";
std::string cmdDirPath = "D:/Banfi/Github/fluently/Commands/cmdForClient/";
const int PORT = 22; //  12345;
const int BUFFER_SIZE = 1024;
const int WAIT_FOR_CMD_FILE = 6000; // Has influence only on simulation

// Global variables
Logger logger(logDirPath);