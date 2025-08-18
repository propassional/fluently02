//#include "LogFile.hpp"

#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <string>
#include <cstring>
#include <winsock2.h>
#include <fstream>
#include <filesystem> 

#pragma comment(lib, "ws2_32.lib")

std::string logDirPath = "D:/Banfi/codeUtility/TCPClient/logs/TCPClient_";
std::unique_ptr<std::ofstream> logFilePtrGlob;

// Add a \n to the end of msg, then writes it to the log file
void logFileWrite(const std::string msg)
{
    char msg_char[3000];

    // Clear the buffer before use
    std::memset(msg_char, 0, sizeof(msg_char));

    // Add the time
    //std::time_t currentTime = std::time(nullptr);
    //struct std::tm* timeInfo = std::localtime(&currentTime);
    //// Format the time as a string (precise to the second)
    //char timeString[20]; // Buffer to hold the formatted time
    //std::strftime(timeString, sizeof(timeString), "%Y_%m_%d__%H_%M_%S: ", timeInfo);

    std::time_t now = std::time(nullptr);
    std::tm timeinfo = *std::localtime(&now);

    // Create a string with the desired filename format
    char timeString[50];
    std::strftime(timeString, sizeof(timeString), "%Y_%m_%d__%H_%M_%S: ", &timeinfo);

    sprintf(msg_char, timeString);

    // Write in msg_char message to logfile
    sprintf(msg_char + strlen(msg_char), msg.c_str());
    // Add \n
    sprintf(msg_char + strlen(msg_char), "\n");
    // Write message to cout
    std::cout << msg_char << std::endl;

    //const char* msg_char = msg.c_str();
    logFilePtrGlob->write(msg_char, strlen(msg_char));
    logFilePtrGlob->flush();
}

// This solution is not elegant: I tried to redirect cout to a file, which is simpler, but redirection made deterministically tcp/ip crash!
int logFileCreate()
{
    // Get the current time
    std::time_t currentTime = std::time(nullptr);
    struct std::tm* timeInfo = std::localtime(&currentTime);

    // Create a filename based on the current date and time
    std::ostringstream logFilename;
    logFilename << std::put_time(timeInfo, "%Y_%m_%d__%H_%M_%S") << ".txt";
    std::string logFilenameComplete = logDirPath + logFilename.str();

    // Create and open the file
    logFilePtrGlob = std::make_unique<std::ofstream>(logFilenameComplete);

    // Check if the file was successfully created
    if (logFilePtrGlob->is_open()) {
        logFileWrite("Server: file '" + logFilenameComplete + "' created successfully. ");
    }
    else {
        logFileWrite("Server: error unable to create the file.");
        return -1;
    }

    return NULL;
}

int main() {

    logFileCreate();

    logFileWrite("Client: error initializing WinSock \n");
    
    return 0;
}