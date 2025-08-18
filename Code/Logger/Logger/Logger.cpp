#include "Logger.hpp"

Logger::Logger(const std::string& logDir) : logDirPath(logDir) {}

void Logger::logFileClose() {
    if (logFilePtr && logFilePtr->is_open()) {
        logFilePtr->close();
    }
}

int Logger::logFileCreate() {
    // Get the current time
    std::time_t currentTime = std::time(nullptr);
    struct std::tm* timeInfo = std::localtime(&currentTime);

    // Create a filename based on the current date and time
    std::ostringstream logFilename;
    logFilename << std::put_time(timeInfo, "%Y_%m_%d__%H_%M_%S") << ".txt";
    std::string logFilenameComplete = logDirPath + logFilename.str();

    // Create and open the file
    logFilePtr = std::make_unique<std::ofstream>(logFilenameComplete);

    // Check if the file was successfully created
    if (logFilePtr->is_open()) {
        logFileWrite("Logger: file '" + logFilenameComplete + "' created successfully");
    }
    else {
        logFileWrite("Logger: error not able to create the log file");
        return -1;
    }

    return 0;
}

void Logger::logFileWrite(const std::string& msg) {
    char msg_char[3000];

    // Clear the buffer before use
    std::memset(msg_char, 0, sizeof(msg_char));

    std::time_t now = std::time(nullptr);
    std::tm timeinfo = *std::localtime(&now);

    // Create a string with the desired filename format
    char timeString[50];
    std::strftime(timeString, sizeof(timeString), "%Y_%m_%d__%H_%M_%S: ", &timeinfo);

    sprintf(msg_char, timeString);

    // Write the message to the log file
    sprintf(msg_char + strlen(msg_char), msg.c_str());
    // Add \n
    sprintf(msg_char + strlen(msg_char), "\n");
    // Write the message to cout
    std::cout << msg_char << std::endl;

    logFilePtr->write(msg_char, strlen(msg_char));
    logFilePtr->flush();
}

//int main() {
//    std::string logDirPath = "D:/temp/TCPClient_";
//    Logger logger(logDirPath);
//    logger.logFileCreate();
//    logger.logFileWrite("Client: log file created \n");
//    logger.logFileClose();
//    return 0;
//}