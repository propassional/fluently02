#ifndef LOGGER_HPP
#define LOGGER_HPP

#define _CRT_SECURE_NO_WARNINGS 1 // for suppressing warning of the so called "deprecated" localtime

#include <string>
#include <memory>
#include <iostream>
#include <string>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <memory>
#include <ctime>
#include <sstream>
#include <iomanip>

class Logger {
private:
    std::string logDirPath;
    std::unique_ptr<std::ofstream> logFilePtr;

public:
    Logger(const std::string& logDir);
    void logFileClose();
    int logFileCreate();
    void logFileWrite(const std::string& msg);
};

#endif