#include "TCPServer.hpp"

void cmdFileCreate(const std::string fileName, const std::string command)
{
    // Create a file stream object
    std::ofstream cmdFile;

    // Open the file in write mode (this will create the file if it doesn't exist)
    cmdFile.open(fileName);

    // Check if the file was successfully opened
    if (!cmdFile.is_open()) {
        logFileWrite(std::to_string(__LINE__) + " Server error: failed to create the file: " + fileName + "\n"); // cerr
        doExit(); 
    }
    else
    {
        cmdFile << command;
        cmdFile.close();
    }
}

void doExit()
{
    logFileWrite(std::to_string(__LINE__) + " Server: exiting server app\n");
    logFileClose();
    exit(1);
}

void logFileClose()
{
    logFileWrite(std::to_string(__LINE__) + " Server: flushing and closing server log file\n");
    logFilePtrGlob->flush();
    logFilePtrGlob->close();
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
        logFileWrite(std::to_string(__LINE__) + " Server: file created " + logFilenameComplete);
    }
    else {
        logFileWrite(std::to_string(__LINE__) + " Server: error unable to create the file.");
        return -1;
    }

    return NULL;
}

int logFileRedirect(std::unique_ptr<std::ofstream> logFilePtr)
{
    // Save the current standard output and error streams
    std::streambuf* coutStreambuf = std::cout.rdbuf();
    std::streambuf* cerrStreambuf = std::cerr.rdbuf();

    // Redirect std::cout and std::cerr to the log file
    std::cout.rdbuf(logFilePtr->rdbuf());
    std::cerr.rdbuf(logFilePtr->rdbuf());

    // Now, anything written to std::cout and std::cerr will be logged in the file

    // Restore the original standard output and error streams
    //std::cout.rdbuf(coutStreambuf);
    //std::cerr.rdbuf(cerrStreambuf);

    return 0;
}

void logFileWrite(const std::string msg)
{
    char msg_char[3000]; 

    // Write message to cout
    std::cout << msg << std::endl;

    // Add the time
    std::time_t currentTime = std::time(nullptr);
    struct std::tm* timeInfo = std::localtime(&currentTime);
    // Format the time as a string (precise to the second)
    char timeString[20]; // Buffer to hold the formatted time
    std::strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", timeInfo);
    sprintf(msg_char, timeString);

    // Write message to logfile
    sprintf(msg_char, msg.c_str());
    //const char* msg_char = msg.c_str();
    logFilePtrGlob->write(msg_char, strlen(msg_char));
    logFilePtrGlob->flush();
}

std::string stringRemoveBadCharacters(std::string stringIn)
{
    std::string stringFiltered;

    // Remove from filename all unacceptable characters:
    // A file name can't contain any of the following characters
    const char* unacceptableChars = "\\/:*?\"<>|¦";
    for (char ch : stringIn) {
        // Check if the character is unacceptable
        if (std::strchr(unacceptableChars, ch) == nullptr) {
            // If not unacceptable, add it to the result
            stringFiltered += ch;
        }
    }

    return stringFiltered;
}

std::string stringRemoveSpaces(std::string input)
{
    std::string result;

    // Find the position of the first non-space character
    size_t start = input.find_first_not_of(" \t");

    // Find the position of the last non-space character
    size_t end = input.find_last_not_of(" \t");

    if (start != std::string::npos && end != std::string::npos) {
        // Extract the substring without leading and trailing spaces
        result = input.substr(start, end - start + 1);
        std::cout << "Original: '" << input << "'" << std::endl;
        std::cout << "Trimmed: '" << result << "'" << std::endl;
    }
    else {
        // The string is either all spaces or empty
        std::cout << "The string is empty or contains only spaces." << std::endl;
    }

    return result;
}

// Version with pointers
int workspaceSetup()
{
    static bool setupWorkspaceDone = false;

    // Insert elements into the command set
    cmdSetGlob.insert("preview");
    cmdSetGlob.insert("record");
    cmdSetGlob.insert("pause");
    cmdSetGlob.insert("stop");
    cmdSetGlob.insert("tools");
    cmdSetGlob.insert("global registration");
    cmdSetGlob.insert("fast fusion");
    cmdSetGlob.insert("save");
    cmdSetGlob.insert("close");
    cmdSetGlob.insert("shutdown");

    if (!setupWorkspaceDone)
    {
        int res = logFileCreate();
    }
    else
        logFileWrite(std::to_string(__LINE__) + " Server: workspace has already been setup previously\n");

    return 0;
}

int main() {

    workspaceSetup();

    // Check which windows socket implementation is needed
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        logFileWrite(std::to_string(__LINE__) + " Server: error initializing WinSock.\n");
        return 1;
    }

    // Create socket, blocking
    //SOCKET serverSocket = socket(AF_INET, SOCK_STREAM | FIONBIO, 0); // Not blocking seems not to work
    SOCKET serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == INVALID_SOCKET) {
        logFileWrite(std::to_string(__LINE__) + " Server: error creating socket.\n");
        return 1;
    }

    // Bind to a port
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    
    // Replace "INADDR_ANY" with the IP address you want to bind to
    // For example: "127.0.0.1" for localhost
    //serverAddress.sin_addr.s_addr = INADDR_ANY;
    //serverAddress.sin_addr.s_addr = inet_addr("192.168.1.131"); // Indirizzo del PC sul quale gira il server, e al quale il client 192.168.1.130 Fluently1_Ubuntu si rivolge
    serverAddress.sin_addr.s_addr = inet_addr(addressIP); // Indirizzo del PC sul quale gira il server, e al quale il client 192.168.1.130 Fluently1_Ubuntu si rivolge
    serverAddress.sin_port = htons(PORT);

    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) {
        logFileWrite(std::to_string(__LINE__) + " Server: error binding to port.\n");
        return 1;
    }

    // Listen for connections
    if (listen(serverSocket, 5) == SOCKET_ERROR) {
        logFileWrite(std::to_string(__LINE__) + " Server: error listening for connections.\n");
        return 1;
    }

    logFileWrite(std::to_string(__LINE__) + " Server: listening on port 22 \n"); //  +PORT);
    
    SOCKET clientSocket = accept(serverSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET) {
        logFileWrite(std::to_string(__LINE__) + " Server: error accepting client connection.\n");
    }

    // Empty the cmd folder
    for (const auto& entry : fs::directory_iterator(cmdForSikulixPath)) {
        if (entry.is_regular_file()) {
            // Check if the file has a specific extension (e.g., ".txt")
            if (entry.path().extension() == ".txt") {
                // Rename the file by adding "_ok" as a postfix
                //fs::path newPath = entry.path().parent_path() / (entry.path().stem().string() + "_ok" + entry.path().extension().string());
                //fs::rename(entry.path(), newPath);
                logFileWrite(std::to_string(__LINE__) + " Server: remove file " + entry.path().string() + "\n");
                fs::remove(entry.path());
            }
        }
    }

    while (true) {
        bool cmdFileFound = true;
        char buffer[BUFFER_SIZE];

        // Receive data from the client
        logFileWrite(std::to_string(__LINE__) + " Server: start receive data, blocking: \n");
        memset(buffer, 0, BUFFER_SIZE);
        // P10r
        // RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV
        int bytesRead = recv(clientSocket, buffer, BUFFER_SIZE, 0);
        if (bytesRead <= 0) {
            logFileWrite(std::to_string(__LINE__) + " Server: error receiving data from client.\n");
            //doExit();
        }
        std::string clientMessage(buffer);
        logFileWrite(std::to_string(__LINE__) + " Server: received message: " + clientMessage + "\n");

        // P20
        // Write the received client command into a text file
        // Convert into lower case
        for (char& c : clientMessage) {
            c = std::tolower(c);
        }
        // File name is constant, only the file contents change
        std::string cmdFileName = cmdForSikulixPath + commandFileName;
        logFileWrite(std::to_string(__LINE__) + " Server: saved cmd file to: " + cmdFileName + "\n");
        logFileWrite(std::to_string(__LINE__) + " Server: client message received: " + clientMessage + "\n");
        cmdFileCreate(cmdFileName, clientMessage);

        // Get feedback from Sikulix that the user command has been executed, blocking
        // Only "cobot" can not be accepted here, otherwise this code is executed, but Sikulix code not, and the state mismatch results into an error
        if ( (strstr(buffer, "cobot start")) || (strstr(buffer, "cobot, start")) || 
             (strstr(buffer, "cobot scan")) || (strstr(buffer, "cobot, scan"))) {

            // P21 Cobot cmd received
            // Get feedback from Sikulix that the user command has been executed, blocking
            bool cobotCmdFound = false;
            do {
                // Check Sikulix cmd acknowledge
                logFileWrite(std::to_string(__LINE__) + " Server: waiting for Sikulix to execute the cobot command \n");
                Sleep(TIME_WAIT_FOR_CHECK_AKNOWLEDGE);

                // Read Sikulix cobot cmd, send it to client
                // Check if the cmd folder FROM Sikulix is empty, if not, send the Sikulix cobot command to the client and blocking wait for feedback
                for (const auto& entry : fs::directory_iterator(cmdFromSikulixPath)) {
                    // A Sikulix cobot cmd has been found
                    cobotCmdFound = true;
                    std::string cmdFromSikulix = entry.path().stem().string(); // file Name Without Extension
                    logFileWrite(std::to_string(__LINE__) + " Server: sending Sikulix cobot command to client: " + cmdFromSikulix + "\n");
                    // SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND
                    send(clientSocket, cmdFromSikulix.c_str(), cmdFromSikulix.size(), 0);
                    logFileWrite(std::to_string(__LINE__) + " Server: removing file " + entry.path().string() + "\n");
                    fs::remove(entry.path()); // xx Sikulix has to remove the Sikulix cmd file
                }
            } while (!cobotCmdFound);

            // Wait for cobot feedback from the client
            logFileWrite(std::to_string(__LINE__) + " Server: waiting for client cobot finished acknowledge\n");
            logFileWrite(std::to_string(__LINE__) + " Server: start receive data, blocking: \n");
            memset(buffer, 0, BUFFER_SIZE);
            // RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV 
            int bytesRead = recv(clientSocket, buffer, BUFFER_SIZE, 0);
            if (bytesRead <= 0) {
                logFileWrite(std::to_string(__LINE__) + " Server: error receiving data from client\n");
                doExit();
            }
            std::string clientMessage(buffer);
            logFileWrite(std::to_string(__LINE__) + " Server: received message: " + clientMessage + "\n");

            const char* found = strstr(buffer, "finished");
            if (found)
            {
                // Write the received client cobot feedback into a text file for Sikulix
                for (char& c : clientMessage) {
                    c = std::tolower(c);
                }
                std::string cmdFileName = cmdFromSikulixPath + feedbackFileName;
                logFileWrite(std::to_string(__LINE__) + " Server: saved feedback file to: " + cmdFileName);
                cmdFileCreate(cmdFileName, clientMessage);
                // Aknowledge is not needed here, since Sikulix reads this file as aknowledge 
            }
            else
            {
                logFileWrite(std::to_string(__LINE__) + " Server: finished was expected, wrong feedback received\n ");
                //doExit();
            }

            // Wait for Sikulix to delete all files in DirCmdFromSikulix, as acknowledge
            do
            {
                // Check Sikulix cmd acknowledge
                // Check if the cmd folder FOR Sikulix is empty, which means that the last user command has been executed by Sikulix
                Sleep(TIME_WAIT_FOR_CHECK_AKNOWLEDGE); // 1) Be sure that the cmd file that server has written has been seen by OS, 2) Slow down checking for acknowledge
                cmdFileFound = false;
                for (const auto& entry : fs::directory_iterator(cmdFromSikulixPath)) {
                    // If there is at least one entry in the directory, it's not empty
                    cmdFileFound = true;
                    logFileWrite(std::to_string(__LINE__) + " Server: waiting for Sikulix to execute last command: " + clientMessage + "\n");
                }
            }while (cmdFileFound);

            // Remove cmd file "cobot start"
            logFileWrite(std::to_string(__LINE__) + " Server: Remove cmd file cobot start\n");
            fs::remove(cmdFileName);
        }
        else if ( (strstr(buffer, "artec")) || (strstr(buffer, "Artec")) )
        {
            // Artec cmd received
            do
            {
                // Check Sikulix cmd acknowledge
                // Check if the cmd folder FOR Sikulix is empty, which means that the last user command has been executed by Sikulix
                Sleep(TIME_WAIT_FOR_CHECK_AKNOWLEDGE); // 1) Be sure that the cmd file that server has written has been seen by OS, 2) Slow down checking for acknowledge
                cmdFileFound = false;
                for (const auto& entry : fs::directory_iterator(cmdForSikulixPath)) {
                    // If there is at least one entry in the directory, it's not empty
                    cmdFileFound = true;
                    logFileWrite(std::to_string(__LINE__) + " Server: waiting for Sikulix to execute last command: " + clientMessage + "\n");
                }
            }while (cmdFileFound);

            // Send aknowledge to the client for last received user command
            std::string response = buffer; // Acknowledge string is the same as the command string
            logFileWrite(std::to_string(__LINE__) + " Server: start send aknowledge msg: " + response + "\n");
            // SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND 
            send(clientSocket, response.c_str(), response.size(), 0);
        }
        else
        {
            // Error: cmd not recognized
            std::string response = "Error: cmd not recognized"; // Acknowledge string is the same as the command string
            logFileWrite(std::to_string(__LINE__) + " Server: send msg: " + response + "\n");
            // SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND 
            //send(clientSocket, response.c_str(), response.size(), 0);
//            doExit();
        }

        // xx Is this part really needed?
        if (clientMessage == "") {
            // Standard startup msg
            std::string response = "Server: Hello from server to client!";
            logFileWrite(std::to_string(__LINE__) + " Server: sending client message: " + response + "\n");
            // SEND SEND SEND
            send(clientSocket, response.c_str(), response.size(), 0);
        }
        else if (clientMessage == "artec shutdown")
        {
            logFileWrite(std::to_string(__LINE__) + " Server: received shutdown command\n");
            doExit();
        }
    }

    // MB
    closesocket(clientSocket);

    // Clean up WinSock
    WSACleanup();

    logFileClose();

    return 0;
}

