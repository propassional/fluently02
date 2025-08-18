// Version 1.0
#include "TCPServer.hpp"

void aknowledgeSikulixCheck(std::string clientMessage, std::string SikulixPath)
{
    bool cmdFileFound = false;
    do
    {
        // Check Sikulix cmd acknowledge
        // Check if the cmd folder FOR Sikulix is empty, which means that the last user command has been executed by Sikulix
        Sleep(TIME_WAIT_FOR_CHECK_AKNOWLEDGE); // 1) Be sure that the cmd file that server has written has been seen by OS, 2) Slow down checking for acknowledge
        cmdFileFound = false;
        for (const auto& entry : fs::directory_iterator(SikulixPath)) {
            // If there is at least one entry in the directory, it's not empty
            cmdFileFound = true;
            logger.logFileWrite(std::to_string(__LINE__) + " Server: waiting for Sikulix to execute last command: " + clientMessage);
        }
    } while (cmdFileFound);
}

void cmdFileCreate(const std::string fileName, const std::string command)
{
    // Create a file stream object
    std::ofstream cmdFile;

    // Open the file in write mode (this will create the file if it doesn't exist)
    cmdFile.open(fileName);

    // Check if the file was successfully opened
    if (!cmdFile.is_open()) {
        logger.logFileWrite(std::to_string(__LINE__) + " Server error: failed to create the file: " + fileName); // cerr
        std::cout << "Server: error, press Enter to exit Server program" << std::endl;
        std::cin.get();
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
    logger.logFileWrite(std::to_string(__LINE__) + " Server: exiting server app");
    logger.logFileClose();
    exit(1);
}

void stringToLowerCase(char* buffer) {
    // Convert characters to lowercase
    //for (char& c : buffer) {
    //    c = std::tolower(c);
    //}
    std::transform(buffer, buffer + strlen(buffer), buffer, ::tolower);
}

void stringRemoveComma(char* buffer) {

    // Remove all commas (',') from the buffer
    char charToRemove = ',';
    int len = std::strlen(buffer);
    int newIndex = 0; // Index for the modified character array

    for (int i = 0; i < len; i++) {
        if (buffer[i] != charToRemove) {
            buffer[newIndex] = buffer[i];
            newIndex++;
        }
    }

    // Null-terminate the modified character array
    buffer[newIndex] = '\0';
}

// Remove from string all unacceptable characters:
void stringRemoveBadCharacters(char* stringIn)
{
    // A command and a file name (file is meant without extension and not a path) can't contain any of the following characters
    //const char* unacceptableChars = ",\\/:*?\"<>|¦#$^!@&*()_+-%=."; // 
    const char unacceptableChars[] = ",\\/:*?\"<>|¦#$^!@&*()_+-%=."; // Null-terminated C string, Last added char (to be tested) is on the right
    char* pIn = stringIn;
    char* pOut = stringIn; // Use the same buffer for input and output

    while (*pIn != '\0') {
        char ch = *pIn;
        // Check if the character is unacceptable
        if (std::strchr(unacceptableChars, ch) == nullptr) {
            // If not unacceptable, copy it to the output position
            *pOut = ch;
            pOut++;
        }
        pIn++;
    }
    // Null-terminate the output string
    *pOut = '\0';
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
        int res = logger.logFileCreate();
        if (res < 0)
        {
            std::cout << "Server: logfile error, press Enter to exit Server program" << std::endl;
            std::cin.get();
            exit(-1); // Error is displayed by Logger
        }
    }
    else
        logger.logFileWrite(std::to_string(__LINE__) + " Server: workspace has already been setup previously");

    return 0;
}

int main() {

    workspaceSetup();

    // Check which windows socket implementation is needed
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        logger.logFileWrite(std::to_string(__LINE__) + " Server: error initializing WinSock");
        return 1;
    }

    // Create socket, blocking
    //SOCKET serverSocket = socket(AF_INET, SOCK_STREAM | FIONBIO, 0); // Not blocking seems not to work
    SOCKET serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == INVALID_SOCKET) {
        logger.logFileWrite(std::to_string(__LINE__) + " Server: error creating socket");
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
        logger.logFileWrite(std::to_string(__LINE__) + " Server: error binding to port");
        return 1;
    }

    // Listen for connections
    if (listen(serverSocket, 5) == SOCKET_ERROR) {
        logger.logFileWrite(std::to_string(__LINE__) + " Server: error listening for connections");
        return 1;
    }

    logger.logFileWrite(std::to_string(__LINE__) + " Server: listening on port 22"); //  +PORT);

    SOCKET clientSocket = accept(serverSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET) {
        logger.logFileWrite(std::to_string(__LINE__) + " Server: error accepting client connection");
    }

    // Precondition: at startup SMB cleans all files from both cmd folders

    while (true) {
        //bool cmdFileFound = true;
        char buffer[BUFFER_SIZE]; // This memory is not inizialized, memset does the initilisation later
        memset(buffer, 0, BUFFER_SIZE);

        // Receive data from whisper through the client
        logger.logFileWrite(std::to_string(__LINE__) + " Server: start receive data, blocking");
        // P10r
        // RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV
        // Blocking only until client is up, otherwise it is not blocking and returns 0
        int bytesRead = recv(clientSocket, buffer, BUFFER_SIZE, 0);
        if (bytesRead <= 0) {
            logger.logFileWrite(std::to_string(__LINE__) + " Server: error receiving data from client");
            break;
            //doExit();
        }

        std::string whisperMessageUnfiltered(buffer);
        logger.logFileWrite(std::to_string(__LINE__) + " Server: received whisper message: " + whisperMessageUnfiltered);
        // P20, cleanup the whisper data
        stringToLowerCase(buffer);
        stringRemoveBadCharacters(buffer);
        std::string clientMessage1(buffer);
        logger.logFileWrite(std::to_string(__LINE__) + " Server: filtered whisper message: " + clientMessage1);

        // File name is constant, only the file contents change
        std::string cmdFileName = cmdForSikulixPath + commandFileName;
        logger.logFileWrite(std::to_string(__LINE__) + " Server: saved cmd file to: " + cmdFileName);
        cmdFileCreate(cmdFileName, clientMessage1);

        // Get feedback from SMB that the user command has been executed, it is blocking
        // Only "cobot" can not be accepted here, otherwise this code is executed, but Sikulix code not, and the state mismatch results into an error
        // xx Read from dir the names of the cobot cmd's
        if ( (strstr(buffer, "scanning functionality")) || (strstr(buffer, "scanning procedure")) ) {

            // P21 Cobot cmd received
            // Get feedback from Sikulix that the user command has been executed, blocking
            bool cobotCmdFound = false;
            do {
                // Check Sikulix cmd acknowledge
                logger.logFileWrite(std::to_string(__LINE__) + " Server: waiting for Sikulix to execute the cobot command");
                Sleep(TIME_WAIT_FOR_CHECK_AKNOWLEDGE);

                // Read Sikulix cobot cmd, send it to client
                // Check if the cmd folder FROM Sikulix is empty, if not, send the Sikulix cobot command to the client and blocking wait for feedback
                for (const auto& cmdFileFromSikulix : fs::directory_iterator(cmdFromSikulixPath)) {
                    // A Sikulix cobot cmd has been found
                    if (!cobotCmdFound) {
                        cobotCmdFound = true;
                        fs::path cmdFileFromSikulixPath = cmdFileFromSikulix.path();
                        std::string cmdFromSikulix = cmdFileFromSikulix.path().stem().string(); // file Name Without Extension
                        logger.logFileWrite(std::to_string(__LINE__) + " Server: sending Sikulix cobot command to client: " + cmdFromSikulix);
                        // SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND
                        send(clientSocket, cmdFromSikulix.c_str(), cmdFromSikulix.size(), 0);
                        logger.logFileWrite(std::to_string(__LINE__) + " Server: removing file " + cmdFileFromSikulix.path().string());
                        // Server removes the Sikulix cmd here, there is no try since if it does not work I must know it
                        // During the demo with Oliver I got a crash here, but had no time to understand why, so now I use a variable
                        //fs::remove(cmdFileFromSikulix.path()); // Error was here
                        fs::remove(cmdFileFromSikulixPath); 
                    }
                    else
                    {
                        // Other cobot cmd files are within this folder: it's an error, thus stop everything NOW
                        logger.logFileWrite(std::to_string(__LINE__) + " Server: error multiple files in " + cmdFromSikulixPath);
                        std::cout << "Server: logfile error, press Enter to exit Server program" << std::endl;
                        std::cin.get();
                        exit(1);
                    }
                }
            } while (!cobotCmdFound);

            const char* found = NULL;
            std::string clientMessage2 = "";
            do
            {
                // Wait for cobot feedback from the client
                logger.logFileWrite(std::to_string(__LINE__) + " Server: waiting for client cobot finished acknowledge");
                logger.logFileWrite(std::to_string(__LINE__) + " Server: start receive data, blocking");
                memset(buffer, 0, BUFFER_SIZE);
                // RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV RECV 
                int bytesRead = recv(clientSocket, buffer, BUFFER_SIZE, 0);
                if (bytesRead <= 0) {
                    logger.logFileWrite(std::to_string(__LINE__) + " Server: error receiving data from client");
                    doExit();
                }

                // Since this data comes from the client and not whisper, there is no need to clean it up
                clientMessage2 = buffer; // "=" invokes the copy constructor of std::string 
                logger.logFileWrite(std::to_string(__LINE__) + " Server: received message: " + clientMessage2);

                found = strstr(buffer, "finished");
                if (found)
                {
                    // Write the received client cobot feedback into a text file for Sikulix
                    for (char& c : clientMessage2) {
                        c = std::tolower(c);
                    }
                    
                    // Added 24.4.24, since feedbackFileName was undefined (I don't know why), but wrong for sure
                    std::string feedbackFileName = "feedbackFileName.txt";

                    std::string cmdFileName = cmdFromSikulixPath + feedbackFileName;
                    logger.logFileWrite(std::to_string(__LINE__) + " Server: saved feedback file to: " + cmdFileName);
                    cmdFileCreate(cmdFileName, clientMessage2);
                    // Aknowledge is done at the end
                }
                else
                {
                    logger.logFileWrite(std::to_string(__LINE__) + " Server: finished was expected, wrong feedback received");
                    //doExit();
                }
            } while (!found);

            // Wait for Sikulix to delete the feedback file in DirCmdFromSikulix, as acknowledge
            aknowledgeSikulixCheck(clientMessage2, cmdFromSikulixPath);
            //do
            //{
            //    // Check Sikulix cmd acknowledge
            //    // Check if the cmd folder FOR Sikulix is empty, which means that the last user command has been executed by Sikulix
            //    Sleep(TIME_WAIT_FOR_CHECK_AKNOWLEDGE); // 1) Be sure that the cmd file that server has written has been seen by OS, 2) Slow down checking for acknowledge
            //    bool cmdFileFound = false;
            //    for (const auto& entry : fs::directory_iterator(cmdFromSikulixPath)) {
            //        // If there is at least one entry in the directory, it's not empty
            //        cmdFileFound = true;
            //        logger.logFileWrite(std::to_string(__LINE__) + " Server: waiting for Sikulix to execute last command: " + clientMessage2);
            //    }
            //} while (cmdFileFound);

            // Waiting for Sikulix to remove the server cmd file "cobot start"
            aknowledgeSikulixCheck(clientMessage1, cmdForSikulixPath);
        }
        else if (strstr(buffer, "scanner"))
        {
            // Artec cmd received
            aknowledgeSikulixCheck(clientMessage1, cmdForSikulixPath);
            //do
            //{
            //    // Check Sikulix cmd acknowledge
            //    // Check if the cmd folder FOR Sikulix is empty, which means that the last user command has been executed by Sikulix
            //    Sleep(TIME_WAIT_FOR_CHECK_AKNOWLEDGE); // 1) Be sure that the cmd file that server has written has been seen by OS, 2) Slow down checking for acknowledge
            //    cmdFileFound = false;
            //    for (const auto& entry : fs::directory_iterator(cmdForSikulixPath)) {
            //        // If there is at least one entry in the directory, it's not empty
            //        cmdFileFound = true;
            //        logger.logFileWrite(std::to_string(__LINE__) + " Server: waiting for Sikulix to execute last command: " + clientMessage);
            //    }
            //} while (cmdFileFound);

            // Send aknowledge to the client, that last received user command has been executed
            std::string response = buffer; // Acknowledge string is the same as the command string
            logger.logFileWrite(std::to_string(__LINE__) + " Server: send aknowledge msg to client: " + response);
            // SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND 
            send(clientSocket, response.c_str(), response.size(), 0);
        }
        else
        {
            // Error: cmd not recognized
            std::string response = "Error: cmd not recognized"; // Acknowledge string is the same as the command string
            logger.logFileWrite(std::to_string(__LINE__) + " Server: send msg: " + response);
            // SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND SEND 
            //send(clientSocket, response.c_str(), response.size(), 0);
//            doExit();
        }

        // xx Is this part really needed?
        if (clientMessage1 == "") {
            // Standard startup msg
            std::string response = "Server: Hello from server to client!";
            logger.logFileWrite(std::to_string(__LINE__) + " Server: sending client message: " + response);
            // SEND SEND SEND
            send(clientSocket, response.c_str(), response.size(), 0);
        }
        else if (clientMessage1 == "scanner shutdown")
        {
            logger.logFileWrite(std::to_string(__LINE__) + " Server: received shutdown command");
            doExit();
        }
    }

    // MB
    closesocket(clientSocket);

    // Clean up WinSock
    WSACleanup();

    logger.logFileClose();

    return 0;
}

