#include "TCPClient.hpp"

std::string readCmdFromFile()
{
    // Directory path
    std::string directoryPath = "D:/Banfi/cmd/cmdForClient/";
    std::string extractedCmd = ""; // Command to be sent from client to server

    try {
        // Iterate through the files in the directory
        //for (const auto& entry : fs::directory_iterator(directoryPath)) {
        for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
            if (entry.is_regular_file()) {

                // Get the filename from the path
                std::string fileName = entry.path().filename().string();
                std::string fileNameComplete = directoryPath + "/" + fileName; // xx Do it seriously....

                // Create an output file stream and open the file
                std::ifstream commandFile(fileNameComplete);

                // Check if the file opened successfully
                if (!commandFile.is_open()) {
                    std::cerr << "Error: Could not open the file for writing." << std::endl;
                    exit(1); // Exit with an error code
                }

                // Read the content from the file into the string
                std::string line;
                while (std::getline(commandFile, line)) {
                    extractedCmd += line + "\n"; // Add each line to the string
                }

                commandFile.close();

                //// Find the positions of '[' and ']' in the filename
                //size_t startPos = fileName.find('[');
                //size_t endPos = fileName.find(']');

                //// If '[' and ']' are found and '[' comes before ']'
                //if (startPos != std::string::npos && endPos != std::string::npos && startPos < endPos) {
                //    // Extract the substring between '[' and ']' (excluding them)
                //    extractedCmd = fileName.substr(startPos + 1, endPos - startPos - 1);

                //    // Print the extracted string
                //    std::cout << "Client: extracted command from " << directoryPath << ": " << extractedCmd << std::endl;
                //}

                // Delete file
                //std::string fileNameComplete = directoryPath + fileNameComplete;
                std::filesystem::remove(fileNameComplete);

                return extractedCmd;
            }
        }

        if (extractedCmd == "")
        {
            logFileWrite("Client: waiting for commands\n");
            extractedCmd = "WaitingForCommands";
            return extractedCmd;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "Client: an error occurred: " << ex.what() << std::endl;
        //return 1;
    }
}
std::string readCmdFromFileOld()
{
    // Directory path
    std::string directoryPath = "D:/Banfi/cmd/cmdForClient/";
    std::string extractedCmd = ""; // Command to be sent from client to server

    try {
        // Iterate through the files in the directory
        //for (const auto& entry : fs::directory_iterator(directoryPath)) {
        for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
            if (entry.is_regular_file()) {
                // Get the filename from the path
                std::string filename = entry.path().filename().string();

                // Find the positions of '[' and ']' in the filename
                size_t startPos = filename.find('[');
                size_t endPos = filename.find(']');

                // If '[' and ']' are found and '[' comes before ']'
                if (startPos != std::string::npos && endPos != std::string::npos && startPos < endPos) {
                    // Extract the substring between '[' and ']' (excluding them)
                    extractedCmd = filename.substr(startPos + 1, endPos - startPos - 1);

                    // Print the extracted string
                    logFileWrite("Client: extracted command from " + directoryPath + ": " + extractedCmd + "\n");
                }

                // Delete file
                std::string fileNameComplete = directoryPath + filename;
                std::filesystem::remove(fileNameComplete);

                return extractedCmd;
            }
        }

        if (extractedCmd == "")
        {
            logFileWrite("Client: waiting for commands\n");
            extractedCmd = "WaitingForCommands";
            return extractedCmd;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "Client: an error occurred: " << ex.what() << std::endl;
        //return 1;
    }
}

void logFileWrite(const std::string msg)
{
    // Write message to cout
    std::cout << msg << std::endl;

    // Write message to logfile
    const char* msg_char = msg.c_str();
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

    // Check which windows socket implementation is needed
    WSADATA wsaData;
    logFileWrite("Client: started WSAStartup\n");
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        logFileWrite("Client: error initializing WinSock \n");
        return 1;
    }

    // Create socket
    logFileWrite("Client: started socket \n");
    SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == INVALID_SOCKET) {
        std::cerr << "Client: error creating socket." << std::endl;
        WSACleanup();
        return 1;
    }

    // Define the server address to connect to
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = inet_addr("192.168.1.142"); // Server IP address
    serverAddress.sin_port = htons(PORT);

    // Connect to the server
    logFileWrite("Client: started connect \n");
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) {
        std::cerr << "Client: error connecting to the server." << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return 1;
    }

    // The server has to start before the client, but
    // the client is expected to startc client/server interactions, so read the user command from file
    std::string msgClientToServer = readCmdFromFile(); // e.g. "preview";
    std::string msgServerToClient = "";

    while (true) {
        
        // Read the next user command only if the aknowledge has come from the server
        // Aknowledge example: client cmd "preview" waits for server reply "previewDone"
        if (msgServerToClient.starts_with(msgClientToServer))
        {
            // P10
            logFileWrite("Client: started readCmdFromFile \n");
            msgClientToServer = readCmdFromFile();
        }
        
        // Send data to the server, blocking
        logFileWrite("Client: send user command to server: " + msgClientToServer + "\n");
        send(clientSocket, msgClientToServer.c_str(), msgClientToServer.size(), 0);

        // Receive data from the server, blocking
        logFileWrite("Client: started receive data, blocking\n");
        char buffer[BUFFER_SIZE];
        memset(buffer, 0, BUFFER_SIZE);
        int bytesRead = recv(clientSocket, buffer, BUFFER_SIZE, 0);
        if (bytesRead <= 0) {
            std::cerr << "Client: error receiving data from server." << std::endl;
        }
        else {
            // Did I receive a feedback or a command?
            if (strstr(buffer, "cobot"))
            {
                // I received a command for the cobot
                std::string msgServerToClientReceived(buffer);
                msgServerToClient = msgServerToClientReceived;
                logFileWrite("Client: received from server: " + msgServerToClient + "\n");

                // Simulate a cobot feedback
                msgClientToServer = "finished";
                logFileWrite("Client: send cobot feedback to the server: " + msgClientToServer + "\n");
                send(clientSocket, msgClientToServer.c_str(), msgClientToServer.size(), 0);

                logFileWrite("Client: started receive data, blocking\n");
                memset(buffer, 0, BUFFER_SIZE);
                int bytesRead = recv(clientSocket, buffer, BUFFER_SIZE, 0);
                if (bytesRead <= 0) {
                    std::cerr << "Client: error receiving data from server." << std::endl;
                }
                else {
                    std::string msgServerToClientReceived(buffer);
                    msgServerToClient = msgServerToClientReceived;
                    logFileWrite("Client: received from server: " + msgServerToClient + "\n");
                }
            }
        }

        // If command read is "shutdown", send it to server (it is done above), then exit client
        if (msgClientToServer == "shutdown")
            break;
    }

    // Close the socket
    logFileWrite("Client: closing socket: \n");
    closesocket(clientSocket);

    logFileWrite("Client: WSACleanup: \n");
    // Clean up WinSock
    WSACleanup();

    return 0;
}