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

const int PORT = 22; //  12345;
const int BUFFER_SIZE = 1024;

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
            std::cout << "Client: waiting for commands" << std::endl;
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
                    std::cout << "Client: extracted command from " << directoryPath << ": " << extractedCmd << std::endl;
                }

                // Delete file
                std::string fileNameComplete = directoryPath + filename;
                std::filesystem::remove(fileNameComplete);

                return extractedCmd;
            }
        }

        if (extractedCmd == "")
        {
            std::cout << "Client: waiting for commands" << std::endl;
            extractedCmd = "WaitingForCommands";
            return extractedCmd;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "Client: an error occurred: " << ex.what() << std::endl;
        //return 1;
    }
}

int main() {

    // Check which windows socket implementation is needed
    WSADATA wsaData;
    std::cout << "Client: started WSAStartup" << std::endl;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Client: error initializing WinSock." << std::endl;
        return 1;
    }

    // Create socket
    std::cout << "Client: started socket" << std::endl;
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
    std::cout << "Client: started connect" << std::endl;
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) {
        std::cerr << "Client: error connecting to the server." << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return 1;
    }

    // The client is expected to start interactions, so read the command from file
    std::string msgClientToServer = readCmdFromFile(); // e.g. "preview";
    std::string msgServerToClient = "";

    while (true) {
        
        // Read the next command only if the aknowledge has come from server
        // Aknowledge example: client cmd "preview" waits for server reply "previewDone"
        if (msgServerToClient.starts_with(msgClientToServer))
        {
            std::cout << "Client: started readCmdFromFile" << std::endl;
            msgClientToServer = readCmdFromFile();
        }
        
        // Send data to the server, blocking
        std::cout << "Client: start send to server: " << msgClientToServer  << std::endl;
        send(clientSocket, msgClientToServer.c_str(), msgClientToServer.size(), 0);

        // Receive data from the server, blocking
        std::cout << "Client: started receive data, blocking" << std::endl;
        char buffer[BUFFER_SIZE];
        memset(buffer, 0, BUFFER_SIZE);
        int bytesRead = recv(clientSocket, buffer, BUFFER_SIZE, 0);
        if (bytesRead <= 0) {
            std::cerr << "Client: error receiving data from server." << std::endl;
        }
        else {
            std::string msgServerToClientReceived(buffer);
            msgServerToClient = msgServerToClientReceived;
            std::cout << "Client: received from server: " << msgServerToClient << std::endl;
        }

        // If command read is "shutdown", send it to server (it is done above), then exit client
        if (msgClientToServer == "shutdown")
            break;
    }

    // Close the socket
    std::cout << "Client: closing socket: " << std::endl;
    closesocket(clientSocket);

    // Clean up WinSock
    std::cout << "Client: WSACleanup: " << std::endl;
    WSACleanup();

    return 0;
}