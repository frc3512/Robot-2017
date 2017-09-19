// Copyright (c) 2016-2017 FRC Team 3512. All Rights Reserved.

#include "DSDisplay.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;

DSDisplay& DSDisplay::GetInstance(uint16_t dsPort) {
    static DSDisplay dsDisplay(dsPort);
    return dsDisplay;
}

void DSDisplay::Clear() { m_packet.clear(); }

void DSDisplay::SendToDS() {
    if (m_dsIP != 0) {
        m_socket.send(m_packet, m_dsIP, m_dsPort);
    }
}

const std::string DSDisplay::ReceiveFromDS() {
    // Send keepalive every 250ms
    auto time = steady_clock::now();
    if (time - prevTime > 250ms) {
        Clear();
        m_packet << static_cast<std::string>("\r\n");
        SendToDS();

        prevTime = time;
    }

    if (m_socket.receive(m_recvBuffer, 256, m_recvAmount, m_recvIP,
                         m_recvPort) == UdpSocket::Done) {
        if (std::strncmp(m_recvBuffer, "connect\r\n", 9) == 0) {
            m_dsIP = m_recvIP;
            m_dsPort = m_recvPort;

            // Send GUI element file to DS

            Clear();

            m_packet << static_cast<std::string>("guiCreate\r\n");

            // Open the file
            std::ifstream guiFile("/home/lvuser/GUISettings.txt",
                                  std::ifstream::binary);

            if (guiFile.is_open()) {
                // Get its length
                guiFile.seekg(0, guiFile.end);
                unsigned int fileSize = guiFile.tellg();
                guiFile.seekg(0, guiFile.beg);

                // Send the length
                m_packet << static_cast<uint32_t>(fileSize);

                // Allocate a buffer for the file
                auto tempBuf = std::make_unique<char>(fileSize);

                // Send the data
                guiFile.read(tempBuf.get(), fileSize);
                m_packet.append(tempBuf.get(), fileSize);

                guiFile.close();
            }

            SendToDS();

            // Send a list of available autonomous modes
            Clear();

            m_packet << static_cast<std::string>("autonList\r\n");

            for (unsigned int i = 0; i < m_autonModes.Size(); i++) {
                m_packet << m_autonModes.Name(i);
            }

            SendToDS();

            // Make sure driver knows which autonomous mode is selected
            Clear();

            m_packet << static_cast<std::string>("autonConfirmed\r\n");
            m_packet << m_autonModes.Name(m_curAutonMode);

            SendToDS();

            return "connect\r\n";
        } else if (std::strncmp(m_recvBuffer, "autonSelect\r\n", 13) == 0) {
            // Next byte after command is selection choice
            m_curAutonMode = m_recvBuffer[13];

            Clear();

            m_packet << static_cast<std::string>("autonConfirmed\r\n");
            m_packet << m_autonModes.Name(m_curAutonMode);

            // Store newest autonomous choice to file for persistent storage
            std::ofstream autonModeFile("/home/lvuser/autonMode.txt",
                                        std::fstream::trunc);
            if (autonModeFile.is_open()) {
                // Selection is stored as ASCII number in file
                char autonNum = '0' + m_curAutonMode;

                if (autonModeFile << autonNum) {
                    std::cout << "DSDisplay: autonSelect: wrote auton "
                              << autonNum << " to file" << std::endl;
                } else {
                    std::cout << "DSDisplay: autonSelect: failed writing auton "
                              << autonNum << " into open file" << std::endl;
                }
            } else {
                std::cout
                    << "DSDisplay: autonSelect: failed to open autonMode.txt"
                    << std::endl;
            }

            SendToDS();

            return "autonSelect\r\n";
        }
    }

    return "NONE";
}

void DSDisplay::AddAutoMethod(const std::string& methodName,
                              std::function<void()> func) {
    m_autonModes.AddMethod(methodName, func);
}

DSDisplay::DSDisplay(uint16_t portNumber) : m_dsPort(portNumber) {
    m_socket.bind(portNumber);
    m_socket.setBlocking(false);

    // Retrieve stored autonomous index
    std::ifstream autonModeFile("/home/lvuser/autonMode.txt");
    if (autonModeFile.is_open()) {
        if (autonModeFile >> m_curAutonMode) {
            std::cout << "DSDisplay: restored auton " << m_curAutonMode
                      << std::endl;

            // Selection is stored as ASCII number in file
            m_curAutonMode -= '0';
        } else {
            std::cout << "DSDisplay: failed restoring auton" << std::endl;
        }
    } else {
        std::cout << "DSDisplay: failed opening autonMode.txt" << std::endl;
        m_curAutonMode = 0;
    }
}

void DSDisplay::DeleteAllMethods() { m_autonModes.DeleteAllMethods(); }

void DSDisplay::ExecAutonomous() {
    m_autonModes.ExecAutonomous(m_curAutonMode);
}

char DSDisplay::GetAutonID() const { return m_curAutonMode; }

void DSDisplay::AddData(std::string ID, StatusLight data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('c');
    m_packet << ID;
    m_packet << static_cast<int8_t>(data);
}

void DSDisplay::AddData(std::string ID, bool data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('c');
    m_packet << ID;

    if (data == true) {
        m_packet << static_cast<int8_t>(DSDisplay::active);
    } else {
        m_packet << static_cast<int8_t>(DSDisplay::inactive);
    }
}

void DSDisplay::AddData(std::string ID, int8_t data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('c');
    m_packet << ID;
    m_packet << data;
}

void DSDisplay::AddData(std::string ID, int32_t data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('i');
    m_packet << ID;
    m_packet << data;
}

void DSDisplay::AddData(std::string ID, uint32_t data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('u');
    m_packet << ID;
    m_packet << data;
}

void DSDisplay::AddData(std::string ID, std::string data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('s');
    m_packet << ID;
    m_packet << data;
}

void DSDisplay::AddData(std::string ID, float data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('s');
    m_packet << ID;
    m_packet << std::to_string(data);
}

void DSDisplay::AddData(std::string ID, double data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('s');
    m_packet << ID;
    m_packet << std::to_string(data);
}