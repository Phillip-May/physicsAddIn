#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Minimal line-oriented serial port, replacing QtSerialPort.
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    struct PortInfo {
        std::string portName;     // e.g. "COM5"
        std::string description;  // friendly name where the platform provides one
    };

    // Ports currently present. On Windows this reads the registry's SERIALCOMM map, which
    // lists exactly the ports the system has open handles available for.
    static std::vector<PortInfo> availablePorts();

    // True where the platform requires the user to pick a device before it can be seen.
    // False on Windows, where enumeration is enough; true in a browser, where Web Serial only
    // exposes a port after an explicit grant.
    static bool needsPortRequest();

    // Asks the user to pick a device, then adds it to availablePorts(). Must be called from
    // the frame that handled a click: browsers only allow the chooser to open while a user
    // activation is live. A no-op where needsPortRequest() is false.
    static void requestPort();

    // Set when the last requestPort() failed for a reason worth showing, such as the browser
    // not implementing Web Serial. Empty when the user simply cancelled.
    static std::string lastRequestError();

    // 8N1 at the given baud, no flow control.
    bool open(const std::string& portName, int baudRate, std::string* errorMessage = nullptr);
    void close();
    bool isOpen() const;

    // Drains whatever the device has sent into the internal buffer. Non-blocking; safe to call
    // every frame whether or not data is waiting.
    void poll();

    // Returns one complete newline-terminated line, or false when none is buffered yet.
    // Partial lines stay buffered until their terminator arrives, so a reply split across
    // polls is not lost or truncated.
    bool takeLine(std::string* line);

    // Appends a newline if the payload does not already end with one.
    bool writeLine(const std::string& payload, std::string* errorMessage = nullptr);

    const std::string& portName() const { return m_portName; }

private:
    void* m_handle = nullptr;  // HANDLE on Windows, kept opaque to avoid leaking windows.h
    std::string m_portName;
    std::string m_readBuffer;
};
