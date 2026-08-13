#include "SerialPort.h"

#if defined(_WIN32) && !defined(__EMSCRIPTEN__)

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace {

HANDLE asHandle(void* handle) {
    return handle == nullptr ? INVALID_HANDLE_VALUE : static_cast<HANDLE>(handle);
}

std::string lastErrorText() {
    const DWORD code = GetLastError();
    if (code == 0) return std::string();
    char* buffer = nullptr;
    const DWORD length = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, code, 0, reinterpret_cast<char*>(&buffer), 0, nullptr);
    std::string message = length > 0 && buffer != nullptr ? std::string(buffer, length) : std::string();
    if (buffer != nullptr) LocalFree(buffer);
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) message.pop_back();
    return message;
}

} // namespace

SerialPort::~SerialPort() {
    close();
}

// Enumeration is sufficient on Windows, so there is nothing for the user to grant.
bool SerialPort::needsPortRequest() { return false; }
void SerialPort::requestPort() {}
std::string SerialPort::lastRequestError() { return {}; }

std::vector<SerialPort::PortInfo> SerialPort::availablePorts() {
    std::vector<PortInfo> ports;

    // HKLM\HARDWARE\DEVICEMAP\SERIALCOMM maps device paths to port names and is the cheapest
    // enumeration that does not pull in SetupAPI.
    HKEY key = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) != ERROR_SUCCESS) {
        return ports;
    }

    char valueName[256];
    char valueData[256];
    DWORD index = 0;
    for (;;) {
        DWORD nameLength = sizeof(valueName);
        DWORD dataLength = sizeof(valueData);
        DWORD type = 0;
        const LONG status = RegEnumValueA(key, index, valueName, &nameLength, nullptr, &type,
                                          reinterpret_cast<BYTE*>(valueData), &dataLength);
        if (status == ERROR_NO_MORE_ITEMS) break;
        ++index;
        if (status != ERROR_SUCCESS || type != REG_SZ) continue;

        PortInfo info;
        info.portName.assign(valueData, dataLength > 0 ? dataLength - 1 : 0);
        info.description.assign(valueName, nameLength);
        if (!info.portName.empty()) ports.push_back(info);
    }
    RegCloseKey(key);
    return ports;
}

bool SerialPort::open(const std::string& portName, int baudRate, std::string* errorMessage) {
    close();

    // The \\.\ prefix is required for COM10 and above; harmless for lower numbers.
    const std::string devicePath = "\\\\.\\" + portName;
    HANDLE handle = CreateFileA(devicePath.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                                OPEN_EXISTING, 0, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        if (errorMessage) *errorMessage = "Failed to open " + portName + ": " + lastErrorText();
        return false;
    }

    DCB settings = {};
    settings.DCBlength = sizeof(settings);
    if (!GetCommState(handle, &settings)) {
        if (errorMessage) *errorMessage = "Failed to read port state: " + lastErrorText();
        CloseHandle(handle);
        return false;
    }
    settings.BaudRate = static_cast<DWORD>(baudRate);
    settings.ByteSize = 8;
    settings.Parity = NOPARITY;
    settings.StopBits = ONESTOPBIT;
    settings.fBinary = TRUE;
    settings.fParity = FALSE;
    settings.fOutxCtsFlow = FALSE;
    settings.fOutxDsrFlow = FALSE;
    settings.fDtrControl = DTR_CONTROL_ENABLE;  // Teensy enumerates only with DTR asserted.
    settings.fRtsControl = RTS_CONTROL_ENABLE;
    settings.fOutX = FALSE;
    settings.fInX = FALSE;
    if (!SetCommState(handle, &settings)) {
        if (errorMessage) *errorMessage = "Failed to configure port: " + lastErrorText();
        CloseHandle(handle);
        return false;
    }

    // All zero except ReadIntervalTimeout = MAXDWORD makes ReadFile return immediately with
    // whatever is buffered, which is what a per-frame poll wants: never block the UI.
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 500;
    SetCommTimeouts(handle, &timeouts);

    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    m_handle = handle;
    m_portName = portName;
    m_readBuffer.clear();
    if (errorMessage) errorMessage->clear();
    return true;
}

void SerialPort::close() {
    if (m_handle != nullptr) {
        CloseHandle(asHandle(m_handle));
        m_handle = nullptr;
    }
    m_portName.clear();
    m_readBuffer.clear();
}

bool SerialPort::isOpen() const {
    return m_handle != nullptr;
}

void SerialPort::poll() {
    if (m_handle == nullptr) return;

    char chunk[2048];
    for (;;) {
        DWORD read = 0;
        if (!ReadFile(asHandle(m_handle), chunk, sizeof(chunk), &read, nullptr)) {
            // A failed read means the device went away, e.g. the board was unplugged.
            close();
            return;
        }
        if (read == 0) return;
        m_readBuffer.append(chunk, read);
        if (read < sizeof(chunk)) return;
    }
}

bool SerialPort::takeLine(std::string* line) {
    const size_t newline = m_readBuffer.find('\n');
    if (newline == std::string::npos) return false;
    if (line) {
        *line = m_readBuffer.substr(0, newline);
        while (!line->empty() && line->back() == '\r') line->pop_back();
    }
    m_readBuffer.erase(0, newline + 1);
    return true;
}

bool SerialPort::writeLine(const std::string& payload, std::string* errorMessage) {
    if (m_handle == nullptr) {
        if (errorMessage) *errorMessage = "Port is not open.";
        return false;
    }
    std::string data = payload;
    if (data.empty() || data.back() != '\n') data.push_back('\n');

    size_t written = 0;
    while (written < data.size()) {
        DWORD chunk = 0;
        if (!WriteFile(asHandle(m_handle), data.data() + written,
                       static_cast<DWORD>(data.size() - written), &chunk, nullptr) || chunk == 0) {
            if (errorMessage) *errorMessage = "Failed to write: " + lastErrorText();
            return false;
        }
        written += chunk;
    }
    if (errorMessage) errorMessage->clear();
    return true;
}

#elif defined(__EMSCRIPTEN__)

#include <cstdlib>

// Web Serial backend.
extern "C" {
void webSerialInit();
void webSerialRequestPort();
int webSerialGrantedCount();
char* webSerialDescribe(int index);
char* webSerialLastError();
int webSerialOpen(int index, int baudRate);
void webSerialClose();
int webSerialIsOpen();
int webSerialPending();
int webSerialTake(unsigned char* destination, int count);
int webSerialWrite(const char* text);
}

namespace {

// Takes ownership of a string the JS side allocated with stringToNewUTF8.
std::string adoptJsString(char* text) {
    if (text == nullptr) return std::string();
    std::string result(text);
    std::free(text);
    return result;
}

} // namespace

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::needsPortRequest() { return true; }

void SerialPort::requestPort() {
    webSerialInit();
    webSerialRequestPort();
}

std::string SerialPort::lastRequestError() { return adoptJsString(webSerialLastError()); }

std::vector<SerialPort::PortInfo> SerialPort::availablePorts() {
    webSerialInit();
    const int count = webSerialGrantedCount();
    std::vector<PortInfo> ports;
    ports.reserve(static_cast<size_t>(count < 0 ? 0 : count));
    for (int i = 0; i < count; ++i) {
        PortInfo info;
        // The index is the name: Web Serial exposes no stable per-device identifier.
        info.portName = std::to_string(i);
        info.description = adoptJsString(webSerialDescribe(i));
        ports.push_back(std::move(info));
    }
    return ports;
}

bool SerialPort::open(const std::string& portName, int baudRate, std::string* errorMessage) {
    webSerialInit();
    const int index = std::atoi(portName.c_str());
    if (!webSerialOpen(index, baudRate)) {
        if (errorMessage) {
            *errorMessage = "No granted serial port at index " + portName +
                            ". Use Request port to pick a device first.";
        }
        return false;
    }
    // The open is still in flight; isOpen() reports the real state on a later frame and writes
    // made in the meantime are queued in JS.
    m_portName = portName;
    if (errorMessage) errorMessage->clear();
    return true;
}

void SerialPort::close() {
    webSerialClose();
    m_portName.clear();
    m_readBuffer.clear();
}

bool SerialPort::isOpen() const { return webSerialIsOpen() != 0; }

void SerialPort::poll() {
    const int pending = webSerialPending();
    if (pending <= 0) return;
    const size_t previous = m_readBuffer.size();
    m_readBuffer.resize(previous + static_cast<size_t>(pending));
    const int taken = webSerialTake(
        reinterpret_cast<unsigned char*>(&m_readBuffer[previous]), pending);
    if (taken < pending) m_readBuffer.resize(previous + static_cast<size_t>(taken < 0 ? 0 : taken));
}

bool SerialPort::takeLine(std::string* line) {
    const size_t newline = m_readBuffer.find('\n');
    if (newline == std::string::npos) return false;
    if (line) {
        *line = m_readBuffer.substr(0, newline);
        while (!line->empty() && line->back() == '\r') line->pop_back();
    }
    m_readBuffer.erase(0, newline + 1);
    return true;
}

bool SerialPort::writeLine(const std::string& payload, std::string* errorMessage) {
    if (!isOpen()) {
        if (errorMessage) *errorMessage = "Port is not open.";
        return false;
    }
    std::string line = payload;
    if (line.empty() || line.back() != '\n') line.push_back('\n');
    webSerialWrite(line.c_str());
    if (errorMessage) errorMessage->clear();
    return true;
}
#else

SerialPort::~SerialPort() = default;
std::vector<SerialPort::PortInfo> SerialPort::availablePorts() { return {}; }
bool SerialPort::needsPortRequest() { return false; }
void SerialPort::requestPort() {}
std::string SerialPort::lastRequestError() { return {}; }
bool SerialPort::open(const std::string&, int, std::string* errorMessage) {
    if (errorMessage) *errorMessage = "Serial ports are not available in this build.";
    return false;
}
void SerialPort::close() {}
bool SerialPort::isOpen() const { return false; }
void SerialPort::poll() {}
bool SerialPort::takeLine(std::string*) { return false; }
bool SerialPort::writeLine(const std::string&, std::string* errorMessage) {
    if (errorMessage) *errorMessage = "Serial ports are not available in this build.";
    return false;
}

#endif
