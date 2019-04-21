#pragma once
#include <stdexcept>
#include <string>

using std::string;

//Exception class about camera
class CameraException: public std::runtime_error{
    public:
    CameraException(const char* message):runtime_error(message){}
    CameraException(const string& message):runtime_error(message){}
    CameraException():runtime_error("Failed something about camera"){}
};