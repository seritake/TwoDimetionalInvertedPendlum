#pragma once
#include <stdexcept>
#include <string>

using std::string;

//Exception class about image processing
class ProcessingException: public std::runtime_error{
    public:
    ProcessingException(const char* message):runtime_error(message){}
    ProcessingException(const string& message):runtime_error(message){}
    ProcessingException():runtime_error("Failed something about image processing"){}
};