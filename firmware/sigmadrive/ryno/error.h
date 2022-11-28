#ifndef _ERROR_H_
#define _ERROR_H_

#include <stdexcept>
#include <sstream>

class NotImplementedError : public std::runtime_error {
public:
	NotImplementedError(const std::string& msg) : std::runtime_error(msg) {}
};

class RelocationError : std::runtime_error {
public:
	RelocationError(const std::string& msg) : std::runtime_error(msg) {}
};

class SyntaxError : public std::runtime_error {
public:
	static std::string format_msg(size_t line, size_t col, size_t offset, const std::string& msg) {
		std::stringstream oss;
		oss << "(" << line << ":" << col << ":" << offset << ") " << msg;
		return oss.str();
	}
	SyntaxError(size_t line, size_t col, size_t offset, const std::string& msg = "Syntax Error") 
		: runtime_error(format_msg(line, col, offset, msg)) { }

	SyntaxError(const std::string& msg = "Syntax Error") 
		: runtime_error(msg) { }

};

class OutOfRangeError : public std::range_error {
public:
	OutOfRangeError(const std::string& msg) : std::range_error(msg) {}
};

static inline void ThrowUnsupportedType()
{
	throw NotImplementedError("Unsupported Type");
}

static inline void ThrowOperationNotImplemented(const std::string& operation)
{
	throw NotImplementedError("'" + operation + "'" + " operation is not implemented.");
}


class ZeroDivisionError : public std::overflow_error {
public:
	ZeroDivisionError(const std::string& msg) : std::overflow_error(msg) {}
};

class InvalidTypeError : public std::domain_error {
public:
	InvalidTypeError(const std::string& msg) : std::domain_error(msg) {}
};


#endif
