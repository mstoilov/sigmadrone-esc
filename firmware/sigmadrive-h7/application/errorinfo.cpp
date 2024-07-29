/*
 * errorinfo.cpp
 *
 *  Created on: Jan 7, 2020
 *      Author: mstoilov
 */

#include <stdarg.h>
#include "errorinfo.h"

ErrorInfo::ErrorInfo()
	: error_(0)
{

}

ErrorInfo::~ErrorInfo()
{

}

void ErrorInfo::ClearError()
{
	error_ = e_none;
	error_msg_.clear();
}

void ErrorInfo::SetError(uint32_t error, const char *msg, ...)
{
	if (error_ == e_none && error) {
		error_ = error;
		va_list args;

		va_start(args, msg);
		char buffer[256];
		vsnprintf(buffer, sizeof(buffer) - 1, msg, args);
		error_msg_ = buffer;
		va_end(args);
	}
}

uint32_t ErrorInfo::GetError() const
{
	return error_;
}

std::string ErrorInfo::GetErrorMessage() const
{
	return error_msg_;
}
