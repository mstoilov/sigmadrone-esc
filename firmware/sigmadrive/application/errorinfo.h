/*
 * errorinfo.h
 *
 *  Created on: Jan 7, 2020
 *      Author: mstoilov
 */

#ifndef _ERRORINFO_H_
#define _ERRORINFO_H_

#include <stdint.h>
#include <map>


enum ErrorCode {
	e_none = 0,
	e_trip_voltage = 1,
	e_trip_current = 2,
};


class ErrorInfo {
public:
	ErrorInfo();
	virtual ~ErrorInfo();

	void ClearError();
	void SetError(uint32_t error, const char *msg, ...);
	uint32_t GetError() const;
	std::string GetErrorMessage() const;


public:
	uint32_t error_;
	std::string error_msg_;
};

#endif /* APPLICATION_ERRORINFO_H_ */
