#ifndef _HELLO_H_
#define _HELLO_H_

#include <string>

class Hello {
public:
	Hello(const std::string& hello);
	void print();

protected:
	std::string hello_msg_;
	uint32_t counter_ = 0;
};

#endif
