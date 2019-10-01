#include "hello.h"

Hello::Hello(const std::string& hello)
	: hello_msg_(hello)
{

}
void Hello::print()
{
	fprintf(stderr, "%s %ld\r\n", hello_msg_.c_str(), counter_++);
}
