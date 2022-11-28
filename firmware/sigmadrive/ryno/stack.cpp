#include "stack.h"

namespace ryno {


size_t Stack::append(value_type&& value)
{
	size_t offset = base::size();
	push_back(std::move(value));
	return offset;
}


void Stack::push(value_type&& value)
{
	push_back(std::move(value));
}

Stack::value_type Stack::pop()
{
	value_type ret = std::move(*rbegin());
	resize(size() - 1);
	return ret;
}

void Stack::dump(std::ostream& os)
{
	for (auto &i : *this) {
		os << i->Repr() << std::endl;
	}
}


}
