#ifndef _RYSTACK_H_
#define _RYSTACK_H_

#include <vector>
#include <ostream>
#include "ryobject.h"
#include "smart_ptr.h"

namespace ryno {


class Stack : public std::vector<RyPointer> {
public:
	using base = std::vector<RyPointer>;
	using base::vector;
	using base::value_type;
	using base::reference;
	using base::const_reference;
	using base::push_back;
	using base::at;
	using base::resize;

	size_t append(value_type&& value);
	void push(value_type&& value);
	value_type pop();
	void dump(std::ostream& os);
    reference at(size_type frame, size_type n) { return base::at(frame + n); }
    const_reference at(size_type frame, size_type n) const { return base::at(frame + n); }
};

}

#endif
