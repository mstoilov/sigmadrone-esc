#ifndef _MODULE_H_
#define _MODULE_H_

#include <map>
#include <functional>
#include "ryobject.h"
#include "floatobject.h"
#include "intobject.h"
#include "boolobject.h"
#include "stringobject.h"
#include "arrayobject.h"
#include "mapobject.h"
#include "codefragment.h"
#include "stack.h"

namespace ryno
{

	class Module
	{
	public:
		Module();

public:
		MapObject map_;
	};

}

#endif
