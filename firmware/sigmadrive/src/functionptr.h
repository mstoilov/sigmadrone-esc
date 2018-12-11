/*
 *  Sigmadrone
 *  Copyright (c) 2013-2015 The Sigmadrone Developers
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Martin Stoilov <martin@sigmadrone.org>
 *  Svetoslav Vassilev <svassilev@sigmadrone.org>
 */
#ifndef FUNCTIONPTR_H
#define FUNCTIONPTR_H

#include <string.h>

typedef void (*pvoidf_t)(void);


class FunctionPointer {
public:

	/** Create a FunctionPointer, attaching a static function
	 *
	 *  @param function The void static function to attach (default is none)
	 */
	FunctionPointer(void (*function)(void) = 0) :
			function_(), object_(), membercaller_(), internal_()
	{
		attach(function);
	}

	/** Create a FunctionPointer, attaching a member function
	 *
	 *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
	 *  @param function The address of the void member function to attach
	 */
	template<typename T>
	FunctionPointer(T *object, void (T::*member)(void)) :
			function_(), object_(), membercaller_(), internal_()
	{
		attach(object, member);
	}

	/** Attach a static function
	 *
	 *  @param function The void static function to attach (default is none)
	 */
	void attach(void (*function)(void) = 0)
	{
		function_ = function;
		object_ = 0;
	}

	/** Attach a member function
	 *
	 *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
	 *  @param function The address of the void member function to attach
	 */
	template<typename T>
	void attach(T *object, void (T::*member)(void))
	{
		object_ = static_cast<void*>(object);
		memcpy(member_, (char*) &member, sizeof(member));
		membercaller_ = &FunctionPointer::membercaller<T>;
		function_ = 0;
	}

	/** Call the attached static or member function
	 */
	void call()
	{
		if (object_) {
			membercaller_(object_, member_);
		} else if (function_) {
			function_();
		}
	}

	pvoidf_t get_function() const
	{
		return (pvoidf_t) function_;
	}

	void operator ()(void)
	{
		call();
	}

private:
	template<typename T>
	static void membercaller(void *object, char *member)
	{
		T* o = static_cast<T*>(object);
		void (T::*m)(void);
		memcpy((char*) &m, member, sizeof(m));
		(o->*m)();
	}

	void (*function_)(void);             // static function pointer - 0 if none attached
	void *object_;                       // object this pointer - 0 if none attached
	char member_[16];                    // raw member function pointer storage - converted back by registered _membercaller
	void (*membercaller_)(void*, char*); // registered membercaller function to convert back and call _member on _object
	int internal_;
};

#endif
