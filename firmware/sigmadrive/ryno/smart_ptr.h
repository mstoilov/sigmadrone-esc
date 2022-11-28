#ifndef _SMART_PTR_H_
#define _SMART_PTR_H_

#include <memory>
#include <assert.h>

namespace ryno {


/**
 * @brief Smart Pointer based on std::unique. This smart pointer
 * template allows the managed object to be constructed and copied.
 *
 * @tparam T The type of the managed object.
 */
template<typename T>
struct smart_ptr : public std::unique_ptr<T, std::default_delete<T>> {
	bool ref = false;
	using base = std::unique_ptr<T, std::default_delete<T>>;
	using base::unique_ptr;
	using base::reset;
	using base::element_type;
	using base::release;

	~smart_ptr()
	{
		if (ref)
			release();
	}

	/**
	 * @brief Copy Constructor - Clone the underlying managed object
	 *
	 * @param rhs
	 */
	smart_ptr(const smart_ptr& rhs) : base(rhs.ref ? rhs.get() : rhs->Clone()), ref(rhs.ref)
	{
		if (!rhs) {
			throw std::runtime_error("smart_ptr: Cannot copy nullptr.");
		}
	}

	smart_ptr(smart_ptr&& rhs) : base(std::move(rhs)), ref(std::move(rhs.ref))
	{
	}

	smart_ptr(T* rhs, bool refonly = false) : base(rhs), ref(refonly)
	{
	}

	/**
	 * @brief Default constructor for the underlying object.
	 *
	 */
	smart_ptr() : base(new T()), ref(false)
	{
	}

	/**
	 * @brief Copy constructor, taking a reference to
	 * an object of the underlying type.
	 *
	 * @param rhs
	 */
	smart_ptr(const T& rhs) : base(new T(rhs)), ref(false)
	{
	}

	/**
	 * @brief Clone the the underlying managed object.
	 *
	 * @param rhs
	 * @return
	 */
	smart_ptr& operator=(const smart_ptr& rhs)
	{
		if (!rhs) {
			throw std::runtime_error("smart_ptr: Assignment operator cannot be passed nullptr.");
		}
		if (ref)
			release();
		reset(rhs.ref ? rhs.get() : rhs->Clone());
		ref = rhs.ref;
		return *this;
	}

	smart_ptr& operator=(smart_ptr&& rhs)
	{
		if (ref)
			release();
		this->reset(rhs.release());
		this->ref = rhs.ref;
		assert(rhs.get() == nullptr);
		return *this;
	}
};


}

#endif // _SMART_PTR_H_
