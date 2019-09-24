#ifndef _RING_H_
#define _RING_H_

#include <stddef.h>
#include <cstdint>
#include <cstddef>

template<typename T, size_t bufsize_>
class Ring
{
public:
	Ring() = default;
	~Ring() = default;
	Ring(const Ring &) = delete;
	Ring(Ring &&) = delete;
	Ring& operator=(const Ring&) = delete;
	Ring& operator=(Ring&&) = delete;

	size_t capacity()		{ return bufsize_; }
	size_t data_size()		{ return ((wp_ + bufsize_ - rp_) % bufsize_); }
	size_t space_size()		{ return (bufsize_ - data_size() - 1); }

	size_t space_to_end()
	{
		return bufsize_ - wp_;
	}

	size_t write_size()
	{
		size_t ret;
		volatile size_t spacesize = space_size();
		volatile size_t spacetoend = bufsize_ - wp_;
		ret = (spacesize < spacetoend) ? spacesize : spacetoend;
		return ret;
	}

	size_t read_size()
	{
		volatile size_t datasize = data_size();
		volatile size_t datatoend = bufsize_ - rp_;
		return (datasize < datatoend) ? datasize : datatoend;
	}

	T* get_read_ptr() { return &buffer_[rp_];	}
	T* get_write_ptr() { return &buffer_[wp_]; 	}
	T* get_data_ptr() { return &buffer_[0]; 	}

	void read_update(size_t size) {	rp_ = (rp_ + bufsize_ + size) % bufsize_;	}
	void write_update(size_t size) { wp_ = (wp_ + bufsize_ + size) % bufsize_; }
	void reset_wp(size_t wp) { wp_ = wp; }
	void reset_rp(size_t rp) { rp_ = rp; }


	T tail()				{ return buffer_[rp_]; }
	void push(const T& e)	{ buffer_[wp_] = e; wp_ = (wp_ + 1) % bufsize_; }
	void pop()				{ rp_ = (rp_ + 1) % bufsize_; }
	bool empty()			{ return wp_ == rp_ ? true : false; }

public:
	volatile size_t wp_ = 0UL;
	volatile size_t rp_ = 0UL;
	T buffer_[bufsize_];
};

#endif
