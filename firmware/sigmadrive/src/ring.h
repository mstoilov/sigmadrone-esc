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
	size_t size()			{ return (rp_ == wp_) ? 0 : ((bufsize_ + wp_ - rp_) % bufsize_); }

	size_t write_size()
	{
		unsigned int spacesize = space();
		unsigned int spacetoend = bufsize_ - wp_;
		return (spacesize < spacetoend) ? spacesize : spacetoend;
	}

	size_t read_size()
	{
		size_t datasize = size();
		size_t datatoend = bufsize_ - rp_;
		return (datasize < datatoend) ? datasize : datatoend;
	}

	T* get_read_ptr() { return &buffer_[rp_];	}
	T* get_write_ptr() { return &buffer_[wp_]; 	}
	T* get_data_ptr() { return &buffer_[0]; 	}

	void read_update(size_t size) {	rp_ = (rp_ + bufsize_ + size) % bufsize_;	}
	void write_update(size_t size) { wp_ = (wp_ + bufsize_ + size) % bufsize_; }
	void reset_wp(size_t wp) { wp_ = wp; }
	void reset_rp(size_t rp) { rp_ = rp; }


	size_t space()			{ return (rp_ == wp_) ? bufsize_ - 1 : ((bufsize_ + rp_ - wp_) % bufsize_ - 1); }
	T front()				{ return buffer_[rp_]; }
	void push(const T& e)	{ buffer_[wp_] = e; wp_ = (wp_ + 1) % bufsize_; }
	void pop()				{ rp_ = (rp_ + 1) % bufsize_; }
	bool empty()			{ return wp_ == rp_ ? true : false; }

public:
	volatile size_t wp_ = 0UL;
	volatile size_t rp_ = 0UL;
	T buffer_[bufsize_];
};

#endif
