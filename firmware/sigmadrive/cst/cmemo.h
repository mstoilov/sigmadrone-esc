#ifndef _CMEMO_H_
#define _CMEMO_H_


#include <array>
#include "prtentry.h"

namespace cst
{

#ifndef CMEMO_CACHE_BITS
#define CMEMO_CACHE_BITS 9
#endif
#define CMEMO_CACHE_SIZE (1 << CMEMO_CACHE_BITS)
#define CMEMO_CACHE_MASK (CMEMO_CACHE_SIZE - 1)

	template<typename Iterator>
	struct MemoEntry {
		size_t serial = 0;
		size_t pos = 0;
		size_t rule = 0;
		bool result = false;
		size_t length = 0;
		std::vector<PRT_Entry<Iterator>> prt;
	};

	template<typename Iterator>
	class CMemo
	{
	public:
		CMemo() : enabled_(true), serial_(1) {}

		void enable(bool enable) { enabled_ = enable; }
		void invalidate() { serial_++; }
		void invalidate(size_t pos, size_t rule)
		{
			size_t index = hash(pos, rule);
			cache_[index] = {0, pos, rule, false, 0, std::vector<PRT_Entry<Iterator>>()};
		}

		size_t hash(size_t pos, size_t rule) const
		{
			constexpr size_t rulebits = CMEMO_CACHE_BITS / 3;
			constexpr size_t rulemask = (1 << rulebits) - 1;
			return ((pos << rulebits) | (rule & rulemask) ) & CMEMO_CACHE_MASK;
		}

		template<typename It>
		void add(size_t pos, size_t rule, bool result, size_t length, const It& begin, const It& end, size_t level)
		{
			if (!enabled_ || level < 1)
				return;
			size_t index = hash(pos, rule);
			cache_[index] = {this->serial_, pos, rule, result, length, std::vector<PRT_Entry<Iterator>>(begin, end)};
			for (auto& e : cache_[index].prt) {
				e.level -= level;
			}
		}

		const MemoEntry<Iterator>* lookup(size_t pos, size_t rule) const
		{
			if (!enabled_)
				return nullptr;
			size_t index = hash(pos, rule);
			const MemoEntry<Iterator>* memo = &cache_[index];
			if (memo->serial == serial_ && memo->pos == pos && memo->rule == rule)
				return memo;
			return nullptr;
		}

		bool enabled_;
		size_t serial_;
		std::array<MemoEntry<Iterator>, CMEMO_CACHE_SIZE> cache_;
	};

}

#endif
