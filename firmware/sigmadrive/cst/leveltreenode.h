#ifndef _LEVELTREENODE_H_
#define _LEVELTREENODE_H_

#include <cstdlib>

namespace cst 
{
	struct LevelTreeNode
	{
		LevelTreeNode(size_t levelval = -1) : level(levelval) {}
		size_t GetLevel() const { return level; }
		void SetLevel(size_t levelval) { level = levelval; }

		size_t level;
	};

	template<typename Iterator>
	Iterator leveltree_next(Iterator rec, Iterator begin, Iterator end)
	{
		if (rec == end)
			return end;
		Iterator ret = std::next(rec);
		while (ret != end && ret->level > rec->level)
			ret++;
		if (ret != end && ret->level == rec->level)
			return ret;
		return end;
	}

	template<typename Iterator>
	Iterator leveltree_prev(Iterator rec, Iterator begin, Iterator end)
	{
		if (rec == end || rec == begin)
			return end;
		auto rend = std::prev(begin);
		Iterator ret = std::prev(rec);
		while (ret != rend && ret->level > rec->level)
			ret--;
		if (ret != rend && ret->level == rec->level)
			return ret;
		return end;
	}

	template<typename Iterator>
	Iterator leveltree_parent(Iterator rec, Iterator begin, Iterator end)
	{
		if (rec == end || rec == begin || rec->level == 0)
			return end;
		auto rend = std::prev(begin);
		Iterator ret = std::prev(rec);
		while (ret != rend && ret->level >= rec->level)
			ret--;
		if (ret != rend && ret->level == rec->level - 1)
			return ret;
		return end;
	}

	template<typename Iterator>
	Iterator leveltree_firstchild(Iterator rec, Iterator begin, Iterator end)
	{
		if (rec == end)
			return end;
		Iterator ret = std::next(rec);
		if (ret != end && ret->level == rec->level + 1)
			return ret;
		return end;
	}

	template<typename Iterator>
	Iterator leveltree_lastchild(Iterator rec, Iterator begin, Iterator end)
	{
		Iterator child = leveltree_firstchild(rec, begin, end);
		Iterator ret = end;

		while (child != end && child->level > rec->level) {
			if (child->level == rec->level + 1)
				ret = child;
			child = leveltree_next(child, begin, end);
		}
		return ret;
	}

	template<typename Iterator>
	size_t leveltree_nchildren(Iterator rec, Iterator begin, Iterator end)
	{
		size_t count = 0;
		for (Iterator it = leveltree_firstchild(rec, begin, end); it != end; it = leveltree_next(it, begin, end))
			count++;
		return count;
	}


	template<typename Iterator, typename PRE, typename PST>
	Iterator leveltree_walk(Iterator rec, Iterator begin, Iterator end, const PRE& pre, const PST& pst)
	{
		if (rec == end)
			return end;
		pre(rec, begin, end);
		for (Iterator it = leveltree_firstchild(rec, begin, end); it != end; it = leveltree_next(it, begin, end) )
			leveltree_walk(it, begin, end, pre, pst);
		pst(rec, begin, end);
		return leveltree_next(rec, begin, end);
	}


	template<typename Iterator, typename CB>
	Iterator leveltree_walkpre(Iterator rec, Iterator begin, Iterator end, const CB& cb)
	{
		if (rec == end)
			return end;
		cb(rec, begin, end);
		for (Iterator it = leveltree_firstchild(rec, begin, end); it != end; it = leveltree_next(it, begin, end) )
			leveltree_walkpre(it, begin, end, cb);
		return leveltree_next(rec, begin, end);
	}

	template<typename Iterator, typename CB>
	Iterator leveltree_walkpost(Iterator rec, Iterator begin, Iterator end, const CB& cb)
	{
		if (rec == end)
			return end;
		for (Iterator it = leveltree_firstchild(rec, begin, end); it != end; it = leveltree_next(it, begin, end) )
			leveltree_walkpost(it, begin, end, cb);
		cb(rec, begin, end);
		return leveltree_next(rec, begin, end);
	}


}

#endif
