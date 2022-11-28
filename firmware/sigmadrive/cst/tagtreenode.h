#ifndef _TAGTREENODE_H_
#define _TAGTREENODE_H_

namespace cst
{

	struct TagTreeNode
	{
		enum Type {
			Begin = 0,
			End = 1
		};

		TagTreeNode() : type(Begin) {}
		TagTreeNode(Type typeval) : type(typeval) {}
		Type GetType() const { return type; }
		void SetType(Type val) { type = val; }

		Type type;
	};

	template<typename Iterator>
	Iterator tagtree_get(Iterator rec, Iterator begin, Iterator end, TagTreeNode::Type type)
	{
		ssize_t s = 0;
		if (rec->type == type) {
			return rec;
		} else if (rec->type == TagTreeNode::Type::Begin) {
			for (Iterator it = rec; it != end; it++) {
				if (it->type == TagTreeNode::Type::Begin)
					s++;
				else if (it->type == TagTreeNode::Type::End)
					s--;
				if (s == 0)
					return it;
			}

		} else if (rec->type == TagTreeNode::Type::End) {
			for (Iterator it = rec; it != std::prev(begin); it--) {
				if (it->type == TagTreeNode::Type::Begin)
					s++;
				else if (it->type == TagTreeNode::Type::End)
					s--;
				if (s == 0)
					return it;
			}

		}
		return end;
	}

	template<typename Iterator>
	Iterator tagtree_firstchild(Iterator rec, Iterator begin, Iterator end, TagTreeNode::Type type)
	{
		if (rec->type == TagTreeNode::Type::End)
			if ((rec = tagtree_get(rec, begin, end, TagTreeNode::Type::Begin)) == end)
				return rec;
		if (++rec == end)
			return rec;
		if (rec->type == TagTreeNode::Type::Begin)
			return tagtree_get(rec, begin, end, type);
		return end;
	}

	template<typename Iterator>
	Iterator tagtree_lastchild(Iterator rec, Iterator begin, Iterator end, TagTreeNode::Type type)
	{
		if (rec->type == TagTreeNode::Type::Begin)
			if ((rec = tagtree_get(rec, begin, end, TagTreeNode::Type::End)) == end)
				return rec;
		if (--rec == (begin - 1))
			return rec;
		if (rec->type == TagTreeNode::Type::End)
			return tagtree_get(rec, begin, end, type);
		return end;
	}

	template<typename Iterator>
	Iterator tagtree_next(Iterator rec, Iterator begin, Iterator end, TagTreeNode::Type type)
	{
		if (rec->type == TagTreeNode::Type::Begin)
			if ((rec = tagtree_get(rec, begin, end, TagTreeNode::Type::End)) == end)
				return rec;
		if (++rec == end)
			return rec;
		if (rec->type == TagTreeNode::Type::Begin)
			return tagtree_get(rec, begin, end, type);
		return end;
	}

	template<typename Iterator>
	Iterator tagtree_prev(Iterator rec, Iterator begin, Iterator end, TagTreeNode::Type type)
	{
		if (rec->type == TagTreeNode::Type::End)
			if ((rec = tagtree_get(rec, begin, end, TagTreeNode::Type::Begin)) == end)
				return rec;
		if (--rec == (begin - 1))
			return rec;
		if (rec->type == TagTreeNode::Type::End)
			return tagtree_get(rec, begin, end, type);
		return end;
	}

	template<typename Iterator>
	Iterator tagtree_parent(Iterator rec, Iterator begin, Iterator end, TagTreeNode::Type type)
	{
		Iterator last;
		if (rec->type == TagTreeNode::Type::End)
			if ((rec = tagtree_get(rec, begin, end, TagTreeNode::Type::Begin)) == end)
				return rec;
		for ( ;rec != end; rec = tagtree_next(rec, begin, end, TagTreeNode::Type::End))
			last = tagtree_get(rec, begin, end, TagTreeNode::Type::End);
		Iterator parnt = last + 1;
		if (parnt != end)
			return tagtree_get(parnt, begin, end, type);
		return end;
	}

	template<typename Iterator>
	size_t tagtree_nchildren(Iterator rec, Iterator begin, Iterator end)
	{
		size_t count = 0;
		for (Iterator it = tagtree_firstchild(rec, begin, end, TagTreeNode::Begin); it != end; it = tagtree_next(it, begin, end, TagTreeNode::Begin))
			count++;
		return count;
	}

	template<typename Iterator>
	size_t tagtree_childindex(Iterator rec, Iterator begin, Iterator end)
	{
		size_t n = 0;
		for (Iterator it = tagtree_prev(rec, begin, end, TagTreeNode::Begin); it != end; it = tagtree_prev(it, begin, end, TagTreeNode::Begin))
			n++;
		return n;
	}


}

#endif

