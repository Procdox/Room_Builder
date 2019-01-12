#pragma once

/*

Contains definition for a forward linked list template

*/

template <class _T>
class FLL;

template <class _T>
class FLL {
	struct FLL_node {

		_T value;
		FLL_node * next;
		FLL_node(_T v, FLL_node * n) {
			value = v;
			next = n;
		};

		//nodes should NEVER be copied or moved, they are purely internal to the creating list
		FLL_node(FLL_node &&) = delete;
		FLL_node(const FLL_node &) = delete;
	};

	FLL_node * head;
	FLL_node * tail;
	int length;

public:

	class FLL_iterator {
		FLL * relevant;
		FLL_node * focus;

	public:
		FLL_iterator(FLL * r, FLL_node * v) {
			relevant = r;
			focus = v;
		}
		FLL_iterator & operator=(FLL_iterator const & target) {
			relevant = target.relevant;
			focus = target.focus;

			return *this;
		}

		FLL_iterator & operator++() {
			focus = focus->next;

			return *this;
		}
		bool operator!=(FLL_iterator const & target) const {
			return focus != target.focus;
		}

		_T & operator*() const {
			return focus->value;
		}
		_T & operator->() const {
			return focus->value;
		}

		FLL_iterator next() const {
			return FLL_iterator(relevant, focus->next);
		}
		FLL_iterator cyclic_next() const {
			if (focus->next == nullptr) {
				return FLL_iterator(relevant, relevant->head);
			}
			else {
				return FLL_iterator(relevant, focus->next);
			}
		}
	};

	class FLL_iterator_c {
		FLL const * relevant;
		FLL_node const * focus;

	public:
		FLL_iterator_c(FLL const * r, FLL_node const * v) {
			relevant = r;
			focus = v;
		}
		FLL_iterator_c & operator=(FLL_iterator_c const & target) {
			relevant = target.relevant;
			focus = target.focus;
			return *this;
		}

		FLL_iterator_c & operator++() {
			focus = focus->next;

			return *this;
		}
		bool operator!=(FLL_iterator_c const & target) const {
			return focus != target.focus;
		}

		_T const & operator*() const {
			return focus->value;
		}
		_T const & operator->() const {
			return focus->value;
		}

		FLL_iterator_c next() const {
			return FLL_iterator_c(relevant, focus->next);
		}
		FLL_iterator_c cyclic_next() const {
			if (focus->next == nullptr) {
				return FLL_iterator_c(relevant, relevant->head);
			}
			else {
				return FLL_iterator_c(relevant, focus->next);
			}
		}
	};


	FLL() {
		head = nullptr;
		tail = nullptr;
		length = 0;
	};
	~FLL() {
		clear();
	};
	FLL(FLL<_T> && refernce) {
		head = refernce.head;
		tail = refernce.tail;
		length = refernce.length;

		refernce.head = nullptr;
		refernce.tail = nullptr;
		refernce.length = 0;

	}
	FLL(FLL<_T> const & reference) {
		FLL_node * focus = reference.head;
		head = nullptr;
		tail = nullptr;
		length = 0;

		while (focus != nullptr) {
			append(focus->value);

			focus = focus->next;
		}
	};

	FLL<_T> & operator=(FLL<_T> const & reference) {
		FLL_node * focus = reference.head;
		head = nullptr;
		tail = nullptr;
		length = 0;

		while (focus != nullptr) {
			append(focus->value);

			focus = focus->next;
		}

		return *this;
	};

	void push(_T value) {
		head = new FLL_node(value, head);

		if (tail == nullptr) tail = head;

		length++;
	};

	void append(_T value) {
		if (head != nullptr) {
			tail->next = new FLL_node(value, nullptr);
			tail = tail->next;
		}
		else {
			head = new FLL_node(value, nullptr);
			tail = head;
		}

		length++;
	}
	void append(FLL<_T> reference) {
		FLL_node * focus = reference.head;

		while (focus != nullptr) {
			append(focus->value);

			focus = focus->next;
		}
	}

	//returns the element at head, undefined behavior if empty
	_T pop() {
		_T product = head->value;
		FLL_node * to_be = head->next;

		delete head;

		head = to_be;

		if (head == nullptr) tail = nullptr;

		length--;

		return product;
	};

	

	bool empty() {
		return head == nullptr;
	}
	bool contains(_T search) const {
		FLL_node* focus = head;

		while (focus != nullptr) {
			if (focus->value == search) return true;
			focus = focus->next;
		}

		return false;
	};

	/*FLL_node * getHead() {
		return head;
	};
	FLL_node const * getHead() const {
		return head;
	};

	FLL_node * getTail() {
		return tail;
	};
	FLL_node const * getTail() const {
		return tail;
	};*/

	FLL_iterator begin() {
		return FLL_iterator(this, head);
	}
	FLL_iterator end() {
		return FLL_iterator(this, nullptr);
	}

	FLL_iterator_c begin() const {
		return FLL_iterator_c(this, head);
	}
	FLL_iterator_c end() const {
		return FLL_iterator_c(this, nullptr);
	}

	_T last() const {
		return tail->value;
	}

	FLL<_T> reverse() const {
		FLL<_T> product;

		FLL_node * focus = head;

		while (focus != nullptr) {
			product.push(focus->value);
			focus = focus->next;
		}

		return product;
	}

	bool remove(_T search) {
		FLL_node * focus = head;
		FLL_node * after;

		//special cases

		if (head == nullptr) return false;

		if (head->value == search) {
			head = head->next;
			if (head == nullptr) tail = nullptr;

			delete focus;
			length--;

			return true;
		}

		//general case

		focus = head;
		after = head->next;

		while (after != nullptr) {

			if (after->value == search) {
				focus->next = after->next;

				if (tail == after) {
					tail = focus;
				}

				delete after;
				length--;

				after = focus->next;
			}
			else {
				focus = after;
				after = after->next;
			}
		}

		return false;
	};

	int removeAll(_T search) {
		FLL_node * focus;
		FLL_node * after;
		int count = 0;

		//special cases

		while (head != nullptr && head->value == search) {
			focus = head->next;

			delete head;
			count++;

			head = focus;
		}

		if (head == nullptr) {
			tail = nullptr;
			return count;
		}

		//general case

		focus = head;
		after = head->next;

		while (after != nullptr) {

			if (after->value == search) {
				focus->next = after->next;

				if (tail == after) {
					tail = focus;
				}

				delete after;
				count++;

				after = focus->next;
			}
			else {
				focus = after;
				after = after->next;
			}
		}

		length -= count;

		return count;
	};

	//appends the target list, and empties it
	void absorb(FLL<_T> & target) {
		if (target.head != nullptr) {

			if (head == nullptr) {
				head = target.head;
			}
			else {
				tail->next = target.head;
			}

			tail = target.tail;

			length += target.length;

			target.head = nullptr;
			target.tail = nullptr;
			target.length = 0;
		}
	}

	int size() const {
		FLL_node * focus = head;
		int s = 0;
		
		while (focus != nullptr) {
			s++;
			focus = focus->next;
		}

		return s;
	}

	int l() const {
		return l;
	}

	void qInsert(_T value, bool (*compare)(_T, _T)) {
		FLL_node * focus = head;
		FLL_node * after;

		if (head == nullptr) {
			push(value);
			return;
		}

		if (compare(head->value, value)) {
			push(value);
			return;
		}

		after = head->next;
		
		while (after != nullptr) {
			if (compare(after->value, value)) {

				focus->next = new FLL_node(value, after);
				length++;
				return;
			}

			focus = after;
			after = after->next;
		}

		append(value);
	}
	void clear() {
		auto focus = head;

		while (head != nullptr) {
			focus = head->next;
			delete head;
			head = focus;
		}

		tail = nullptr;

		length = 0;
	};

	_T operator[](int index) {
		FLL_node * focus = head;

		while (focus != nullptr && index > 0) {
			focus = focus->next;
			index-=1;
		}

		return focus->value;
	}
};