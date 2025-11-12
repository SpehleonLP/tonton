#ifndef SIZEDCOUNTEDARRAY_H
#define SIZEDCOUNTEDARRAY_H
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <algorithm>

//#pragma GCC push_options
//#pragma GCC optimize ("O3")

static_assert(sizeof(std::atomic<int>) == sizeof(int));

template<typename T>
struct CountedArray
{
	CountedArray(void * heap, uint32_t size)
	{
		new(&refCount()) std::atomic<int>(1);
		*((int32_t*)this - 3) =  (uint8_t*)this - (uint8_t*)heap;
		*((uint32_t*)this - 1) = size;

		//ensure array is 16 byte aligned
		assert(((intptr_t)&m_data[0] & 0x0F) == 0);

		for(uint32_t i = 1; i < size; ++i)
			new(&m_data[i]) T();

		assert((void*)&m_data[0] == (void*)this);
	}
	template<typename S=T>
	CountedArray(void * heap, uint32_t size, S value)
	{
		new(&refCount()) std::atomic<int>(1);
		*((int32_t*)this - 3) =  (uint8_t*)this - (uint8_t*)heap;
		*((uint32_t*)this - 1) = size;

		//ensure array is 16 byte aligned
		assert(((intptr_t)&m_data[0] & 0x0F) == 0);

		m_data[0] = value;
		for(uint32_t i = 1; i < size; ++i)
		{
			new(&m_data[i]) T();
			m_data[i] = value;
		}

		assert((void*)&m_data[0] == (void*)this);
	}
	~CountedArray()
	{
		(&refCount())->~atomic();

		for(uint32_t i = 1; i < size(); ++i)
			m_data[i].~T();

	}

	inline void AddRef() { ++refCount(); }
	inline void Release() { if(--refCount() == 0) { this->~CountedArray(); std::free((uint8_t*)this - offsetFromFront()); } }

	inline int32_t            offsetFromFront() const { return *((int32_t*)this - 3); }
	inline uint32_t           size() const { return *((uint32_t*)this - 1); }
	inline std::atomic<int> & refCount() { return *((std::atomic<int>*)this - 2); }

	T                m_data[1];
};

template<typename T, int alignment>
union immutable_array;

///refcounted array of fixed size; data[0] is guaranteed to be 16 byte aligned.
template<typename T, int alignment = 16>
union shared_array
{
typedef CountedArray<T> Array;
	friend union immutable_array<T, alignment>;
public:
friend class asFixedArray;
typedef T value_type;
typedef shared_array<T, alignment> self_type;

static_assert(alignment >= 16);

enum
{
	ByteAlignment = alignment,
	BytePadding   = 16 + ByteAlignment
};

	inline shared_array(Array * it) { _pointer = it; if(it) it->AddRef(); };
	inline shared_array() { _pointer = nullptr; };
	inline shared_array(shared_array && it) { _pointer = it._pointer; it._pointer = nullptr; }
	inline shared_array(shared_array const& it)  { _pointer = it._pointer; if(_pointer) _pointer->AddRef(); }
	inline ~shared_array() { if(_pointer) _pointer->Release(); };

	explicit shared_array(size_t size)
	{
		_pointer = 0;

		if(size)
		{
			auto _size = sizeof(T) * (size) + BytePadding + ByteAlignment;
			void * heap = std::malloc(_size);
			void * centered = (void*)(((intptr_t)heap + (BytePadding-1)) & ~(intptr_t)(ByteAlignment-1));
			_pointer = new(centered) Array(heap, size);
		}
	}

	template<typename S=T>
	shared_array(size_t size, S value)
	{
		_pointer = 0;

		if(size)
		{
			void * heap = std::malloc(sizeof(T) * (size) + BytePadding + ByteAlignment);
			void * centered = (void*)(((intptr_t)heap + (BytePadding-1)) & ~(intptr_t)(ByteAlignment-1));
			assert(heap <= centered && centered <= (uint8_t*)heap + BytePadding);
			assert((intptr_t)centered % ByteAlignment == 0);

			_pointer = new(centered) Array(heap, size, value);
		}
	}

	template<typename S>
	static self_type FromArray(S * begin, size_t size)
	{
		if(!size) return self_type();

		shared_array r(size);

		for(size_t i = 0; i < size; ++i)
			r[i] = begin[i];

		return r;
	}

	template<typename S>
	static self_type FromArray(S * begin, S * end)
	{
		if((end - begin) <= 0)
			 return self_type();

		shared_array r(end - begin);

		for(size_t i = 0; i < r.size(); ++i)
			r[i] = begin[i];

		return r;
	}

	template<typename S>
	static self_type FromArray(S const& array)
	{
		return FromArray(array.data(), array.size());
	}

	template<typename Lambda>
	static self_type Build(size_t size, Lambda labmda)
	{
		if(!size) return self_type();

		shared_array r(size);

		for(size_t i = 0; i < size; ++i)
			r[i] = labmda(i);

		return r;
	}

	inline shared_array clone() const
	{
		if(empty())
			return *this;

		return FromArray(&at(0), size());
	}
#if 0
	template<typename S=T>
	inline shared_array reset(size_t size, S value)
	{
		return *this = shared_array(size, value);
	}

	template<typename S=T>
	inline shared_array reset(S * begin, S * end)
	{
		return *this = shared_array::FromArray(begin, end);
	}

	template<typename S=T>
	inline shared_array reset(S * begin, size_t size)
	{
		return *this = shared_array::FromArray(begin, size);
	}

	template<typename S>
	inline shared_array reset(S const& array)
	{
		return *this = shared_array::FromArray(array.data(), array.size());
	}
#endif

	inline bool operator==(self_type const& it) const { return _pointer == it._pointer; }
	inline bool operator!=(self_type const& it) const { return _pointer != it._pointer; }

	inline self_type & operator=(shared_array const& it)
	{
		auto old = _pointer;
		_pointer = it._pointer;

		if(_pointer) _pointer->AddRef();
		if(old) old->Release();

		return *this;
	}

	inline self_type & operator=(shared_array && it)
	{
		auto old = _pointer;
		_pointer = it._pointer;

		if(old) old->Release();
		it._pointer = nullptr;
		return *this;
	}

	int find(T it) const
	{
		for(auto i = 0u; i < size(); ++i)
			if(it == (((const shared_array*)this)->at(i)))
				return i;

		return -1;
	}

	template<typename Lambda>
	int find(Lambda lambda) const
	{
		for(auto i = 0u; i < size(); ++i)
			if(lambda(((const shared_array*)this)->at(i)))
				return i;

		return -1;
	}

	template<typename Lambda>
	int findClosest(Lambda lambda) const
	{
		int distance = 0x7FFFFFFF;
		int best = -1;

		for(auto i = 0u; i < size(); ++i)
		{
            int r = lambda(((const shared_array*)this)->at(i));

			if(r == 0) return i;
			if(r < distance)
			{
				distance = r;
				best = i;
			}
		}

		return best;
	}


	inline uint32_t size()  const { return _pointer? _pointer->size() : 0uL; }
	inline uint32_t byteLength()  const { return size() * sizeof(T); }
	inline bool     empty() const { return _pointer == nullptr; }
	inline void     clear() { if(_pointer) { _pointer->Release(); _pointer = nullptr; } }

	inline int GetRefCount() const { return _pointer->refCount(); }

	inline       value_type & at(uint32_t i) { return _pointer->m_data[i]; }
	inline const value_type & at(uint32_t i) const { return _pointer->m_data[i]; }

	inline       value_type & operator[](uint32_t i) { return _pointer->m_data[i]; }
	inline const value_type & operator[](uint32_t i) const { return _pointer->m_data[i]; }

	inline 	     value_type * begin()       { return (_pointer->m_data); }
	inline const value_type * begin() const { return (_pointer->m_data); }

	inline 	     value_type * end()       { return begin() + size(); }
	inline const value_type * end() const { return begin() + size(); }

	inline       value_type & front()       { return _pointer->m_data[0]; }
	inline const value_type & front() const { return _pointer->m_data[0]; }

	inline 	  value_type & back()       { return _pointer->m_data[_pointer->size()-1]; }
	inline const value_type & back() const { return _pointer->m_data[_pointer->size()-1]; }

	inline bool IsAligned(int value) const { return (intptr_t)_pointer % value == 0 && byteLength() % value < ByteAlignment; }
	inline bool contains(void const* ptr) const { return begin() <= ptr && ptr < end(); }

	inline void swap(self_type & it)
	{
		auto old = _pointer;
		_pointer = it._pointer;
		it._pointer = old;
	}

	inline bool merge(self_type const& it)
	{
		if(CanMerge(it))
		{
			*this = it;
			return true;
		}

		return false;
	}

	inline bool CanMerge(self_type const& it) const
	{
		if(_pointer == it._pointer)
			return true;

		if(_pointer->size() != it._pointer->size()
		|| memcmp((_pointer->m_data),(it._pointer->m_data), sizeof(value_type) * _pointer->size()))
			return false;

		return true;
	}

	inline T       * data()       { return _pointer->m_data; }
	inline T  const* data() const { return _pointer->m_data; }

private:
	Array * _pointer;
	T * _contents;
};

template<typename T, int alignment = 16>
union immutable_array
{
typedef CountedArray<T> Array;
public:
friend class asFixedArray;
typedef T value_type;
typedef immutable_array<T, alignment> self_type;
typedef shared_array<T, alignment> non_const_type;

	enum
	{
		ByteAlignment = alignment,
		BytePadding   = 16 + ByteAlignment
	};

	inline bool IsAligned(int value) const { return (intptr_t)_pointer % value == 0 && byteLength() % value < ByteAlignment; }

	template<typename S>
	static self_type FromArray(S * begin, size_t size)
	{
		if(!size) return self_type();

		shared_array<T> r(size);

		for(size_t i = 0; i < size; ++i)
			r[i] = begin[i];

		return r;
	}

	inline immutable_array() { _pointer = nullptr; };
	inline immutable_array(self_type && it) { _pointer = it._pointer; it._pointer = nullptr; }
	inline immutable_array(self_type const& it)  { _pointer = it._pointer; if(_pointer) _pointer->AddRef(); }
	inline immutable_array(non_const_type && it) { _pointer = it._pointer; it._pointer = nullptr; }
	inline immutable_array(non_const_type const& it)  { _pointer = it._pointer; if(_pointer) _pointer->AddRef(); }
	inline ~immutable_array() { if(_pointer) _pointer->Release(); };

	non_const_type clone() const
	{
		if(empty())
			return non_const_type();

		return non_const_type::FromArray(&at(0), size());
	}

	template<typename Lambda>
	int find(Lambda lambda) const
	{
		for(auto i = 0u; i < size(); ++i)
					if(lambda(((const immutable_array*)this)->at(i)))
				return i;

		return -1;
	}

	int find(T it) const
	{
		for(auto i = 0u; i < size(); ++i)
					if(it == (((const immutable_array*)this)->at(i)))
				return i;

		return -1;
	}

	template<typename Lambda>
	int findClosest(Lambda lambda) const
	{
		size_t distance = ~0llu;
		int best = -1;

		for(auto i = 0u; i < size(); ++i)
		{
					auto r = lambda(((const immutable_array*)this)->at(i));

			if(r == 0) return i;
			if(r < distance)
			{
				distance = r;
				best = i;
			}
		}

		return best;
	}

	inline bool operator==(non_const_type const& it) const { return _pointer == it._pointer; }
	inline bool operator!=(non_const_type const& it) const { return _pointer != it._pointer; }
	inline bool operator==(self_type const& it) const { return _pointer == it._pointer; }
	inline bool operator!=(self_type const& it) const { return _pointer != it._pointer; }

	inline self_type & operator=(self_type const& it)
	{
		auto old = _pointer;
		_pointer = it._pointer;

		if(_pointer) _pointer->AddRef();
		if(old) old->Release();

		return *this;
	}

	inline self_type & operator=(self_type && it)
	{
		auto old = _pointer;
		_pointer = it._pointer;

		if(old) old->Release();
		it._pointer = nullptr;
		return *this;
	}

	inline self_type & operator=(non_const_type const& it)
	{
		auto old = _pointer;
		_pointer = it._pointer;

		if(_pointer) _pointer->AddRef();
		if(old) old->Release();

		return *this;
	}

	inline self_type & operator=(non_const_type && it)
	{
		auto old = _pointer;
		_pointer = it._pointer;

		if(old) old->Release();
		it._pointer = nullptr;
		return *this;
	}

	inline uint32_t size()  const { return _pointer? _pointer->size() : 0uL; }
	inline uint32_t byteLength()  const { return size() * sizeof(T); }
	inline bool     empty() const { return _pointer == nullptr; }
	inline void     clear() { if(_pointer) { _pointer->Release(); _pointer = nullptr; } }

	inline int GetRefCount() const { return _pointer->refCount(); }

	inline const value_type & at(uint32_t i) const { return _pointer->m_data[i]; }
	inline const value_type & operator[](uint32_t i) const { return _pointer->m_data[i]; }

	inline const value_type * begin() const { return (_pointer->m_data); }
	inline const value_type * end() const { return begin() + size(); }

	inline const value_type & front() const { return _pointer->m_data[0]; }
	inline const value_type & back() const { return _pointer->m_data[_pointer->size()-1]; }

	inline bool contains(void const* ptr) const { return begin() <= ptr && ptr < end(); }

	inline void swap(self_type & it)
	{
		auto old = _pointer;
		_pointer = it._pointer;
		it._pointer = old;
	}

	inline bool merge(self_type const& it)
	{
		if(CanMerge(it))
		{
			*this = it;
			return true;
		}

		return false;
	}

	inline bool CanMerge(self_type const& it) const
	{
		if(_pointer == it._pointer)
			return true;

		if(_pointer->size() != it._pointer->size()
		|| memcmp((_pointer->m_data),(it._pointer->m_data), sizeof(value_type) * _pointer->size()))
			return false;

		return true;
	}

	inline T  const* data() const { return _pointer->m_data; }

private:
	Array * _pointer;
	T * _contents;
};

template<typename T>
inline T max(immutable_array<T> const& it)
{
	if(it.empty())
		return {};

	auto k = it[0];

	for(auto const& item : it)
		k = std::max(k, item);

	return k;
}

template<typename T>
inline T min(immutable_array<T> const& it)
{
	if(it.empty())
		return {};

	auto k = it[0];

	for(auto const& item : it)
        k = min(k, item);

	return k;
}

template<typename T>
inline T max(shared_array<T> const& it)
{
	if(it.empty())
		return {};

	auto k = it[0];

	for(auto const& item : it)
		k = std::max(k, item);

	return k;
}

template<typename T>
inline T min(shared_array<T> const& it)
{
	if(it.empty())
		return {};

	auto k = it[0];

	for(auto const& item : it)
		k = std::min(k, item);

	return k;
}

template<typename T, typename Lambda>
inline shared_array<T> BuildArray(size_t size, Lambda labmda)
{
	return shared_array<T>::Build(size, std::move(labmda));
}

template<typename T> inline shared_array<T> shared_storage(T const* begin, size_t size) { return shared_array<T>::FromArray(begin, size); }
template<typename T> inline shared_array<T> shared_storage(T const* begin, T const* end) { return shared_array<T>::FromArray(begin, end); }


//#pragma GCC pop_options

#endif // SIZEDCOUNTEDARRAY_H
