#ifndef TONTON_OPTIONAL_HPP
#define TONTON_OPTIONAL_HPP
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <cstddef>
#include <span>

namespace TonTon
{

template<typename T>
class optional {
private:
    uint32_t _offset{};


public:
	optional() = default;
    ~optional() {
        if(_offset)
			((T*)((uint8_t*)this + _offset))->~T();
		_offset = 0;	
    }

	void load(std::span<uint8_t> & heap, std::optional<T> const& content)
	{
		if(content.has_value() == false) return;
		if(_offset) throw std::logic_error("tried to reconstruct tonton-optional that already has a value");
		
		if(!((void*)this < heap.data()+heap.size_bytes()))
		{
			throw std::logic_error("tried to reconstruct tonton-optional that does not live in the given memory arena");
		}
		
		uint8_t* where = heap.data();
		heap = heap.subspan(sizeof(T));
		_offset = (uint8_t*)where - (uint8_t*)this;
		
		new(&value()) T(content.value());
	}
	
    // Delete copy/move constructors and assignment operators
    // (we don't own the memory, just manage an object at a location)
    optional(const optional&) = delete;
    optional& operator=(const optional&) = delete;
    optional(optional&&) = delete;
    optional& operator=(optional&&) = delete;

    // std::optional-like interface
    inline bool has_value() const noexcept { return _offset; }
    inline explicit operator bool() const noexcept { return _offset; }

    inline T& value() {
        if (!_offset) throw std::bad_optional_access();
        return *((T*)((uint8_t*)this + _offset));
    }

    inline const T& value() const {
        if (!_offset) throw std::bad_optional_access();
        return *((T*)((uint8_t*)this + _offset));
    }

    inline T& operator*() & { return value(); }
    inline const T& operator*() const& { return value(); }

    inline T* operator->() { return &value(); }
    inline const T* operator->() const { return &value(); }

    template<typename U>
    T value_or(U&& default_value) const& {
        return _offset ? *value() : static_cast<T>(std::forward<U>(default_value));
    }

    template<typename U>
    T value_or(U&& default_value) && {
        return _offset ? std::move(*value()) : static_cast<T>(std::forward<U>(default_value));
    }

    // Size of managed memory
    static constexpr size_t size() noexcept { return (sizeof(T)); }
};

// Comparison operators
template<typename T>
bool operator==(const optional<T>& lhs, const optional<T>& rhs) {
    if (lhs.has_value() != rhs.has_value()) return false;
    if (!lhs.has_value()) return true;
    return *lhs == *rhs;
}

template<typename T>
bool operator!=(const optional<T>& lhs, const optional<T>& rhs) {
    return !(lhs == rhs);
}

template<typename T>
bool operator<(const optional<T>& lhs, const optional<T>& rhs) {
    if (!rhs.has_value()) return false;
    if (!lhs.has_value()) return true;
    return *lhs < *rhs;
}

// Compare with std::nullopt
template<typename T>
bool operator==(const optional<T>& opt, std::nullopt_t) noexcept {
    return !opt.has_value();
}

template<typename T>
bool operator==(std::nullopt_t, const optional<T>& opt) noexcept {
    return !opt.has_value();
}

// Compare with value
template<typename T, typename U>
bool operator==(const optional<T>& opt, const U& value) {
    return opt.has_value() && *opt == value;
}

template<typename T, typename U>
bool operator==(const U& value, const optional<T>& opt) {
    return opt.has_value() && value == *opt;
}

}

#endif // TONTON_OPTIONAL_HPP
