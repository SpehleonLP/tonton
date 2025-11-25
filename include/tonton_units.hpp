#ifndef TONTON_UNITS_HPP
#define TONTON_UNITS_HPP
#include <cmath>
#include <compare>

#include <cmath>
#include <compare>
#include <ratio>

namespace TonTon
{

// Compile-time dimensional analysis using template parameters
// Dimensions: Mass, Length, Time, Temperature, etc.
template<int M, int L, int T, int Temp = 0>
struct Quantity
{
	float value;
	
	// Constructors
	constexpr Quantity() : value(0.0f) {}
	constexpr Quantity(float v) : value(v) {}
	
	// Explicit conversion to float
	constexpr explicit operator float() const { return value; }
	
	// Same-dimension operations
	constexpr Quantity operator+(Quantity other) const { return Quantity(value + other.value); }
	constexpr Quantity operator-(Quantity other) const { return Quantity(value - other.value); }
	
	constexpr Quantity& operator+=(Quantity other) { value += other.value; return *this; }
	constexpr Quantity& operator-=(Quantity other) { value -= other.value; return *this; }
	
	constexpr Quantity operator-() const { return Quantity(-value); }
	
	// Scalar multiplication/division
	constexpr Quantity operator*(float scalar) const { return Quantity(value * scalar); }
	constexpr Quantity operator/(float scalar) const { return Quantity(value / scalar); }
	
	constexpr Quantity& operator*=(float scalar) { value *= scalar; return *this; }
	constexpr Quantity& operator/=(float scalar) { value /= scalar; return *this; }
	
	constexpr float operator/(Quantity scalar) const { return (value / scalar.value); }
	
	// Comparison operators
	constexpr auto operator<=>(const Quantity&) const = default;
	constexpr bool operator==(const Quantity&) const = default;
	
	// Comparison with float (dimensionless only)
	constexpr auto operator<=>(float other) const  {
		return value <=> other;
	}
};

// Scalar * Quantity
template<int M, int L, int T, int Temp>
constexpr Quantity<M, L, T, Temp> operator*(float scalar, Quantity<M, L, T, Temp> q) {
	return Quantity<M, L, T, Temp>(scalar * q.value);
}

// Quantity * Quantity (adds dimensions)
template<int M1, int L1, int T1, int Temp1, int M2, int L2, int T2, int Temp2>
constexpr auto operator*(Quantity<M1, L1, T1, Temp1> a, Quantity<M2, L2, T2, Temp2> b) {
	return Quantity<M1+M2, L1+L2, T1+T2, Temp1+Temp2>(a.value * b.value);
}

// Quantity / Quantity (subtracts dimensions)
template<int M1, int L1, int T1, int Temp1, int M2, int L2, int T2, int Temp2>
constexpr auto operator/(Quantity<M1, L1, T1, Temp1> a, Quantity<M2, L2, T2, Temp2> b) {
	return Quantity<M1-M2, L1-L2, T1-T2, Temp1-Temp2>(a.value / b.value);
}

// inverse
template<int M, int L, int T, int Temp>
constexpr auto operator/(float a, Quantity<M, L, T, Temp> b) {
	return Quantity<-M, -L, -T, -Temp>(a / b.value);
}

// Square root (halves all dimensions - only works for even dimensions)
template<int M, int L, int T, int Temp>
constexpr auto sqrt(Quantity<M, L, T, Temp> q) 
requires (M % 2 == 0 && L % 2 == 0 && T % 2 == 0 && Temp % 2 == 0)
{
	return Quantity<M/2, L/2, T/2, Temp/2>(std::sqrt(q.value));
}

// Named unit types - only define the ones you actually use!
using Dimensionless = Quantity<0, 0, 0>;

// Base units
using mass_kg = Quantity<1, 0, 0>;        // kg
using length_m = Quantity<0, 1, 0>;      // m
using time_s = Quantity<0, 0, 1>;        // s
using temp_K = Quantity<0, 0, 0, 1>; // K

// Derived units (only the ones you commonly store)
using area_m2 = Quantity<0, 2, 0>;         // m²
using volume_m3 = Quantity<0, 3, 0>;       // m³
using velocity_m_s = Quantity<0, 1, -1>;    // m/s
using acceleration_m_s2 = Quantity<0, 1, -2>; // m/s²
using force_N = Quantity<1, 1, -2>;       // N (kg⋅m/s²)
using energy_J = Quantity<1, 2, -2>;      // J (N⋅m = kg⋅m²/s²)
using power_W = Quantity<1, 2, -3>;       // W (J/s = kg⋅m²/s³)
using density_kg_m3 = Quantity<1, -3, 0>;     // kg/m³
using freq_Hz = Quantity<0, 0, -1>;   // Hz (1/s)

using inertia_kgm2 = Quantity<1, 2, 0>;      // kg⋅m²

using angle_rad = Quantity<0, 0, 0>;        // rad (dimensionless in SI)
using omega_rad_s = Quantity<0, 0, -1>; // rad/s

using pressure_Pa = Quantity<1, -1, -2>;   // Pa (N/m² = kg/(m⋅s²))
using viscosity_Pa_s = Quantity<1, -1, -1>; // Pa⋅s
using cost_W_kg = Quantity<0, 2, -3>; // W / N
using cost_W_N =  Quantity<0, 1, -1>; // same as velocity... weird.
using load_N_m2 = Quantity<1, 1, -4>;


} // namespace TonTon


#endif // TONTON_UNITS_HPP
