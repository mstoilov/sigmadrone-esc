#ifndef _SDMATH_H_
#define _SDMATH_H_

#include <complex>

namespace sdmath {

template<typename T>
inline T cross(const std::complex<T>& a, const std::complex<T>& b)
{
	return a.real() * b.imag() - a.imag() * b.real();
}

template<typename T>
inline T dot(const std::complex<T>& a, const std::complex<T>& b)
{
	return a.real() * b.real() + a.imag() * b.imag();
}

}

#endif /* _SDMATH_H_ */
