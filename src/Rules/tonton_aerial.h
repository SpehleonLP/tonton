#ifndef TONTON_AERIAL_H
#define TONTON_AERIAL_H
#include "../../include/tonton_output.h"
#include <span>

namespace TonTon {

struct Scratch;

std::optional<Output_Aerial>  ComputeAerial(Input const& in, Scratch &out);


std::vector<glm::vec3> GetGaitGroupCenters(Input const& in, Output_Appendage * data, size_t size, size_t stride);

template<typename T>
inline std::vector<glm::vec3> GetGaitGroupCenters(Input const& input, std::span<T> const& in)
{
	return GetGaitGroupCenters(input, static_cast<Output_Appendage*>(in.data()), in.size(), sizeof(T));
}

} // namespace TonTon

#endif // TONTON_AERIAL_H
