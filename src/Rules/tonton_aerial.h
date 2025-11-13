#ifndef TONTON_AERIAL_H
#define TONTON_AERIAL_H
#include <optional>

namespace TonTon {

struct Scratch;
struct Output_Appendage;
struct Output_Aerial;
struct Input;

std::optional<Output_Aerial>  ComputeAerial(Input const& in, Scratch &out);


} // namespace TonTon

#endif // TONTON_AERIAL_H
