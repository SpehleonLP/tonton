#ifndef TONTON_AERIAL_H
#define TONTON_AERIAL_H
#include <optional>

namespace TonTon {

struct Scratch;
struct Analysis_Appendage;
struct Analysis_Aerial;
struct Input;

std::optional<Analysis_Aerial>  ComputeAerial(Input const& in);


} // namespace TonTon

#endif // TONTON_AERIAL_H
