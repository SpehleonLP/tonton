#ifndef TONTON_AQUATIC_H
#define TONTON_AQUATIC_H
#include <optional>

namespace TonTon
{

struct Input;
struct Scratch;
struct Output_Aquatic;

std::optional<Output_Aquatic>   ComputeAquatic(Input const& in, Scratch & s);

}

#endif // TONTON_AQUATIC_H
