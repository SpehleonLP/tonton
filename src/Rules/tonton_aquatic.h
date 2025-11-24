#ifndef TONTON_AQUATIC_H
#define TONTON_AQUATIC_H
#include <optional>

namespace TonTon
{

struct Input;
struct Scratch;
struct Analysis_Aquatic;

std::optional<Analysis_Aquatic>   ComputeAquatic(Input const& in, Scratch &s);

}

#endif // TONTON_AQUATIC_H
