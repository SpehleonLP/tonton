#ifndef TONTON_SPECIALIZED_H
#define TONTON_SPECIALIZED_H

#include <optional>

namespace TonTon
{
struct Input;
struct Scratch;
struct Analysis_Digging;
struct Analysis_Constriction;

std::optional<Analysis_Digging> ComputeDigging(Input const& in, Scratch & s);
std::optional<Analysis_Constriction> ComputeConstriction(Input const& in, Scratch & s);

}

#endif // TONTON_SPECIALIZED_H
