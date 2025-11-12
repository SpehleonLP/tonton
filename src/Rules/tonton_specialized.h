#ifndef TONTON_SPECIALIZED_H
#define TONTON_SPECIALIZED_H

#include <optional>

namespace TonTon
{
struct Input;
struct Scratch;
struct Output_Digging;
struct Output_Constriction;

std::optional<Output_Digging> ComputeDigging(Input const& in, Scratch & s);
std::optional<Output_Constriction> ComputeConstriction(Input const& in, Scratch & s);

}

#endif // TONTON_SPECIALIZED_H
