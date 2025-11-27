#ifndef TONTON_CLIMBING_H
#define TONTON_CLIMBING_H
#include "tonton_wordlist.h"
#include <optional>
#include <vector>

namespace TonTon
{

struct Input;
struct Scratch;
struct Analysis_Manipulator;
struct Analysis_Appendage;
struct Analysis_Climbing;
struct Analysis_Brachiation;
struct Builder_Appendage;

std::optional<Analysis_Climbing>  ComputeClimbing(Input const& in, Scratch & s);
std::optional<Analysis_Brachiation> ComputeBrachiation(Input const& in, Scratch & s);
std::vector<Analysis_Manipulator>   ComputeManipulation(Input const& in, SemanticFlags requirements);
std::optional<TonTon::Analysis_Manipulator> ComputeManipulation(Input const& in, uint32_t idx);



}

#endif // TONTON_CLIMBING_H
