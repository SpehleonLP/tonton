#ifndef TONTON_CLIMBING_H
#define TONTON_CLIMBING_H
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

std::optional<Analysis_Climbing>  ComputeClimbing(Input const& in, Scratch & s);
std::optional<Analysis_Brachiation> ComputeBrachiation(Input const& in, Scratch & s);
std::vector<Analysis_Manipulator>   ComputeManipulation(Input const& in, std::vector<Analysis_Appendage> & appendages);

}

#endif // TONTON_CLIMBING_H
