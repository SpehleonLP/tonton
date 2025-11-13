#ifndef TONTON_CLIMBING_H
#define TONTON_CLIMBING_H
#include <optional>
#include <vector>

namespace TonTon
{

struct Input;
struct Scratch;
struct Output_Manipulator;
struct Output_Appendage;
struct Output_Climbing;
struct Output_Brachiation;

std::optional<Output_Climbing>  ComputeClimbing(Input const& in, Scratch & s);
std::optional<Output_Brachiation> ComputeBrachiation(Input const& in, Scratch & s);
std::vector<Output_Manipulator>   ComputeManipulation(Input const& in, std::vector<Output_Appendage> & appendages);

}

#endif // TONTON_CLIMBING_H
