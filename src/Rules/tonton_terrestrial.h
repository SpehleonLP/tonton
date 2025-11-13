#ifndef TONTON_TERRESTRIAL_H
#define TONTON_TERRESTRIAL_H
#include <optional>

namespace TonTon {

struct Scratch;
struct Scratch;
struct Output_Appendage;
struct Output_Terrestrial;
struct Output_Jumping;
struct Input;


std::optional<Output_Terrestrial>  ComputeTerrestrial(Input const& in, Scratch &out);
std::optional<Output_Jumping>  ComputeJumping(Input const& in, Scratch & s);


}

#endif // TONTON_TERRESTRIAL_H
