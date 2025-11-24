#ifndef TONTON_TERRESTRIAL_H
#define TONTON_TERRESTRIAL_H
#include <optional>

namespace TonTon {

struct Scratch;
struct Scratch;
struct Analysis_Appendage;
struct Analysis_Terrestrial;
struct Analysis_Jumping;
struct Input;


std::optional<Analysis_Terrestrial>  ComputeTerrestrial(Input const& in, Scratch &out);
std::optional<Analysis_Jumping>  ComputeJumping(Input const& in, Scratch & s);


}

#endif // TONTON_TERRESTRIAL_H
