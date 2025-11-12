#ifndef TONTON_SENSORY_H
#define TONTON_SENSORY_H
#include "../../include/tonton_output.h"
#include <optional>

namespace TonTon
{

struct Input;
struct Scratch;

Output_Sensory<std::optional>  ComputeSensory(Input const& in, Scratch & s);

}

#endif // TONTON_SENSORY_H
