#ifndef TONTON_SENSORY_H
#define TONTON_SENSORY_H
#include "../../include/tonton_analysis.h"
#include <optional>

namespace TonTon
{

struct Input;
struct Scratch;

Analysis_Sensory<std::optional>  ComputeSensory(Input const& in, Scratch & s);

}

#endif // TONTON_SENSORY_H
