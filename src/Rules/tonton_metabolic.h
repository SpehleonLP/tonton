#ifndef TONTON_METABOLIC_H
#define TONTON_METABOLIC_H
#include <optional>

namespace TonTon
{

struct Input;
struct Scratch;
struct Analysis_Metabolic;

// Compute metabolic rates with multi-clade blending
// Called after physical analysis (clade flags available)
// Called before locomotion modes (they need metabolic budget)
Analysis_Metabolic ComputeMetabolic(Input const& in, Scratch & s);

}

#endif // TONTON_METABOLIC_H
