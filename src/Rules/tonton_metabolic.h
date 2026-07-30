#ifndef TONTON_METABOLIC_H
#define TONTON_METABOLIC_H
#include <optional>

namespace TonTon
{

struct Input;
struct Scratch;
struct Analysis_Metabolic;

// What locomotion turned out to DEMAND, fed back into a second metabolic pass.
//
// The dependency is genuinely circular -- locomotion modes spend the metabolic
// budget, but the budget a creature evolved is a consequence of what it does for
// a living. Pass 1 runs with a clade-only budget and leaves this zeroed; the
// locomotion modes then publish their demands, and pass 2 asks whether a
// physiology exists that funds them. Because the only permitted move is an
// UPGRADE (ectotherm -> endotherm, higher aerobic scope), the loop is monotone
// and terminates after exactly two passes; it cannot oscillate.
struct MetabolicDemand
{
	// Sustained mechanical power that level flight requires, including reserve.
	// Zero if the creature has no wings or no aerial analysis.
	float sustained_flight_W{0.0f};
};

// Compute metabolic rates with multi-clade blending
// Called after physical analysis (clade flags available)
// Pass 1 runs before locomotion with a default-constructed demand; pass 2 (only
// when a demand went unmet) re-runs with what locomotion actually asked for.
Analysis_Metabolic ComputeMetabolic(Input const& in, Scratch & s,
                                    MetabolicDemand const& demand = {});

}

#endif // TONTON_METABOLIC_H
