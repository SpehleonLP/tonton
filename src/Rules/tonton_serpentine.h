#ifndef TONTON_SERPENTINE_H
#define TONTON_SERPENTINE_H

#include "../../include/tonton_analysis.h"
#include <optional>

namespace TonTon
{

struct Input;
struct Scratch;

/**
 * Compute serpentine locomotion capabilities
 *
 * Analyzes creature morphology to determine if it can perform terrestrial
 * undulation-based locomotion. This includes:
 * - True legless forms (snakes)
 * - Forms with vestigial legs (some lizards, skinks)
 * - Hybrid forms with arms but serpentine lower body (lamias)
 *
 * Serpentine locomotion is defined as "can move by undulating tail on land"
 * rather than strictly being legless.
 *
 * Prerequisites (must be computed before calling):
 * - physical: body mass, length, tail data
 * - sensory: sensory capabilities
 * - manipulation: limb/hand data
 * - tails: tail structure and capabilities
 * - terrestrial: leg-based locomotion (if present)
 *
 * References:
 * - Gray, J. (1936). Studies in animal locomotion VI. The propulsive powers
 *   of the dolphin. J. Exp. Biol. 13(2), 192-199.
 * - Garland, T., Jr. (1994). Quantitative genetics of locomotor behavior and
 *   physiology in a garter snake. In Quantitative Genetic Studies of
 *   Behavioral Evolution (pp. 251-277). University of Chicago Press.
 *
 * @param in Input armature, environment, and parameters
 * @param s Scratch workspace with previously computed outputs
 * @return Analysis_Serpentine data if capable, std::nullopt otherwise
 */
std::optional<Analysis_Serpentine> ComputeSerpentine(Input const& in, Scratch &s);

} // namespace TonTon

#endif // TONTON_SERPENTINE_H
