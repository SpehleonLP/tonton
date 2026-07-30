#ifndef TONTON_BEHAVIOR_H
#define TONTON_BEHAVIOR_H

namespace TonTon
{
struct Input;
struct Scratch;
struct Analysis_Behavior;

// ComputeMetabolic is declared in tonton_metabolic.h -- it used to be duplicated
// here, which became an ambiguous overload once it took a demand argument.
Analysis_Behavior   ComputeBehavior(Input const& in, Scratch & s);

}

#endif // TONTON_BEHAVIOR_H
