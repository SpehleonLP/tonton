#ifndef TONTON_BEHAVIOR_H
#define TONTON_BEHAVIOR_H

namespace TonTon
{
struct Input;
struct Scratch;
struct Analysis_Metabolic;
struct Analysis_Behavior;

Analysis_Metabolic    ComputeMetabolic(Input const& in, Scratch & s);
Analysis_Behavior   ComputeBehavior(Input const& in, Scratch & s);

}

#endif // TONTON_BEHAVIOR_H
