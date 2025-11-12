#ifndef TONTON_BEHAVIOR_H
#define TONTON_BEHAVIOR_H

namespace TonTon
{
struct Input;
struct Scratch;
struct Output_Metabolic;
struct Output_Behavior;

Output_Metabolic    ComputeMetabolic(Input const& in, Scratch & s);
Output_Behavior   ComputeBehavior(Input const& in, Scratch & s);

}

#endif // TONTON_BEHAVIOR_H
