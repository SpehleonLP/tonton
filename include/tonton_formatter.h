#ifndef TONTON_FORMATTER_H
#define TONTON_FORMATTER_H
#include "tonton_output.h"
#include <ostream>

namespace TonTon
{

enum class Verbosity
{
	SILENT,
	BASIC,
	INFO,
	DETAILED,
};

// Output streaming operators
std::ostream& operator<<(std::ostream& os, const Output_Physical& p);
std::ostream& operator<<(std::ostream& os, const Output_Metabolic& m);
std::ostream& operator<<(std::ostream& os, const Output_Behavior& b);
std::ostream& operator<<(std::ostream& os, const Output_Sensory<optional>& s);
std::ostream& operator<<(std::ostream& os, const Output_Diagnostics& d);

std::ostream& operator<<(std::ostream& os, const Output_BodyWave& bw);

std::ostream& operator<<(std::ostream& os, const Output_Terrestrial::Leg& leg);
std::ostream& operator<<(std::ostream& os, const Output_Terrestrial::Gait& gait);
std::ostream& operator<<(std::ostream& os, const Output_Serpentine& serp);
std::ostream& operator<<(std::ostream& os, const Output_Terrestrial& t);

std::ostream& operator<<(std::ostream& os, const Output_Aerial::Wing& wing);
std::ostream& operator<<(std::ostream& os, const Output_Aerial& a);

std::ostream& operator<<(std::ostream& os, const Output_Aquatic::Fin& fin);
std::ostream& operator<<(std::ostream& os, const Output_Aquatic::CStartResponse& cstart);
std::ostream& operator<<(std::ostream& os, const Output_Aquatic::JetPropulsion& jet);
std::ostream& operator<<(std::ostream& os, const Output_Aquatic& aq);

std::ostream& operator<<(std::ostream& os, const Output_Climbing::Climber& limb);
std::ostream& operator<<(std::ostream& os, const Output_Climbing& c);

std::ostream& operator<<(std::ostream& os, const Output_Jumping& j);

std::ostream& operator<<(std::ostream& os, const Output_Manipulator& manip);

std::ostream& operator<<(std::ostream& os, const Output_Brachiation::Arm& arm);
std::ostream& operator<<(std::ostream& os, const Output_Brachiation& br);

std::ostream& operator<<(std::ostream& os, const Output_Tail::Branch& branch);
std::ostream& operator<<(std::ostream& os, const Output_Tail& tail);
std::ostream& operator<<(std::ostream& os, const Output_Antenna& ant);
std::ostream& operator<<(std::ostream& os, const Output::Appendages& app);

std::ostream& operator<<(std::ostream& os, const Output& output);

}

#endif // TONTON_FORMATTER_H
