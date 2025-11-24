#ifndef TONTON_FORMATTER_H
#define TONTON_FORMATTER_H
#include "tonton_analysis.h"
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
std::ostream& operator<<(std::ostream& os, const Analysis_Physical& p);
std::ostream& operator<<(std::ostream& os, const Analysis_Metabolic& m);
std::ostream& operator<<(std::ostream& os, const Analysis_Behavior& b);
std::ostream& operator<<(std::ostream& os, const Analysis_Sensory<optional>& s);
std::ostream& operator<<(std::ostream& os, const Analysis_Diagnostics& d);

std::ostream& operator<<(std::ostream& os, const Analysis_BodyWave& bw);

std::ostream& operator<<(std::ostream& os, const Analysis_Serpentine& serp);
std::ostream& operator<<(std::ostream& os, const Analysis_Terrestrial& t);

std::ostream& operator<<(std::ostream& os, const Analysis_Aerial::Wing& wing);
std::ostream& operator<<(std::ostream& os, const Analysis_Aerial& a);

std::ostream& operator<<(std::ostream& os, const Analysis_Aquatic::Fin& fin);
std::ostream& operator<<(std::ostream& os, const Analysis_Aquatic::CStartResponse& cstart);
std::ostream& operator<<(std::ostream& os, const Analysis_Aquatic::JetPropulsion& jet);
std::ostream& operator<<(std::ostream& os, const Analysis_Aquatic& aq);

std::ostream& operator<<(std::ostream& os, const Analysis_Climbing& c);

std::ostream& operator<<(std::ostream& os, const Analysis_Jumping& j);

std::ostream& operator<<(std::ostream& os, const Analysis_Manipulator& manip);

std::ostream& operator<<(std::ostream& os, const Analysis_Brachiation::Arm& arm);
std::ostream& operator<<(std::ostream& os, const Analysis_Brachiation& br);

std::ostream& operator<<(std::ostream& os, const Analysis_Tail& tail);
std::ostream& operator<<(std::ostream& os, const Output::Appendages& app);

std::ostream& operator<<(std::ostream& os, const Output& output);

std::ostream& operator<<(std::ostream& os, const Word& output);
std::ostream& operator<<(std::ostream& os, const SemanticFlags& output);
std::ostream& operator<<(std::ostream& os, const CladeFlags& output);

template<typename T>
std::ostream& operator<<(std::ostream& os, immutable_array<T> app) {
    os << '[';
    
    for(auto & item : app)
    {
		os << item;
    }
	
	os << ']';

    return os;
}

}

#endif // TONTON_FORMATTER_H
