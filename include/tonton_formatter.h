#ifndef TONTON_FORMATTER_H
#define TONTON_FORMATTER_H
#include "tonton_analysis.h"
#include "tonton_builder.h"
#include <string>

namespace TonTon
{

enum class Verbosity
{
	SILENT,
	BASIC,
	INFO,
	DETAILED,
};

// Format all Analysis types
std::string format(const Analysis_Physical& p);
std::string format(const Analysis_Metabolic& m);
std::string format(const Analysis_Behavior& b);
std::string format(const Analysis_Sensory<optional>& s);
std::string format(const Analysis_Diagnostics& d);

std::string format(const Analysis_BodyWave& bw);
std::string format(const Analysis_Serpentine& serp);
std::string format(const Analysis_Terrestrial& t);

std::string format(const Analysis_Aerial::Wing& wing);
std::string format(const Analysis_Aerial& a);

std::string format(const Analysis_Aquatic::Fin& fin);
std::string format(const Analysis_Aquatic::CStartResponse& cstart);
std::string format(const Analysis_Aquatic::JetPropulsion& jet);
std::string format(const Analysis_Aquatic& aq);

std::string format(const Analysis_Climbing& c);
std::string format(const Analysis_Jumping& j);
std::string format(const Analysis_Manipulator& manip);

std::string format(const Analysis_Brachiation::Arm& arm);
std::string format(const Analysis_Brachiation& br);

std::string format(const Analysis_Tail& tail);
std::string format(const Output::Appendages& app);

std::string format(const Output& output);

// Format Builder types
std::string format(const Builder_Chain& chain);
std::string format(const Builder::SemanticAnalysis& sa);
std::string format(const Builder::Physical& p);
std::string format(const Builder::Sensory::Vision::EyeInfo& eye);
std::string format(const Builder::Sensory::Vision& vision);
std::string format(const Builder::Sensory::Antennae& antennae);
std::string format(const Builder::Sensory& sensory);
std::string format(const Builder_Appendage& appendage);
std::string format(const Builder& builder);

// Format enum/flags types
std::string format(const Word& w);
std::string format(const SemanticFlags& flags);
std::string format(const CladeFlags& flags);
std::string format(const NicheFlags& flags);

// Single generic operator<< that works for ANY TonTon type with format()
template<typename T>
auto operator<<(std::ostream& os, const T& value)
  -> decltype(TonTon::format(value), os)  // SFINAE: only exists if format(value) is valid
{
  return os << format(value);
}
// Single generic operator<< that works for ANY TonTon type with format()
template<int M, int L, int T, int Temp, int Stage>
std::ostream& operator<<(std::ostream& os, const Quantity<M, L, T, Temp, Stage>& value)
{
  return os << float(value);
}

}

#if __cplusplus >= 202002L
#include <format>

// std::formatter specializations
template<>
struct std::formatter<TonTon::Analysis_Physical> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Physical& p, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(p), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Metabolic> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Metabolic& m, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(m), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Behavior> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Behavior& b, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(b), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Sensory<TonTon::optional>> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Sensory<TonTon::optional>& s, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(s), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Diagnostics> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Diagnostics& d, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(d), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_BodyWave> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_BodyWave& bw, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(bw), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Serpentine> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Serpentine& serp, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(serp), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Terrestrial> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Terrestrial& t, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(t), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Aerial::Wing> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Aerial::Wing& wing, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(wing), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Aerial> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Aerial& a, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(a), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Aquatic::Fin> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Aquatic::Fin& fin, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(fin), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Aquatic::CStartResponse> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Aquatic::CStartResponse& cstart, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(cstart), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Aquatic::JetPropulsion> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Aquatic::JetPropulsion& jet, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(jet), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Aquatic> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Aquatic& aq, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(aq), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Climbing> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Climbing& c, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(c), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Jumping> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Jumping& j, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(j), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Manipulator> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Manipulator& manip, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(manip), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Brachiation::Arm> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Brachiation::Arm& arm, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(arm), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Brachiation> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Brachiation& br, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(br), ctx);
    }
};

template<>
struct std::formatter<TonTon::Analysis_Tail> : std::formatter<std::string> {
    auto format(const TonTon::Analysis_Tail& tail, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(tail), ctx);
    }
};

template<>
struct std::formatter<TonTon::Output::Appendages> : std::formatter<std::string> {
    auto format(const TonTon::Output::Appendages& app, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(app), ctx);
    }
};

template<>
struct std::formatter<TonTon::Output> : std::formatter<std::string> {
    auto format(const TonTon::Output& output, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(output), ctx);
    }
};

// Builder formatters
template<>
struct std::formatter<TonTon::Builder_Chain> : std::formatter<std::string> {
    auto format(const TonTon::Builder_Chain& chain, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(chain), ctx);
    }
};

template<>
struct std::formatter<TonTon::Builder::SemanticAnalysis> : std::formatter<std::string> {
    auto format(const TonTon::Builder::SemanticAnalysis& sa, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(sa), ctx);
    }
};

template<>
struct std::formatter<TonTon::Builder::Physical> : std::formatter<std::string> {
    auto format(const TonTon::Builder::Physical& p, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(p), ctx);
    }
};

template<>
struct std::formatter<TonTon::Builder::Sensory::Vision::EyeInfo> : std::formatter<std::string> {
    auto format(const TonTon::Builder::Sensory::Vision::EyeInfo& eye, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(eye), ctx);
    }
};

template<>
struct std::formatter<TonTon::Builder::Sensory::Vision> : std::formatter<std::string> {
    auto format(const TonTon::Builder::Sensory::Vision& vision, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(vision), ctx);
    }
};

template<>
struct std::formatter<TonTon::Builder::Sensory::Antennae> : std::formatter<std::string> {
    auto format(const TonTon::Builder::Sensory::Antennae& antennae, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(antennae), ctx);
    }
};

template<>
struct std::formatter<TonTon::Builder::Sensory> : std::formatter<std::string> {
    auto format(const TonTon::Builder::Sensory& sensory, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(sensory), ctx);
    }
};

template<>
struct std::formatter<TonTon::Builder_Appendage> : std::formatter<std::string> {
    auto format(const TonTon::Builder_Appendage& appendage, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(appendage), ctx);
    }
};

template<>
struct std::formatter<TonTon::Builder> : std::formatter<std::string> {
    auto format(const TonTon::Builder& builder, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(builder), ctx);
    }
};

// Enum/flags formatters
template<>
struct std::formatter<TonTon::Word> : std::formatter<std::string> {
    auto format(const TonTon::Word& w, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(w), ctx);
    }
};

template<>
struct std::formatter<TonTon::SemanticFlags> : std::formatter<std::string> {
    auto format(const TonTon::SemanticFlags& flags, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(flags), ctx);
    }
};

template<>
struct std::formatter<TonTon::CladeFlags> : std::formatter<std::string> {
    auto format(const TonTon::CladeFlags& flags, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(flags), ctx);
    }
};

template<>
struct std::formatter<TonTon::NicheFlags> : std::formatter<std::string> {
    auto format(const TonTon::NicheFlags& flags, auto& ctx) const {
        return std::formatter<std::string>::format(TonTon::format(flags), ctx);
    }
};

// Generic formatters for templates
template<typename T>
struct std::formatter<immutable_array<T>> : std::formatter<std::string> {
    auto format(const immutable_array<T>& arr, auto& ctx) const {
        std::string result = "[";
        for (size_t i = 0; i < arr.size(); ++i) {
            if (i > 0) result += ", ";
            result += std::format("{}", arr[i]);
        }
        result += "]";
        return std::formatter<std::string>::format(result, ctx);
    }
};

template<int M, int L, int T, int Temp, int Stage>
struct std::formatter<TonTon::Quantity<M, L, T, Temp, Stage>> : std::formatter<float> {
    auto format(const TonTon::Quantity<M, L, T, Temp, Stage>& q, auto& ctx) const {
        return std::formatter<float>::format(static_cast<float>(q), ctx);
    }
};

#endif

#endif // TONTON_FORMATTER_H
