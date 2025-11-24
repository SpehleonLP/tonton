#ifndef TONTON_SCRATCH_H
#define TONTON_SCRATCH_H
#include "../include/tonton_analysis.h"
#include <optional>

namespace TonTon
{

struct Input;

struct Scratch
{
	Scratch(Input const& in);	
	
	Analysis_Physical physical{};
	Analysis_Metabolic metabolic{};
	Analysis_Behavior  behavior{};
	Analysis_Sensory<std::optional>   sensory{};
	
	struct Appendages {
		immutable_array<Analysis_Tail> tails{};  // usually 1, but kitsune has 9!
		immutable_array<Analysis_Manipulator> manipulation{};
	} appendages{};
	
	std::optional<Analysis_Terrestrial> terrestrial{};
	std::optional<Analysis_Serpentine> serpentine{};
	std::optional<Analysis_Aerial> aerial{};
	std::optional<Analysis_Aquatic> aquatic{};
	std::optional<Analysis_Climbing> climbing{};
	std::optional<Analysis_Brachiation> brachiation{};
	std::optional<Analysis_Jumping> jumping{};

	struct SpecializedBehaviors {		
		std::optional<Analysis_Digging> digging{};
		std::optional<Analysis_Constriction> constriction{};
	} specialized{};
		
	Analysis_Diagnostics diagnostics;
};

std::vector<Analysis_Chain> GetChainsFromRoot(Input const& in, SemanticFlags flags, SemanticFlags child_flags = SemanticFlags::NONE);
std::vector<Analysis_Chain> GetChainsFromRoot(Input const& in, std::span<Word> words);
std::vector<Analysis_Appendage> GetAppendages(Input const& in, std::vector<Analysis_Chain> && chains);

std::vector<glm::vec3> GetGaitGroupCenters(Input const& in, Analysis_Appendage * data, size_t size, size_t stride);

template<typename T>
inline std::vector<glm::vec3> GetGaitGroupCenters(Input const& input, std::span<T> const& in)
{
	return GetGaitGroupCenters(input, static_cast<Analysis_Appendage*>(in.data()), in.size(), sizeof(T));
}

}

#endif // TONTON_SCRATCH_H
