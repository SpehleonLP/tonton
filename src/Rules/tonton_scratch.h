#ifndef TONTON_SCRATCH_H
#define TONTON_SCRATCH_H
#include "../include/tonton_output.h"
#include <optional>

namespace TonTon
{

struct Input;

struct Scratch
{
	Scratch(Input const& in);	
	
	Output_Physical physical{};
	Output_Metabolic metabolic{};
	Output_Behavior  behavior{};
	Output_Sensory<std::optional>   sensory{};
	
	struct Appendages {
		immutable_array<Output_Tail> tails{};  // usually 1, but kitsune has 9!
		immutable_array<Output_Manipulator> manipulation{};
	} appendages{};
	
	std::optional<Output_Terrestrial> terrestrial{};
	std::optional<Output_Serpentine> serpentine{};
	std::optional<Output_Aerial> aerial{};
	std::optional<Output_Aquatic> aquatic{};
	std::optional<Output_Climbing> climbing{};
	std::optional<Output_Brachiation> brachiation{};
	std::optional<Output_Jumping> jumping{};

	struct SpecializedBehaviors {		
		std::optional<Output_Digging> digging{};
		std::optional<Output_Constriction> constriction{};
	} specialized{};
		
	Output_Diagnostics diagnostics;
};

std::vector<Output_Chain> GetChainsFromRoot(Input const& in, SemanticFlags flags);
std::vector<Output_Chain> GetChainsFromRoot(Input const& in, std::span<Word> words);
std::vector<Output_Chain> GetChainsFromTip(Input const& in, SemanticFlags include_flags, SemanticFlags exclude_flags);
std::vector<Output_Appendage> GetAppendages(Input const& in, std::vector<Output_Chain> && chains);

std::vector<glm::vec3> GetGaitGroupCenters(Input const& in, Output_Appendage * data, size_t size, size_t stride);

template<typename T>
inline std::vector<glm::vec3> GetGaitGroupCenters(Input const& input, std::span<T> const& in)
{
	return GetGaitGroupCenters(input, static_cast<Output_Appendage*>(in.data()), in.size(), sizeof(T));
}

}

#endif // TONTON_SCRATCH_H
