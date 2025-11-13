#ifndef TONTON_ARMATURE_MEMO_H
#define TONTON_ARMATURE_MEMO_H
#include "tonton_shared_array.hpp"
#include "../../include/tonton_wordlist.h"
#include <cstdint>
#include <mutex>
#include <span>

namespace TonTon
{

struct Armature;

struct ArmatureMemo
{
	struct FlagPair
	{
		SemanticFlags parent_flags;
		SemanticFlags child_flags;
	};

	struct CladeFlagPair
	{
		CladeFlags parent_flags;
		CladeFlags child_flags;
	};
	
	ArmatureMemo(Armature const* ptr);
	ArmatureMemo(ArmatureMemo const&) = delete;
	~ArmatureMemo();
		
	const Armature & in;

	std::vector<uint16_t> GetAllChildren(int32_t of, SemanticFlags including, SemanticFlags excluding = SemanticFlags::NONE, std::span<int32_t> not_in = {});
	std::vector<uint16_t> GetAllChildrenOfRoot(uint32_t begin);
	
	// each is indexed by bone.
	immutable_array<std::span<const uint16_t>> GetChildren();
	immutable_array<immutable_array<uint16_t>> GetDirectedGraph();
	immutable_array<SemanticFlags> GetSemanticFlags();
	immutable_array<CladeFlags> GetCladeFlags();
	immutable_array<FlagPair> GetRelativeFlags();
	immutable_array<CladeFlagPair> GetRelativeCladeFlags();
	immutable_array<uint16_t> GetDfsOrdering();
	immutable_array<uint16_t> GetLeaves();
	immutable_array<uint16_t> GetGcrTable();
//	immutable_array<Clique> GetCliques();
		
private:	
	std::recursive_mutex _mutex;
	
	immutable_array<std::span<const uint16_t>>	children;
	immutable_array<immutable_array<uint16_t>> directed_graph;
	immutable_array<SemanticFlags>             semantic_flags;
	immutable_array<CladeFlags>                clade_flags;
	immutable_array<FlagPair>                  relative_flags;
	immutable_array<CladeFlagPair>             relative_clade_flags;
	immutable_array<uint16_t>                  dfs_ordering;
	immutable_array<uint16_t>                  leaves;
	immutable_array<uint16_t>                  gcr_table;
//	immutable_array<Clique>                    cliques;
	
};


}

#endif
