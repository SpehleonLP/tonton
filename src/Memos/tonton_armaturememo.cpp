#include "tonton_armaturememo.h"
#include "../../include/tonton_input.h"

TonTon::ArmatureMemo::ArmatureMemo(Armature const* ptr) : in(*ptr)
{
}

TonTon::ArmatureMemo::~ArmatureMemo() = default;

immutable_array<TonTon::ArmatureMemo::FlagPair> TonTon::ArmatureMemo::GetRelativeFlags()
{
	std::lock_guard lock(_mutex);
	
	if(relative_flags.size())
		return relative_flags;

	auto dfs_ordering = GetDfsOrdering();
	auto tags = GetSemanticFlags();
	auto children = GetChildren();

	shared_array<TonTon::ArmatureMemo::FlagPair> flags(dfs_ordering.size(), FlagPair{SemanticFlags::NONE, SemanticFlags::NONE});
		
	for(auto i = 0u; i < dfs_ordering.size(); ++i)
	{
		auto j = dfs_ordering[(dfs_ordering.size()-1)-i];
		auto p = in.parents[j];
		
		if(p >= 0)
		{
			flags[p].child_flags |= (flags[j].child_flags | tags[j]); 
		}
		
		// root to leaves
		j = dfs_ordering[i];
		p = in.parents[j];
		
		if(p >= 0)
		{
			flags[j].parent_flags |= (flags[p].parent_flags | tags[p]); 
		}
	}
	
	return (relative_flags = flags);
}

immutable_array<TonTon::ArmatureMemo::CladeFlagPair> TonTon::ArmatureMemo::GetRelativeCladeFlags()
{
	std::lock_guard lock(_mutex);
	
	if(relative_flags.size())
		return relative_clade_flags;

	auto dfs_ordering = GetDfsOrdering();
	auto tags = GetCladeFlags();
	auto children = GetChildren();

	shared_array<TonTon::ArmatureMemo::CladeFlagPair> flags(dfs_ordering.size(), CladeFlagPair{CladeFlags::NONE, CladeFlags::NONE});
		
	for(auto i = 0u; i < dfs_ordering.size(); ++i)
	{
		auto j = dfs_ordering[(dfs_ordering.size()-1)-i];
		auto p = in.parents[j];
		
		if(p >= 0)
		{
			flags[p].child_flags |= (flags[j].child_flags | tags[j]); 
		}
		
		// root to leaves
		j = dfs_ordering[i];
		p = in.parents[j];
		
		if(p >= 0)
		{
			flags[j].parent_flags |= (flags[p].parent_flags | tags[p]); 
		}
	}
	
	return (relative_clade_flags = flags);
}


immutable_array<uint16_t> TonTon::ArmatureMemo::GetDfsOrdering()
{
	std::lock_guard lock(_mutex);
	
	if(dfs_ordering.size())
		return dfs_ordering;

	auto _noJoints = in.parents.size();
	auto children = GetChildren();
	
	shared_array<uint16_t> joints(_noJoints, 0);
	std::vector<uint32_t> stack;
	stack.reserve(_noJoints);
	
	for(auto i = 0u, j = 0u; i < _noJoints; ++i)
	{
		if(in.parents[i] < 0)
		{
			stack.push_back(i);
			
			while(stack.size())
			{
				auto node = stack.back();
				stack.pop_back();
				
				joints[j++] = node;
				
				for(auto child : children[node])
				{
					stack.push_back(child);
				}
			}
		}
	}
	
	return (dfs_ordering = joints);
}

immutable_array<immutable_array<uint16_t>> TonTon::ArmatureMemo::GetDirectedGraph()
{
	std::lock_guard lock(_mutex);
	
	if(children.size()) return directed_graph;
	
	auto _noJoints = in.parents.size();
	std::vector<std::vector<int>> children(_noJoints);
		
	for(auto i = 0u; i < _noJoints; ++i)
	{
		if(in.parents[i] >= 0)
			children[in.parents[i]].push_back(i);
	}
	
	for(auto i = 0u; i < _noJoints; ++i)
	{
		if(in.parents[i] >= 0)
			children[i].push_back(in.parents[i]);
	}	
	
	return (this->directed_graph = shared_array<immutable_array<uint16_t>>::Build(_noJoints, [&](int i) 
	{ 
		return children[i].size()? 
			shared_array<uint16_t>::FromArray(children[i].data(), children[i].size())
			: shared_array<uint16_t>{}; 
	}));
}

immutable_array<std::span<const uint16_t> > TonTon::ArmatureMemo::GetChildren()
{
	std::lock_guard lock(_mutex);
	
	if(children.size()) return children;
	
	auto _noJoints = in.parents.size();
	auto directed_graph = GetDirectedGraph();
		
	return (this->children = shared_array<std::span<const uint16_t>>::Build(_noJoints, 
	[&](int i) -> std::span<const uint16_t> 
	{ 
		return in.parents[i] >= 0? 
			std::span(directed_graph[i].data(), directed_graph[i].size()-1) 
		:   std::span(directed_graph[i].data(), directed_graph[i].size());
	}));
}

immutable_array<TonTon::SemanticFlags> TonTon::ArmatureMemo::GetSemanticFlags()
{
	std::lock_guard lock(_mutex);
	
	if(semantic_flags.size()) 
		return semantic_flags;
	
	auto children = GetChildren();
		
	shared_array<TonTon::SemanticFlags> flags(in.parents.size(), TonTon::SemanticFlags::NONE);

	auto HasWord = [&](int i, std::span<Word> tokens) -> bool
	{
		for(auto & word : in.tags[i])
		{
			if(std::find(tokens.begin(), tokens.end(), word) != tokens.end())
				return true;
		}
		
		return false;
	};

	for(auto i = 0u; i < in.parents.size(); ++i)
	{
		for(auto & word : in.tags[i])
		{
			flags[i] |= ::TonTon::GetSemanticFlags(word);
		}
	}
	
	auto leg_words = std::array<Word, 1>{Word::leg};
	
	for(auto i = 0u; i < in.parents.size(); ++i)
	{
		if(children[i].empty())
		{
			if(HasWord(i, leg_words))
			{
				flags[i] |= SemanticFlags::CONTACT;		
			}
		}
	}
	
	return (semantic_flags = flags);
}

immutable_array<TonTon::CladeFlags> TonTon::ArmatureMemo::GetCladeFlags()
{
	std::lock_guard lock(_mutex);
	
	if(semantic_flags.size()) 
		return clade_flags;
	
	auto children = GetChildren();
		
	shared_array<TonTon::CladeFlags> flags(in.parents.size(), TonTon::CladeFlags::NONE);

	auto HasWord = [&](int i, std::span<Word> tokens) -> bool
	{
		for(auto & word : in.tags[i])
		{
			if(std::find(tokens.begin(), tokens.end(), word) != tokens.end())
				return true;
		}
		
		return false;
	};

	for(auto i = 0u; i < in.parents.size(); ++i)
	{
		for(auto & word : in.tags[i])
		{
			flags[i] |= ::TonTon::GetCladeFlags(word);
		}
	}
		
	return (clade_flags = flags);
}

immutable_array<uint16_t> TonTon::ArmatureMemo::GetLeaves()
{
	std::lock_guard lock(_mutex);
	
	if(leaves.size()) return leaves;
	
	auto children = GetChildren();
	auto _noJoints = children.size();
	
	std::vector<uint16_t> retn;

	for(auto i = 0u; i < _noJoints; ++i)
	{
		if(children[i].empty())
			retn.push_back(i);
	}

	return leaves = shared_array<uint16_t>::FromArray(retn.data(), retn.size());
}

immutable_array<uint16_t> TonTon::ArmatureMemo::GetGcrTable()
{
	std::lock_guard lock(_mutex);
	
	if(gcr_table.size()) return gcr_table;
	auto _noJoints = in.parents.size();
	
	shared_array<uint16_t> table(_noJoints*_noJoints, 0);

// parents are in strict DFS order.
	auto get_gcr = [p=in.parents](int i, int j) -> int
	{
		while(i != j)
		{
			if(i > j)
				i = p[i];
			else
				j = p[j];
		}
		
		return i;
	};

	for(uint32_t i = 0; i < _noJoints; ++i)
	{
		table[i*_noJoints+i] = i;
		for(uint32_t j = i+1; j < _noJoints; ++j)
		{
			auto gcr = get_gcr(i, j);
			table[i*_noJoints+j] = gcr;
			table[j*_noJoints+i] = gcr;
		}
	}

	return (gcr_table = table);
}

std::vector<uint16_t> TonTon::ArmatureMemo::GetAllChildren(int32_t of, SemanticFlags including, SemanticFlags excluding, std::span<int32_t> not_in)
{


	std::vector<uint16_t> r;
	auto common_ancestor = GetGcrTable();
	auto flags = GetSemanticFlags();
	auto dfs_ordering = GetDfsOrdering();
	auto noJoints = in.parents.size();
	
	std::vector<uint16_t> also_exclude(not_in.begin(), not_in.end());
	
	auto should_exclude_subtree = [&](int i)
	{
		for(auto exclude_subtree : also_exclude)
		{
			if(common_ancestor[i*noJoints+exclude_subtree] == (uint32_t)exclude_subtree)
				return true;
		}
		
		return false;
	};
	
	for(auto idx = 0u; idx < noJoints; ++idx)
	{
		auto i = dfs_ordering[idx];
		
		if(common_ancestor[i*noJoints+of] != (uint32_t)of)
			continue;
			
		if(should_exclude_subtree(i))
			continue;
			
		bool has_included_flags = HasFlag(flags[i], including);
		bool has_excluded_flags = HasFlag(flags[i], excluding);
		if(!has_included_flags
		||  has_excluded_flags)
		{
			also_exclude.push_back(i);
			continue;
		}
		
		r.push_back(i);
	}	
	
	return r;
}


std::vector<uint16_t>  TonTon::ArmatureMemo::GetAllChildrenOfRoot(uint32_t begin)
{
	std::lock_guard lock(_mutex);
	
	std::vector<uint16_t> items;
	items.reserve(8);
	
	auto common_ancestor = GetGcrTable();
	uint32_t noJoints = in.parents.size();
	
	for(auto idx = 0u; idx < noJoints; ++idx)
	{
		if(idx != begin && common_ancestor[begin*noJoints + idx] == (uint32_t)begin) 
			items.push_back(idx);
	}

	items.push_back(begin); // make sure root is always last so we can find it. 
	return  items;
}


/*
immutable_array<TonTon::Tensor> TonTon::GetTensors(TonTon::Armature const& in, int32_t from, int32_t to, SemanticFlags include, SemanticFlags exclude)
{
	auto table = in.Get(GetGcrTable);
	auto p =  in.skinnedMesh.parents;
	auto _noJoints = p.size();
	
	if(to >= 0 && table[from*_noJoints+to] != (uint32_t)to)
		throw std::runtime_error("from not a child of to.");

	std::vector<int32_t> relevant_bones;
	relevant_bones.reserve(8);
	
	for(int32_t i = from; i >= to; i = p[from])
	{
		relevant_bones.push_back(i);
	}
	
	return GetTensors(in, std::span(relevant_bones.data(), relevant_bones.data()+relevant_bones.size()), include, exclude);
}*/
