#include "tonton_meshmemo.h"
#include "../../include/tonton_input.h"
#include "dodeedum.h"
#include "dodeedum_mesh.h"

#ifndef __unused__
#if defined(__GNUC__) || defined(__clang__)
#define __unused __attribute__((unused))
#else
#define __unused
#endif
#endif

TonTon::MeshMemo::MeshMemo(Mesh const* ptr) : in(*ptr)
{
}

TonTon::MeshMemo::~MeshMemo() = default;

static inline bool __unused operator<(const immutable_array<uint16_t>& fk, const std::span<uint16_t>& lk) 
{ 
	if(fk.size() != lk.size())
		return fk.size() < lk.size();
	
	return memcmp(fk.data(), lk.data(), fk.byteLength()) < 0;
}

// YOU NEED THIS ONE TOO (the reverse direction)
static inline bool __unused operator<(const std::span<uint16_t>& lhs,
                             const immutable_array<uint16_t>& rhs) 
{ 
    if(lhs.size() != rhs.size())
        return lhs.size() < rhs.size();
    
    return memcmp(lhs.data(), rhs.data(), lhs.size() * sizeof(uint16_t)) < 0;
}

static inline bool __unused operator<(const immutable_array<uint16_t>& fk, const immutable_array<uint16_t>& lk) 
{ 
	if(fk.size() != lk.size())
		return fk.size() < lk.size();
	
	return memcmp(fk.data(), lk.data(), fk.byteLength()) < 0;
}


uint32_t TonTon::MeshMemo::get_index(std::span<uint16_t> list)
{
	std::lock_guard lock(_mutex);
	
	auto itr = _span_indices.find(list);

	if(itr != _span_indices.end())
		return itr->second;
		
	immutable_array<uint16_t> fk = shared_array<uint16_t>::FromArray(list);
	
	auto idx = _span_indices.size();
	_span_indices[fk] = idx;
	return idx;
}

TonTon::Silhouette & TonTon::MeshMemo::GetSilhouettes(glm::mat4 const& projection, glm::dvec3 const& scale, std::span<uint16_t> joints,  float cutoff, bool secondMoment)
{
	Key key =
	{
		.joints = get_index(joints),
		.cutoff = uint8_t(std::clamp<int>(cutoff * 255, 0, 255)),
		.secondMoment=secondMoment,
		.projection = projection * 
			glm::mat4(
				scale.x, 0, 0, 0, 
				0, scale.y, 0, 0, 
				0, 0, scale.z, 0,
				0, 0, 0, 1)
	};
	
	std::lock_guard lock(_mutex);	
	
	auto itr = _cache.find(key);
	
	if(itr != _cache.end())
		return itr->second;
	
	immutable_array<uint32_t> fk = shared_array<uint32_t>::FromArray(joints);
	
	DoDeeDum::Input input
	{
		.mesh=in.mesh,
		.projection=projection,
		.scale=scale,
		.cutoff=cutoff,
		.getSecondMoment=secondMoment,
		.joints=std::span(fk.data(), fk.size())
	};
	
	_cache[key] = DoDeeDum::GetSilhouette(input, "/home/anyuser/claude/", "dragonfly");
	
	return _cache[key];
}

bool TonTon::MeshMemo::Key::operator<(Key const& k) const
{
	if(joints != k.joints) return joints < k.joints;
	if(cutoff != k.cutoff) return cutoff < k.cutoff;
	if(secondMoment != k.secondMoment) return secondMoment < k.secondMoment;
	return memcmp(&projection, &k.projection, sizeof(projection)) < 0;
}

int TonTon::MeshMemo::GetVertexOverlap(int a, int b)
{
	BuildVertexOverlapMemos();
	return _vertexOverlapTable[a*_maxJoint+b];
}

float TonTon::MeshMemo::GetVertexOverlapPercent(int a, int b)
{
	BuildVertexOverlapMemos();
	
	int32_t numerator = _vertexOverlapTable[a*_maxJoint+b];
	int32_t denominator = std::max(1, _totalVertsAffected[a] + _totalVertsAffected[b]);
	
	return (numerator*2) / double(denominator);
}

bool TonTon::MeshMemo::isTube(int parent, int vertex, std::span<int16_t> children)
{
	BuildVertexOverlapMemos();
	
	int32_t count = _vertexOverlapTable[vertex*_maxJoint+vertex];
	if(parent >= 0)
		count += _vertexOverlapTable[vertex*_maxJoint+parent];
		
	for(auto child : children)
	{
		count += _vertexOverlapTable[vertex*_maxJoint+child];	
	}
	
	return (count == _totalVertsAffected[vertex]);
}
	
void TonTon::MeshMemo::BuildVertexOverlapMemos()
{
	if(_vertexOverlapTable != nullptr)
		return;
		
// the mesh doesn't have access to the number of joints in the armature
// so we can't just use an array for the hash.
	std::unordered_map<uint64_t, int> memo;
	memo.reserve(64);
	int32_t max_joint = 0;
	
	auto DocumentVertex = [&](int32_t a, int32_t b)
	{
		uint64_t index = (uint64_t(a) << 32) | b;
		memo[index] += 1;
		
		max_joint = std::max(max_joint, a);
		max_joint = std::max(max_joint, b);
	};

	for(auto & primitive : in.mesh)
	{
		primitive.for_each_vertex([&](auto const& vert) -> bool
		{
			int count1 = 0;
			int count2 = 0;
			
			for(int i = 0; i < 3; ++i)
			{
				if(vert.weights[i] != 0)
				{
					++count1;
					count2 = 0;
					DocumentVertex(vert.joints[i], vert.joints[i]);
					
					for(int j = i+1; j < 4; ++j)
					{
						if(vert.weights[j] != 0)
						{
							++count2;
							DocumentVertex(vert.joints[i], vert.joints[j]);
						}
					}
				}
				
				// this vertex is the only thing affecting this thing.
				if(count1 == 1 && count2 == 0)
					DocumentVertex(vert.joints[i], -1);
			}
		
			return false;
		});
	}
	
	shared_array<int> table((max_joint+1)*(max_joint+1), 0);
	shared_array<int> counts(max_joint+1, 0);
	_maxJoint = max_joint;
	
	for(auto item : memo)
	{
		int a = item.first & 0xFFFFFFFF;
		int b = (item.first >> 32);
		
		if(a < 0)
		{
			counts[b] = item.second;
		}
		else
		{
			table[a*_maxJoint+b] = item.second;
			table[b*_maxJoint+a] = item.second;
		}
	}
	
	_vertexOverlapTable = table;
	_totalVertsAffected = counts;
}
