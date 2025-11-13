/* This file contains the word list
 * 
 * Basically its how we map bone names in the armature to numeric IDs.
 * 
 * we need to analyze the skeleton 
 * 
 */ 

#ifndef TONTON_WORDLIST_H
#define TONTON_WORDLIST_H
#include <cstdint>
#include <vector>
#include <string_view>

namespace TonTon
{

enum class Word : uint16_t
{
//Directional/Positional:
	left, right, front, fore, anterior, hind, rear, back, posterior, upper, lower, mid, middle, inner, outer, medial, lateral, proximal, distal, tip, end, base, root, top, bottom,
	
//Body regions:
	flank, loin, rump, haunch, withers, croup, breast, brisket, dewlap, pouch, crop, gizzard,
	butt, 
	
//Head/Face details:
	fang, tooth, teeth, incisor, canine, molar, trunk, rostrum, barbel, snoot, jowl,
	head, skull, cranium, jaw, mandible, maxilla, chin, mouth, muzzle, snout, beak, bill, 
	proboscis, nose, nostril, nare, eye, orbit, ear, lobe, pinna, horn, 
	antler, tusk, crest, crown, comb, wattle, whisker, vibrissa, tongue, throat, gullet, cheek,
	maw, trap, face,
//Spine & Torso:
	vertebra, vertebrae, neck, cervical, chest,  sternum, rib, ribcage, belly, abdomen, ventral, stomach, pelvis, hip, sacrum, tail, coccyx,
//Tail types:
	dock, brush, bob, scut, fan, plume, tuft,
//Extremities:
	paw, dewclaw, fetlock, pastern, cannon, coronet, frog, stifle, gaskin, hock, digit, mitt,
//Arm/Forelimb:
	shoulder, scapula, clavicle, humerus, arm, elbow, radius, ulna, forearm, wrist, carpal, carpus, hand, grasper, palm, metacarpal, thumb, finger, knuckle, phalanx, phalange,  
//Leg/Hindlimb:
	thigh, femur, leg,  knee, patella, kneecap, shin, tibia, fibula, calf, ankle, tarsal, tarsus, heel, calcaneus, foot, sole, metatarsal, toe, claw, talon, nail, spur, pad, hoof,
//Scales/Armor:
	scale, plate, scute, shell, carapace, plastron, shield, spike, 
//Insects/Arthropods:
	antenna, segment, coxa, trochanter, cercus, ovipositor, setae,
//Aquatic specific:
	ray, operculum, siphon, mantle,
	fin, flipper, fluke, pectoral, dorsal, anal, caudal, tentacle, web, gill,
//Wings/Flight:
	covert, patagium, aileron, 
	wing, pinion, alula, 
//Other:
	gland, duct, sac, chamber, valve, muscle, tendon, ligament, cartilage, sensory, chemoreceptor,
	primary, secondary, compound,
	
//Technical/Rigging:
	joint, bone, twist, roll, IK, FK, control, ctrl, deform, def, target, pole, master, mstr, pivot,
	
// Multiple Meanings:
	membrane, socket, sucker, spine, thorax, adhesive,
	
// Jiggle bones:
	jiggle, phys, sym, hair, feather, mane, fur,
	
	TOTAL_WORDS
};

enum class SemanticFlags : uint64_t
{
	NONE = 0,
    // Laterality
    LEFT        = 1ULL << 0,
    RIGHT       = 1ULL << 1,
    
    // Anterior/Posterior
    ANTERIOR    = 1ULL << 2,  // front, fore
    POSTERIOR   = 1ULL << 3,  // hind, rear, back
    
    // Dorsal/Ventral
    DORSAL      = 1ULL << 4,  // top, back
    VENTRAL     = 1ULL << 5,  // bottom, belly
    
    // Medial/Lateral
    MEDIAL      = 1ULL << 6,  // inner, middle
    LATERAL     = 1ULL << 7,  // outer
    
    // Proximal/Distal
    PROXIMAL    = 1ULL << 8,  // upper, base, root
    DISTAL      = 1ULL << 9,  // lower, tip, end
    TERMINAL    = 1ULL << 10, // hand, foot, paw
    
    // Major body regions
    HEAD        = 1ULL << 11,
    NECK        = 1ULL << 12,
    SPINE       = 1ULL << 13,
    ABDOMEN     = 1ULL << 14, // belly
    TAIL        = 1ULL << 15,
    LIMB        = 1ULL << 16, // arm/wing structures
        
    // Specific structures
    DIGIT       = 1ULL << 17, // finger, toe, thumb
    FACIAL      = 1ULL << 18, // eyes, nose, mouth, ears
    TEETH       = 1ULL << 19,
    NAIL		= 1ULL << 20,
    GRASPER     = 1ULL << 21,
    HORN_ANTLER = 1ULL << 22,
    
    // Animal type indicators
    AVIAN       = 1ULL << 23, // bird-specific
    AQUATIC     = 1ULL << 24, // fish/marine
    ARTHROPOD   = 1ULL << 25, // insect/spider
    EQUINE      = 1ULL << 26,
    
    // Appendage types
    WING        = 1ULL << 27,
    FIN         = 1ULL << 28,
    TENTACLE    = 1ULL << 29,
    MEMBRANE    = 1ULL << 20,
    
    // External features
    ARMOR       = 1ULL << 31, // scales, plates, shell
    ALPHA_CARD  = 1ULL << 32,
    WEAPON      = 1ULL << 33,
    ASSMETRICAL = 1ULL << 34,
    
    // Rigging/Technical (not semantically important for animal type)
    RIGGING_CONTROL = 1ULL << 35, // IK, FK, control, target, pole
    DEFORMER        = 1ULL << 36, // deform bones
    TWIST_ROLL      = 1ULL << 37, // twist/roll helpers
    JIGGLE_BONE     = 1ULL << 38, // twist/roll helpers
    
    // Additional semantic categories
    INTERNAL    = 1ULL << 39, // glands, organs, internal structures
    SENSORY     = 1ULL << 40, // whiskers, vibrissae, barbels
    RESPIRATORY = 1ULL << 41, // gills, nostrils
    MOUTH_PARTS = 1ULL << 42, // mandibles, beaks, muzzles, snouts
    
    HEARING = 1ULL << 43,
    VISION = 1ULL << 44,
    CONTACT = 1ULL << 45,
    
    FORELIMB   = LIMB | ANTERIOR,
    HINDLIMB   = LIMB | POSTERIOR,
    UPPER_LIMB = LIMB | PROXIMAL,
    MID_LIMB   = LIMB| MEDIAL,
    LOWER_LIMB = LIMB | DISTAL,
    THORAX     = ABDOMEN | ANTERIOR,
    PELVIS     = ABDOMEN | POSTERIOR,
    
    POSITION_MASK = LEFT|RIGHT|ANTERIOR|POSTERIOR|DORSAL|VENTRAL,
};

enum class CladeFlags : uint32_t
{
	NONE = 0,

	// Vertebrates
	CHORDATA    = 1ULL << 0,  // Vertebrates (has spine/vertebrae)

	// Tetrapods
	AMPHIBIA    = 1ULL << 1,  // Amphibians

	// Amniotes
	REPTILIA    = 1ULL << 2,  // Reptiles
	CHELONIA    = 1ULL << 3,  // Turtles/tortoises
	AVES        = 1ULL << 4,  // Birds
	MAMMALIA    = 1ULL << 5,  // Mammals

	// Mammal subgroups
	UNGULATA    = 1ULL << 6,  // Hoofed mammals
	EQUIDAE     = 1ULL << 7,  // Horses, zebras, donkeys
	CETACEA     = 1ULL << 8,  // Whales, dolphins, porpoises

	// Fish
	PISCES      = 1ULL << 9,  // Fish (general)

	// Invertebrates
	ARTHROPODA  = 1ULL << 10, // Arthropods (general)
	INSECTA     = 1ULL << 11, // Insects
	ARACHNIDA   = 1ULL << 12, // Spiders, scorpions
	CRUSTACEA   = 1ULL << 13, // Crustaceans

	MOLLUSCA    = 1ULL << 14, // Mollusks (general)
	CEPHALOPODA = 1ULL << 15, // Octopuses, squids, cuttlefish
};

size_t StringToWords(std::vector<Word> & dst, std::string_view word);
SemanticFlags GetSemanticFlags(Word word);
CladeFlags GetCladeFlags(Word word);

inline SemanticFlags operator|(SemanticFlags a, SemanticFlags b) 
{
	return SemanticFlags((uint64_t)a | (uint64_t)b);
}

inline void operator|=(SemanticFlags & a, SemanticFlags b) 
{
	a = TonTon::SemanticFlags((uint64_t)a | (uint64_t)b);
}

inline SemanticFlags operator&(TonTon::SemanticFlags a, TonTon::SemanticFlags b) 
{
	return TonTon::SemanticFlags((uint64_t)a & (uint64_t)b);
}

inline void operator&=(TonTon::SemanticFlags & a, TonTon::SemanticFlags b) 
{
	a = TonTon::SemanticFlags((uint64_t)a & (uint64_t)b);
}

inline bool HasFlag(SemanticFlags a, SemanticFlags b)
{
	return ((uint64_t)a & (uint64_t)b) != 0;
}

inline bool ExactMatch(SemanticFlags a, SemanticFlags b)
{
	return ((uint64_t)a & (uint64_t)b) == (uint64_t)b;
}

inline CladeFlags operator|(CladeFlags a, CladeFlags b)
{
	return CladeFlags((uint64_t)a | (uint64_t)b);
}

inline void operator|=(CladeFlags & a, CladeFlags b)
{
	a = TonTon::CladeFlags((uint64_t)a | (uint64_t)b);
}

inline CladeFlags operator&(TonTon::CladeFlags a, TonTon::CladeFlags b)
{
	return TonTon::CladeFlags((uint64_t)a & (uint64_t)b);
}

inline void operator&=(TonTon::CladeFlags & a, TonTon::CladeFlags b)
{
	a = TonTon::CladeFlags((uint64_t)a & (uint64_t)b);
}

inline bool HasFlag(CladeFlags a, CladeFlags b)
{
	return ((uint64_t)a & (uint64_t)b) != 0;
}

inline bool ExactMatch(CladeFlags a, CladeFlags b)
{
	return ((uint64_t)a & (uint64_t)b) == (uint64_t)b;
}

}

#endif // TONTON_WORDLIST_H
