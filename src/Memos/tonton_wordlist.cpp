#include "../../include/tonton_wordlist.h"
#include <algorithm>
#include <cstring>
#include <string>
#include <cassert>
#include <array>

namespace TonTon
{
struct WordDef
{
	Word word;
	const char * string;
};

constexpr std::array<WordDef, (int)Word::TOTAL_WORDS+3> word_strings = [] {
    std::array<WordDef, (int)Word::TOTAL_WORDS+3> arr = {
		WordDef{ Word::left, "left"},
		{ Word::right, "right"},
		{ Word::left, "l"},
		{ Word::right, "r"},
		{ Word::front, "front"},
		{ Word::fore, "fore"},
		{ Word::anterior, "anterior"},
		{ Word::hind, "hind"},
		{ Word::rear, "rear"},
		{ Word::back, "back"},
		{ Word::posterior, "posterior"},
		{ Word::upper, "upper"},
		{ Word::lower, "lower"},
		{ Word::mid, "mid"},
		{ Word::middle, "middle"},
		{ Word::inner, "inner"},
		{ Word::outer, "outer"},
		{ Word::medial, "medial"},
		{ Word::lateral, "lateral"},
		{ Word::proximal, "proximal"},
		{ Word::distal, "distal"},
		{ Word::tip, "tip"},
		{ Word::end, "end"},
		{ Word::base, "base"},
		{ Word::root, "root"},
		{ Word::top, "top"},
		{ Word::bottom, "bottom"},
		{ Word::flank, "flank"},
		{ Word::loin, "loin"},
		{ Word::rump, "rump"},
		{ Word::haunch, "haunch"},
		{ Word::withers, "withers"},
		{ Word::croup, "croup"},
		{ Word::breast, "breast"},
		{ Word::brisket, "brisket"},
		{ Word::dewlap, "dewlap"},
		{ Word::pouch, "pouch"},
		{ Word::crop, "crop"},
		{ Word::gizzard, "gizzard"},
		{ Word::butt, "butt"},
		{ Word::fang, "fang"},
		{ Word::tooth, "tooth"},
		{ Word::teeth, "teeth"},
		{ Word::incisor, "incisor"},
		{ Word::canine, "canine"},
		{ Word::molar, "molar"},
		{ Word::trunk, "trunk"},
		{ Word::rostrum, "rostrum"},
		{ Word::barbel, "barbel"},
		{ Word::snoot, "snoot"},
		{ Word::jowl, "jowl"},
		{ Word::head, "head"},
		{ Word::skull, "skull"},
		{ Word::cranium, "cranium"},
		{ Word::jaw, "jaw"},
		{ Word::mandible, "mandible"},
		{ Word::maxilla, "maxilla"},
		{ Word::chin, "chin"},
		{ Word::mouth, "mouth"},
		{ Word::muzzle, "muzzle"},
		{ Word::snout, "snout"},
		{ Word::beak, "beak"},
		{ Word::bill, "bill"},
		{ Word::proboscis, "proboscis"},
		{ Word::nose, "nose"},
		{ Word::nostril, "nostril"},
		{ Word::nare, "nare"},
		{ Word::eye, "eye"},
		{ Word::orbit, "orbit"},
		{ Word::ear, "ear"},
		{ Word::lobe, "lobe"},
		{ Word::pinna, "pinna"},
		{ Word::horn, "horn"},
		{ Word::antler, "antler"},
		{ Word::tusk, "tusk"},
		{ Word::crest, "crest"},
		{ Word::crown, "crown"},
		{ Word::comb, "comb"},
		{ Word::wattle, "wattle"},
		{ Word::whisker, "whisker"},
		{ Word::vibrissa, "vibrissa"},
		{ Word::tongue, "tongue"},
		{ Word::throat, "throat"},
		{ Word::gullet, "gullet"},
		{ Word::cheek, "cheek"},
		{ Word::maw, "maw"},
		{ Word::trap, "trap"},
		{ Word::face, "face"},
		{ Word::vertebra, "vertebra"},
		{ Word::vertebrae, "vertebrae"},
		{ Word::neck, "neck"},
		{ Word::cervical, "cervical"},
		{ Word::chest, "chest"},
		{ Word::sternum, "sternum"},
		{ Word::rib, "rib"},
		{ Word::ribcage, "ribcage"},
		{ Word::belly, "belly"},
		{ Word::abdomen, "abdomen"},
		{ Word::ventral, "ventral"},
		{ Word::stomach, "stomach"},
		{ Word::pelvis, "pelvis"},
		{ Word::hip, "hip"},
		{ Word::sacrum, "sacrum"},
		{ Word::tail, "tail"},
		{ Word::coccyx, "coccyx"},
		{ Word::dock, "dock"},
		{ Word::brush, "brush"},
		{ Word::bob, "bob"},
		{ Word::scut, "scut"},
		{ Word::fan, "fan"},
		{ Word::plume, "plume"},
		{ Word::tuft, "tuft"},
		{ Word::paw, "paw"},
		{ Word::dewclaw, "dewclaw"},
		{ Word::fetlock, "fetlock"},
		{ Word::pastern, "pastern"},
		{ Word::cannon, "cannon"},
		{ Word::coronet, "coronet"},
		{ Word::frog, "frog"},
		{ Word::stifle, "stifle"},
		{ Word::gaskin, "gaskin"},
		{ Word::hock, "hock"},
		{ Word::digit, "digit"},
		{ Word::mitt, "mitt"},
		{ Word::shoulder, "shoulder"},
		{ Word::scapula, "scapula"},
		{ Word::clavicle, "clavicle"},
		{ Word::humerus, "humerus"},
		{ Word::arm, "arm"},
		{ Word::elbow, "elbow"},
		{ Word::radius, "radius"},
		{ Word::ulna, "ulna"},
		{ Word::forearm, "forearm"},
		{ Word::wrist, "wrist"},
		{ Word::carpal, "carpal"},
		{ Word::carpus, "carpus"},
		{ Word::hand, "hand"},
		{ Word::grasper, "grasper"},
		{ Word::palm, "palm"},
		{ Word::metacarpal, "metacarpal"},
		{ Word::thumb, "thumb"},
		{ Word::finger, "finger"},
		{ Word::knuckle, "knuckle"},
		{ Word::phalanx, "phalanx"},
		{ Word::phalange, "phalange"},
		{ Word::thigh, "thigh"},
		{ Word::femur, "femur"},
		{ Word::leg, "leg"},
		{ Word::knee, "knee"},
		{ Word::patella, "patella"},
		{ Word::kneecap, "kneecap"},
		{ Word::shin, "shin"},
		{ Word::tibia, "tibia"},
		{ Word::fibula, "fibula"},
		{ Word::calf, "calf"},
		{ Word::ankle, "ankle"},
		{ Word::tarsal, "tarsal"},
		{ Word::tarsus, "tarsus"},
		{ Word::heel, "heel"},
		{ Word::calcaneus, "calcaneus"},
		{ Word::foot, "foot"},
		{ Word::sole, "sole"},
		{ Word::metatarsal, "metatarsal"},
		{ Word::toe, "toe"},
		{ Word::claw, "claw"},
		{ Word::talon, "talon"},
		{ Word::setae, "setae"},
		{ Word::nail, "nail"},
		{ Word::spur, "spur"},
		{ Word::pad, "pad"},
		{ Word::hoof, "hoof"},
		{ Word::scale, "scale"},
		{ Word::plate, "plate"},
		{ Word::scute, "scute"},
		{ Word::shell, "shell"},
		{ Word::carapace, "carapace"},
		{ Word::plastron, "plastron"},
		{ Word::shield, "shield"},
		{ Word::spike, "spike"},
		{ Word::antenna, "antenna"},
		{ Word::antenna, "antennae"},
		{ Word::segment, "segment"},
		{ Word::coxa, "coxa"},
		{ Word::trochanter, "trochanter"},
		{ Word::cercus, "cercus"},
		{ Word::ovipositor, "ovipositor"},
		{ Word::ray, "ray"},
		{ Word::operculum, "operculum"},
		{ Word::siphon, "siphon"},
		{ Word::mantle, "mantle"},
		{ Word::fin, "fin"},
		{ Word::flipper, "flipper"},
		{ Word::fluke, "fluke"},
		{ Word::pectoral, "pectoral"},
		{ Word::dorsal, "dorsal"},
		{ Word::anal, "anal"},
		{ Word::caudal, "caudal"},
		{ Word::tentacle, "tentacle"},
		{ Word::web, "web"},
		{ Word::gill, "gill"},
		{ Word::covert, "covert"},
		{ Word::patagium, "patagium"},
		{ Word::aileron, "aileron"},
		{ Word::wing, "wing"},
		{ Word::pinion, "pinion"},
		{ Word::alula, "alula"},
		{ Word::primary, "primary"},
		{ Word::secondary, "secondary"},
		{ Word::compound, "compound"},
		{ Word::gland, "gland"},
		{ Word::duct, "duct"},
		{ Word::sac, "sac"},
		{ Word::chamber, "chamber"},
		{ Word::valve, "valve"},
		{ Word::muscle, "muscle"},
		{ Word::tendon, "tendon"},
		{ Word::ligament, "ligament"},
		{ Word::cartilage, "cartilage"},
		{ Word::joint, "joint"},
		{ Word::bone, "bone"},
		{ Word::twist, "twist"},
		{ Word::roll, "roll"},
		{ Word::IK, "ik"},
		{ Word::FK, "fk"},
		{ Word::control, "control"},
		{ Word::ctrl, "ctrl"},
		{ Word::deform, "deform"},
		{ Word::def, "def"},
		{ Word::target, "target"},
		{ Word::pole, "pole"},
		{ Word::master, "master"},
		{ Word::mstr, "mstr"},
		{ Word::pivot, "pivot"},
		{ Word::membrane, "membrane"},
		{ Word::socket, "socket"},
		{ Word::sucker, "sucker"},
		{ Word::spine, "spine"},
		{ Word::thorax, "thorax"},
		{ Word::jiggle, "jiggle"},
		{ Word::phys, "phys"},
		{ Word::sym, "sym"},
		{ Word::hair, "hair"},
		{ Word::mane, "mane"},
		{ Word::fur, "fur"},
		{ Word::feather, "feather"},  
		{ Word::adhesive, "adhesive"},  
		{ Word::sensory, "sensory"},  
		{ Word::predator, "predator"},  
		{ Word::herbivore, "herbivore"},  
		{ Word::chemoreceptor, "chemoreceptor"},  
		{ Word::carnivore, "carnivore"},  
		{ Word::stinger, "stinger"},  
		{ Word::venom, "venom"},  
    };
    
    std::sort(arr.begin(), arr.end(), [](const WordDef& a, const WordDef& b) {
				return std::string_view(a.string) < std::string_view(b.string);
			});
			
    return arr;
}();

}


static std::string to_lower(std::string_view s)
{
	std::string r = std::string(s);
	
	for(auto & c : r)
	{
		c = tolower(c);
	}
	
	return r;
}


std::string_view TonTon::WordToString(Word w)
{
	for(auto i = 0u; i < word_strings.size(); ++i)
	{
		if(word_strings[i].word == w)
			return word_strings[i].string;
	}
	
	return "<UNKNOWN>";
}

size_t TonTon::StringToWords(std::vector<Word> & dst, std::string_view word)
{
	size_t old_size = dst.size();
	auto lowa = to_lower(word);
	
	for(auto begin = 0u, end = 0u; begin < word.size(); begin = end)
	{
		for(;begin < word.size(); ++begin)
		{
			if(('A' <= word[begin] && word[begin] <= 'Z')
			|| ('a' <= word[begin] && word[begin] <= 'z'))
				break;
		}
	
		bool is_lowa = false;
		for(end = begin; end < word.size(); ++end)
		{
			if('a' <= word[end] && word[end] <= 'z')
			{
				if(!is_lowa && end > begin+1)
					break;
				else 
					is_lowa = true;
			}
			else if('A' <= word[end] && word[end] <= 'Z')
			{
				if(is_lowa)
					break;
			}
			else
				break;
		}
		
		if(begin == end)
			continue;
		
		auto token = std::string_view(lowa).substr(begin, end-begin);
		
		auto itr = std::lower_bound(std::begin(word_strings), std::end(word_strings),
				token, [](WordDef const& it, std::string_view const& value) -> bool
				{
					return std::string_view(it.string) < value;
				});
	
		if(itr != std::end(word_strings) 
		&& std::string_view(itr->string) == token)
			dst.push_back(itr->word);
	}

	return dst.size() - old_size;
}


TonTon::SemanticFlags TonTon::GetSemanticFlags(Word word)
{
    using SF = SemanticFlags;
    
    switch(word)
    {
    case Word::left: return SF::LEFT;
    case Word::right: return SF::RIGHT;
    
    case Word::front:
    case Word::fore:
    case Word::anterior: return SF::ANTERIOR;
    
    case Word::hind:
    case Word::rear:
    case Word::back:
    case Word::posterior: return SF::POSTERIOR;
    
    case Word::upper:
    case Word::proximal:
    case Word::base:
    case Word::root: return SF::PROXIMAL;
    
    case Word::lower:
    case Word::distal:
    case Word::tip:
    case Word::end: return SF::DISTAL;
    
    case Word::mid:
    case Word::middle:
    case Word::inner:
    case Word::medial: return SF::MEDIAL;
    
    case Word::outer:
    case Word::lateral: return SF::LATERAL;
    
    case Word::top: return SF::DORSAL;
    case Word::bottom: return SF::VENTRAL;
    
    case Word::dorsal: return SF::DORSAL;
    case Word::ventral:
    case Word::belly:
    case Word::abdomen:
    case Word::stomach: return SF::VENTRAL | SF::ABDOMEN;
    
    // Body regions
    case Word::flank:
    case Word::loin:
    case Word::rump:
    case Word::haunch:
    case Word::butt: return SF::POSTERIOR | SF::ABDOMEN;
    
    case Word::withers:
    case Word::croup: return SF::POSTERIOR | SF::ABDOMEN | SF::EQUINE;
    
    case Word::breast:
    case Word::brisket:
    case Word::chest:
    case Word::sternum:
    case Word::ribcage:
    case Word::rib:
    case Word::thorax: return SF::ABDOMEN|SF::POSTERIOR;
    
    case Word::dewlap:
    case Word::pouch:
    case Word::crop:
    case Word::gizzard: return SF::VENTRAL | SF::INTERNAL | SF::AVIAN;
    
    // Teeth
    case Word::fang:
    case Word::tooth:
    case Word::teeth:
    case Word::incisor:
    case Word::canine:
    case Word::molar: return SF::HEAD | SF::FACIAL | SF::TEETH;
    
    // Mouth/nose structures
    case Word::trunk: return SF::HEAD | SF::FACIAL | SF::RESPIRATORY;
    case Word::rostrum: return SF::HEAD | SF::FACIAL | SF::MOUTH_PARTS | SF::AVIAN;
    case Word::barbel: return SF::HEAD | SF::SENSORY | SF::AQUATIC;
    case Word::snoot:
    case Word::muzzle:
    case Word::snout: return SF::HEAD | SF::FACIAL | SF::MOUTH_PARTS;
    case Word::beak:
    case Word::bill: return SF::HEAD | SF::FACIAL | SF::MOUTH_PARTS | SF::AVIAN;
    case Word::proboscis: return SF::HEAD | SF::MOUTH_PARTS | SF::ARTHROPOD;
    
    case Word::nose:
    case Word::nostril:
    case Word::nare: return SF::HEAD | SF::FACIAL | SF::RESPIRATORY;
    
    case Word::jowl:
    case Word::chin:
    case Word::cheek: return SF::HEAD | SF::FACIAL;
    
    case Word::mouth:
    case Word::maw:
    case Word::jaw:
    case Word::trap: return SF::HEAD | SF::FACIAL | SF::MOUTH_PARTS;
    
    case Word::mandible:
    case Word::maxilla: return SF::HEAD | SF::MOUTH_PARTS;
    
    case Word::head:
    case Word::skull:
    case Word::cranium:
    case Word::face: return SF::HEAD;
    
    case Word::eye:
    case Word::orbit: return SF::HEAD | SF::FACIAL | SF::VISION;
    
    case Word::ear:
    case Word::lobe:
    case Word::pinna: return SF::HEAD | SF::FACIAL | SF::HEARING;
    
    case Word::horn:
    case Word::antler:
    case Word::tusk:
    case Word::crest:
    case Word::crown: return SF::HEAD | SF::HORN_ANTLER;
    
    case Word::comb:
    case Word::wattle: return SF::HEAD | SF::AVIAN;
    
    case Word::whisker:
    case Word::vibrissa: return SF::HEAD | SF::FACIAL | SF::SENSORY;
    
    case Word::tongue:
    case Word::throat:
    case Word::gullet: return SF::HEAD | SF::INTERNAL;
    
    // Spine/Neck/Tail
    case Word::vertebra:
    case Word::vertebrae:
    case Word::spine: return SF::SPINE;
    
    case Word::neck:
    case Word::cervical: return SF::NECK;
    
    case Word::pelvis:
    case Word::hip:
    case Word::sacrum: return SF::ABDOMEN|SF::POSTERIOR;
    
    case Word::tail:
    case Word::coccyx:
    case Word::caudal:
    case Word::dock:
    case Word::brush:
    case Word::bob:
    case Word::scut:
    case Word::fan:
    case Word::plume:
    case Word::tuft: return SF::TAIL;
    
    // Forelimb - upper
    case Word::shoulder:
    case Word::scapula:
    case Word::clavicle:
    case Word::humerus: return SF::FORELIMB | SF::UPPER_LIMB;
    
    case Word::arm: return SF::FORELIMB;
    
    // Forelimb - mid
    case Word::elbow: return SF::FORELIMB | SF::MID_LIMB;
    case Word::radius:
    case Word::ulna:
    case Word::forearm: return SF::FORELIMB | SF::MID_LIMB;
    
    // Forelimb - lower
    case Word::wrist: return SF::FORELIMB | SF::LOWER_LIMB;
    case Word::carpal:
    case Word::carpus: return SF::FORELIMB | SF::LOWER_LIMB;
    
    // Forelimb - terminal
    case Word::hand:
    case Word::palm:		return SF::FORELIMB | SF::TERMINAL | SF::GRASPER;
    case Word::metacarpal: return SF::FORELIMB | SF::TERMINAL;
    
    case Word::grasper: return SF::GRASPER;
    
    case Word::thumb:
    case Word::finger: return SF::FORELIMB | SF::DIGIT;
    
    case Word::knuckle: return SF::FORELIMB | SF::DIGIT;
    
    case Word::phalanx:
    case Word::phalange: return SF::DIGIT;
    
    // Hindlimb - upper
    case Word::thigh:
    case Word::femur: return SF::HINDLIMB | SF::UPPER_LIMB;
    
    case Word::leg: return SF::HINDLIMB;
    
    // Hindlimb - mid
    case Word::knee:
    case Word::patella:
    case Word::kneecap: return SF::HINDLIMB | SF::MID_LIMB;
    
    case Word::shin:
    case Word::tibia:
    case Word::fibula:
    case Word::calf:
    case Word::gaskin: return SF::HINDLIMB | SF::MID_LIMB;
    
    // Hindlimb - lower
    case Word::ankle: return SF::HINDLIMB | SF::LOWER_LIMB;
    case Word::tarsal:
    case Word::tarsus:
    case Word::hock: return SF::HINDLIMB | SF::LOWER_LIMB;
    
    case Word::heel:
    case Word::calcaneus: return SF::HINDLIMB | SF::LOWER_LIMB;
    
    // Hindlimb - terminal
    case Word::foot:
    case Word::sole:
    case Word::metatarsal:
    case Word::paw:
    case Word::mitt: return SF::HINDLIMB | SF::TERMINAL | SF::CONTACT;
    
    case Word::toe: return SF::HINDLIMB | SF::DIGIT;
    
    // Terminal structures (could be fore or hind)
    case Word::digit: return SF::DIGIT;
    
    case Word::claw:
    case Word::talon: return SF::WEAPON | SF::AVIAN | SF::NAIL;
    case Word::nail: return SF::WEAPON | SF::NAIL;
    case Word::spur: return SF::WEAPON | SF::AVIAN;
    case Word::hoof: return SF::NAIL | SF::EQUINE;
    
    case Word::pad: return SF::TERMINAL;
    
    // Equine-specific lower leg
    case Word::dewclaw: return SF::LOWER_LIMB |  SF::NAIL;
    case Word::fetlock:
    case Word::pastern:
    case Word::cannon:
    case Word::coronet:
    case Word::frog: return SF::LOWER_LIMB | SF::EQUINE;
    
    case Word::stifle: return SF::HINDLIMB | SF::EQUINE;
    
    // Armor/Scales
    case Word::scale:
    case Word::plate:
    case Word::scute:
    case Word::spike: return SF::ARMOR | SF::AQUATIC; // Could be reptile too
    case Word::shell:
    case Word::carapace:
    case Word::plastron:
    case Word::shield: return SF::ARMOR;
    
    // Arthropod
    case Word::antenna: return SF::HEAD | SF::SENSORY | SF::ARTHROPOD;
    case Word::segment: return SF::ARTHROPOD;
    case Word::coxa:
    case Word::trochanter: return SF::UPPER_LIMB | SF::ARTHROPOD;
    case Word::cercus:
    case Word::ovipositor: return SF::ABDOMEN | SF::ARTHROPOD;
    
    // Aquatic
    case Word::ray: return SF::FIN | SF::AQUATIC;
    case Word::operculum: return SF::HEAD | SF::RESPIRATORY | SF::AQUATIC;
    case Word::siphon:
    case Word::mantle: return SF::INTERNAL | SF::AQUATIC;
    case Word::gill: return SF::RESPIRATORY | SF::AQUATIC;
    
    case Word::fin:
    case Word::flipper:
    case Word::fluke: return SF::FIN | SF::AQUATIC;
    
    case Word::pectoral: return SF::FIN | SF::AQUATIC | SF::FORELIMB;
    case Word::anal: return SF::FIN | SF::AQUATIC | SF::VENTRAL;
    
    case Word::tentacle: return SF::TENTACLE | SF::AQUATIC;
    case Word::web: return SF::TERMINAL | SF::AQUATIC; // webbed feet
    case Word::sucker: return SF::TENTACLE | SF::AQUATIC;
    
    // Wings & feathers
    case Word::covert:
    case Word::primary:
    case Word::secondary: return SF::WING | SF::ALPHA_CARD | SF::AVIAN;
    
    case Word::patagium:
    case Word::aileron: return SF::WING | SF::FORELIMB;
    
    case Word::wing:
    case Word::pinion:
    case Word::alula: return SF::WING | SF::FORELIMB | SF::AVIAN;
    
    // Soft tissue / Internal
    case Word::gland:
    case Word::duct:
    case Word::sac:
    case Word::chamber:
    case Word::valve: return SF::INTERNAL;
    
    case Word::muscle:
    case Word::tendon:
    case Word::ligament:
    case Word::cartilage: return SF::INTERNAL;
    
    case Word::membrane: return SF::AQUATIC | SF::AVIAN | SF::MEMBRANE; // Could be either (webbed feet or wing membranes)
    
    // Generic - don't flag
    case Word::joint:
    case Word::socket:
    case Word::pivot:
    case Word::bone: return (SF)0;
    
    // Rigging controls
    case Word::IK:
    case Word::FK:
    case Word::control:
    case Word::ctrl:
    case Word::target:
    case Word::pole:
    case Word::master:
    case Word::mstr: return SF::RIGGING_CONTROL;
    
    case Word::deform:
    case Word::def: return SF::DEFORMER;
    
    case Word::twist:
    case Word::roll: return SF::TWIST_ROLL;
    
    case Word::jiggle:
    case Word::phys:
    case Word::sym:  return SF::ALPHA_CARD;	 
    case Word::hair:
    case Word::mane:
    case Word::fur:
    case Word::feather: return SF::ALPHA_CARD | SF::JIGGLE_BONE;	 
    case Word::TOTAL_WORDS: return SF::NONE;
	case Word::setae:
	case Word::adhesive:
		break;
	case Word::sensory:
	case Word::chemoreceptor:
	case Word::compound:
    case Word::predator:
    case Word::carnivore:
    case Word::herbivore:
		break;
    case Word::stinger:
    case Word::venom: return SF::WEAPON;
	}
	
	return (SF)0;
}


TonTon::CladeFlags TonTon::GetCladeFlags(Word word)
{
    using CF = CladeFlags;

    switch(word)
    {
    // Generic directional/positional terms - no specific clade
    case Word::left:
    case Word::right:
    case Word::front:
    case Word::fore:
    case Word::anterior:
    case Word::hind:
    case Word::rear:
    case Word::back:
    case Word::posterior:
    case Word::upper:
    case Word::lower:
    case Word::mid:
    case Word::middle:
    case Word::inner:
    case Word::outer:
    case Word::medial:
    case Word::lateral:
    case Word::proximal:
    case Word::distal:
    case Word::tip:
    case Word::end:
    case Word::base:
    case Word::root:
    case Word::top:
    case Word::primary:
    case Word::secondary:
    case Word::predator:
    case Word::herbivore:
    case Word::leg:
    case Word::foot:
    case Word::venom:
    case Word::bottom: return CF::NONE;

    // Body regions - general vertebrate
    case Word::flank:
    case Word::loin:
    case Word::rump:
    case Word::haunch:
    case Word::butt: return CF::CHORDATA;

    // Equine-specific body regions
    case Word::withers:
    case Word::croup: return CF::EQUIDAE;

    case Word::breast:
    case Word::brisket:
    case Word::chest:
    case Word::sternum:
    case Word::ribcage:
    case Word::rib: return CF::CHORDATA;

    // Avian-specific
    case Word::crop:
    case Word::gizzard: return CF::AVES;

    // Could be mammal (marsupials) or bird (pelicans), or reptile (lizards)
    case Word::dewlap:
    case Word::pouch: return CF::MAMMALIA | CF::AVES | CF::REPTILIA;

    // Teeth - primarily mammals, some reptiles
    case Word::tooth:
    case Word::teeth: return CF::CHORDATA;
    
    case Word::fang:
    case Word::incisor:
    case Word::canine:
    case Word::molar: return CF::MAMMALIA;

    // Trunk is elephant-specific (mammal)
    case Word::trunk: return CF::MAMMALIA;

    // Rostrum - primarily birds, also some marine mammals
    case Word::rostrum: return CF::NONE; // too general

    // Barbel - fish
    case Word::barbel: return CF::PISCES;

    case Word::snoot:
    case Word::jowl: return CF::MAMMALIA;

    case Word::skull:
    case Word::cranium: return CF::CHORDATA;
    
    case Word::jaw:    
    case Word::head:
    case Word::mouth:
    case Word::face: return CF::NONE; 

    // Mandible/maxilla - vertebrates, but also arthropods
    case Word::mandible:
    case Word::maxilla: return CF::ARTHROPODA; // to general


    case Word::chin:
    case Word::muzzle:
    case Word::snout: return CF::CHORDATA;

    case Word::beak:
    case Word::bill: return CF::AVES;

    // Proboscis - insects or elephants
    case Word::proboscis: return CF::ARTHROPODA;

    case Word::eye:
    case Word::ear: return CF::NONE; // to general
    
    case Word::nose:
    case Word::nostril:
    case Word::nare:
    case Word::orbit:
    case Word::lobe:
    case Word::pinna: return CF::CHORDATA;

    case Word::horn:
    case Word::antler:
    case Word::tusk: return CF::MAMMALIA;

    case Word::crest:
    case Word::crown: return CF::NONE; // too general

    case Word::comb:
    case Word::wattle: return CF::AVES;

    case Word::whisker:
    case Word::vibrissa: return CF::MAMMALIA;

    case Word::tongue:
    case Word::throat:
    case Word::gullet:
    case Word::cheek:
    case Word::maw:
    case Word::trap: return CF::CHORDATA;

    // Spine - defining feature of chordates
    case Word::vertebra:
    case Word::vertebrae:
    case Word::spine:
    case Word::neck:
    case Word::cervical: return CF::CHORDATA;

    case Word::belly:
    case Word::abdomen:
    case Word::ventral:
    case Word::stomach:
    case Word::dorsal: return CF::NONE; // directional or too general

    case Word::pelvis:
    case Word::hip:
    case Word::sacrum:
    case Word::coccyx: return CF::CHORDATA;
    
    case Word::tail:
    case Word::caudal: return CF::NONE; // directional term

    // Mammalian tail types
    case Word::dock:
    case Word::brush:
    case Word::bob:
    case Word::scut: return CF::MAMMALIA;

    case Word::fan:
    case Word::plume:
    case Word::tuft: return CF::NONE; // could be bird or mammal

    // Mammal-specific limb terminals
    case Word::paw:
    case Word::mitt:
    case Word::pad: return CF::MAMMALIA;

    case Word::dewclaw: return CF::MAMMALIA;

    // Equine-specific lower leg
    case Word::fetlock:
    case Word::pastern:
    case Word::cannon:
    case Word::coronet:
    case Word::frog:
    case Word::stifle:
    case Word::gaskin:
    case Word::hock: return CF::EQUIDAE;

    case Word::digit: return CF::CHORDATA;

    // General limb anatomy - vertebrate
    case Word::shoulder:
    case Word::scapula:
    case Word::clavicle:
    case Word::humerus:
    case Word::arm:
    case Word::elbow:
    case Word::radius:
    case Word::ulna:
    case Word::forearm:
    case Word::wrist:
    case Word::carpal:
    case Word::carpus:
    case Word::hand:
    case Word::palm:
    case Word::metacarpal:
    case Word::thumb:
    case Word::finger:
    case Word::knuckle:
    case Word::phalanx:
    case Word::phalange:
    case Word::thigh:
    case Word::femur:
    case Word::knee:
    case Word::patella:
    case Word::kneecap:
    case Word::shin:
    case Word::tibia:
    case Word::fibula:
    case Word::calf:
    case Word::ankle:
    case Word::tarsal:
    case Word::tarsus:
    case Word::heel:
    case Word::calcaneus:
    case Word::sole:
    case Word::metatarsal:
    case Word::toe: return CF::CHORDATA;

    case Word::grasper: return CF::NONE; // could be various clades

    // Terminal structures
    case Word::claw: return CF::CHORDATA;
    case Word::talon: return CF::AVES;
    case Word::nail: return CF::MAMMALIA;
    case Word::spur: return CF::AVES;
    case Word::hoof: return CF::UNGULATA;

    // Armor/Scales
    case Word::scale:
    case Word::scute:
    case Word::plate:
    case Word::spike: return CF::REPTILIA;
    case Word::shield: return CF::NONE;
    case Word::shell:
    case Word::carapace:
    case Word::plastron: return CF::CHELONIA;

    // Arthropod structures
    case Word::stinger:
    case Word::antenna: return CF::ARTHROPODA;
    case Word::segment: return CF::ARTHROPODA;
    case Word::coxa:
    case Word::trochanter: return CF::ARTHROPODA;
    case Word::cercus:
    case Word::ovipositor: return CF::INSECTA;
    case Word::setae: return CF::ARTHROPODA;

    // Aquatic - fish
    case Word::ray: return CF::PISCES;
    case Word::operculum:
    case Word::gill: return CF::PISCES;

    // Mollusks
    case Word::siphon:
    case Word::mantle: return CF::MOLLUSCA;

    // Fins - fish and cetaceans
    case Word::fin:
    case Word::pectoral:
    case Word::anal: return CF::PISCES;

    // Marine mammals
    case Word::flipper: return CF::CHORDATA;
    case Word::fluke: return CF::CETACEA;

    // Cephalopods
    case Word::tentacle:
    case Word::sucker: return CF::CEPHALOPODA;

    // Webbed feet - amphibians primarily
    case Word::web: return CF::AMPHIBIA;

    // Wings & feathers
    case Word::covert:
    case Word::feather: return CF::AVES;

    // Wing membrane - bats (mammals)
    case Word::patagium: return CF::MAMMALIA;
    case Word::aileron: return CF::NONE; // rigging term

    case Word::wing: return CF::NONE;
    case Word::pinion:
    case Word::alula: return CF::AVES;

    // Soft tissue / Internal - too general
    case Word::gland:
    case Word::duct:
    case Word::sac:
    case Word::chamber:
    case Word::valve:
    case Word::muscle:
    case Word::tendon:
    case Word::ligament:
    case Word::cartilage:
    case Word::joint:
    case Word::bone:
    case Word::socket:
    case Word::pivot: return CF::NONE;

    // Membrane - too general (wings, webbing, etc.)
    case Word::membrane: return CF::NONE;

    // Arthropod body section
    case Word::thorax: return CF::ARTHROPODA;

    // Rigging controls - not biological
    case Word::twist:
    case Word::roll:
    case Word::IK:
    case Word::FK:
    case Word::control:
    case Word::ctrl:
    case Word::deform:
    case Word::def:
    case Word::target:
    case Word::pole:
    case Word::master:
    case Word::mstr:
    case Word::jiggle:
    case Word::phys:
    case Word::sym: return CF::NONE;

    // Mammalian integument
    case Word::hair:
    case Word::mane:
    case Word::fur: return CF::MAMMALIA;

    // Adhesive - could be gecko (reptile), frog (amphibian), or arthropod
    case Word::adhesive: return CF::NONE; // too general

    case Word::sensory:
    case Word::chemoreceptor: return CF::NONE;

	case Word::compound:
    case Word::carnivore:
    case Word::TOTAL_WORDS: return CF::NONE;
    }

    return CF::NONE;
}

std::string_view TonTon::WordToString(SemanticFlags f)
{

#define CASE(x) case SemanticFlags::x: return #x

	switch(f)
	{
	CASE(LEFT);
	CASE(RIGHT);
	CASE(ANTERIOR);
	CASE(POSTERIOR);
	CASE(DORSAL);
	CASE(VENTRAL);
	CASE(MEDIAL);
	CASE(LATERAL);
	CASE(PROXIMAL);
	CASE(DISTAL);
	CASE(TERMINAL);
	CASE(HEAD);
	CASE(NECK);
	CASE(SPINE);
	CASE(ABDOMEN);
	CASE(TAIL);
	CASE(LIMB);
	CASE(DIGIT);
	CASE(FACIAL);
	CASE(TEETH);
	CASE(NAIL);
	CASE(GRASPER);
	CASE(HORN_ANTLER);
	CASE(AVIAN);
	CASE(AQUATIC);
	CASE(ARTHROPOD);
	CASE(EQUINE);
	CASE(WING);
	CASE(FIN);
	CASE(TENTACLE);
	CASE(ARMOR);
	CASE(ALPHA_CARD);
	CASE(WEAPON);
	CASE(ASSMETRICAL);
	CASE(RIGGING_CONTROL);
	CASE(DEFORMER);
	CASE(TWIST_ROLL);
	CASE(JIGGLE_BONE);
	CASE(INTERNAL);
	CASE(SENSORY);
	CASE(RESPIRATORY);
	CASE(MOUTH_PARTS);
	CASE(HEARING);
	CASE(VISION);
	CASE(CONTACT);
	CASE(FORELIMB);
	CASE(HINDLIMB);
	CASE(UPPER_LIMB);
	CASE(MID_LIMB);
	CASE(LOWER_LIMB);
	default: return "UNKNOWN";
	}
#undef CASE

}

std::string_view TonTon::WordToString(CladeFlags f)
{

#define CASE(x) case CladeFlags::x: return #x

	switch(f)
	{
	CASE(NONE);
	CASE(CHORDATA);
	CASE(AMPHIBIA);
	CASE(REPTILIA);
	CASE(CHELONIA);
	CASE(AVES);
	CASE(MAMMALIA);
	CASE(UNGULATA);
	CASE(EQUIDAE);
	CASE(CETACEA);
	CASE(PISCES);
	CASE(ARTHROPODA);
	CASE(INSECTA);
	CASE(ARACHNIDA);
	CASE(CRUSTACEA);
	CASE(MOLLUSCA);
	CASE(CEPHALOPODA);
	default: return "UNKNOWN";
	}
#undef CASE
}

TonTon::NicheFlags TonTon::GetNicheFlags(Word word)
{
    using NF = NicheFlags;

    switch(word)
    {
    case Word::predator: return NF::PREDATOR;
    case Word::herbivore: return NF::HERBIVORE;
    case Word::carnivore: return NF::CARNIVORE;
    default: return NF::NONE;
    }

    return NF::NONE;
}

std::string_view TonTon::WordToString(NicheFlags f)
{

#define CASE(x) case NicheFlags::x: return #x

	switch(f)
	{
	CASE(NONE);
	CASE(UNINITIALIZED);
	CASE(PREDATOR);
	CASE(HERBIVORE);
	CASE(CARNIVORE);
	default: return "UNKNOWN";
	}
#undef CASE

}
