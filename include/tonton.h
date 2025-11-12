#ifndef TONTON_H
#define TONTON_H

#include "tonton_input.h"
#include "tonton_output.h"
#include "tonton_wordlist.h"
#include "tonton_formatter.h"

/* IMPORTANT!
 * you must define this! 
 * tonton does not define it!
 *
 * Expected output: 
 * - vec3 should be sorted from least to greatest
 * - quat should be the proper rotation given that sorting
 */
 
std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::dmat3 const& I);

#endif

