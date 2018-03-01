/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file GraphCut.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief GraphCut algorithm to cut graphs into pieces for segmentation.
 */

#ifndef GC_GRAPHCUT_H
#define GC_GRAPHCUT_H

#include <set>
#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include <algorithm>

#include "Graph.h"
#include "Edge.h"
#include "disjoint-set.h"

#include "Relation.h"
#include "SurfaceModel.hpp"

namespace gc
{

// THRESHOLD_CONSTANT:
// CAREFULL: Float value!!!
#define THRESHOLD_CONSTANT 0.5    /// good value 0.5 
#define MIN_SIZE 1                /// minimum size of element-sets

#define THRESHOLD(size, c) (c/size)

/**
 * @brief Class GraphCut
 */
class GraphCut
{
public:
  
private:
  bool print;
  bool createAllRelations;                          ///< create all combinatorial combinations of relations (if graph is not fully connected)

  bool initialized;                                 ///< flag to process
  bool processed;                                   ///< flag to get results
  unsigned num_edges;                               ///< Number of edges

  surface::View *view;                              ///< View with relations
  gc::Edge *edges;                                  ///< Edges between the nodes, representing a probability
  universe *u;                                      ///< universe to cut graph
  
  public:
  GraphCut();
  ~GraphCut();
  
  /** Initialize the graph cut algorithm with number of nodes and with all available relations **/
  bool init(surface::View *_view);

  /** Process graph cut **/
  void process();
  
  /** Print the results of the graph cut **/
  void printResults(bool _printResults) {print = _printResults;}
};

}

#endif

