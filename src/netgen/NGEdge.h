/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    NGEdge.h
/// @author  Markus Hartinger
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Mar, 2003
/// @version $Id$
///
// A netgen-representation of an edge
/****************************************************************************/
#ifndef NGNetElements_h
#define NGNetElements_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <list>
#include <utils/common/Named.h>
#include <utils/common/UtilExceptions.h>
#include <utils/geom/Position.h>
#include <utils/geom/GeomHelper.h>


// ===========================================================================
// class declarations
// ===========================================================================
class NGNode;
class NBNode;
class NBEdge;
class NBNetBuilder;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NGEdge
 * @brief A netgen-representation of an edge
 *
 * Please note that the edge makes itself known to the from- and the to-nodes
 *  on initialisation and removes this information from the nodes when being
 *  deleted. This implicates that nodes have to be deleted after the edges.
 */
class NGEdge : public Named {
public:
    /** @brief Constructor
     *
     * Adds itself to the start and the end node's lists of connections.
     *
     * @param[in] id The id of the link
     * @param[in] StartNode The begin node
     * @param[in] EndNode The end node
	 * @param[in] Shape Shape of the edge 
     */
    NGEdge(const std::string& id, NGNode* startNode, NGNode* endNode);
	NGEdge(const std::string& id, NGNode* startNode, NGNode* endNode, PositionVector shape, std::string type);



    /** @brief Destructor
     *
     * Removes itself from the start and the end node's lists of connections.
     */
    ~NGEdge();


    /** @brief Returns this link's start node
     *
     * @return The start node of the link
     */
    NGNode* getStartNode() {
        return myStartNode;
    };


    /** @brief Returns this link's end node
     *
     * @return The end node of the link
     */
    NGNode* getEndNode() {
        return myEndNode;
    };


    /** @brief Builds and returns this link's netbuild-representation
     *
     * Returns an edge built using the known values. Other values, such as the
     *  number of lanes, are gathered from defaults.
     * The starting and the ending node must have been built in prior.
     *
     * @param[in] nb The netbuilder to retrieve the referenced nodes from
     * @return The built edge
     */
    NBEdge* buildNBEdge(NBNetBuilder& nb) const;


private:
    /// @brief The node the edge starts at
    NGNode* myStartNode;

    /// @brief The node the edge ends at
    NGNode* myEndNode;

	PositionVector myShape;

	std::string myType;
};


/**
 * @typedef NGEdgeList
 * @brief A list of edges (edge pointers)
 */
typedef std::list<NGEdge*> NGEdgeList;

#endif

/****************************************************************************/

