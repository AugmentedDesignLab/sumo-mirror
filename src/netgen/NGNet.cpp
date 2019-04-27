/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2003-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    NGNet.cpp
/// @author  Markus Hartinger
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Mar, 2003
/// @version $Id$
///
// The class storing the generated network
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <netbuild/NBNode.h>
#include <netbuild/NBNodeCont.h>
#include <netbuild/NBEdge.h>
#include <netbuild/NBEdgeCont.h>
#include <netbuild/NBNetBuilder.h>
#include <utils/common/ToString.h>
#include <utils/common/RandHelper.h>
#include <utils/options/OptionsCont.h>
#include "NGNet.h"


// ===========================================================================
// method definitions
// ===========================================================================
NGNet::NGNet(NBNetBuilder& nb) :
    myLastID(0),
    myAlphaIDs(OptionsCont::getOptions().getBool("alphanumerical-ids")),
    myNetBuilder(nb) {
}


NGNet::~NGNet() {
    for (NGEdgeList::iterator ni = myEdgeList.begin(); ni != myEdgeList.end(); ++ni) {
        delete *ni;
    }
    for (NGNodeList::iterator ni = myNodeList.begin(); ni != myNodeList.end(); ++ni) {
        delete *ni;
    }
}


std::string
NGNet::getNextFreeID() {
    return toString<int>(++myLastID);
}


NGNode*
NGNet::findNode(int xID, int yID) {
    for (NGNodeList::iterator ni = myNodeList.begin(); ni != myNodeList.end(); ++ni) {
        if ((*ni)->samePos(xID, yID)) {
            return *ni;
        }
    }
    return 0;
}

std::string
NGNet::alphabeticalCode(int i, int iMax) {
    // lazy mans 26th root to determine number of characters for x-label
    int xn = 1;
    for (; std::pow(26, xn) < iMax; xn++) {};
    std::string result = "";
    for (int j = 0; j < xn; j++) {
        result = char('A' + (i % 26)) + result;
        i /= 26;
    }
    return result;
}

void
NGNet::createChequerBoard(int numX, int numY, double spaceX, double spaceY, double attachLength) {

    for (int ix = 0; ix < numX; ix++) {
        const std::string nodeIDStart = (myAlphaIDs ? alphabeticalCode(ix, numX) : toString<int>(ix) + "/");
        for (int iy = 0; iy < numY; iy++) {
            // create Node
            NGNode* node = new NGNode(nodeIDStart + toString(iy), ix, iy);
            node->setX(ix * spaceX + attachLength);
            node->setY(iy * spaceY + attachLength);
            myNodeList.push_back(node);
            // create Links
            if (ix > 0) {
                connect(node, findNode(ix - 1, iy));
            }
            if (iy > 0) {
                connect(node, findNode(ix, iy - 1));
            }
        }
    }
    if (attachLength > 0.0) {
        for (int ix = 0; ix < numX; ix++) {
            // create nodes
            NGNode* topNode = new NGNode("top" + toString<int>(ix), ix, numY);
            NGNode* bottomNode = new NGNode("bottom" + toString<int>(ix), ix, numY + 1);
            topNode->setX(ix * spaceX + attachLength);
            bottomNode->setX(ix * spaceX + attachLength);
            topNode->setY((numY - 1) * spaceY + 2 * attachLength);
            bottomNode->setY(0);
            myNodeList.push_back(topNode);
            myNodeList.push_back(bottomNode);
            // create links
            connect(topNode, findNode(ix, numY - 1));
            connect(bottomNode, findNode(ix, 0));
        }
        for (int iy = 0; iy < numY; iy++) {
            // create nodes
            NGNode* leftNode = new NGNode("left" + toString<int>(iy), numX, iy);
            NGNode* rightNode = new NGNode("right" + toString<int>(iy), numX + 1, iy);
            leftNode->setX(0);
            rightNode->setX((numX - 1) * spaceX + 2 * attachLength);
            leftNode->setY(iy * spaceY + attachLength);
            rightNode->setY(iy * spaceY + attachLength);
            myNodeList.push_back(leftNode);
            myNodeList.push_back(rightNode);
            // create links
            connect(leftNode, findNode(0, iy));
            connect(rightNode, findNode(numX - 1, iy));
        }
    }
}

void
NGNet::createTownSim(std::string filename) {

	std::ifstream file(filename);
	std::string input;

	std::string delim1 = "(";
	std::string delim2 = "),(";
	std::string delim3 = "),[";
	std::string delim4 = "]";

	std::string delimA = ",";
	std::string delimB = "(";
	std::string delimC = "),(";
	std::string delimD = ")";

	while (std::getline(file, input)) {
		input = input.substr(input.find(delim1) + 1);
		std::string node1 = input.substr(0, input.find(delim2));
		int x1 = std::stoi(node1.substr(0, input.find(delimA)));
		int y1 = std::stoi(node1.substr(input.find(delimA) + 1));
		
		NGNode* n1 = findNode(x1, y1);
		if (n1 == NULL) {
			std::string id1 = std::to_string(x1) + "_" + std::to_string(y1);
			n1 = new NGNode(id1, x1, y1);
			n1->setX(x1 * 10);
			n1->setY(y1 * 10);
			myNodeList.push_back(n1);
		}
		
		input = input.substr(input.find(delim2) + 3);
		std::string node2 = input.substr(0, input.find(delim3));
		int x2 = std::stoi(node2.substr(0, input.find(delimA)));
		int y2 = std::stoi(node2.substr(input.find(delimA) + 1));
		
		NGNode* n2 = findNode(x2, y2);
		if (n2 == NULL) {
			std::string id2 = std::to_string(x2) + "_" + std::to_string(y2);
			n2 = new NGNode(id2, x2, y2);
			n2->setX(x2 * 10);
			n2->setY(y2 * 10);
			myNodeList.push_back(n2);
		}
		
		input = input.substr(input.find(delim3) + 3);
		std::string shape = input.substr(0, input.find(delim4));

		std::vector<Position> v;
		
		if (shape.find(delimB) != std::string::npos) {
			v.push_back(*(new Position(x1 * 10, y1 * 10, 0)));
			while (shape.find(delimB) != std::string::npos) {
				shape = shape.substr(shape.find(delimB) + 1);
				int turnX = std::stoi(shape.substr(0, shape.find(delimA)));
				int turnY = std::stoi(shape.substr(shape.find(delimA) + 1, shape.find(delimD)));
				v.push_back(*(new Position(turnX * 10.0, turnY * 10.0, 0.0)));
			}
			v.push_back(*(new Position(x2 * 10, y2 * 10, 0)));
		}

		input = input.substr(input.find(delim4) + 1);
		std::string type = std::string("minor");
		if (input.length() > 0) {
			type = input.substr(1);
		}
		PositionVector pos = *(new PositionVector(v));
		connect(n1, n2, pos, type);
	}
}



double
NGNet::radialToX(double radius, double phi) {
    return cos(phi) * radius;
}


double
NGNet::radialToY(double radius, double phi) {
    return sin(phi) * radius;
}


void
NGNet::createSpiderWeb(int numRadDiv, int numCircles, double spaceRad, bool hasCenter) {
    if (numRadDiv < 3) {
        numRadDiv = 3;
    }
    if (numCircles < 1) {
        numCircles = 1;
    }

    int ir, ic;
    double angle = (double)(2 * M_PI / numRadDiv); // angle between radial divisions
    NGNode* Node;
    for (ic = 1; ic < numCircles + 1; ic++) {
        const std::string nodeIDStart = alphabeticalCode(ic, numCircles);
        for (ir = 1; ir < numRadDiv + 1; ir++) {
            // create Node
            const std::string nodeID = (myAlphaIDs ?
                                        nodeIDStart + toString<int>(ir) :
                                        toString<int>(ir) + "/" + toString<int>(ic));
            Node = new NGNode(nodeID, ir, ic);
            Node->setX(radialToX((ic) * spaceRad, (ir - 1) * angle));
            Node->setY(radialToY((ic) * spaceRad, (ir - 1) * angle));
            myNodeList.push_back(Node);
            // create Links
            if (ir > 1) {
                connect(Node, findNode(ir - 1, ic));
            }
            if (ic > 1) {
                connect(Node, findNode(ir, ic - 1));
            }
            if (ir == numRadDiv) {
                connect(Node, findNode(1, ic));
            }
        }
    }
    if (hasCenter) {
        // node
        Node = new NGNode(myAlphaIDs ? "A1" : "1", 0, 0, true);
        Node->setX(0);
        Node->setY(0);
        myNodeList.push_back(Node);
        // links
        for (ir = 1; ir < numRadDiv + 1; ir++) {
            connect(Node, findNode(ir, 1));
        }
    }
}


void
NGNet::connect(NGNode* node1, NGNode* node2) {
    std::string id1 = node1->getID() + (myAlphaIDs ? "" : "to") + node2->getID();
    std::string id2 = node2->getID() + (myAlphaIDs ? "" : "to") + node1->getID();
    NGEdge* link1 = new NGEdge(id1, node1, node2);
    NGEdge* link2 = new NGEdge(id2, node2, node1);
    myEdgeList.push_back(link1);
    myEdgeList.push_back(link2);
}

void
NGNet::connect(NGNode* node1, NGNode* node2, PositionVector shape, std::string type) {
	std::string id1 = node1->getID() + (myAlphaIDs ? "" : "to") + node2->getID();
	std::string id2 = node2->getID() + (myAlphaIDs ? "" : "to") + node1->getID();
	NGEdge* link1 = new NGEdge(id1, node1, node2, shape, type);
	//NGEdge* link1 = new NGEdge(id1, node1, node2);
	NGEdge* link2 = new NGEdge(id2, node2, node1, shape.reverse(), type);
	//NGEdge* link2 = new NGEdge(id2, node2, node1);
	myEdgeList.push_back(link1);
	myEdgeList.push_back(link2);
}


void
NGNet::toNB() const {
    std::vector<NBNode*> nodes;
    for (NGNodeList::const_iterator i1 = myNodeList.begin(); i1 != myNodeList.end(); i1++) {
        NBNode* node = (*i1)->buildNBNode(myNetBuilder);
        nodes.push_back(node);
        myNetBuilder.getNodeCont().insert(node);
    }
    for (NGEdgeList::const_iterator i2 = myEdgeList.begin(); i2 != myEdgeList.end(); i2++) {
        NBEdge* edge = (*i2)->buildNBEdge(myNetBuilder);
        myNetBuilder.getEdgeCont().insert(edge);
    }
    // now, let's append the reverse directions...
    double bidiProb = OptionsCont::getOptions().getFloat("rand.bidi-probability");
    for (std::vector<NBNode*>::const_iterator i = nodes.begin(); i != nodes.end(); ++i) {
        NBNode* node = *i;
        for (NBEdge* e : node->getIncomingEdges()) {
            if (node->getConnectionTo(e->getFromNode()) == 0 && RandHelper::rand() <= bidiProb) {
                NBEdge* back = new NBEdge("-" + e->getID(), node, e->getFromNode(),
                                          "", myNetBuilder.getTypeCont().getSpeed(""),
                                          e->getNumLanes(),
                                          e->getPriority(),
                                          myNetBuilder.getTypeCont().getWidth(""), NBEdge::UNSPECIFIED_OFFSET);
                myNetBuilder.getEdgeCont().insert(back);
            }
        }
    }
    // add splits depending on turn-lane options
    const int turnLanes = OptionsCont::getOptions().getInt("turn-lanes");
    const bool lefthand =  OptionsCont::getOptions().getBool("lefthand");
    if (turnLanes > 0) {
        const double turnLaneLength = OptionsCont::getOptions().getFloat("turn-lanes.length");
        NBEdgeCont& ec = myNetBuilder.getEdgeCont();
        EdgeVector allEdges;
        for (auto it = ec.begin(); it != ec.end(); ++it) {
            allEdges.push_back(it->second);
        }
        for (NBEdge* e : allEdges) {
            if (e->getToNode()->geometryLike()) {
                continue;
            }
            std::vector<NBEdgeCont::Split> splits;
            NBEdgeCont::Split split;
            for (int i = 0; i < e->getNumLanes() + turnLanes; ++i) {
                split.lanes.push_back(i);
            }
            split.pos = MAX2(0.0, e->getLength() - turnLaneLength);
            split.speed = e->getSpeed();
            split.node = new NBNode(e->getID() + "." + toString(split.pos), e->getGeometry().positionAtOffset(split.pos));
            split.idBefore = e->getID();
            split.idAfter = split.node->getID();
            split.offsetFactor = lefthand ? -1 : 1;
            if (turnLaneLength <= e->getLength() / 2) {
                split.offset = -0.5 * split.offsetFactor * turnLanes * e->getLaneWidth(0);
                if (e->getFromNode()->geometryLike()) {
                    // shift the reverse direction explicitly as it will not get a turn lane
                    NBEdge* reverse = 0;
                    for (NBEdge* reverseCand : e->getFromNode()->getIncomingEdges()) {
                        if (reverseCand->getFromNode() == e->getToNode()) {
                            reverse = reverseCand;
                        }
                    }
                    if (reverse != 0) {
                        PositionVector g = reverse->getGeometry();
                        g.move2side(-split.offset);
                        reverse->setGeometry(g);
                    }
                }
            }
            splits.push_back(split);
            ec.processSplits(e, splits,
                             myNetBuilder.getNodeCont(),
                             myNetBuilder.getDistrictCont(),
                             myNetBuilder.getTLLogicCont());
        }
    }
}


void
NGNet::add(NGNode* node) {
    myNodeList.push_back(node);
}


void
NGNet::add(NGEdge* edge) {
    myEdgeList.push_back(edge);
}


int
NGNet::nodeNo() const {
    return (int)myNodeList.size();
}


/****************************************************************************/

