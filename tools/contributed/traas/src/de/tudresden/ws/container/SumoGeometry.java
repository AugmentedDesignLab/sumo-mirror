/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2017-2017 German Aerospace Center (DLR) and others.
// TraaS module
// Copyright (C) 2016-2017 Dresden University of Technology
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    SumoGeometry.java
/// @author  Mario Krumnow
/// @author  Evamarie Wiessner
/// @date    2016
/// @version $Id$
///
//
/****************************************************************************/
package de.tudresden.ws.container;

import java.util.LinkedList;

/**
 * 
 * @author Mario Krumnow
 *
 */

public class SumoGeometry implements SumoObject {

	public LinkedList<SumoPosition2D> coords;
	
	public SumoGeometry(){
		this.coords = new LinkedList<SumoPosition2D>();
	}
	
	public void add(SumoPosition2D pos){
		this.coords.add(pos);
	}
	
	public String toString(){
		
		StringBuilder sb = new StringBuilder();
		for(SumoPosition2D pos : coords){
			sb.append(pos.x+",");
			sb.append(pos.y+" ");
		}
		
		return sb.toString().trim();
		
	}
	
	public void fromString(String shape){
		
		String[] arr = shape.split(" ");
		for(String s : arr){
			String[] tmp = s.split(",");
			double x = Double.valueOf(tmp[0]);
			double y = Double.valueOf(tmp[1]);
			this.add(new SumoPosition2D(x, y));
		}
		
	}
	
}