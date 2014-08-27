/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Dula Nad
 *  Created: 30.10.2013.
 *********************************************************************/
#include <labust/graph/PNGraph.hpp>
#include <labust/graph/BFSTools.hpp>
#include <boost/graph/graphviz.hpp>

#include <stdexcept>

using namespace labust::graph;

template <class Vector>
void print_vec(const Vector& vec)
{
	std::cout<<"(";
	int i;
	for(i=0; i<vec.size()-1;++i)
	{
		std::cout<<vec(i)<<",";
	}
	std::cout<<vec(i)<<")";
}

PNGraph::VertexProperty PNGraph::addPlace(const std::string& name)
{
	int pnum = Dm.rows() + 1;
	int pidx = Dm.rows();
	int tnum = Dm.cols();

	//Resize matrices to fit new place
	Dm.conservativeResize(pnum, tnum);
	Dp.conservativeResize(pnum, tnum);

	//Resize the marking to fit new place
	marking.conservativeResize(pnum);
	marking(pidx) = 0;

	//Zero new rows
	Dm.row(pidx) = Eigen::VectorXi::Zero(tnum);
	Dp.row(pidx) = Eigen::VectorXi::Zero(tnum);

	//Add to graph
	IntType place = boost::add_vertex(pngraph);
	pngraph[place].type = VertexProperty::place;
	pngraph[place].idx = pidx;
	pngraph[place].p_num = pidx;
	pngraph[place].name = name;
	pngraph[place].vertexIdx = place;
	placeToVertex[pidx] = place;

	return pngraph[place];
}

PNGraph::VertexProperty PNGraph::addTransition(const std::string& name)
{
	int pnum = Dm.rows();
	int tidx = Dm.cols();
	int tnum = Dm.cols()+1;

	//Resize matrices to fit new place
	Dm.conservativeResize(pnum, tnum);
	Dp.conservativeResize(pnum, tnum);

	//Zero new rows
	Dm.col(tidx) = Eigen::VectorXi::Zero(pnum);
	Dp.col(tidx) = Eigen::VectorXi::Zero(pnum);

	//Add to graph
	typedef GraphType::vertex_descriptor IntType;
	IntType transition = boost::add_vertex(pngraph);
	pngraph[transition].type = VertexProperty::transition;
	pngraph[transition].idx = tidx;
	pngraph[transition].t_num = tidx;
	pngraph[transition].name = name;
	pngraph[transition].vertexIdx = transition;
	transitionToVertex[tidx] = transition;

	return pngraph[transition];
}

void PNGraph::connect(const VertexProperty& out, const VertexProperty& in, int weight)
{
	if ((out.type == VertexProperty::place) &&
			(in.type == VertexProperty::transition))
	{
		//std::cout<<"Connecting place to transition: ";
		Dm(out.idx, in.idx) = weight;
		boost::add_edge(placeToVertex[out.idx],
				transitionToVertex[in.idx],	weight, pngraph);
	}
	else if ((in.type == VertexProperty::place) &&
			(out.type == VertexProperty::transition))
	{
		//std::cout<<"Connecting transition to place: ";
		Dp(in.idx, out.idx) = weight;
		boost::add_edge(transitionToVertex[out.idx],
				placeToVertex[in.idx], weight, pngraph);
	}
	else
	{
		throw std::runtime_error("Only possible connection is between places and transitions.");
	}
	//std::cout<<out.name<<"("<<out.idx<<") -> "<<in.name<<"("<<in.idx<<")"<<std::endl;
}

bool PNGraph::isEnabled(const VertexProperty& transition)
{
	Eigen::VectorXi all_trans = marking.transpose()*Dm;
	return (all_trans(transition.idx) >= Dm.col(transition.idx).sum());
}

bool PNGraph::fire(const std::vector<VertexProperty>& transitions)
{
	Eigen::VectorXi fire = Eigen::VectorXi::Zero(Dm.cols());
	Eigen::VectorXi m(marking);
	//Evaluate I
	I = Dp - Dm;

	//Fire transitions sequentially
	std::vector<int> fired;
	bool retVal = true;
 	for(std::vector<VertexProperty>::const_iterator it=transitions.begin();
 			it != transitions.end();
 			++it)
 	{
 		if (it->type != VertexProperty::transition)
 			throw std::runtime_error("Only transitions can be in the firing sequence.");

 		Eigen::VectorXi all_trans = m.transpose()*Dm;
 		if (all_trans(it->idx) == Dm.col(it->idx).sum())
 		{
 		  fire(it->idx) = 1;
 			m = m + I*fire;
 			fire(it->idx) = 0;
 			fired.push_back(it->idx);
 		}
 		else
 		{
 			retVal = false;
 			std::cout<<"Transition "<<it->name<<" is not enabled, skipping."<<std::endl;
 		}
 	}

 	//Update the net marking if no errors occurred
 	marking = m;

 	return retVal;
}

void PNGraph::addToken(const VertexProperty& place,	const std::string& label)
{
	if (place.type == VertexProperty::place)
	{
		if (!label.empty())
		{
			pngraph[place.vertexIdx].tokens.push_back(label);
			tokenTracker[label] = pngraph[place.vertexIdx].idx;
		}
		marking(place.idx) += 1;
	}
}

PNGraph::TSequencePtr PNGraph::findTransitions(const VertexProperty& from,
		const VertexProperty& to, bool allowInvariants)
{
	if ((from.type != VertexProperty::place) || (to.type != VertexProperty::place))
		throw std::runtime_error("Transition sequence can only be searched between places.");

	TSequencePtr retVal(new TSequence());

	//If it is the same place
	if (from.vertexIdx == to.vertexIdx) return retVal;

	//Search the graph
  std::vector <IntType> distance(boost::num_vertices(pngraph));
	std::vector <IntType> predmap(boost::num_vertices(pngraph));
	//Build the predecessor map
	boost::breadth_first_search(pngraph, from.vertexIdx,
			boost::visitor(makeBFSDistanceVisitor(&distance[0],&predmap[0])));

	if (distance[to.vertexIdx] != 0)
	{
		std::cout<<"Directed graph distance from "<<from.name<<" to ";
		std::cout<<to.name<<" is "<<distance[to.vertexIdx]<<std::endl;

  	IntType pred = to.vertexIdx;
  	bool validSeq = true;
  	std::set<int> firingseq;
  	while (pred != from.vertexIdx)
  	{
  		if (pngraph[pred].type == VertexProperty::transition)
  		{
   			//Note that this is a simplified invariance detection
  			//that assumes enable/disable transition pairs added
  			//one after the other
  			if (!allowInvariants)
  			{
  				int tnum = pngraph[pred].idx, tpair = 0;
  				if (tnum % 2 == 0) tpair = tnum+1; else tpair = tnum - 1;
  				if (firingseq.find(tpair) != firingseq.end())
  				{
  					std::cout<<"Invariant detected"<<std::endl;
  					validSeq = false;
  					break;
  				}
  				firingseq.insert(pngraph[pred].idx);
  			}

  			std::cout<<"Adding transition "<<pngraph[pred].name<<std::endl;
  			retVal->push_back(pngraph[pred]);
  		}

  		//Next predecessor
  		pred = predmap[pred];
  	}

  	//Clear the return sequence if iteration failed
  	if (!validSeq) retVal.reset(new TSequence());
	}
	else
	{
		std::cout<<"Place "<<to.name<<" is unreachable from place "<<from.name<<std::endl;
	}

	return retVal;
}

std::ostream& labust::graph::operator<<(std::ostream& os, const PNGraph& obj)
{
	boost::write_graphviz(os, obj.pngraph,
			labust::graph::PNGraph::PNGraphDotWriter(obj));
	return os;
}
