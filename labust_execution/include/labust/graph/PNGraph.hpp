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
 *********************************************************************/
#ifndef PNGRAPH_HPP_
#define PNGRAPH_HPP_
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <Eigen/Dense>

#include <iosfwd>

namespace labust
{
	namespace graph
	{
		/**
		 * The Petri-Net container class. The class contains the Petri-Net graph and
		 * Petri-Net matrix representation. The Petri-Net places are named for debugging
		 * convenience.
		 *
		 * \todo Token labeling and tracking
		 * \todo Return std::pair<TYPE(P/T), idx> outside of PNGraph (keep vertex property
		 *  only inside class or upon request)
		 * \todo Switch vertex property to pointer type ? (return CPtr to disallow changing
		 * parts of vertex)
		 * \todo Token class containers
		 * \todo Solve the potential problem of some tokens not having names (split to subclasses?)
		 * \todo Incidence matrix is reevaluated on each change (add lazy evaluation ?)
		 * \todo Optimize idx<->vertex mapping (add a large number xxxx|yyyy == p/tidx|vertexIdx) ?
		 * \todo Add general transition invariance detection
		 * \todo Build BFS only till the target is reached
		 * \todo For offline/unchangable trees add reachability calculation
		 * \todo For transition searches add BFS map saving.
		 * \todo Consider list instead of vectors for sequences that are manipulated
		 */
		class PNGraph
		{
		public:
			/**
			 * Petri-Net directional graph vertex property
			 * \todo Add only single index instead of t_num, p_num
			 * \todo Separate to private and public interface
			 * \todo Create two sub-classes with separate properties (P/T)
			 */
			struct VertexProperty
			{
				///Two vertex types - place or transition
				enum {transition=0, place=1};
				///Vertex type
				int type;
				///Matrix index of this transition or place
				int idx;
				///Matrix index of this transition (deprecated)
				int t_num;
				///Matrix index of this place (deprecated)
				int p_num;
				///Vertex index in the graph
				int vertexIdx;

				///Current token labels
				std::vector<std::string> tokens;
				///Debug name of the place or transition
				std::string name;
			};

			///The directional graph of the Petri-Net type
			typedef boost::adjacency_list<boost::vecS, boost::vecS,
				boost::bidirectionalS, VertexProperty,
				boost::property<boost::edge_name_t, int> > GraphType;

			typedef std::map<int, GraphType::vertex_descriptor> MGMap;
			typedef std::map<std::string, GraphType::vertex_descriptor> TMap;
			typedef GraphType::vertex_descriptor IntType;
			typedef std::vector<VertexProperty> TSequence;
			typedef boost::shared_ptr<TSequence> TSequencePtr;

			/**
			 * The graphviz writer class for the PN graph.
			 */
			struct PNGraphDotWriter {
				PNGraphDotWriter(const PNGraph& pngraph):graph(pngraph.pngraph){}
				template <class Vertex>
				void operator()(std::ostream &out, const Vertex& e) const
				{
					if (graph[e].type ==
							GraphType::vertex_property_type::value_type::place)
					{
						out << "[label="<< graph[e].name<<"]";
					}
					else
					{
						out << "[height=0.05, style=filled, shape=rectangle, color=black, label=\"\"]";
					}
				}
				const PNGraph::GraphType& graph;
			};

			/**
			 * Add single (unconnected) place into the Petri-Net.
			 * @param name Debug name of the place. Empty by default.
			 * @param label The token label
			 * @return The place index.
			 */
			VertexProperty addPlace(const std::string& name = "");

			/**
			 * Add single (unconnected) transition into the Petri-Net.
			 * @param name Debug name of the place. Empty by default.
			 * @return The transition index.
			 */
			VertexProperty addTransition(const std::string& name = "" );
			/**
			 * Connect a place and transition.
			 * Called with:
			 *  (place, transition) - place -> transition
			 *  (transition, place) - transition -> place
			 * otherwise throws a runtime error.
			 * @param out Outgoing place or outgoing transition.
			 * @param in Incoming transition or incoming place.
			 * @param weight The weight of the connection.
			 */
			void connect(const VertexProperty& out, const VertexProperty& in, int weight = 1);

			void addToken(const VertexProperty& place,	const std::string& label = "");

			template <class Vector>
			void setMarking(const Vector& vector)
			{
				if (vector.size() == marking.size()) marking = vector;
			}

			TSequencePtr findTransitions(const VertexProperty& from,
					const VertexProperty& to, bool allowInvariants = false);

			bool fire(const std::vector<VertexProperty>& transitions);

			inline bool fire(const VertexProperty& transition)
			{
				std::vector<VertexProperty> transitions(1, transition);
				return fire(transitions);
			}

			bool isEnabled(const VertexProperty& transition);

			inline bool isMarked(const VertexProperty& place)
			{
				if (place.type == VertexProperty::place)
					return marking(place.idx) > 0;
				else
					return false;
			}

			///Input matrix of the Petri-Net
			Eigen::MatrixXi Dm;
			///Output matrix of the Petri-Net
			Eigen::MatrixXi Dp;
			///Incidence matrix of the Petri-Net
			Eigen::MatrixXi I;
			///Marking of the Petri-Net
			Eigen::VectorXi marking;

			///The directional graph holding the Petri-Net
			GraphType pngraph;
			///Matrix row (place) index to vertex index map
			MGMap placeToVertex;
			///Matrix col (transition) index to vertex index map
			MGMap transitionToVertex;
			///Token marking
			TMap tokenTracker;


			///Friend output function
			friend std::ostream& operator<<(std::ostream& os, const PNGraph& obj);
		};

		/**
		 * Output DOT description of the graph.
		 * @param os
		 * @param obj
		 * @return
		 */
		std::ostream& operator<<(std::ostream& os, const PNGraph& obj);
	}
}

/* PNGRAPH_HPP_ */
#endif
