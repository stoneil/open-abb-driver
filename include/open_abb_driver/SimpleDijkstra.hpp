#ifndef _SIMPLE_DIJKSTRA_H_
#define _SIMPLE_DIJKSTRA_H_

#include <set>
#include <memory>
#include <unordered_map>
#include <queue>

#include <boost/foreach.hpp>

namespace open_abb_driver
{
	
class DijkstraNode;

struct DijkstraEdge
{
	std::shared_ptr<DijkstraNode> parent;
	std::shared_ptr<DijkstraNode> child;
	double cost;
};

class DijkstraNode
{
public:
	
	typedef std::shared_ptr<DijkstraNode> Ptr;
	
	DijkstraNode() {}
	virtual ~DijkstraNode() {}
	
	void AddEdge( const DijkstraEdge& edge ) { edges.push_back( edge ); }
	std::vector<DijkstraEdge> GetEdges() { return edges; }
	
	double GetCost() const { return cost; }
	void SetCost( double c ) { cost = c; }
	
	DijkstraNode::Ptr GetParent() const { return parent; }
	void SetParent( const DijkstraNode::Ptr& p ) { parent = p; }
	
private:
	
	double cost;
	std::vector<DijkstraEdge> edges;
	DijkstraNode::Ptr parent;
	
};

class DijkstraComparator
{
public:
	bool operator()( const DijkstraNode::Ptr& a, const DijkstraNode::Ptr& b )
	{
		return a->GetCost() > b->GetCost();
	}
};

class DijkstraSearch
{
public:
	
	DijkstraSearch() {}
	
	// NOTE Don't double add! There is no checking in here.
	void AddNode( const DijkstraNode::Ptr& node, bool initial = false, bool goal = false )
	{
		if( initial )
		{
			current.push( node );
			node->SetCost( 0 );
		}
		else
		{
			node->SetCost( std::numeric_limits<double>::infinity() );
		}
		
		if( goal )
		{
			goals.insert( node.get() );
		}
	}
	
	std::vector<DijkstraNode::Ptr> Execute()
	{
		while( current.size() > 0 )
		{
			DijkstraNode::Ptr currentNode = current.top();
			current.pop();
			
			if( goals.count( currentNode.get() ) > 0 ) 
			{
				return Backtrace( currentNode );
			}
			
			if( visited.count( currentNode.get() ) > 0 ) { continue; }
			
			std::vector<DijkstraEdge> edges = currentNode->GetEdges();
// 				std::cout << "Node " << currentNode.get() << " has " << edges.size() << " edges." << std::endl;
			BOOST_FOREACH( const DijkstraEdge& edge, edges )
			{
				if( visited.count( edge.child.get() ) > 0 ) { continue; }
				
				double acc = currentNode->GetCost() + edge.cost;
				if( acc < edge.child->GetCost() )
				{
					edge.child->SetParent( currentNode );
					edge.child->SetCost( acc );
				}
				current.push( edge.child );
			}
			visited.insert( currentNode.get() );
		}
		
// 			std::cout << "Dijkstra failed!" << std::endl;
		std::vector<DijkstraNode::Ptr> ret;
		return ret;
	}
	
	std::vector<DijkstraNode::Ptr> Backtrace( const DijkstraNode::Ptr& start )
	{
		std::vector<DijkstraNode::Ptr> trace;
		trace.push_back( start );
		DijkstraNode::Ptr parent = start->GetParent();
		while( parent )
		{
			trace.push_back( parent );
			parent = parent->GetParent();
		}
		std::reverse( trace.begin(), trace.end() );
		return trace;
	}
	
private:
	
	typedef std::set<DijkstraNode*> NodeSet;

	typedef std::priority_queue< DijkstraNode::Ptr, 
								 std::vector<DijkstraNode::Ptr>, 
								 DijkstraComparator > PQueue;
	
	PQueue current;
	NodeSet visited;
	NodeSet goals;
	
};
	
}

#endif