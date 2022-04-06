#include "SingleAgentSolver.h"


list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
	list<int> rst = instance.getNeighbors(curr);
	rst.emplace_back(curr);
	return rst;
}


void SingleAgentSolver::compute_heuristics()
{
	struct Node
	{
		int location;
		int value;

		Node() = default;
		Node(int location, int value) : location(location), value(value) {}
		// the following is used to comapre nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};
	//std::cout << "SAS " << __LINE__ << std::endl;
	my_heuristic.resize(instance.map_size, MAX_TIMESTEP);
	//std::cout << "SAS " << __LINE__ << std::endl;

	// generate a heap that can save nodes (and a open_handle)
	boost::heap::pairing_heap< Node, boost::heap::compare<Node::compare_node> > heap;
	//std::cout << "SAS " << __LINE__ << std::endl;

	Node root(goal_location, 0);
	//std::cout << "SAS " << __LINE__ << std::endl;
	//std::cout << "SAS " <<  "goal_loc " << goal_location << std::endl;
	my_heuristic[goal_location] = 0;
	//std::cout << "SAS " << __LINE__ << std::endl;
	heap.push(root);  // add root to heap
	//std::cout << "SAS " << __LINE__ << std::endl;
	while (!heap.empty())
	{
		Node curr = heap.top();
		//std::cout << "SAS " << __LINE__ << std::endl;
		heap.pop();
		//std::cout << "SAS " << __LINE__ << std::endl;
		for (int next_location : instance.getNeighbors(curr.location))
		{
			//std::cout << "SAS " << __LINE__ << std::endl;
			if (my_heuristic[next_location] > curr.value + 1)
			{
				//std::cout << "SAS " << __LINE__ << std::endl;
				my_heuristic[next_location] = curr.value + 1;
				Node next(next_location, curr.value + 1);
				heap.push(next);
			}
		}
	}
}