
#include "./render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree() : root(NULL) {}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node* &node, Node* &inNode, uint axis = 0)
	{
		if (node == NULL)
			node = inNode;
		else if (inNode->point[axis % 3] < node->point[axis % 3])
			insertHelper(node->left, inNode, axis+1);
		else 
			insertHelper(node->right, inNode, axis+1);	
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly within the root
		Node* inNode = new Node(point, id); 
		insertHelper(root, inNode); 
	}

	void searchHelper(std::vector<float> target, std::vector<int> &ids, Node* n, float distanceTol, uint axis = 0)
	{
		// check if node is null
		if (n == NULL)
			return;
		if (target[axis%3]+distanceTol < n->point[axis%3])
			searchHelper(target, ids, n->left, distanceTol, axis+1);
		else if (target[axis%3]-distanceTol > n->point[axis%3])
			searchHelper(target, ids, n->right, distanceTol, axis+1);
		// check both sub-spaces if the node is inside the box around the target point
		else 
		{
			if ((n->point[0] >= target[0]-distanceTol)&&(n->point[0] <= target[0]+distanceTol) &&
				(n->point[1] >= target[1]-distanceTol)&&(n->point[1] <= target[1]+distanceTol) &&		
				(n->point[2] >= target[2]-distanceTol)&&(n->point[2] <= target[2]+distanceTol))
			{
				// see if the point is within the distace tolerance 
				if (std::sqrt((target[0]-n->point[0])*(target[0]-n->point[0])
							+ (target[1]-n->point[1])*(target[1]-n->point[1])
							+ (target[2]-n->point[2])*(target[2]-n->point[2])) <= distanceTol)
					ids.push_back(n->id);
			}

			searchHelper(target, ids, n->left, distanceTol, axis+1);
			searchHelper(target, ids, n->right, distanceTol, axis+1);
		}
	}
			

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, ids, root, distanceTol);
		return ids;
	}
	

};




