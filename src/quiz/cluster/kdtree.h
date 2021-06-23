/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree() 
	: root(NULL) 
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node* &node, Node* &inNode, uint axis = 0)
	{
		if (node == NULL)
			node = inNode;
		else if (inNode->point[axis % 2] < node->point[axis % 2])
			insertHelper(node->left, inNode, ++axis);
		else 
			insertHelper(node->right, inNode, ++axis);	
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly within the root
		Node* inNode = new Node(point, id); 
		insertHelper(root, inNode); 
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




